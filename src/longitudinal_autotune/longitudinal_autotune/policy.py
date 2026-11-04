"""Hardware-independent two-line experiment state machine and event evaluator."""
from dataclasses import asdict, dataclass
import math
import numpy as np


@dataclass
class Sample:
    t: float
    speed: float
    raw_speed: float
    duty: float = 0.
    brake: float = 0.
    target_speed: float = 0.
    curvature: float = 0.
    yaw_rate: float = 0.
    cte: float = 0.
    line: float = math.nan  # Front-bumper distance, capture-age compensated.
    lane_sequence: int = 0
    lane_ok: bool = True
    telemetry_ok: bool = True
    command_ok: bool = True
    voltage: float = math.nan
    motor_current: float = math.nan
    actual_duty: float = math.nan
    applied_duty: float = math.nan  # Handover-forwarded command, not measured motor duty.
    applied_brake: float = math.nan


def evaluate(rows, final_distance, hold_creep, direction, turn, expected_line, e):
    moving = [r for r in rows if r['phase'] != 'HOLD']
    failures = []
    if not e['stop_range_min_m'] <= final_distance <= e['stop_range_max_m']:
        failures.append('stop_outside_range')
    if hold_creep > e['maximum_hold_creep_m']:
        failures.append('hold_creep')
    sign = -1 if direction == 'cw' else 1
    if abs(sign * turn - e['expected_half_lap_turn_rad']) > e['half_lap_turn_tolerance_rad']:
        failures.append('unexpected_half_lap_turn_or_missed_line')
    if len(moving) < 20:
        failures.append('insufficient_segment_data')
    ts = np.array([r['t'] for r in rows], dtype=float)
    raw = np.array([r['raw_speed'] for r in rows], dtype=float)
    acceleration, jerks, filtered, previous_a = [], [], raw[0], None
    for i in range(1, len(rows)):
        dt = ts[i] - ts[i-1]
        if not .005 <= dt <= .10:
            failures.append('sample_gap')
            previous_a = None
            continue
        next_v = filtered + (1 - math.exp(-dt / .08)) * (raw[i] - filtered)
        a = (next_v - filtered) / dt
        acceleration.append(a)
        if previous_a is not None:
            jerks.append((a - previous_a) / dt)
        filtered, previous_a = next_v, a
    jerk_rms = math.sqrt(float(np.mean(np.square(jerks)))) if jerks else math.inf
    peak_decel = max([0.] + [-x for x in acceleration])
    if jerk_rms > e['maximum_stop_jerk_rms']:
        failures.append('excessive_jerk')
    if peak_decel > e['maximum_deceleration_mps2']:
        failures.append('excessive_deceleration')
    mode = [('brake' if r['brake'] > .05 else 'drive' if r['duty'] > .001 else 'coast') for r in moving]
    # One coast/brake application and one release are normal, not chattering.
    actuated = [m for m in mode if m != 'coast']
    switches = sum(a != b for a, b in zip(actuated, actuated[1:]))
    corner = [r for r in moving if abs(r['curvature']) >= .5]
    overspeed = [max(0., r['speed'] - math.sqrt(e['maximum_lateral_acceleration_mps2'] /
                  max(abs(r['curvature']), 1e-6))) for r in corner]
    error = math.sqrt(float(np.mean([(r['speed'] - r['target_speed'])**2 for r in moving]))) if moving else math.inf
    voltages = [r['voltage'] for r in rows if math.isfinite(r['voltage'])]
    duration = rows[-1]['t'] - rows[0]['t']
    metrics = dict(line_id=expected_line, direction=direction, estimated_stop_distance_m=final_distance,
                   position_error_m=abs(final_distance - e['stop_target_m']), hold_creep_m=hold_creep,
                   jerk_rms_mps3=jerk_rms, peak_deceleration_mps2=peak_decel,
                   drive_brake_switches=switches, speed_rmse_mps=error,
                   corner_overspeed_mps=max(overspeed, default=0.), segment_sec=duration,
                   signed_turn_rad=turn, voltage_v=float(np.median(voltages)) if voltages else None,
                   maximum_cte_m=max(abs(r['cte']) for r in rows), failures=sorted(set(failures)))
    w = e['score_weights']
    metrics['score'] = (w['stop_position'] * metrics['position_error_m'] / .1 +
        w['jerk'] * jerk_rms / 3. + w['creep'] * hold_creep / .015 +
        w['switching'] * max(0, switches - 2) + w['speed_tracking'] * error / .2 +
        w['segment_time'] * duration / 15. + w['corner_overspeed'] * metrics['corner_overspeed_mps'] / .1)
    metrics['feasible'] = not metrics['failures'] and math.isfinite(metrics['score'])
    return metrics


class Experiment:
    """No timers replay lane results. Only fresh samples can advance a trial."""
    def __init__(self, settings, direction):
        if direction not in ('cw', 'ccw'):
            raise ValueError('direction must be cw or ccw')
        self.e, self.direction = settings, direction
        self.phase, self.reason = 'IDLE', 'not_armed'
        self.fault_details = {}
        self.previous = None
        self.remaining = None
        self.last_line_t = None
        self.last_line_distance = None
        self.last_sequence = None
        self.confirmations = 0
        self.stopped_since = self.clear_since = self.hold_since = None
        self.started = self.approach_started = 0.
        self.segment_travel = self.release_travel = self.turn = self.hold_creep = 0.
        self.anchor = True
        self.results, self.rows = [], []

    @property
    def running(self):
        return self.phase in ('DRIVE', 'APPROACH', 'HOLD', 'RELEASE')

    @property
    def red_request(self):
        return self.phase in ('DRIVE', 'APPROACH', 'HOLD')

    def arm(self, now):
        if self.phase != 'IDLE':
            raise ValueError('A block can only be armed once; create a new block after a stop/fault')
        self.phase, self.reason, self.started = 'DRIVE', 'finding_anchor', now

    def fail(self, reason, details=None):
        if self.phase == 'FAULT':
            return  # Preserve the first cause even after stop diagnostics reset.
        self.phase, self.reason = 'FAULT', reason
        self.fault_details = dict(details or {})

    def update(self, s):
        if not self.running:
            return None
        e = self.e
        numbers = (s.t, s.speed, s.raw_speed, s.duty, s.brake, s.target_speed, s.curvature, s.yaw_rate, s.cte)
        if not all(math.isfinite(v) for v in numbers):
            self.fail('nonfinite_observation'); return None
        if not s.lane_ok or not s.telemetry_ok or not s.command_ok:
            self.fail('stale_or_invalid_lane_telemetry_or_command'); return None
        if s.speed < -.03:
            self.fail('reverse_speed_limit', dict(value=s.speed, limit=-.03, unit='m/s')); return None
        if s.speed > e['maximum_speed_mps'] + .3:
            self.fail('overspeed_limit', dict(value=s.speed, limit=e['maximum_speed_mps']+.3, unit='m/s')); return None
        if abs(s.cte) > e['maximum_cte_m']:
            self.fail('cross_track_error_limit', dict(value=s.cte, limit=e['maximum_cte_m'], unit='m')); return None
        if e['require_voltage'] and not math.isfinite(s.voltage):
            self.fail('voltage_unavailable'); return None
        dt = 0. if self.previous is None else s.t - self.previous.t
        if dt < 0 or dt > .15:
            self.fail('clock_or_sample_gap'); return None
        self.previous = s
        travel = max(0., s.raw_speed) * dt
        self.segment_travel += travel
        if s.speed > e['stopped_speed_mps']:
            self.turn += s.yaw_rate * dt
        sign = -1 if self.direction == 'cw' else 1
        if sign * self.turn < -e['maximum_direction_error_rad']:
            self.fail('wrong_travel_direction'); return None
        if s.t - self.started > e['maximum_segment_sec']:
            self.fail('segment_timeout'); return None
        if self.remaining is not None:
            self.remaining -= travel
        new_lane = s.lane_sequence != self.last_sequence
        if new_lane:
            self.last_sequence = s.lane_sequence
            if math.isfinite(s.line):
                consistent = self.remaining is None or abs(s.line - self.remaining) <= e['stop_match_tolerance_m']
                if consistent:
                    self.remaining = s.line if self.remaining is None else .65 * self.remaining + .35 * s.line
                    self.last_line_t, self.last_line_distance = s.t, s.line
                    self.confirmations += 1
                else:
                    self.confirmations = 0
            elif self.phase == 'DRIVE':
                self.confirmations = 0
                self.remaining = None
        row = asdict(s)
        row['phase'] = self.phase
        self.rows.append(row)
        if self.phase == 'RELEASE':
            self.release_travel += travel
            self.clear_since = (self.clear_since or s.t) if not math.isfinite(s.line) else None
            if (self.release_travel >= e['release_min_travel_m'] and self.clear_since is not None and
                    s.t - self.clear_since >= e['release_clear_sec']):
                self.phase = 'DRIVE'
                self.remaining, self.last_line_t = None, None
                self.confirmations = 0
            return None
        if self.phase == 'DRIVE':
            if self.confirmations >= e['stop_confirm_frames'] and self.remaining is not None:
                self.phase, self.approach_started = 'APPROACH', s.t
            return None
        if self.remaining is not None and self.remaining < e['stop_range_min_m'] - .03:
            self.fail('stop_line_overshoot'); return None
        if self.phase == 'APPROACH':
            if s.t - self.approach_started > e['maximum_approach_sec']:
                self.fail('approach_timeout'); return None
            age = math.inf if self.last_line_t is None else s.t - self.last_line_t
            terminal = (self.last_line_distance is not None and
                        self.last_line_distance <= e['terminal_observation_distance_m'] and
                        age <= e['terminal_observation_timeout_sec'])
            if age > e['stop_observation_timeout_sec'] and not terminal:
                self.fail('stop_line_lost_before_terminal'); return None
            stopped = abs(s.raw_speed) <= e['stopped_speed_mps'] and s.target_speed <= .01
            self.stopped_since = (self.stopped_since or s.t) if stopped else None
            near = self.remaining is not None and self.remaining <= e['stop_range_max_m'] + .02
            if self.stopped_since is not None:
                stopped_for = s.t - self.stopped_since
                if not near and stopped_for >= e['maximum_stopped_before_line_sec']:
                    self.fail('premature_stop'); return None
                if near and stopped_for >= e['stopped_confirm_sec']:
                    self.phase, self.hold_since, self.hold_creep = 'HOLD', s.t, 0.
            return None
        if self.phase == 'HOLD':
            # Forward the tested controller during this window; an external hold
            # would hide its twitching. Large movement aborts instead of scoring well.
            self.hold_creep += abs(s.raw_speed) * dt
            if abs(s.raw_speed) > e['maximum_hold_speed_mps'] or s.duty > .001:
                self.fail('hold_motion_or_positive_drive'); return None
            if s.t - self.hold_since < e['hold_sec']:
                return None
            if self.anchor:
                self.anchor = False
                event = {'anchor': True, 'estimated_stop_distance_m': self.remaining}
            else:
                if self.segment_travel < e['minimum_stop_spacing_m']:
                    self.fail('same_line_retrigger_or_invalid_spacing'); return None
                event = evaluate(self.rows, self.remaining, self.hold_creep, self.direction,
                                 self.turn, (len(self.results)+1) % 2, e)
                self.results.append(event)
                if not event['feasible']:
                    self.fail('evaluation_failed'); return event
            self.phase = 'BLOCK_DONE' if len(self.results) >= 2 * e['laps_per_direction'] else 'RELEASE'
            self.reason = 'block_complete' if self.phase == 'BLOCK_DONE' else 'released_after_hold'
            self.remaining = self.last_line_t = self.last_line_distance = None
            self.confirmations = 0
            self.segment_travel = self.release_travel = self.turn = 0.
            self.stopped_since = self.clear_since = self.hold_since = None
            self.started, self.rows = s.t, []
            return event
