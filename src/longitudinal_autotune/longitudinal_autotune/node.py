"""Experiment coordinator. All hardware commands pass through auto_handover."""
from dataclasses import asdict
from pathlib import Path
import fcntl
import hashlib
import json
import math
import time
import uuid
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from ament_index_python.packages import get_package_prefix
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32, Int32, String
from std_srvs.srv import SetBool, Trigger
from line_detactor.msg import LaneResult
from traffic_detection_test.msg import TrafficLightState
from auto_control_tunnel.path_validation import PathRules, metric_path, stamp_ns
from .config import PREFIX as P, fingerprint
from .controller import Controller
from .gate import Inputs, motor_output
from .policy import Experiment, Sample, POLICY_VERSION
from .search import Campaign
from .storage import Trace, atomic_write


def file_hash(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            h.update(chunk)
    return h.hexdigest()


class Session(Node):
    def __init__(self):
        super().__init__('autotune_session')
        self._autotune_closed = False
        for k, v in dict(inputs_file='', campaign_dir='', direction='cw', model_path='').items():
            self.declare_parameter(k, v)
        data = yaml.safe_load(Path(self.get_parameter('inputs_file').value).read_text())
        self.document, self.base = data['document'], data['controller']
        self.e = self.document['experiment']
        self.direction = self.get_parameter('direction').value
        root = Path(self.get_parameter('campaign_dir').value).expanduser()
        root.mkdir(parents=True, exist_ok=True)
        self.lock = (root/'.session.lock').open('a')
        fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        executable = Path(get_package_prefix('auto_control'))/'lib/auto_control/auto_control_node'
        identity = dict(controller_sha256=file_hash(executable),
                        model_sha256=file_hash(self.get_parameter('model_path').value),
                        baseline_sha256=fingerprint(self.base), perception_sha256=fingerprint(data['perception']), format_version=2, experiment_policy_version=POLICY_VERSION)
        self.campaign = Campaign(root, self.document, self.base, identity)
        # Resume at the next candidate only after a fully recorded paired trial.
        if not self.campaign.state['complete'] and (not self.campaign.needed() or self.campaign.current.get('feasible') is False):
            self.campaign.advance()
        self.run_id = time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:8]
        self.run_root = root/'runs'/self.run_id
        self.run_root.mkdir(parents=True)
        atomic_write(self.run_root/'inputs.yaml', data, True)
        self.trace = Trace(self.run_root/'trace.jsonl')
        self.controller = Controller(self.run_root)
        self.inputs = Inputs()
        self.experiment = Experiment(self.e, self.direction)
        self.mode, self.connected, self.model_ready = '', False, False
        self.raw_mode, self.sequence, self.lane_stamp = 'brake_current', 0, 0
        self.line_at_rx, self.lane_at_rx = math.nan, -math.inf
        self.lane_valid = False
        self.servo = float(self.base['servo_center'])
        self.submitted = self.auto_arm = False
        self.barrier = None
        self.last_phase, self.last_status = '', 0.
        self.tracking_warning = False
        self.last_enable, self.last_enable_at = None, -math.inf
        self.publisher_conflict = False
        self.graph_checked = -math.inf
        self.generation = 0
        ratio = (self.base['spur_gear_teeth']/self.base['motor_pinion_teeth'] *
                 self.base['differential_ring_teeth']/self.base['differential_pinion_teeth'])
        self.erpm_scale = (math.pi*self.base['wheel_diameter_m']/(60*self.base['motor_pole_pairs']*ratio) *
                           self.base['erpm_direction_sign']*self.base['speed_scale_correction'])
        self.rules = PathRules(frame_id=self.base['lane_result_frame_id'],
            minimum_points=self.base['path_minimum_points'], minimum_span=self.base['path_minimum_span_m'],
            minimum_x=self.base['path_minimum_x_m'], maximum_x=self.base['path_maximum_x_m'],
            maximum_gap=self.base['path_maximum_gap_m'])
        best_effort = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        state = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.gate = {c: self.create_publisher(Float32, P+'/gate/'+c, best_effort)
                     for c in ('duty', 'brake_current', 'servo_position')}
        self.enable = self.create_publisher(Bool, P+'/controller_enabled', best_effort)
        self.stop = self.create_publisher(TrafficLightState, P+'/stop_request', 1)
        self.status = self.create_publisher(String, P+'/status', state)
        self.tracking_status = self.create_publisher(String, P+'/tracking', state)
        self.fault_status = self.create_publisher(String, P+'/fault', state)
        self._autotune_subscriptions = []
        def sub(kind, topic, callback, qos=best_effort):
            self._autotune_subscriptions.append(self.create_subscription(kind, topic, callback, qos))
        sub(Int32, P+'/vesc/measured_erpm', lambda m: self.inputs.set('raw_speed', m.data*self.erpm_scale, time.monotonic()))
        sub(Bool, P+'/vesc/connected', lambda m: setattr(self, 'connected', m.data), state)
        sub(String, P+'/drive/mode', self.on_mode, state)
        sub(Bool, P+'/operator_enabled', self.on_operator)
        for topic, key in [('current_duty', 'applied_duty'), ('current_brake_current', 'applied_brake')]:
            sub(Float32, P+'/drive/'+topic, lambda m, k=key: self.inputs.set(k, m.data, time.monotonic()))
        sub(String, P+'/autotune_lane/model_status', lambda m: setattr(self, 'model_ready', m.data.startswith('READY |')), state)
        sub(LaneResult, P+'/lane', self.on_lane)
        sub(Imu, P+'/camera/imu', self.on_imu)
        for channel in ('duty', 'brake_current', 'servo_position'):
            sub(Float32, P+'/raw/'+channel, lambda m, c=channel: self.on_raw(c, m.data))
        for topic, key in [('current_speed', 'speed'), ('target_speed', 'target_speed'),
                           ('path_curvature', 'curvature'), ('cross_track_error_m', 'cte')]:
            sub(Float32, P+'/controller/'+topic, lambda m, k=key: self.inputs.set(k, m.data, time.monotonic()))
        for topic, key in [('input_voltage_v', 'voltage'), ('motor_current_a', 'motor_current'), ('duty_cycle', 'actual_duty')]:
            sub(Float32, P+'/dynamics/'+topic, lambda m, k=key: self.inputs.set(k, m.data, time.monotonic()))
        self.create_service(SetBool, P+'/arm', self.arm)
        self.create_service(Trigger, P+'/finish', self.finish)
        self.configure()
        self.create_timer(.02, self.tick)
        self.get_logger().info(f'Autotune policy_version={POLICY_VERSION} | session_code={Path(__file__).resolve()}')
        self.get_logger().info('Disarmed. Candidate '+str(self.campaign.current['id'])+
            ', directions still needed: '+str(self.campaign.needed())+'. Arm service starts motion; X hands over to manual.')

    def record(self, kind, **data):
        self.trace.put(dict(kind=kind, run=self.run_id, candidate=self.campaign.current['id'],
                            direction=self.direction, **data))

    def configure(self):
        p = dict(self.base, **self.campaign.current['values'])
        self.controller.request(p)
        self.record('configuration', parameters=p)

    def on_mode(self, msg):
        self.mode = msg.data
        if self.mode != 'AUTO' and self.experiment.running:
            self.experiment.fail('manual_handover')
            self.auto_arm = False

    def on_operator(self, msg):
        if not msg.data:
            self.auto_arm = False
            if self.experiment.running:
                self.experiment.fail('operator_stop')

    def on_raw(self, channel, value):
        now = time.monotonic()
        if channel == 'servo_position':
            self.inputs.set('servo', value, now)
        else:
            self.raw_mode = channel
            self.inputs.set('duty', value if channel == 'duty' else 0., now)
            self.inputs.set('brake', value if channel == 'brake_current' else 0., now)
            self.record('controller_motor', t=now, mode=channel, value=value)

    def on_imu(self, msg):
        # Direct gyro freshness is required: a periodic dynamics publisher could
        # otherwise continue reporting a zero yaw after the IMU goes stale.
        p = self.document.get('vehicle_dynamics_monitor', {}).get('ros__parameters', {})
        axis = p.get('imu_yaw_axis', 'y')
        value = getattr(msg.angular_velocity, axis)
        captured = msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds-captured)*1e-9
        if not captured or not -.05 <= age <= self.e['telemetry_timeout_sec']:
            return
        if msg.angular_velocity_covariance[0] == -1:
            return
        yaw = p.get('imu_yaw_rate_sign', -1.) * (value-p.get('imu_yaw_rate_bias_radps', 0.))
        self.inputs.set('yaw_rate', yaw, time.monotonic())

    def on_lane(self, msg):
        now = time.monotonic()
        capture = stamp_ns(msg)
        age = (self.get_clock().now().nanoseconds-capture)*1e-9
        valid_time = capture > self.lane_stamp and -.05 <= age <= self.base['path_capture_maximum_age_sec']
        if not valid_time:
            self.lane_valid = False
            return
        self.lane_stamp = capture
        self.sequence += 1
        self.lane_at_rx = now
        self.lane_valid = metric_path(msg, self.rules) is not None
        distance = float(msg.stop_line_distance_m)
        self.line_at_rx = (distance-self.base['traffic_front_axle_to_bumper_m']-
                           max(0., self.inputs.get('speed', 0.))*max(0., age)
                           if msg.stop_line_present and math.isfinite(distance) and 0 <= distance <= 6. else math.nan)
        self.record('lane', t=now, capture_ns=capture, capture_age_sec=age, sequence=self.sequence,
                    valid=self.lane_valid, stop_distance_bumper_m=self.line_at_rx)

    def sample(self, now):
        i, e = self.inputs, self.e
        command = i.fresh(('duty', 'brake', 'servo', 'speed', 'target_speed', 'curvature', 'cte', 'applied_duty', 'applied_brake'),
                          now, e['command_timeout_sec'], after=self.controller.started)
        telemetry = self.connected and i.fresh(('raw_speed', 'yaw_rate'), now, e['telemetry_timeout_sec'])
        lane = self.lane_valid and 0 <= now-self.lane_at_rx <= e['lane_timeout_sec']
        optional = lambda k: i.get(k) if i.fresh((k,), now, .3) else math.nan
        # Distance is only assimilated once per unique LaneResult in Experiment.
        return Sample(t=now, speed=i.get('speed'), raw_speed=i.get('raw_speed'), duty=i.get('duty'),
            brake=i.get('brake'), target_speed=i.get('target_speed'), curvature=i.get('curvature'),
            yaw_rate=i.get('yaw_rate'), cte=i.get('cte'), line=(self.line_at_rx-max(0., i.get('raw_speed', 0.))*(now-self.lane_at_rx)
                               if math.isfinite(self.line_at_rx) else math.nan),
            lane_sequence=self.sequence, lane_ok=lane, telemetry_ok=telemetry, command_ok=command,
            voltage=optional('voltage'), motor_current=optional('motor_current'), actual_duty=optional('actual_duty'),
            applied_duty=i.get('applied_duty'), applied_brake=i.get('applied_brake'))

    def stopped(self, now):
        return self.connected and self.inputs.fresh(('raw_speed',), now, self.e['telemetry_timeout_sec']) and abs(self.inputs.get('raw_speed')) <= self.e['stopped_speed_mps']

    def ready(self, now):
        s = self.sample(now)
        return (self.controller.alive() and self.model_ready and self.mode == 'AUTO' and
                s.lane_ok and s.telemetry_ok and s.command_ok and not self.publisher_conflict and
                now-self.graph_checked < 2. and not self.trace.failed and not self.trace.dropped)

    def arm(self, request, response):
        now = time.monotonic()
        if not request.data:
            self.auto_arm = False
            if self.experiment.running: self.experiment.fail('operator_stop')
            response.success, response.message = True, 'Disarmed; interrupted block cannot become best'
        elif self.experiment.phase != 'IDLE':
            response.success, response.message = False, 'Restart launch for a stopped/interrupted block'
        elif self.campaign.state['complete'] or self.direction not in self.campaign.needed():
            response.success, response.message = False, 'No pending block in this direction; inspect /longitudinal_autotune/status'
        elif not self.ready(now) or not self.stopped(now):
            response.success, response.message = False, 'Waiting for valid lane, model, IMU, AUTO handover, stopped ERPM, controller and unique publishers'
        else:
            self.experiment.arm(now)
            self.record('block_started')
            response.success, response.message = True, 'Armed: finding unscored anchor, then two laps'
        return response

    def finish(self, request, response):
        if self.experiment.running or not self.stopped(time.monotonic()):
            response.success, response.message = False, 'Disarm and confirm fresh stopped ERPM first'
        else:
            self.auto_arm = False
            self.submitted = True
            self.campaign.state['complete'] = True
            self.campaign.save()
            response.success, response.message = True, 'Campaign finished; only previously validated best remains saved'
        return response

    def audit(self, now):
        self.graph_checked = now
        # No camera/bridge is created here. Reject extra owners on the three
        # exclusive command paths, including unrelated publishers in this domain.
        conflict = False
        for channel in ('duty', 'brake_current', 'servo_position'):
            for topic, owner in [(P+'/gate/'+channel, 'autotune_session'),
                                  (P+'/vesc/'+channel, 'autotune_handover'),
                                  (P+'/raw/'+channel, f'autotune_controller_{self.controller.generation}')]:
                pubs = self.get_publishers_info_by_topic(topic)
                if len(pubs) != 1 or any(x.node_name != owner or x.node_namespace != P for x in pubs):
                    conflict = True
        if self.get_publishers_info_by_topic(P+'/controller/maximum_speed_request'):
            conflict = True
        self.publisher_conflict = conflict

    def tick(self):
        now = time.monotonic()
        if self.controller.tick():
            self.publisher_conflict = True
            self.graph_checked = -math.inf
            self.inputs.values = {k:v for k,v in self.inputs.values.items() if k not in ('duty','brake','servo','speed','target_speed','curvature','cte')}
        if now-self.graph_checked >= 1.: self.audit(now)
        if self.auto_arm and self.ready(now) and self.stopped(now):
            self.auto_arm = False
            self.experiment.arm(now)
            self.record('block_started')
        was_running = self.experiment.running
        if was_running:
            if not self.controller.alive() or self.mode != 'AUTO' or self.publisher_conflict:
                self.experiment.fail('controller_or_handover_or_publisher_conflict')
            elif self.trace.failed or self.trace.dropped:
                self.experiment.fail('trace_incomplete')
            elif not self.model_ready:
                self.experiment.fail('model_not_ready')
            else:
                s = self.sample(now)
                try:
                    motor_output(s.duty, s.brake, self.inputs.get('servo'), self.raw_mode, True,
                                 self.e['emergency_brake_current_a'], self.e['hold_current_a'], False)
                except ValueError:
                    self.experiment.fail('invalid_actuator_command')
                event = self.experiment.update(s)
                self.record('sample', data=dict(asdict(s), phase=self.experiment.phase, segment=len(self.experiment.results), anchor=self.experiment.anchor))
                if event: self.record('stop_evaluation', data=event)
        permitted = self.experiment.running and self.ready(now)
        if self.experiment.running and not permitted:
            self.experiment.fail('readiness_lost')
        channel, value = motor_output(self.inputs.get('duty'), self.inputs.get('brake'),
            self.inputs.get('servo'), self.raw_mode, permitted, self.e['emergency_brake_current_a'],
            self.e['hold_current_a'], self.stopped(now))
        if permitted != self.last_enable or now-self.last_enable_at >= .5:
            self.enable.publish(Bool(data=permitted))
            self.last_enable, self.last_enable_at = permitted, now
        self.gate[channel].publish(Float32(data=float(value)))
        if permitted:
            self.servo = self.inputs.get('servo')
        self.gate['servo_position'].publish(Float32(data=self.servo))
        if was_running: self.record('gate_motor', t=now, mode=channel, value=value, permitted=permitted)
        self.generation += 1
        signal = TrafficLightState()
        signal.header.stamp = self.get_clock().now().to_msg()
        signal.header.frame_id = 'autotune_internal_stop_request'
        signal.state = TrafficLightState.RED if self.experiment.red_request else TrafficLightState.GREEN
        signal.detection_score, signal.source_generation = 1., self.generation
        self.stop.publish(signal)
        # Flush/fsync in the logging worker, never block a moving control timer.
        if self.experiment.phase == 'BLOCK_DONE' and not self.submitted:
            if self.barrier is None:
                self.record('block_complete', results=self.experiment.results)
                self.barrier = self.trace.barrier()
            elif self.barrier.is_set() and self.stopped(now):
                if self.trace.failed or self.trace.dropped:
                    self.experiment.fail('trace_incomplete')
                else:
                    self.campaign.submit(self.direction, self.experiment.results)
                    self.submitted = True
                    if not self.campaign.needed() and self.campaign.advance():
                        self.experiment = Experiment(self.e, self.direction)
                        self.tracking_warning = False
                        self.submitted, self.barrier, self.auto_arm = False, None, True
                        self.configure()  # Vehicle has completed its two-second stopped hold.
        # Only measured comfort failures reject a candidate. Perception, logger,
        # telemetry and operator interruptions remain incomplete and are retried.
        if (self.experiment.phase == 'FAULT' and not self.submitted and
                self.experiment.reason in ('evaluation_failed', 'hold_motion_or_positive_drive')):
            if self.barrier is None:
                self.record('candidate_rejected', reason=self.experiment.reason, results=self.experiment.results)
                self.barrier = self.trace.barrier()
            elif self.barrier.is_set() and self.stopped(now) and not self.trace.failed and not self.trace.dropped:
                self.campaign.submit(self.direction, self.experiment.results, failure=self.experiment.reason)
                self.submitted = True
        tracking = self.experiment.tracking.metrics()
        if tracking['cte_warning'] != self.tracking_warning:
            self.tracking_warning = tracking['cte_warning']
            self.record('tracking_warning', active=self.tracking_warning, data=tracking)
            state = 'exceeded; continuing evaluation' if self.tracking_warning else 'recovered'
            self.get_logger().info(f'Tracking target {state}: cte={tracking["current_cte_m"]:+.4f}m '
                                   f'soft={self.e["maximum_cte_m"]:.3f}m hard={self.e["hard_cte_limit_m"]:.3f}m')
        if self.experiment.phase != self.last_phase:
            details = self.experiment.fault_details
            self.record('phase', phase=self.experiment.phase, reason=self.experiment.reason, details=details)
            text = self.experiment.reason
            if details:
                text += f" value={details['value']:+.4f}{details['unit']} limit={details['limit']:.4f}{details['unit']}"
            self.get_logger().info(f'Candidate {self.campaign.current["id"]} {self.direction}: {self.experiment.phase} ({text})')
            if self.experiment.phase == 'FAULT':
                self.fault_status.publish(String(data=text))
            self.last_phase = self.experiment.phase
        if now-self.last_status >= .5:
            self.last_status = now
            self.tracking_status.publish(String(data=json.dumps(tracking)))
            self.status.publish(String(data=json.dumps(dict(phase=self.experiment.phase, reason=self.experiment.reason,
                fault=self.experiment.fault_details, policy_version=POLICY_VERSION, tracking=tracking, candidate=self.campaign.current['id'], direction=self.direction, needed=self.campaign.needed(),
                complete=self.campaign.state['complete'], best_id=(self.campaign.state['best'] or {}).get('id'),
                ready=self.ready(now), trace=str(self.trace.path), trace_dropped=self.trace.dropped))))

    def close(self):
        if self._autotune_closed:
            return
        self._autotune_closed = True
        self.auto_arm = False
        try:
            # SIGINT may already have invalidated the ROS context. Publishing
            # failure must never skip termination of the owned controller.
            try:
                self.enable.publish(Bool(data=False))
                self.gate['brake_current'].publish(Float32(data=float(self.e['emergency_brake_current_a'])))
            except Exception as error:
                self.record('shutdown_publish_error', error=str(error))
            self.record('session_closed', phase=self.experiment.phase, incomplete=not self.submitted)
        finally:
            try:
                self.controller.close()
            finally:
                try:
                    self.trace.close()
                finally:
                    self.lock.close()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Session()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node is not None:
                try: node.close()
                finally: node.destroy_node()
        finally:
            if rclpy.ok(): rclpy.shutdown()
