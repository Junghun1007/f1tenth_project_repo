#!/usr/bin/env python3
"""터널 IR 지연 원인 측정 (일회용, 측정 뒤 폴더째 삭제).

가설: IMU 영상 안정화 보정이 끝나기 전에는 터널 IR 프레임이 모두 버려져 IR 차선이 늦게(보정 완료 뒤에) 들어온다.
터널 없이 웹 "터널 IR 강제"로 진입 뒤 경로를 돌리고, 각 단계 첫 도착 시각을 IMU 보정 완료 시각과 비교한다.

  T1 기준        : 보정 완료 상태에서 IR ON → IR이 바로(1 s 안) 와야 함
  T2 REFIT 직후  : REFIT(보정 재시작) → 바로 IR ON → 첫 IR ≈ 보정 완료 시각이면 가설 지지
  T3 REFIT+흔들기: T2 + 안내에 따라 8 s 흔든 뒤 내려놓기 → 흔드는 동안 IR 없음 (--with-motion)
  T4 대조        : 보정 완료 상태에서 IR ON + 흔들기 → IR 계속 도착 (--with-motion)

전제: ros2 launch vehicle_bringup auto_drive.launch.py tunnel_enabled:=true mode:=detect (290514c 이후 빌드),
VESC 연결, 차는 바닥에 정지. 실행: source install/setup.bash && python3 tools/tunnel_ir_probe/probe.py
Python 3.8 호환 (Jetson Humble).
"""
import argparse
import csv
import json
import os
import re
import statistics
import sys
import threading
import time
import urllib.request

IR_CAMERA_RE = re.compile(r'\[IR CAMERA\].*?imu_wait_expired=(\d+).*?imu_initialized=(\w+)')
ROSOUT_KEYS = ('[IR CAMERA]', 'IMU stabilization', 'Refit ground reference', 'recalibrating IMU',
               'Tunnel ', 'lane source', 'refit', 'Refit', 'tunnel ')


def now():
    return time.monotonic()


class Recorder:
    """모든 관측을 받아 시각(monotonic)과 함께 보관한다. 스레드 안전."""

    def __init__(self):
        self.lock = threading.Lock()
        self.events = []          # (t, kind, detail)
        self.telemetry = None     # 마지막 웹 텔레메트리 dict
        self.telemetry_t = None
        self.imu_calibrated = None   # True/False/None(알 수 없음: 예전 빌드 또는 안정화 안 씀)
        self.state = None
        self.tunnel_state = None
        self.counts = {'ir_bev': 0, 'ir_raw': 0, 'ir_raw_valid': 0, 'ir_fwd': 0}
        self.first = {}           # kind -> [t, ...] 도착 시각 목록 (창 분석용)
        self.imu_wait_expired = []   # (t, 값)
        self.imu_initialized = []    # (t, 'yes'/'no')
        self.rosout_lines = []

    def event(self, kind, detail=''):
        with self.lock:
            self.events.append((now(), kind, detail))

    def arrival(self, kind, valid=None):
        t = now()
        with self.lock:
            self.counts[kind] += 1
            self.first.setdefault(kind, []).append(t)
            if valid:
                self.counts['ir_raw_valid'] += 1
                self.first.setdefault('ir_raw_valid', []).append(t)

    def on_telemetry(self, data):
        t = now()
        readiness = data.get('readiness') or {}
        imu = readiness.get('imu_calibrated')
        state = data.get('state')
        tunnel_state = (data.get('tunnel') or {}).get('state')
        with self.lock:
            self.telemetry, self.telemetry_t = data, t
            if imu is not None and imu != self.imu_calibrated:
                self.events.append((t, 'imu_calibrated', str(imu)))
            if imu is not None:
                self.imu_calibrated = imu
            if state != self.state:
                self.events.append((t, 'state', str(state)))
                self.state = state

    def on_rosout(self, name, text):
        t = now()
        with self.lock:
            self.rosout_lines.append('%.3f [%s] %s' % (t, name, text))
            match = IR_CAMERA_RE.search(text)
            if match:
                self.imu_wait_expired.append((t, int(match.group(1))))
                self.imu_initialized.append((t, match.group(2)))
            if 'IMU stabilization calibrated' in text:
                self.events.append((t, 'log_imu_calibrated', ''))
            if 'recalibrating IMU' in text:
                self.events.append((t, 'log_imu_recalibrating', ''))

    def snapshot(self):
        with self.lock:
            return dict(telemetry=self.telemetry, telemetry_t=self.telemetry_t, imu=self.imu_calibrated,
                        state=self.state)

    def times(self, kind, start, end=None):
        with self.lock:
            return [t for t in self.first.get(kind, []) if t >= start and (end is None or t <= end)]

    def events_of(self, kind, start, end=None):
        with self.lock:
            return [(t, d) for t, k, d in self.events if k == kind and t >= start and (end is None or t <= end)]

    def rosout_between(self, start, end):
        with self.lock:
            expired = sum(v for t, v in self.imu_wait_expired if start <= t <= end)
            init = [v for t, v in self.imu_initialized if start <= t <= end]
        return expired, init


class Web:
    """auto_drive 웹 API (웹 GUI 버튼과 같은 경로) + SSE 텔레메트리."""

    def __init__(self, host, port, rec, client='tunnel_ir_probe'):
        self.base = 'http://%s:%d' % (host, port)
        self.rec = rec
        self.client = client
        self.stop = threading.Event()
        self.connected = threading.Event()

    def post(self, path, body=None):
        data = json.dumps(dict(body or {}, client=self.client)).encode()
        request = urllib.request.Request(self.base + path, data=data, headers={'Content-Type': 'application/json'})
        try:
            with urllib.request.urlopen(request, timeout=5) as response:
                return json.loads(response.read().decode() or '{}')
        except Exception as error:   # noqa: BLE001 - 측정 도구: 실패 사유만 남김
            return {'ok': False, 'reason': 'request failed: %s' % error}

    def start(self):
        # 이 스크립트도 웹 클라이언트로 등록 (heartbeat 없으면 서버가 텔레메트리를 만들지 않음). 프리뷰 켬 = 30 Hz 전체 텔레메트리
        self.post('/api/view', {'preview': True, 'bev': False, 'camera': False})
        threading.Thread(target=self._heartbeat, daemon=True).start()
        threading.Thread(target=self._events, daemon=True).start()

    def _heartbeat(self):
        while not self.stop.is_set():
            self.post('/api/heartbeat')
            self.stop.wait(0.2)

    def _events(self):
        while not self.stop.is_set():
            try:
                with urllib.request.urlopen(self.base + '/api/events', timeout=5) as stream:
                    for raw in stream:
                        if self.stop.is_set():
                            return
                        line = raw.decode('utf-8', 'replace').strip()
                        if line.startswith('data: '):
                            self.connected.set()
                            self.rec.on_telemetry(json.loads(line[6:]))
            except Exception:   # noqa: BLE001 - 끊기면 다시 붙는다
                self.stop.wait(0.5)


def start_ros(rec):
    """ROS 토픽·/rosout 구독 (별도 스레드에서 spin)."""
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
    from std_msgs.msg import String
    from sensor_msgs.msg import Image
    from rcl_interfaces.msg import Log
    from line_detactor_msgs.msg import LaneResult

    rclpy.init()
    node = Node('tunnel_ir_probe')
    sensor = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE,
                        history=HistoryPolicy.KEEP_LAST)
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST)
    rosout = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE,
                        history=HistoryPolicy.KEEP_LAST)

    def on_tunnel_state(message):
        rec.tunnel_state = message.data
        rec.event('tunnel_state', message.data)

    subs = [
        node.create_subscription(String, '/tunnel/state', on_tunnel_state, latched),
        node.create_subscription(String, '/tunnel/selected_source', lambda m: rec.event('selected_source', m.data), latched),
        node.create_subscription(Image, '/camera/image_bev_ir', lambda m: rec.arrival('ir_bev'), sensor),
        node.create_subscription(LaneResult, '/ir_bev_line_detection/result',
                                 lambda m: rec.arrival('ir_raw', valid=bool(m.centerline_valid)), sensor),
        node.create_subscription(LaneResult, '/tunnel/lane_result/ir', lambda m: rec.arrival('ir_fwd'), sensor),
        node.create_subscription(Log, '/rosout', lambda m: rec.on_rosout(m.name, m.msg)
                                 if any(k in m.msg for k in ROSOUT_KEYS) else None, rosout),
    ]
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    def shutdown():
        del subs[:]
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return shutdown


class Aborted(Exception):
    pass


class Prober:
    def __init__(self, web, rec, args, out):
        self.web, self.rec, self.args, self.out = web, rec, args, out
        self.ir_on = False
        self.motion = False   # 흔드는 동안은 속도 검사를 하지 않음 (detect 모드: 자동 출력 없음)
        self.rows = []

    # ---- 안전 ----
    def check_safe(self):
        snap = self.rec.snapshot()
        data = snap['telemetry'] or {}
        if snap['state'] in ('DRIVING', 'ARMING', 'STOPPING'):
            raise Aborted('state %s: 자동 주행 중에는 측정하지 않음' % snap['state'])
        speed = (data.get('vehicle') or {}).get('speed')
        if not self.motion and speed is not None and abs(speed) > 0.05:
            raise Aborted('speed %.2f m/s: 차를 세운 뒤 다시 실행' % speed)
        if self.rec.telemetry_t is None or now() - self.rec.telemetry_t > 2.0:
            raise Aborted('웹 텔레메트리 끊김')

    def wait(self, predicate, timeout, step=0.02):
        end = now() + timeout
        while now() < end:
            self.check_safe()
            if predicate():
                return True
            time.sleep(step)
        return False

    def sleep(self, seconds):
        self.wait(lambda: False, seconds)

    def set_ir(self, enabled):
        if enabled:
            self.check_safe()
            data = self.rec.snapshot()['telemetry'] or {}
            if not (data.get('tunnel') or {}).get('force_ir_change_allowed', False):
                raise Aborted('강제 IR 조건 불충족 (READY/MANUAL·정지·최신 ERPM·안전 MANUAL 필요)')
        reply = self.web.post('/api/tunnel/force-ir', {'enabled': enabled})
        if not reply.get('ok'):
            raise Aborted('force-ir %s 실패: %s' % (enabled, reply.get('reason')))
        self.ir_on = enabled

    def ir_off_safely(self):
        if self.ir_on:
            reply = self.web.post('/api/tunnel/force-ir', {'enabled': False})
            self.ir_on = not reply.get('ok', False)

    def prompt(self, text, seconds):
        end = now() + seconds
        while now() < end:
            sys.stdout.write('\r\a  >>> %s  (%2d s) ' % (text, int(end - now()) + 1))
            sys.stdout.flush()
            self.sleep(min(1.0, max(0.0, end - now())))
        sys.stdout.write('\n')

    # ---- 준비 ----
    def wait_calibrated(self, timeout=40.0):
        if self.rec.snapshot()['imu'] is None:
            print('  ! readiness.imu_calibrated 없음 (290514c 이전 빌드?) → [IR CAMERA] 로그로만 판정')
            return
        print('  IMU 보정 완료 대기 (정지 유지)...')
        if not self.wait(lambda: self.rec.snapshot()['imu'] is True, timeout):
            raise Aborted('IMU 보정이 %.0f s 안에 끝나지 않음' % timeout)

    def refit(self):
        t0 = now()
        reply = self.web.post('/api/refit')
        if not reply.get('ok'):
            raise Aborted('refit 실패: %s' % reply.get('reason'))
        if not self.wait(lambda: self.rec.snapshot()['state'] == 'REFITTING', 5):
            raise Aborted('REFITTING 상태로 바뀌지 않음')
        if not self.wait(lambda: self.rec.snapshot()['state'] in ('READY', 'MANUAL'), 60):
            raise Aborted('REFIT이 60 s 안에 끝나지 않음')
        # 카메라가 새 지면 기준을 받아 보정을 다시 시작하는 시점은 READY와 약간 어긋날 수 있다 → 잠깐 기다린다
        if self.rec.snapshot()['imu'] is not None and not self.wait(lambda: self.rec.snapshot()['imu'] is False, 3):
            print('  ! REFIT 뒤 IMU 보정이 다시 시작되지 않음 (지면 기준이 같으면 재보정하지 않음)')
        return t0

    # ---- 측정 창 ----
    def window(self, test, index, motion_after_on=None):
        """IR ON → 도착 측정 → IR OFF. motion_after_on: (흔들기 s, 정지 안내) 또는 None."""
        snap = self.rec.snapshot()
        imu_at_on = snap['imu']
        t_on = now()
        self.set_ir(True)
        t_still = None
        if motion_after_on:
            self.motion = True
            try:
                self.prompt('지금 차를 손으로 계속 흔드세요', motion_after_on)
            finally:
                self.motion = False
            self.prompt('내려놓고 가만히 두세요', 1)
            t_still = now()
        max_wait = self.args.max_wait
        self.wait(lambda: bool(self.rec.times('ir_bev', t_on)), max(0.0, t_on + max_wait - now()))
        first_bev = (self.rec.times('ir_bev', t_on) or [None])[0]
        hold_end = (first_bev or now()) + self.args.hold
        self.sleep(max(0.0, hold_end - now()))
        t_end = now()
        self.set_ir(False)
        self.wait(lambda: self.rec.tunnel_state == 'OUTSIDE', 5)
        row = self.measure(test, index, t_on, t_end, imu_at_on, t_still)
        self.rows.append(row)
        self.write()
        print('  %s #%d: 첫 IR BEV %s / 보정 완료 %s / 판정 %s' % (
            test, index, fmt(row['first_ir_bev_s']), fmt(row['imu_calibrated_s']), row['verdict']))
        self.sleep(self.args.settle)
        return row

    def measure(self, test, index, t_on, t_end, imu_at_on, t_still):
        rel = lambda t: None if t is None else round(t - t_on, 3)   # noqa: E731
        inside = self.rec.events_of('tunnel_state', t_on, t_end)
        inside_t = next((t for t, d in inside if d == 'INSIDE'), None)
        calibrated = next((t for t, d in self.rec.events_of('imu_calibrated', t_on, t_end) if d == 'True'), None)
        uncalibrated_seen = any(d == 'False' for _, d in self.rec.events_of('imu_calibrated', t_on - 70, t_end))
        bev = self.rec.times('ir_bev', t_on, t_end)
        raw = self.rec.times('ir_raw', t_on, t_end)
        valid = self.rec.times('ir_raw_valid', t_on, t_end)
        fwd = self.rec.times('ir_fwd', t_on, t_end)
        expired, init = self.rec.rosout_between(t_on, t_end)
        fps = (len(bev) - 1) / (bev[-1] - bev[0]) if len(bev) > 1 and bev[-1] > bev[0] else 0.0
        motion_gap = None   # T4: 흔드는 동안 IR BEV 최대 간격
        if t_still and bev:
            during = [t for t in bev if t <= t_still]
            gaps = [b - a for a, b in zip(during, during[1:])]
            motion_gap = round(max(gaps), 3) if gaps else None
        row = dict(test=test, trial=index, imu_calibrated_at_on=imu_at_on,
                   tunnel_inside_s=rel(inside_t), imu_calibrated_s=rel(calibrated),
                   imu_reset_seen=uncalibrated_seen, still_s=rel(t_still),
                   first_ir_bev_s=rel(bev[0] if bev else None), first_ir_raw_s=rel(raw[0] if raw else None),
                   first_ir_valid_s=rel(valid[0] if valid else None), first_ir_fwd_s=rel(fwd[0] if fwd else None),
                   ir_bev_fps=round(fps, 1), ir_raw_count=len(raw), ir_valid_ratio=round(len(valid) / len(raw), 2) if raw else None,
                   imu_wait_expired=expired, imu_initialized_log='/'.join(sorted(set(init))) or None,
                   max_ir_gap_while_moving_s=motion_gap, window_s=round(t_end - t_on, 2))
        row['verdict'] = verdict(row)
        return row

    def write(self):
        if not self.rows:
            return
        with open(os.path.join(self.out, 'trials.csv'), 'w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(self.rows[0].keys()))
            writer.writeheader()
            writer.writerows(self.rows)

    # ---- 시험 ----
    def run(self):
        tests = [t.strip().upper() for t in self.args.tests.split(',') if t.strip()]
        print('시험: %s × %d회' % (', '.join(tests), self.args.trials))
        for test in tests:
            for index in range(1, self.args.trials + 1):
                print('\n[%s #%d]' % (test, index))
                if test == 'T1':
                    self.wait_calibrated()
                    self.window('T1', index)
                elif test == 'T2':
                    self.wait_calibrated()
                    self.refit()
                    self.window('T2', index)
                elif test == 'T3':
                    self.wait_calibrated()
                    self.refit()
                    self.window('T3', index, motion_after_on=self.args.motion_sec)
                elif test == 'T4':
                    self.wait_calibrated()
                    self.window('T4', index, motion_after_on=self.args.motion_sec)
                else:
                    raise Aborted('알 수 없는 시험 %s' % test)


def fmt(value):
    return '-' if value is None else '%.2fs' % value


def verdict(row):
    """한 시험의 가설 판정. 지지 / 불일치 / 판단불가."""
    bev, cal, test = row['first_ir_bev_s'], row['imu_calibrated_s'], row['test']
    if test in ('T1', 'T4'):
        if row['imu_calibrated_at_on'] is False:
            return '판단불가(시작 때 보정 안 됨)'
        if bev is None:
            return '불일치(보정 완료인데 IR 없음)'
        if test == 'T4' and row['max_ir_gap_while_moving_s'] is not None and row['max_ir_gap_while_moving_s'] > 1.0:
            return '불일치(보정 후 흔들 때 IR 끊김)'
        return '지지' if bev < 1.0 else '불일치(보정 완료인데 IR %.2fs 지연)' % bev
    # T2/T3: REFIT 직후
    if not row['imu_reset_seen'] and row['imu_calibrated_at_on'] is not False:
        return '판단불가(REFIT이 IMU 보정을 다시 시작하지 않음)'
    if bev is None:
        return '판단불가(창 안에 IR 없음: --max-wait 늘리기)' if cal is None else '불일치(보정 후에도 IR 없음)'
    if cal is None:
        return '불일치(보정 전에 IR 도착)' if row['imu_calibrated_at_on'] is False else '판단불가(보정 시각 없음)'
    if bev < cal - 0.3:
        return '불일치(보정 %.2fs 전에 IR 도착)' % (cal - bev)
    if test == 'T3' and row['still_s'] is not None and bev < row['still_s']:
        return '불일치(흔드는 중에 IR 도착)'
    return '지지' if bev - cal <= 1.0 else '불일치(보정 뒤 %.2fs 늦게 IR)' % (bev - cal)


def summarize(rows):
    lines = ['터널 IR 지연 측정 요약', '가설: IMU 보정 완료 전에는 IR 프레임이 모두 버려진다', '']
    for test in sorted({r['test'] for r in rows}):
        group = [r for r in rows if r['test'] == test]
        support = sum(1 for r in group if r['verdict'] == '지지')
        lines.append('[%s] %d회 중 지지 %d' % (test, len(group), support))
        for key in ('first_ir_bev_s', 'imu_calibrated_s', 'first_ir_fwd_s'):
            values = [r[key] for r in group if r[key] is not None]
            if values:
                lines.append('  %-18s 중앙값 %.2fs  최대 %.2fs  (n=%d)' % (key, statistics.median(values), max(values), len(values)))
        deltas = [r['first_ir_bev_s'] - r['imu_calibrated_s'] for r in group
                  if r['first_ir_bev_s'] is not None and r['imu_calibrated_s'] is not None]
        if deltas:
            lines.append('  첫 IR − 보정 완료   중앙값 %.2fs  범위 %.2f~%.2fs' % (statistics.median(deltas), min(deltas), max(deltas)))
        for r in group:
            if r['verdict'] != '지지':
                lines.append('  #%d %s' % (r['trial'], r['verdict']))
    judged = [r for r in rows if not r['verdict'].startswith('판단불가')]
    lines += ['', '전체: 판정 가능 %d회 중 지지 %d회' % (len(judged), sum(1 for r in judged if r['verdict'] == '지지'))]
    return '\n'.join(lines) + '\n'


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--tests', default='T1,T2', help='T1,T2,T3,T4 중 쉼표로 (기본 T1,T2)')
    parser.add_argument('--with-motion', action='store_true', help='T3,T4(흔들기 안내)도 추가')
    parser.add_argument('--trials', type=int, default=5)
    parser.add_argument('--hold', type=float, default=3.0, help='첫 IR 뒤 계속 볼 시간 (s)')
    parser.add_argument('--max-wait', type=float, default=25.0, help='IR ON 뒤 첫 IR을 기다릴 최대 시간 (s)')
    parser.add_argument('--motion-sec', type=float, default=8.0)
    parser.add_argument('--settle', type=float, default=3.0, help='IR OFF 뒤 다음 시험까지 (s)')
    parser.add_argument('--out', default='')
    args = parser.parse_args()
    if args.with_motion and 'T3' not in args.tests.upper():
        args.tests += ',T3,T4'
    out = args.out or 'tunnel_ir_probe_%s' % time.strftime('%Y%m%d_%H%M%S')
    os.makedirs(out, exist_ok=True)

    rec = Recorder()
    shutdown_ros = start_ros(rec)
    web = Web(args.host, args.port, rec)
    web.start()
    prober = Prober(web, rec, args, out)
    status = 0
    try:
        if not web.connected.wait(10):
            raise Aborted('웹 텔레메트리에 연결하지 못함 (%s)' % web.base)
        data = rec.snapshot()['telemetry'] or {}
        if not (data.get('tunnel') or {}).get('enabled'):
            raise Aborted('터널 기능이 꺼져 있음 (tunnel_enabled:=true로 실행)')
        if data.get('mode') != 'detect':
            raise Aborted('mode=%s: 안전을 위해 mode:=detect 로 실행' % data.get('mode'))
        if (data.get('tunnel') or {}).get('force_ir'):   # 이전 실행이 켜 두고 끝났으면 끄고 시작
            prober.ir_on = True
            prober.ir_off_safely()
            time.sleep(args.settle)
        prober.run()
    except Aborted as error:
        print('\n중단: %s' % error)
        status = 1
    except KeyboardInterrupt:
        print('\n사용자 중단')
        status = 1
    finally:
        prober.ir_off_safely()
        web.stop.set()
        with open(os.path.join(out, 'summary.txt'), 'w') as stream:
            stream.write(summarize(prober.rows) if prober.rows else '측정된 시험 없음\n')
        with open(os.path.join(out, 'rosout.log'), 'w') as stream:
            stream.write('\n'.join(rec.rosout_lines) + '\n')
        with open(os.path.join(out, 'events.csv'), 'w', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(['t', 'kind', 'detail'])
            for t, kind, detail in rec.events:
                writer.writerow(['%.3f' % t, kind, detail])
        prober.write()
        shutdown_ros()
        print('\n결과: %s/ (trials.csv, summary.txt, events.csv, rosout.log)' % out)
        if prober.rows:
            print(summarize(prober.rows))
    return status


if __name__ == '__main__':
    sys.exit(main())
