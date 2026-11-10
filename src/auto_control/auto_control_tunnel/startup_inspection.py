"""Main-thread OpenCV startup inspection; never publishes actuator commands."""
import os
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from .inspection_state import InspectionState
from .inspection_speed import InspectionSpeed


DISPLAY_SCALE = .85
START = (25, 655, 740, 720)
STOP = (760, 655, 975, 720)
TIMINGS = (25, 740, 200, 790)
PARAMS = (215, 740, 390, 790)
SPEED = (415, 735, 1175, 790)
SPEED_LEFT, SPEED_RIGHT, SPEED_Y = 440, 1145, 775


def contains(rect, x, y):
    return rect[0] <= x <= rect[2] and rect[1] <= y <= rect[3]


class StartupInspection(Node):
    def __init__(self):
        super().__init__('startup_inspection')
        expected = self.declare_parameter('expected_nodes', ['line_detactor']).value
        self.state = InspectionState(expected)
        self.image = None
        self.image_at = 0.
        self.pending = None
        self.speed = InspectionSpeed()
        from std_msgs.msg import Float64
        from line_detactor.msg import SpeedLimitState
        self.action_pub = self.create_publisher(String, '/auto/ui/action', 10)
        self.speed_pub = self.create_publisher(Float64, '/auto/maximum_speed_request', 1)
        self.speed_sub = self.create_subscription(
            SpeedLimitState, '/auto/maximum_speed_state', self.speed.observe,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        image_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscriptions_owned = [
            self.create_subscription(String, '/bev_processor/startup_status', self.on_camera, qos),
            self.create_subscription(String, '/auto_control/startup_state', self.on_controller, qos),
            self.create_subscription(Image, '/bev_processor/startup_preview', self.on_image, image_qos),
        ]
        self.subscriptions_owned += [self.create_subscription(
            String, f'/{name}/model_status', lambda msg, name=name: self.on_model(name, msg), qos)
            for name in expected]
        self.client = self.create_client(Trigger, '/auto_control/start')
        self.stop_client = self.create_client(Trigger, '/auto_control/stop')

    def on_camera(self, message):
        self.state.camera = message.data

    def on_model(self, name, message):
        self.state.update_model(name, message.data)

    def on_controller(self, message):
        self.state.update_controller(message.data)

    def on_image(self, message):
        import numpy as np
        if (message.encoding != 'bgr8' or message.height <= 0 or message.width <= 0
                or message.step < message.width * 3 or len(message.data) < message.step * message.height):
            return
        rows = np.frombuffer(message.data, dtype=np.uint8,
            count=message.height*message.step).reshape(message.height, message.step)
        self.image = rows[:, :message.width * 3].reshape(message.height, message.width, 3).copy()
        self.image_at = time.monotonic()

    def on_mouse(self, event, x, y, flags, userdata):
        import cv2
        # HighGUI receives coordinates in the displayed (85%) image, not the
        # logical 1200x860 drawing. Keep drawing and hit targets in one space.
        x, y = x / DISPLAY_SCALE, y / DISPLAY_SCALE
        if event == cv2.EVENT_LBUTTONDOWN and contains(SPEED, x, y):
            if self.speed.fresh(time.monotonic()):
                self.speed.dragging = True
                self.speed.select(x, SPEED_LEFT, SPEED_RIGHT)
        elif event == cv2.EVENT_MOUSEMOVE and self.speed.dragging:
            if flags & cv2.EVENT_FLAG_LBUTTON:
                self.speed.select(x, SPEED_LEFT, SPEED_RIGHT)
            else:
                self.speed.dragging = False
                self.speed.released = self.speed.dirty
        elif event == cv2.EVENT_LBUTTONUP:
            if self.speed.dragging:
                self.speed.select(x, SPEED_LEFT, SPEED_RIGHT)
                self.speed.dragging = False
                self.speed.released = self.speed.dirty
                return
            if contains(START, x, y):
                self.request_start()
            elif contains(STOP, x, y):
                self.request_stop()
            elif contains(TIMINGS, x, y):
                self.action_pub.publish(String(data='metrics'))
                self.state.error = 'Timings window requested.'
            elif contains(PARAMS, x, y):
                self.action_pub.publish(String(data='parameters'))
                self.state.error = 'Parameters window requested.'

    def flush_speed_request(self):
        from std_msgs.msg import Float64
        value = self.speed.request(time.monotonic())
        if value is not None:
            self.speed_pub.publish(Float64(data=value))

    def request_stop(self):
        if self.stop_client.service_is_ready():
            self.stop_client.call_async(Trigger.Request()).add_done_callback(self.on_stop_response)
        else:
            self.state.error = 'STOP service unavailable'

    def on_stop_response(self, future):
        try:
            self.state.error = future.result().message
        except Exception as error:
            self.state.error = f'STOP failed: {error}'

    def request_start(self):
        if not self.state.can_start() or self.pending is not None:
            return
        if not self.client.service_is_ready():
            self.state.error = 'Controller START service unavailable; duty stays blocked.'
            return
        self.pending = self.client.call_async(Trigger.Request())
        self.pending.add_done_callback(self.on_response)

    def on_response(self, future):
        self.pending = None
        try:
            response = future.result()
            self.state.error = response.message
        except Exception as error:
            self.state.error = f'START failed: {error}'


def render(state, image=None, image_fresh=False, pending=False, now=None, speed=None):
    import cv2
    import numpy as np
    now = time.monotonic() if now is None else now
    canvas = np.full((860,1200,3), (32,36,40), dtype=np.uint8)
    def text(line, x, y, color=(225,225,225), scale=.55):
        cv2.putText(canvas, line[:112], (x,y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)
    text('STARTUP INSPECTION', 22,35, (255,220,80), .9)
    if image is not None:
        canvas[60:360,20:500] = cv2.resize(image, (480,300))
        text('Live measurement view' if image_fresh else 'Last measurement view (frozen)', 25,385)
    else:
        text('Waiting for camera measurement view...', 35,210)
    passed = state.camera.startswith('READY | ')
    color = (100,230,120) if passed else ((70,80,250) if state.camera.startswith('FAILED') else (80,210,250))
    text('1. CAMERA ATTITUDE: ' + ('PASS' if passed else 'MEASURING / WAITING'), 22,415,color)
    import textwrap
    detail = state.camera.split(' | ',1)[-1]
    for row, line in enumerate(textwrap.wrap(detail, width=53)[:4]):
        text(line,22,445+row*20,color,.41)
    ready_count = sum(value.startswith('READY | ') for value in state.models.values())
    text(f'2. INFERENCE MODELS ({ready_count}/{len(state.models)} READY)',525,65,(255,220,80),.65)
    for index, name in enumerate(state.models):
        info = state.model_info(name, now)
        tag = info['tag']
        y = 95 + index * 100
        color = (100,230,120) if tag=='READY' else ((70,80,250) if tag=='FAILED' or
            'STALE' in info['health'] else (80,210,250))
        text(f'{name}: {tag}',525,y,color,.54)
        phase = info['phase']
        if tag == 'READY':
            phase = 'Engine loaded; trial inference PASS'
        elif phase.startswith('model='):
            phase = 'Opening model: ' + os.path.basename(phase[6:])
        for row, line in enumerate(textwrap.wrap(phase, width=77)[:2]):
            text(line,525,y+20+row*17,color,.40)
        text(f"Total {info['elapsed']:.0f}s | Stage {info['stage_age']:.0f}s | Last report {info['age']:.1f}s ago",
             525,y+57,scale=.40)
        text(info['health'],525,y+77,color,.40)
    text('Stage changes reflect loader activity; a heartbeat alone cannot prove GPU progress.',525,505,scale=.37)
    text('3. DEPARTURE', 22,535,(255,220,80),.65)
    controller = state.controller if state.controller_fresh(now) else 'WAITING | controller heartbeat unavailable'
    text(controller,22,567,scale=.53)
    text('The controller checks models, measured camera pose, VESC feedback and a fresh valid lane.',22,600,scale=.48)
    text('Automatic duty is blocked until an accepted START request. Manual mode skips this button.',22,625,scale=.48)
    ready = state.can_start(now) and not pending
    active = state.completed(now)
    cv2.rectangle(canvas,START[:2],START[2:],(65,155,60) if ready else (65,65,65),-1)
    label = ('MANUAL CONTROL' if active and state.controller.startswith('MANUAL | ') else
             'DRIVING / START ACCEPTED' if active else 'START AUTOMATIC DRIVING' if ready else
             'REQUESTING START...' if pending else 'WAITING FOR CHECKS')
    text(label,70,698,(255,255,255),.78)
    for rect, label in ((STOP, 'STOP'), (TIMINGS, 'Timings'), (PARAMS, 'Params')):
        cv2.rectangle(canvas,rect[:2],rect[2:],(50,50,190) if label=='STOP' else (95,85,65),-1)
        text(label,rect[0]+20,rect[1]+(43 if label=='STOP' else 33),(255,255,255),.70)
    speed = speed or dict(bounds=None, selected=None, applied=None, status='CONTROLLER OFFLINE')
    if speed['bounds'] is not None:
        minimum, ceiling = speed['bounds']
        selected, applied = speed['selected'], speed['applied']
        text(f'MAX SPEED {selected:.2f} m/s | Applied {applied:.2f} | {speed["status"]}',420,754,scale=.45)
        ratio = (selected-minimum)/(ceiling-minimum) if ceiling > minimum else 0.
        thumb = round(SPEED_LEFT+ratio*(SPEED_RIGHT-SPEED_LEFT))
        accent = (100,230,120) if speed['status']=='LIVE' else (80,210,250)
        cv2.line(canvas,(SPEED_LEFT,SPEED_Y),(SPEED_RIGHT,SPEED_Y),(100,100,100),7)
        cv2.line(canvas,(SPEED_LEFT,SPEED_Y),(thumb,SPEED_Y),accent,7)
        cv2.circle(canvas,(thumb,SPEED_Y),9,accent,-1)
        text(f'{minimum:.2f}',415,800,scale=.36)
        text(f'{ceiling:.2f} m/s',1100,800,scale=.36)
    else:
        text('MAX SPEED: waiting for controller',420,765,scale=.48)
    text(state.error,22,815,(100,170,255),.43)
    text('START requires checks. STOP, Timings, Params and speed controls remain available here after START.',22,843,scale=.43)
    return cv2.resize(canvas, (round(canvas.shape[1]*DISPLAY_SCALE), round(canvas.shape[0]*DISPLAY_SCALE)),
                      interpolation=cv2.INTER_AREA)


def main(args=None):
    rclpy.init(args=args)
    node = StartupInspection()
    shown = False
    dismissed = False
    window = 'Safety inspection and driving controls'
    gui = bool(os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY') or os.sys.platform=='darwin')
    cv2 = None
    if gui:
        try:
            import cv2
        except ImportError as error:
            node.get_logger().error(f'Inspection GUI unavailable: {error}; automatic duty stays blocked.')
            gui = False
    else:
        node.get_logger().error('No display for startup inspection. Automatic duty requires an explicit controller START service request.')
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.03)
            node.flush_speed_request()
            if not gui:
                continue
            if dismissed:
                continue
            if not shown:
                cv2.namedWindow(window, cv2.WINDOW_AUTOSIZE)
                cv2.setMouseCallback(window, node.on_mouse)
                shown = True
            cv2.imshow(window, render(node.state,node.image,
                time.monotonic()-node.image_at<.5,node.pending is not None,
                speed=node.speed.view(time.monotonic())))
            key = cv2.waitKey(1)&255
            if key in (10,13):
                node.request_start()
            if key in (ord('q'),27) or cv2.getWindowProperty(window,cv2.WND_PROP_VISIBLE)<1:
                cv2.destroyWindow(window)
                shown = False
                dismissed = True
                node.speed.dragging = node.speed.dirty = node.speed.released = False
    except KeyboardInterrupt:
        pass
    except Exception as error:
        node.get_logger().error(f'Inspection GUI stopped: {error}. No automatic START approval was issued by this error.')
    finally:
        if shown and cv2 is not None:
            cv2.destroyWindow(window)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
