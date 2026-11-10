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


class StartupInspection(Node):
    def __init__(self):
        super().__init__('startup_inspection')
        expected = self.declare_parameter('expected_nodes', ['line_detactor']).value
        self.state = InspectionState(expected)
        self.image = None
        self.image_at = 0.
        self.pending = None
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

    def request_stop(self):
        if self.stop_client.service_is_ready():
            self.stop_client.call_async(Trigger.Request())
        else:
            self.state.error = 'STOP service unavailable'

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


def render(state, image=None, image_fresh=False, pending=False, now=None):
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
    cv2.rectangle(canvas,(25,655),(740,730),(65,155,60) if ready else (65,65,65),-1)
    label = 'START AUTOMATIC DRIVING' if ready else ('REQUESTING START...' if pending else 'WAITING FOR CHECKS')
    text(label,70,702,(255,255,255),.85)
    cv2.rectangle(canvas,(760,655),(975,730),(50,50,190),-1)
    text('STOP',800,702,(255,255,255),.85)
    text(state.error,22,768,(100,170,255),.48)
    text('Click START or press Enter when READY. Q / closing this window never authorizes duty.',22,810,scale=.48)
    text('Only this inspection window closes after approval; the driving BEV preview remains open.',22,838,scale=.48)
    return canvas


def main(args=None):
    rclpy.init(args=args)
    node = StartupInspection()
    shown = False
    dismissed = False
    window = 'Startup inspection - START required'
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
            if not gui:
                continue
            if node.state.completed():
                if shown:
                    cv2.destroyWindow(window)
                    cv2.waitKey(1)
                    shown = False
                continue
            if dismissed:
                continue
            if not shown:
                cv2.namedWindow(window, cv2.WINDOW_AUTOSIZE)
                cv2.setMouseCallback(window, lambda event,x,y,flags,data: node.request_start()
                    if event==cv2.EVENT_LBUTTONUP and 25<=x<=740 and 655<=y<=730 else
                    node.request_stop() if event==cv2.EVENT_LBUTTONUP and 760<=x<=975 and 655<=y<=730 else None)
                shown = True
            cv2.imshow(window, render(node.state,node.image,
                time.monotonic()-node.image_at<.5,node.pending is not None))
            key = cv2.waitKey(1)&255
            if key in (10,13):
                node.request_start()
            if key in (ord('q'),27) or cv2.getWindowProperty(window,cv2.WND_PROP_VISIBLE)<1:
                cv2.destroyWindow(window)
                shown = False
                dismissed = True
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
