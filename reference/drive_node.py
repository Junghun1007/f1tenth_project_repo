import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import serial
import struct
import time
import numpy as np
import cv2

# ===== VESC 통신 =====
COMM_SET_DUTY = 5
COMM_SET_SERVO_POS = 12

def crc16_xmodem(data):
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc

def make_vesc_packet(payload):
    crc = crc16_xmodem(payload)
    return bytes([2, len(payload)]) + payload + struct.pack(">H", crc) + bytes([3])

def make_duty_packet(duty):
    duty = max(-1.0, min(1.0, duty))
    return make_vesc_packet(bytes([COMM_SET_DUTY]) + struct.pack(">i", int(duty * 100000)))

def make_servo_packet(pos):
    pos = max(0.0, min(1.0, pos))
    return make_vesc_packet(bytes([COMM_SET_SERVO_POS]) + struct.pack(">h", int(pos * 1000)))

# ===== 튜닝 파라미터 (60cm 도로) =====
PORT = "/dev/ttyACM0"
KP = 0.004           # 오차→서보 세기 (곡선 대응으로 올림)
SERVO_CENTER = 0.50  # 중앙
SERVO_RANGE = 0.25   # 좌우 최대 편차
DUTY = 0.06          # 모터 속도
ERROR_DEADZONE = 5   # 이 이하 오차는 직진
LANE_LOST_SIGNAL = 9999

class DriveNode(Node):
    def __init__(self):
        super().__init__('drive_node')
        self.ser = serial.Serial(PORT, 115200, timeout=0.1, write_timeout=2.0)
        time.sleep(0.2)
        self.get_logger().info(f'VESC 연결됨 ({PORT})')
        self.running = False   # 처음엔 정지!
        self.sub = self.create_subscription(Int32, '/lane_error', self.on_error, 10)  # 구독
        cv2.namedWindow('drive control', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('drive control', 400, 150)
        self.key_timer = self.create_timer(0.05, self.check_key)
        self.get_logger().info('주행 노드 시작 - [정지] 창에서 SPACE=출발/정지, q=종료')

    def on_error(self, msg):
        error = msg.data
        if error == LANE_LOST_SIGNAL:              # 차선 완전히 잃으면 정지
            self.send(0.0, SERVO_CENTER)
            self.get_logger().warn('차선 잃음 - 정지', throttle_duration_sec=1.0)
            return
        if abs(error) < ERROR_DEADZONE:            # 오차 → 서보 (P제어)
            servo = SERVO_CENTER
        else:
            offset = max(-SERVO_RANGE, min(SERVO_RANGE, KP * error))
            servo = SERVO_CENTER + offset
        duty = DUTY if self.running else 0.0       # 주행중일 때만 모터
        self.send(duty, servo)
        state = 'RUN ' if self.running else 'STOP'
        self.get_logger().info(f'[{state}] error={error:+d} servo={servo:.3f} duty={duty}', throttle_duration_sec=0.5)

    def send(self, duty, servo):
        try:
            self.ser.write(make_servo_packet(servo))
            self.ser.write(make_duty_packet(duty))
        except Exception as e:
            self.get_logger().error(f'VESC 전송 실패: {e}')

    def check_key(self):                           # 스페이스 시작/정지 창
        img = np.zeros((150, 400, 3), dtype=np.uint8)
        if self.running:
            img[:] = (0, 80, 0); txt, color = "RUNNING", (0, 255, 0)
        else:
            img[:] = (0, 0, 80); txt, color = "STOPPED", (0, 0, 255)
        cv2.putText(img, txt, (110, 70), cv2.FONT_HERSHEY_SIMPLEX, 1.2, color, 3)
        cv2.putText(img, "SPACE=start/stop  q=quit", (40, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
        cv2.imshow('drive control', img)
        key = cv2.waitKey(1) & 0xFF
        if key == ord(' '):
            self.running = not self.running
            self.get_logger().info(f'>>> {"출발!" if self.running else "정지!"}')
            if not self.running:
                self.send(0.0, SERVO_CENTER)
        elif key == ord('q'):
            self.running = False
            raise KeyboardInterrupt

    def stop(self):
        for _ in range(5):
            self.send(0.0, SERVO_CENTER); time.sleep(0.02)
        cv2.destroyAllWindows()

def main():
    rclpy.init()
    node = DriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
