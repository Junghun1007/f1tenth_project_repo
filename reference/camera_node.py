import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import cv2
import numpy as np
import depthai as dai

CALIB = "oak_rgb_calibration.npz"
W, H = 640, 480
SOBEL_THRESH = 40
N_WINDOWS, MARGIN, MINPIX = 12, 60, 25

SHOW_DEBUG = True          # 6단계 화면 (튜닝용). 실전엔 False
MAX_LOST = 30              # 이만큼 연속 놓치면 정지신호
LANE_LOST_SIGNAL = 9999    # "차선 잃음" 특수값
MIN_PIX_LANE = 80          # 이 픽셀 이상이면 그 차선 "있음"
HALF_LANE_PX = 180         # 한쪽 차선 주행 시 도로폭 절반(픽셀)
MAX_ERROR = 200            # 오차 폭주 제한

def _label(img, text):
    img = img.copy()
    cv2.rectangle(img, (0, 0), (img.shape[1], 26), (0, 0, 0), -1)
    cv2.putText(img, text, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
    return img

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.pub = self.create_publisher(Int32, '/lane_error', 10)
        data = np.load(CALIB)
        self.K = data["camera_matrix"]; self.D = data["dist_coeffs"]
        self.newK, _ = cv2.getOptimalNewCameraMatrix(self.K, self.D, (W, H), 1, (W, H))
        # 버드아이뷰 사다리꼴 (곡선 대응: 넓게)
        self.src = np.float32([[180,240],[460,240],[40,470],[600,470]])
        dst = np.float32([[140,0],[500,0],[140,480],[500,480]])
        self.M = cv2.getPerspectiveTransform(self.src, dst)
        self.pipeline = dai.Pipeline()
        cam = self.pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        out = cam.requestOutput((W, H), resizeMode=dai.ImgResizeMode.STRETCH, fps=30)
        self.q = out.createOutputQueue(maxSize=4, blocking=False)
        self.pipeline.start()
        self.last_error = 0      # 직전 오차 (놓쳤을 때 유지용)
        self.lost_count = 0      # 연속으로 놓친 프레임 수
        self.timer = self.create_timer(0.033, self.loop)
        self.get_logger().info('카메라 노드 시작 - 곡선대응(사다리꼴 확대)')

    def loop(self):
        frame = self.q.get().getCvFrame()
        undist = cv2.undistort(frame, self.K, self.D, None, self.newK)   # 왜곡보정
        bird = cv2.warpPerspective(undist, self.M, (W, H))               # 버드아이뷰
        L = cv2.cvtColor(bird, cv2.COLOR_BGR2HLS)[:, :, 1]               # 밝기(L) 채널
        sxf = np.absolute(cv2.Sobel(L, cv2.CV_64F, 1, 0, ksize=3))       # Sobel 엣지
        sx = np.uint8(255 * sxf / (sxf.max() + 1e-6))
        _, binary = cv2.threshold(sx, SOBEL_THRESH, 255, cv2.THRESH_BINARY)  # 이진화
        binary = cv2.medianBlur(binary, 3)
        h, w = binary.shape
        hist = np.sum(binary[h//2:, :], axis=0)
        mid = w // 2
        lx = np.argmax(hist[:mid]); rx = np.argmax(hist[mid:]) + mid
        win_h = h // N_WINDOWS
        nz = binary.nonzero(); nzy, nzx = np.array(nz[0]), np.array(nz[1])
        left_idx, right_idx = [], []
        binvis = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR) if SHOW_DEBUG else None
        for win in range(N_WINDOWS):                                     # 슬라이딩 윈도우
            ylo = h - (win+1)*win_h; yhi = h - win*win_h
            if SHOW_DEBUG:
                cv2.rectangle(binvis, (lx-MARGIN,ylo),(lx+MARGIN,yhi),(0,255,0),2)
                cv2.rectangle(binvis, (rx-MARGIN,ylo),(rx+MARGIN,yhi),(0,0,255),2)
            gl = ((nzy>=ylo)&(nzy<yhi)&(nzx>=lx-MARGIN)&(nzx<lx+MARGIN)).nonzero()[0]
            gr = ((nzy>=ylo)&(nzy<yhi)&(nzx>=rx-MARGIN)&(nzx<rx+MARGIN)).nonzero()[0]
            left_idx.append(gl); right_idx.append(gr)
            if len(gl)>MINPIX: lx = int(np.mean(nzx[gl]))
            if len(gr)>MINPIX: rx = int(np.mean(nzx[gr]))
        left_idx = np.concatenate(left_idx); right_idx = np.concatenate(right_idx)

        # 오차 계산: 양쪽 / 왼쪽만 / 오른쪽만 / 없음
        has_left = len(left_idx) > MIN_PIX_LANE
        has_right = len(right_idx) > MIN_PIX_LANE
        ploty = h - 1
        lane_center = mid
        mode = "LOST"
        if has_left and has_right:
            lf = np.polyfit(nzy[left_idx], nzx[left_idx], 2)
            rf = np.polyfit(nzy[right_idx], nzx[right_idx], 2)
            lane_center = (lf[0]*ploty**2+lf[1]*ploty+lf[2] + rf[0]*ploty**2+rf[1]*ploty+rf[2]) / 2
            mode = "BOTH"
        elif has_left:
            lf = np.polyfit(nzy[left_idx], nzx[left_idx], 2)
            lane_center = (lf[0]*ploty**2+lf[1]*ploty+lf[2]) + HALF_LANE_PX
            mode = "L-only"
        elif has_right:
            rf = np.polyfit(nzy[right_idx], nzx[right_idx], 2)
            lane_center = (rf[0]*ploty**2+rf[1]*ploty+rf[2]) - HALF_LANE_PX
            mode = "R-only"

        if mode != "LOST":
            error = int(lane_center - w/2)
            error = max(-MAX_ERROR, min(MAX_ERROR, error))
            self.last_error = error
            self.lost_count = 0
        else:
            self.lost_count += 1
            error = self.last_error if self.lost_count <= MAX_LOST else LANE_LOST_SIGNAL

        msg = Int32(); msg.data = error
        self.pub.publish(msg)                                           # /lane_error 발행

        if SHOW_DEBUG:
            sz = (320, 240)
            s1 = _label(cv2.resize(frame, sz), "1.original")
            vis2 = undist.copy()
            cv2.polylines(vis2, [self.src.astype(np.int32)[[0,1,3,2]]], True, (0,255,0), 2)
            s2 = _label(cv2.resize(vis2, sz), "2.undistort+src")
            s3 = _label(cv2.resize(bird, sz), "3.birdeye")
            s4 = _label(cv2.resize(cv2.cvtColor(sx, cv2.COLOR_GRAY2BGR), sz), "4.sobel")
            s5 = _label(cv2.resize(cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR), sz), "5.binary")
            if mode != "LOST":
                cv2.line(binvis, (int(lane_center),0),(int(lane_center),h),(255,255,0),3)
            cv2.line(binvis, (mid,0),(mid,h),(255,0,255),2)
            st = mode if mode != "LOST" else ("HOLD" if self.lost_count<=MAX_LOST else "LOST!")
            s6 = _label(cv2.resize(binvis, sz), f"6.{st} err={error} L={len(left_idx)} R={len(right_idx)}")
            grid = np.vstack([np.hstack([s1,s2,s3]), np.hstack([s4,s5,s6])])
            cv2.imshow("camera_node debug", grid)
            cv2.waitKey(1)

def main():
    rclpy.init()
    node = CameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
