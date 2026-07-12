import argparse
import time
from pathlib import Path

import cv2
import depthai as dai
import numpy as np


# 체커보드의 "내부 코너" 개수입니다.
# 예: 가로 8칸 x 세로 7칸 체커보드는 내부 코너가 7 x 6 입니다.
BOARD_SIZE = (5, 5)

# 체커보드 한 칸의 실제 크기입니다.
# mm 단위로 넣으면 보정 결과의 tvec도 mm 스케일이 됩니다.
# 실제 칸 크기를 모르면 1.0으로 둬도 렌즈 왜곡 보정에는 사용할 수 있습니다.
SQUARE_SIZE_MM = 30.0

# 캘리브레이션할 때 사용할 RGB 출력 해상도입니다.
# 실제 주행/인식에 사용할 해상도와 같게 맞추는 것이 좋습니다.
FRAME_SIZE = (1280, 800)
TARGET_FPS = 30

CALIB_DIR = Path("calibration_images")
CALIB_FILE = Path("oak_rgb_calibration.npz")


def create_oak_rgb_queue(frame_size=FRAME_SIZE, fps=TARGET_FPS):
    """OAK RGB 카메라 프레임을 받을 DepthAI 파이프라인과 큐를 만듭니다."""
    pipeline = dai.Pipeline()
    device = pipeline.getDefaultDevice()

    connected_sockets = {cam.socket for cam in device.getConnectedCameraFeatures()}
    print("Connected camera sockets:", [socket.name for socket in connected_sockets])

    rgb_cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)

    rgb_out = rgb_cam.requestOutput(
        frame_size,
        resizeMode=dai.ImgResizeMode.STRETCH,
        fps=fps,
    )
    q_rgb = rgb_out.createOutputQueue(maxSize=4, blocking=False)

    return pipeline, q_rgb


def make_object_points(board_size=BOARD_SIZE, square_size=SQUARE_SIZE_MM):
    """체커보드 내부 코너들의 실제 3D 좌표를 만듭니다. 체커보드는 Z=0 평면에 있다고 둡니다."""
    cols, rows = board_size
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= square_size
    return objp


def find_chessboard_corners(gray, board_size=BOARD_SIZE):
    """이미지에서 체커보드 내부 코너를 찾고, sub-pixel 수준으로 정밀화합니다."""
    flags = (
        cv2.CALIB_CB_ADAPTIVE_THRESH
        + cv2.CALIB_CB_NORMALIZE_IMAGE
        + cv2.CALIB_CB_FAST_CHECK
    )
    found, corners = cv2.findChessboardCorners(gray, board_size, flags)
    if not found:
        return False, None

    criteria = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
        30,
        0.001,
    )
    refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    return True, refined


def collect_images(target_count=20, min_interval_sec=0.7):
    """카메라를 켜고 체커보드가 보이면 캘리브레이션용 이미지를 저장합니다."""
    CALIB_DIR.mkdir(exist_ok=True)

    pipeline, q_rgb = create_oak_rgb_queue()
    pipeline.start()

    saved_count = len(list(CALIB_DIR.glob("calib_*.jpg")))
    last_save_time = 0.0

    print()
    print("Collect mode")
    print(f"- Target images: {target_count}")
    print(f"- Save folder: {CALIB_DIR.resolve()}")
    print("- Move/tilt the chessboard between captures.")
    print("- Press 's' to save when corners are found, 'q' to quit.")

    while saved_count < target_count:
        in_rgb = q_rgb.get()
        frame = in_rgb.getCvFrame()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        found, corners = find_chessboard_corners(gray)
        preview = frame.copy()

        if found:
            cv2.drawChessboardCorners(preview, BOARD_SIZE, corners, found)
            cv2.putText(
                preview,
                "Corners found - press s",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 255, 0),
                2,
            )
        else:
            cv2.putText(
                preview,
                "Show chessboard",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 0, 255),
                2,
            )

        cv2.putText(
            preview,
            f"Saved: {saved_count}/{target_count}",
            (20, 80),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (255, 255, 0),
            2,
        )
        cv2.imshow("Collect calibration images", preview)

        key = cv2.waitKey(1) & 0xFF
        now = time.perf_counter()

        if key == ord("q"):
            break
        if key == ord("s") and found and now - last_save_time >= min_interval_sec:
            saved_count += 1
            filename = CALIB_DIR / f"calib_{saved_count:03d}.jpg"
            cv2.imwrite(str(filename), frame)
            print(f"Saved {filename}")
            last_save_time = now

    cv2.destroyAllWindows()
    print(f"Finished. Saved images: {saved_count}")


def calibrate_from_images():
    """저장된 체커보드 이미지들로 camera matrix와 distortion coefficients를 계산합니다."""
    image_paths = sorted(CALIB_DIR.glob("*.jpg"))
    if not image_paths:
        raise FileNotFoundError(f"No calibration images found in {CALIB_DIR.resolve()}")

    objp = make_object_points()
    objpoints = []
    imgpoints = []
    image_size = None
    used_images = 0

    for image_path in image_paths:
        img = cv2.imread(str(image_path))
        if img is None:
            print(f"Skip unreadable image: {image_path}")
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        image_size = gray.shape[::-1]

        found, corners = find_chessboard_corners(gray)
        if not found:
            print(f"Pattern not found: {image_path.name}")
            continue

        objpoints.append(objp)
        imgpoints.append(corners)
        used_images += 1
        print(f"Use image: {image_path.name}")

    if used_images < 10:
        raise RuntimeError(
            f"Only {used_images} valid images found. Use at least 10, preferably 15-25."
        )

    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        objpoints,
        imgpoints,
        image_size,
        None,
        None,
    )

    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
        camera_matrix,
        dist_coeffs,
        image_size,
        1,
        image_size,
    )

    mean_error = compute_reprojection_error(
        objpoints,
        imgpoints,
        rvecs,
        tvecs,
        camera_matrix,
        dist_coeffs,
    )

    np.savez(
        CALIB_FILE,
        image_size=np.array(image_size),
        board_size=np.array(BOARD_SIZE),
        square_size_mm=np.array([SQUARE_SIZE_MM]),
        camera_matrix=camera_matrix,
        dist_coeffs=dist_coeffs,
        new_camera_matrix=new_camera_matrix,
        roi=np.array(roi),
        rms_error=np.array([ret]),
        reprojection_error=np.array([mean_error]),
    )

    print()
    print(f"Calibration saved: {CALIB_FILE.resolve()}")
    print(f"Valid images: {used_images}/{len(image_paths)}")
    print(f"RMS error: {ret:.6f}")
    print(f"Mean reprojection error: {mean_error:.6f}")
    print("Camera matrix:")
    print(camera_matrix)
    print("Distortion coefficients:")
    print(dist_coeffs.ravel())


def compute_reprojection_error(objpoints, imgpoints, rvecs, tvecs, camera_matrix, dist_coeffs):
    """캘리브레이션 결과가 실제 코너 위치를 얼마나 잘 재현하는지 평균 오차를 계산합니다."""
    total_error = 0.0
    for i, objp in enumerate(objpoints):
        projected, _ = cv2.projectPoints(
            objp,
            rvecs[i],
            tvecs[i],
            camera_matrix,
            dist_coeffs,
        )
        error = cv2.norm(imgpoints[i], projected, cv2.NORM_L2) / len(projected)
        total_error += error
    return total_error / len(objpoints)


def live_undistort():
    """저장된 캘리브레이션 값을 이용해 실시간 원본/보정 화면을 비교합니다."""
    if not CALIB_FILE.exists():
        raise FileNotFoundError(f"Calibration file not found: {CALIB_FILE.resolve()}")

    data = np.load(CALIB_FILE)
    camera_matrix = data["camera_matrix"]
    dist_coeffs = data["dist_coeffs"]
    new_camera_matrix = data["new_camera_matrix"]
    roi = data["roi"].astype(int)

    pipeline, q_rgb = create_oak_rgb_queue()
    pipeline.start()

    print()
    print("Live undistort mode")
    print("- Press 'q' to quit.")

    while True:
        frame = q_rgb.get().getCvFrame()
        undistorted = cv2.undistort(frame, camera_matrix, dist_coeffs, None, new_camera_matrix)

        x, y, w, h = roi
        if w > 0 and h > 0:
            cropped = undistorted[y : y + h, x : x + w]
        else:
            cropped = undistorted

        cv2.imshow("Raw", frame)
        cv2.imshow("Undistorted", cropped)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cv2.destroyAllWindows()


def undistort_image(input_path, output_path="undistorted_result.jpg"):
    """파일 이미지 1장을 캘리브레이션 값으로 보정해서 저장합니다."""
    if not CALIB_FILE.exists():
        raise FileNotFoundError(f"Calibration file not found: {CALIB_FILE.resolve()}")

    img = cv2.imread(str(input_path))
    if img is None:
        raise FileNotFoundError(f"Cannot read image: {input_path}")

    data = np.load(CALIB_FILE)
    camera_matrix = data["camera_matrix"]
    dist_coeffs = data["dist_coeffs"]

    h, w = img.shape[:2]
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
        camera_matrix,
        dist_coeffs,
        (w, h),
        1,
        (w, h),
    )

    undistorted = cv2.undistort(img, camera_matrix, dist_coeffs, None, new_camera_matrix)
    x, y, rw, rh = roi
    if rw > 0 and rh > 0:
        undistorted = undistorted[y : y + rh, x : x + rw]

    cv2.imwrite(str(output_path), undistorted)
    print(f"Saved undistorted image: {Path(output_path).resolve()}")


def parse_args():
    parser = argparse.ArgumentParser(description="OAK RGB camera calibration with a chessboard.")
    parser.add_argument(
        "mode",
        choices=["collect", "calibrate", "live", "undistort"],
        help="collect: save chessboard images, calibrate: calculate parameters, live: preview undistortion, undistort: fix one image",
    )
    parser.add_argument("--count", type=int, default=20, help="Number of images to collect.")
    parser.add_argument("--input", type=str, help="Input image path for undistort mode.")
    parser.add_argument("--output", type=str, default="undistorted_result.jpg", help="Output image path.")
    return parser.parse_args()


def main():
    args = parse_args()

    if args.mode == "collect":
        collect_images(target_count=args.count)
    elif args.mode == "calibrate":
        calibrate_from_images()
    elif args.mode == "live":
        live_undistort()
    elif args.mode == "undistort":
        if not args.input:
            raise ValueError("undistort mode requires --input")
        undistort_image(args.input, args.output)


if __name__ == "__main__":
    main()
