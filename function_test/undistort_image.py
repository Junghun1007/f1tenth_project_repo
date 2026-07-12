import argparse
import time
from pathlib import Path

import numpy as np

try:
    import cv2
except ImportError:
    cv2 = None


def require_opencv():
    if cv2 is None:
        raise SystemExit(
            "OpenCV(cv2)가 필요합니다. ROS/Jetson 환경에서 실행하거나 "
            "`python3 -m pip install opencv-python`으로 설치한 뒤 다시 실행하세요."
        )


BASE_DIR = Path(__file__).resolve().parent
PROJECT_DIR = BASE_DIR.parent
DEFAULT_INPUT = BASE_DIR / "input"
DEFAULT_RESULT_DIR = BASE_DIR / "result"
DEFAULT_CALIB = PROJECT_DIR / "reference" / "oak_rgb_calibration.npz"
IMAGE_EXTENSIONS = {".bmp", ".jpeg", ".jpg", ".png", ".tif", ".tiff", ".webp"}


def read_image(path):
    data = np.fromfile(str(path), dtype=np.uint8)
    image = cv2.imdecode(data, cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f"이미지를 읽을 수 없습니다: {path}")
    return image


def write_image(path, image):
    path.parent.mkdir(parents=True, exist_ok=True)
    ok, encoded = cv2.imencode(path.suffix, image)
    if not ok:
        raise ValueError(f"이미지를 저장할 수 없습니다: {path}")
    encoded.tofile(str(path))


def load_calibration(calib_path):
    if not calib_path.exists():
        raise FileNotFoundError(f"캘리브레이션 파일이 없습니다: {calib_path}")

    data = np.load(calib_path)
    required_keys = {"camera_matrix", "dist_coeffs"}
    missing = required_keys - set(data.files)
    if missing:
        raise KeyError(f"캘리브레이션 파일에 필요한 값이 없습니다: {sorted(missing)}")

    calib_size = None
    if "image_size" in data.files:
        calib_size = tuple(int(v) for v in data["image_size"])

    return {
        "camera_matrix": data["camera_matrix"].astype(np.float64),
        "dist_coeffs": data["dist_coeffs"].astype(np.float64),
        "image_size": calib_size,
    }


def scale_camera_matrix(camera_matrix, calib_size, image_size):
    if calib_size is None or calib_size == image_size:
        return camera_matrix.copy()

    calib_w, calib_h = calib_size
    image_w, image_h = image_size
    sx = image_w / calib_w
    sy = image_h / calib_h

    scaled = camera_matrix.copy()
    scaled[0, :] *= sx
    scaled[1, :] *= sy
    return scaled


class Undistorter:
    def __init__(self, calibration, alpha=1.0, crop_roi=False):
        self.calibration = calibration
        self.alpha = alpha
        self.crop_roi = crop_roi
        self.map_cache = {}

    def get_maps(self, image_size):
        cache_key = (image_size, self.alpha)
        if cache_key in self.map_cache:
            map1, map2, roi = self.map_cache[cache_key]
            return map1, map2, roi, 0.0

        map_start = time.perf_counter()
        camera_matrix = scale_camera_matrix(
            self.calibration["camera_matrix"],
            self.calibration["image_size"],
            image_size,
        )
        dist_coeffs = self.calibration["dist_coeffs"]
        new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
            camera_matrix,
            dist_coeffs,
            image_size,
            self.alpha,
            image_size,
        )
        map1, map2 = cv2.initUndistortRectifyMap(
            camera_matrix,
            dist_coeffs,
            None,
            new_camera_matrix,
            image_size,
            cv2.CV_16SC2,
        )
        map_elapsed = time.perf_counter() - map_start
        self.map_cache[cache_key] = (map1, map2, roi)
        return map1, map2, roi, map_elapsed

    def undistort(self, image):
        h, w = image.shape[:2]
        map1, map2, roi, map_elapsed = self.get_maps((w, h))
        remap_start = time.perf_counter()
        undistorted = cv2.remap(image, map1, map2, cv2.INTER_LINEAR)
        remap_elapsed = time.perf_counter() - remap_start

        if self.crop_roi:
            x, y, rw, rh = roi
            if rw > 0 and rh > 0:
                undistorted = undistorted[y : y + rh, x : x + rw]

        return undistorted, map_elapsed, remap_elapsed


def is_image_file(path):
    return path.is_file() and path.suffix.lower() in IMAGE_EXTENSIONS


def collect_input_images(input_path, recursive=False):
    if input_path.is_file():
        if not is_image_file(input_path):
            raise ValueError(f"지원하지 않는 이미지 확장자입니다: {input_path}")
        return [input_path]

    if not input_path.is_dir():
        raise FileNotFoundError(f"입력 경로가 없습니다: {input_path}")

    iterator = input_path.rglob("*") if recursive else input_path.iterdir()
    return sorted(path for path in iterator if is_image_file(path))


def make_output_path(image_path, input_path, result_dir):
    if input_path.is_dir():
        relative = image_path.relative_to(input_path)
        return result_dir / relative.parent / f"{relative.stem}_undistorted{relative.suffix}"
    return result_dir / f"{image_path.stem}_undistorted{image_path.suffix}"


def parse_args():
    parser = argparse.ArgumentParser(
        description="사진 파일을 oak_rgb_calibration.npz 보정계수로 왜곡 보정합니다."
    )
    parser.add_argument(
        "input",
        nargs="?",
        default=str(DEFAULT_INPUT),
        help="입력 이미지 파일 또는 이미지 폴더. 기본값: function_test/input",
    )
    parser.add_argument(
        "--calib",
        default=str(DEFAULT_CALIB),
        help="캘리브레이션 npz 경로. 기본값: reference/oak_rgb_calibration.npz",
    )
    parser.add_argument(
        "--result-dir",
        default=str(DEFAULT_RESULT_DIR),
        help="결과 저장 폴더. 기본값: function_test/result",
    )
    parser.add_argument(
        "--alpha",
        type=float,
        default=1.0,
        help="0이면 검은 여백을 줄이고, 1이면 원본 시야를 최대한 유지합니다.",
    )
    parser.add_argument(
        "--crop-roi",
        action="store_true",
        help="OpenCV가 계산한 유효 영역만 잘라서 저장합니다.",
    )
    parser.add_argument(
        "--recursive",
        action="store_true",
        help="입력 폴더의 하위 폴더까지 이미지 파일을 찾습니다.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    require_opencv()

    input_path = Path(args.input).expanduser().resolve()
    calib_path = Path(args.calib).expanduser().resolve()
    result_dir = Path(args.result_dir).expanduser().resolve()

    if not 0.0 <= args.alpha <= 1.0:
        raise ValueError("--alpha는 0.0부터 1.0 사이여야 합니다.")

    calibration = load_calibration(calib_path)
    image_paths = collect_input_images(input_path, recursive=args.recursive)
    if not image_paths:
        raise FileNotFoundError(f"입력 이미지가 없습니다: {input_path}")

    print(f"Calibration: {calib_path}")
    print(f"Input: {input_path}")
    print(f"Result: {result_dir}")
    print(f"Images: {len(image_paths)}")

    undistorter = Undistorter(
        calibration,
        alpha=args.alpha,
        crop_roi=args.crop_roi,
    )
    total_start = time.perf_counter()
    processing_times = []
    timing_totals = {
        "read": 0.0,
        "map": 0.0,
        "undistort": 0.0,
        "write": 0.0,
    }

    for image_path in image_paths:
        image_start = time.perf_counter()
        read_start = time.perf_counter()
        image = read_image(image_path)
        read_elapsed = time.perf_counter() - read_start

        undistorted, map_elapsed, undistort_elapsed = undistorter.undistort(image)

        output_path = make_output_path(image_path, input_path, result_dir)
        write_start = time.perf_counter()
        write_image(output_path, undistorted)
        write_elapsed = time.perf_counter() - write_start

        elapsed = time.perf_counter() - image_start
        processing_times.append(elapsed)
        timing_totals["read"] += read_elapsed
        timing_totals["map"] += map_elapsed
        timing_totals["undistort"] += undistort_elapsed
        timing_totals["write"] += write_elapsed

        print(
            f"Saved: {output_path} "
            f"(total={elapsed * 1000:.1f}ms, "
            f"read={read_elapsed * 1000:.1f}ms, "
            f"map={map_elapsed * 1000:.1f}ms, "
            f"undistort={undistort_elapsed * 1000:.1f}ms, "
            f"write={write_elapsed * 1000:.1f}ms)"
        )

    total_elapsed = time.perf_counter() - total_start
    avg_elapsed = sum(processing_times) / len(processing_times)
    print()
    print("Processing time")
    print(f"- Total: {total_elapsed:.4f}s")
    print(f"- Average per image: {avg_elapsed:.4f}s ({avg_elapsed * 1000:.1f}ms)")
    print(f"- Average read: {timing_totals['read'] / len(image_paths) * 1000:.1f}ms")
    print(f"- Average map build: {timing_totals['map'] / len(image_paths) * 1000:.1f}ms")
    print(f"- Average undistort: {timing_totals['undistort'] / len(image_paths) * 1000:.1f}ms")
    print(f"- Average write: {timing_totals['write'] / len(image_paths) * 1000:.1f}ms")
    print(f"- Cached map resolutions: {len(undistorter.map_cache)}")


if __name__ == "__main__":
    main()
