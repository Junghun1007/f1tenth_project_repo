#!/usr/bin/env python3
"""기존 패키지(origin/1113ML)에서 필요한 소스를 복사해 cone_drive_test 네임스페이스로 바꾼다.

    python3 src/cone_drive_test/tools/copy_from_1113ML.py <group> [...]   # group: cpu | gpu | all

- 파일 첫 줄에 `// Copied from <원래 경로> (origin/1113ML)` 주석을 남긴다
- 네임스페이스(auto_control, line_detactor, ...)를 cone_drive_test로, include 경로를 cone_drive_test/copied/로 바꾼다
- 복사 후 동작 변경(분기 제거 등)은 파일에서 직접 하고, 다시 이 스크립트를 돌리면 덮어쓰므로 주의
"""
import re
import subprocess
import sys
from pathlib import Path

REF = "origin/1113ML"
PKG = Path(__file__).resolve().parents[1]
INC = PKG / "include/cone_drive_test/copied"
SRC = PKG / "src/copied"
NAMESPACES = ["auto_control", "line_detactor", "multiclass_yolo", "bev_processor", "oak_startup", "camera_driver"]

CPU = {
    "src/auto_control/include/auto_control/control_core.hpp": INC,
    "src/auto_control/src/control_core.cpp": SRC,
    "src/multiclass_yolo/include/multiclass_yolo/object_localization.hpp": INC,
    "src/multiclass_yolo/include/multiclass_yolo/postprocess.hpp": INC,
    "src/multiclass_yolo/include/multiclass_yolo/yolox_detector.hpp": INC,
    "src/line_detactor/include/line_detactor/centerline.hpp": INC,
    "src/line_detactor/src/centerline.cpp": SRC,
    "src/line_detactor/include/line_detactor/lane_connector.hpp": INC,
    "src/line_detactor/src/lane_connector.cpp": SRC,
    "src/line_detactor/include/line_detactor/processing_profile.hpp": INC,
    "src/line_detactor/src/processing_profile.cpp": SRC,
    "src/bev_processor/include/bev_processor/bev_geometry.hpp": INC,
    "src/bev_processor/src/bev_geometry.cpp": SRC,
    "src/oak_startup/include/oak_startup/startup_attitude.hpp": INC,
    "src/oak_startup/src/startup_attitude.cpp": SRC,
    "src/oak_startup/include/oak_startup/ground_plane_estimator.hpp": INC,
    "src/oak_startup/src/ground_plane_estimator.cpp": SRC,
}
GPU = {
    "src/camera_driver/include/camera_driver/imu_image_stabilizer.hpp": INC,
    "src/camera_driver/src/imu_image_stabilizer.cpp": SRC,
    "src/camera_driver/include/camera_driver/vehicle_motion_estimator.hpp": INC,
    "src/camera_driver/src/vehicle_motion_estimator.cpp": SRC,
    "src/bev_processor/include/bev_processor/cuda_bev_processor.hpp": INC,
    "src/bev_processor/src/cuda_bev_processor.cu": SRC,
    "src/line_detactor/include/line_detactor/tensorrt_lane_backend.hpp": INC,
    "src/line_detactor/src/tensorrt_lane_backend.cpp": SRC,
    "src/line_detactor/src/cuda_lane_pipeline.hpp": INC,
    "src/line_detactor/src/cuda_lane_pipeline.cu": SRC,
    "src/multiclass_yolo/src/yolox_detector.cpp": SRC,
    "src/multiclass_yolo/include/multiclass_yolo/tensorrt_yolox_backend.hpp": INC,
    "src/multiclass_yolo/src/tensorrt_yolox_backend.cpp": SRC,
    "src/multiclass_yolo/src/tensorrt_disabled.cpp": SRC,
    "src/multiclass_yolo/src/cuda_yolox_preprocessor.hpp": INC,
    "src/multiclass_yolo/src/cuda_yolox_preprocessor.cu": SRC,
    "src/multiclass_yolo/src/yolox_geometry.hpp": INC,
    "src/oak_startup/include/oak_startup/oak_startup_measurement.hpp": INC,
    "src/oak_startup/src/oak_startup_measurement.cpp": SRC,
}
MODELS = [
    "src/multiclass_yolo/models/multiclass_yolox_tiny_512x320_holdout10.onnx",
    "src/multiclass_yolo/models/multiclass_yolox_tiny_512x320_holdout10.labels.txt",
    "src/multiclass_yolo/models/multiclass_yolox_tiny_512x320_holdout10.metadata.json",
    "src/line_detactor/models/fast_scnn_stop_line_120x300_batch_1.onnx",
    "src/line_detactor/models/fast_scnn_stop_line_120x300_batch_1.json",
    "src/line_detactor/models/fast_scnn_stop_line_120x300_batch_1.int8.qdq.onnx",   # line_detactor 기본 (INT8)
    "src/line_detactor/models/fast_scnn_stop_line_120x300_batch_1.int8.qdq.json",
]


def convert(text: str, origin: str) -> str:
    # include "pkg/foo.hpp" 와 같은 패키지 안의 "foo.hpp"(src 내부 헤더) 모두 copied/로
    text = re.sub(r'#include\s+"(?:%s)/([^"]+)"' % "|".join(NAMESPACES), r'#include "cone_drive_test/copied/\1"', text)
    text = re.sub(r'#include\s+"(cuda_lane_pipeline|cuda_yolox_preprocessor|yolox_geometry)\.hpp"',
                  r'#include "cone_drive_test/copied/\1.hpp"', text)
    for ns in NAMESPACES:
        text = re.sub(r"\bnamespace\s+%s\b" % ns, "namespace cone_drive_test", text)
        text = re.sub(r"\b%s::" % ns, "cone_drive_test::", text)
        text = re.sub(r"\b%s__" % ns.upper(), "CONE_DRIVE_TEST_COPIED__", text)
        text = text.replace("%s_PUBLIC" % ns.upper(), "")
    return f"// Copied from {origin} ({REF})\n" + text


def copy(mapping):
    for origin, dest in mapping.items():
        text = subprocess.run(["git", "show", f"{REF}:{origin}"], check=True, capture_output=True, text=True).stdout
        dest.mkdir(parents=True, exist_ok=True)
        (dest / Path(origin).name).write_text(convert(text, origin))
        print("copied", origin)


def copy_models():
    out = PKG / "models"
    out.mkdir(exist_ok=True)
    for origin in MODELS:
        data = subprocess.run(["git", "show", f"{REF}:{origin}"], check=True, capture_output=True).stdout
        (out / Path(origin).name).write_bytes(data)
        print("copied", origin)


if __name__ == "__main__":
    groups = sys.argv[1:] or ["cpu"]
    if "cpu" in groups or "all" in groups:
        copy(CPU)
        copy_models()
    if "gpu" in groups or "all" in groups:
        copy(GPU)
