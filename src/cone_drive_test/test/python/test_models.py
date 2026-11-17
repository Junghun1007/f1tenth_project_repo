"""ONNX 모델 스모크: onnxruntime CPU로 입력·출력 형식만 확인 (TensorRT 엔진은 Jetson에서 생성)."""
from pathlib import Path

import numpy as np
import onnxruntime as ort

MODELS = Path(__file__).resolve().parents[2] / "models"


def _run(name, shape):
    sess = ort.InferenceSession(str(MODELS / name), providers=["CPUExecutionProvider"])
    inp = sess.get_inputs()[0]
    assert list(inp.shape) == shape, inp.shape
    return sess.run(None, {inp.name: np.full(shape, 114, np.float32)})[0]


def test_yolo_output_shape_and_finite():
    out = _run("multiclass_yolox_tiny_512x320_holdout10.onnx", [1, 3, 320, 512])
    assert out.shape == (1, 3360, 9)
    assert np.isfinite(out).all()


def test_lane_output_shape_and_finite():
    out = _run("fast_scnn_stop_line_120x300_batch_1.onnx", [1, 3, 300, 120])
    assert out.shape == (1, 3, 300, 120)
    assert np.isfinite(out).all()


def test_yolo_labels_order():
    labels = (MODELS / "multiclass_yolox_tiny_512x320_holdout10.labels.txt").read_text().split()
    assert labels == ["con", "truck", "sedan", "traffic_light"]


def test_lane_int8_qdq_model_matches_fp32_contract():
    """기존 line_detactor 기본값과 같은 INT8 Q/DQ 차선 모델 (외부 입출력은 FP32)."""
    out = _run("fast_scnn_stop_line_120x300_batch_1.int8.qdq.onnx", [1, 3, 300, 120])
    assert out.shape == (1, 3, 300, 120)
    assert np.isfinite(out).all()
