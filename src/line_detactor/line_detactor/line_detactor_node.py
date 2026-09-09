"""Preview Fast-SCNN lane inference from the raw BEV ROS image."""

from collections import deque
from contextlib import nullcontext
import os
from pathlib import Path
import threading
import time

from ament_index_python.packages import get_package_share_directory
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image
import torch

from line_detactor.model import FastSCNNHighRes, validate_checkpoint


INPUT_WIDTH = 120
INPUT_HEIGHT = 300
LEFT_BGR = (255, 0, 0)
RIGHT_BGR = (0, 0, 255)


def _default_model_path():
    return str(
        Path(get_package_share_directory("line_detactor")) /
        "models" /
        "best.pt"
    )


def _display_available():
    if os.name != "posix":
        return True
    return bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))


def _load_checkpoint(path):
    try:
        return torch.load(
            str(path),
            map_location="cpu",
            weights_only=True,
        )
    except TypeError:
        # NVIDIA Jetson wheels based on older PyTorch do not expose
        # weights_only. The checkpoint is a trusted asset bundled here.
        return torch.load(str(path), map_location="cpu")


class LineDetactorNode(Node):
    def __init__(self):
        super().__init__("line_detactor")

        self.declare_parameter("input_topic", "/camera/image_bev")
        self.declare_parameter("model_path", _default_model_path())
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("amp_enabled", True)
        self.declare_parameter("mask_threshold", 0.5)
        self.declare_parameter("warmup_iterations", 20)
        self.declare_parameter("preview_enabled", True)
        self.declare_parameter("preview_fps", 30.0)
        self.declare_parameter("preview_scale", 2.0)
        self.declare_parameter(
            "preview_window_name",
            "BEV lane model preview",
        )
        self.declare_parameter("status_log_interval_sec", 1.0)
        self.declare_parameter("torch_num_threads", 2)

        self._input_topic = str(self.get_parameter("input_topic").value)
        self._model_path = Path(str(self.get_parameter("model_path").value))
        self._device_name = str(self.get_parameter("device").value)
        self._amp_requested = bool(self.get_parameter("amp_enabled").value)
        self._threshold = float(self.get_parameter("mask_threshold").value)
        self._warmup_iterations = int(
            self.get_parameter("warmup_iterations").value
        )
        self._preview_enabled = bool(
            self.get_parameter("preview_enabled").value
        )
        self._preview_fps = float(self.get_parameter("preview_fps").value)
        self._preview_scale = float(
            self.get_parameter("preview_scale").value
        )
        self._window_name = str(
            self.get_parameter("preview_window_name").value
        )
        self._status_interval = float(
            self.get_parameter("status_log_interval_sec").value
        )
        self._torch_num_threads = int(
            self.get_parameter("torch_num_threads").value
        )
        self._validate_parameters()

        torch.set_num_threads(self._torch_num_threads)
        cv2.setNumThreads(0)
        self._device = torch.device(self._device_name)
        if self._device.type == "cuda":
            if not torch.cuda.is_available():
                raise RuntimeError(
                    "CUDA is unavailable. Install the NVIDIA PyTorch build "
                    "matching this JetPack, or explicitly use device:=cpu."
                )
            torch.cuda.set_device(self._device)
            torch.backends.cudnn.benchmark = True

        checkpoint = _load_checkpoint(self._model_path)
        validate_checkpoint(checkpoint)
        config = checkpoint.get("config", {})
        auxiliary = float(config.get("aux_weight", 0.0)) > 0.0
        self._amp_enabled = bool(
            self._amp_requested and
            bool(config.get("amp", True)) and
            self._device.type == "cuda"
        )
        self._model = FastSCNNHighRes(auxiliary=auxiliary)
        self._model.load_state_dict(checkpoint["model"], strict=True)
        self._model.to(self._device).eval()

        self._host_bgr = None
        self._device_bgr = None
        self._input_tensor = None
        self._preview_host = None
        self._colors = None
        if self._device.type == "cuda":
            self._host_bgr = torch.empty(
                (INPUT_HEIGHT, INPUT_WIDTH, 3),
                dtype=torch.uint8,
                pin_memory=True,
            )
            self._device_bgr = torch.empty(
                (INPUT_HEIGHT, INPUT_WIDTH, 3),
                dtype=torch.uint8,
                device=self._device,
            )
            self._input_tensor = torch.empty(
                (1, 3, INPUT_HEIGHT, INPUT_WIDTH),
                dtype=torch.float32,
                device=self._device,
            )
            self._preview_host = torch.empty(
                (INPUT_HEIGHT, INPUT_WIDTH, 3),
                dtype=torch.uint8,
                pin_memory=True,
            )
            self._colors = {
                "left": torch.tensor(
                    LEFT_BGR,
                    dtype=torch.float32,
                    device=self._device,
                ),
                "right": torch.tensor(
                    RIGHT_BGR,
                    dtype=torch.float32,
                    device=self._device,
                ),
            }
            self._events = [
                torch.cuda.Event(enable_timing=True) for _ in range(6)
            ]
        else:
            self._events = None

        self._warmup_model()

        self._condition = threading.Condition()
        self._latest_message = None
        self._generation = 0
        self._received_interval = 0
        self._received_total = 0
        self._processed_total = 0
        self._skipped_total = 0
        self._stop_event = threading.Event()
        self._worker_error = None

        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._subscription = self.create_subscription(
            Image,
            self._input_topic,
            self._on_image,
            image_qos,
        )
        self._worker = threading.Thread(
            target=self._preview_loop,
            name="line-detactor-preview",
            daemon=True,
        )
        self._worker.start()

        self.get_logger().info(
            "Line detector ready: input={} (bgr8 {}x{}), model={}, "
            "device={}, AMP={}, threshold={:.3f}, preview={} @ {:.1f} FPS".format(
                self._input_topic,
                INPUT_WIDTH,
                INPUT_HEIGHT,
                self._model_path,
                self._device,
                "on" if self._amp_enabled else "off",
                self._threshold,
                "on" if self._preview_enabled else "off",
                self._preview_fps,
            )
        )
        self.get_logger().info(
            "GPU path: pinned BGR8 H2D -> CUDA BGR/RGB NCHW -> model -> "
            "CUDA sigmoid/masks/overlay -> one BGR8 D2H"
            if self._device.type == "cuda"
            else "CPU path selected explicitly; CUDA acceleration is disabled."
        )

    @property
    def stopped(self):
        return self._stop_event.is_set()

    def _validate_parameters(self):
        if not self._input_topic:
            raise ValueError("input_topic must not be empty")
        if not self._model_path.is_file():
            raise FileNotFoundError(
                "Model checkpoint not found: {}".format(self._model_path)
            )
        if not 0.0 <= self._threshold <= 1.0:
            raise ValueError("mask_threshold must be in [0,1]")
        if self._warmup_iterations < 0:
            raise ValueError("warmup_iterations must be non-negative")
        if self._preview_fps <= 0.0 or not np.isfinite(self._preview_fps):
            raise ValueError("preview_fps must be finite and positive")
        if self._preview_scale <= 0.0 or not np.isfinite(self._preview_scale):
            raise ValueError("preview_scale must be finite and positive")
        if self._status_interval <= 0.0 or not np.isfinite(
            self._status_interval
        ):
            raise ValueError("status_log_interval_sec must be positive")
        if self._torch_num_threads <= 0:
            raise ValueError("torch_num_threads must be positive")
        if self._preview_enabled and not _display_available():
            raise RuntimeError(
                "Preview is enabled but DISPLAY/WAYLAND_DISPLAY is unavailable"
            )

    def _autocast(self):
        if not self._amp_enabled:
            return nullcontext()
        if hasattr(torch, "amp") and hasattr(torch.amp, "autocast"):
            return torch.amp.autocast("cuda")
        return torch.cuda.amp.autocast()

    def _forward(self, tensor):
        with torch.inference_mode(), self._autocast():
            return self._model(tensor)["out"]

    def _synchronize(self):
        if self._device.type == "cuda":
            torch.cuda.synchronize(self._device)

    def _warmup_model(self):
        tensor = torch.zeros(
            (1, 3, INPUT_HEIGHT, INPUT_WIDTH),
            dtype=torch.float32,
            device=self._device,
        )
        for _ in range(self._warmup_iterations):
            self._forward(tensor)
        self._synchronize()
        del tensor
        self.get_logger().info(
            "Model warmup completed: {} iterations".format(
                self._warmup_iterations
            )
        )

    def _on_image(self, message):
        with self._condition:
            self._latest_message = message
            self._generation += 1
            self._received_interval += 1
            self._received_total += 1
            self._condition.notify()

    @staticmethod
    def _message_bgr_view(message):
        if message.encoding.lower() != "bgr8":
            raise ValueError(
                "Expected bgr8, received {}".format(message.encoding)
            )
        if message.width != INPUT_WIDTH or message.height != INPUT_HEIGHT:
            raise ValueError(
                "Expected {}x{} BEV, received {}x{}".format(
                    INPUT_WIDTH,
                    INPUT_HEIGHT,
                    message.width,
                    message.height,
                )
            )
        minimum_step = INPUT_WIDTH * 3
        if message.step < minimum_step:
            raise ValueError(
                "Invalid image step {}; expected at least {}".format(
                    message.step,
                    minimum_step,
                )
            )
        expected_bytes = int(message.step) * INPUT_HEIGHT
        if len(message.data) < expected_bytes:
            raise ValueError(
                "Image data is truncated: {} < {} bytes".format(
                    len(message.data),
                    expected_bytes,
                )
            )
        rows = np.frombuffer(
            message.data,
            dtype=np.uint8,
            count=expected_bytes,
        ).reshape(
            INPUT_HEIGHT, int(message.step)
        )
        return rows[:, :minimum_step].reshape(INPUT_HEIGHT, INPUT_WIDTH, 3)

    def _gpu_predict_overlay(self, image, alpha):
        self._host_bgr.copy_(torch.from_numpy(image))
        pre_start, pre_end, forward_start, forward_end, post_start, post_end = (
            self._events
        )
        pre_start.record()
        self._device_bgr.copy_(self._host_bgr, non_blocking=True)
        self._input_tensor[0, 0].copy_(self._device_bgr[:, :, 2])
        self._input_tensor[0, 1].copy_(self._device_bgr[:, :, 1])
        self._input_tensor[0, 2].copy_(self._device_bgr[:, :, 0])
        self._input_tensor.mul_(1.0 / 255.0)
        pre_end.record()

        forward_start.record()
        logits = self._forward(self._input_tensor)
        forward_end.record()

        post_start.record()
        masks = torch.sigmoid(logits[0]) >= self._threshold
        left = masks[0]
        right = masks[1]
        left_display = left & (~right | (logits[0, 0] >= logits[0, 1]))
        right_display = right & (~left | (logits[0, 1] > logits[0, 0]))
        any_lane = left_display | right_display
        color_image = torch.zeros(
            (INPUT_HEIGHT, INPUT_WIDTH, 3),
            dtype=torch.float32,
            device=self._device,
        )
        color_image[left_display] = self._colors["left"]
        color_image[right_display] = self._colors["right"]
        overlay = self._device_bgr.to(dtype=torch.float32)
        overlay[any_lane] = (
            overlay[any_lane] * (1.0 - alpha) +
            color_image[any_lane] * alpha
        )
        overlay = overlay.to(dtype=torch.uint8)
        self._preview_host.copy_(overlay, non_blocking=True)
        post_end.record()
        self._synchronize()

        timing = {
            "preprocess": pre_start.elapsed_time(pre_end),
            "inference": forward_start.elapsed_time(forward_end),
            "postprocess": post_start.elapsed_time(post_end),
        }
        return self._preview_host.numpy(), timing

    def _cpu_predict_overlay(self, image, alpha):
        started = time.perf_counter()
        rgb = np.ascontiguousarray(
            image[:, :, ::-1].transpose(2, 0, 1),
            dtype=np.float32,
        )
        tensor = torch.from_numpy(rgb).unsqueeze(0).div_(255.0)
        preprocessed = time.perf_counter()
        logits = self._forward(tensor)
        forwarded = time.perf_counter()
        logits_array = logits[0].numpy()
        masks = torch.sigmoid(logits[0]).numpy() >= self._threshold
        left_display = masks[0] & (
            ~masks[1] | (logits_array[0] >= logits_array[1])
        )
        right_display = masks[1] & (
            ~masks[0] | (logits_array[1] > logits_array[0])
        )
        colors = np.zeros_like(image)
        colors[left_display] = LEFT_BGR
        colors[right_display] = RIGHT_BGR
        any_lane = left_display | right_display
        overlay = image.copy()
        overlay[any_lane] = (
            overlay[any_lane].astype(np.float32) * (1.0 - alpha) +
            colors[any_lane].astype(np.float32) * alpha
        ).astype(np.uint8)
        finished = time.perf_counter()
        timing = {
            "preprocess": (preprocessed - started) * 1000.0,
            "inference": (forwarded - preprocessed) * 1000.0,
            "postprocess": (finished - forwarded) * 1000.0,
        }
        return overlay, timing

    @staticmethod
    def _preview_canvas(overlay, inference_ms, preview_fps):
        banner = np.zeros((58, INPUT_WIDTH, 3), dtype=np.uint8)
        inference_fps = 1000.0 / inference_ms if inference_ms > 0.0 else 0.0
        lines = (
            ("left:blue right:red", (220, 220, 220)),
            ("inference: {:.2f} ms".format(inference_ms), (0, 255, 255)),
            ("model FPS: {:.1f}".format(inference_fps), (0, 255, 255)),
            ("preview FPS: {:.1f}".format(preview_fps), (255, 255, 255)),
        )
        for index, (text, color) in enumerate(lines):
            cv2.putText(
                banner,
                text,
                (3, 12 + index * 14),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.28,
                color,
                1,
                cv2.LINE_AA,
            )
        return np.vstack((overlay, banner))

    def _window_quit_requested(self, window_seen):
        key = cv2.waitKey(1) & 0xFF
        visible = cv2.getWindowProperty(
            self._window_name,
            cv2.WND_PROP_VISIBLE,
        )
        if visible >= 1.0:
            window_seen = True
        quit_requested = (
            key in (ord("q"), ord("Q"), 27) or
            (window_seen and visible < 1.0)
        )
        return quit_requested, window_seen

    def _preview_loop(self):
        window_seen = False
        if self._preview_enabled:
            cv2.namedWindow(self._window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(
                self._window_name,
                int(INPUT_WIDTH * self._preview_scale),
                int((INPUT_HEIGHT + 58) * self._preview_scale),
            )

        minimum_period = 1.0 / self._preview_fps
        next_allowed_at = time.monotonic()
        processed_generation = 0
        report_started_at = time.monotonic()
        preview_interval = 0
        stage_samples = {
            "preprocess": deque(),
            "inference": deque(),
            "postprocess": deque(),
        }
        latest_preview_fps = 0.0
        last_error_at = 0.0

        try:
            while not self._stop_event.is_set():
                with self._condition:
                    self._condition.wait_for(
                        lambda: (
                            self._stop_event.is_set() or
                            self._generation != processed_generation
                        ),
                        timeout=0.1,
                    )
                    if self._stop_event.is_set():
                        break
                    message = self._latest_message
                    generation = self._generation

                now = time.monotonic()
                if now < next_allowed_at:
                    if self._stop_event.wait(next_allowed_at - now):
                        break
                    with self._condition:
                        message = self._latest_message
                        generation = self._generation

                if message is None or generation == processed_generation:
                    if self._preview_enabled:
                        quit_requested, window_seen = self._window_quit_requested(
                            window_seen
                        )
                        if quit_requested:
                            self._stop_event.set()
                    continue

                if (
                    processed_generation > 0 and
                    generation > processed_generation + 1
                ):
                    self._skipped_total += (
                        generation - processed_generation - 1
                    )
                processed_generation = generation

                try:
                    image = self._message_bgr_view(message)
                    if self._device.type == "cuda":
                        overlay, timing = self._gpu_predict_overlay(image, 0.75)
                    else:
                        overlay, timing = self._cpu_predict_overlay(image, 0.75)
                except Exception as exception:  # noqa: BLE001
                    now = time.monotonic()
                    if now - last_error_at >= 5.0:
                        self.get_logger().error(
                            "BEV inference frame rejected: {}".format(exception)
                        )
                        last_error_at = now
                    continue

                self._processed_total += 1
                preview_interval += 1
                for name in stage_samples:
                    stage_samples[name].append(timing[name])

                if self._preview_enabled:
                    canvas = self._preview_canvas(
                        overlay,
                        timing["inference"],
                        latest_preview_fps,
                    )
                    cv2.imshow(self._window_name, canvas)
                    quit_requested, window_seen = self._window_quit_requested(
                        window_seen
                    )
                    if quit_requested:
                        self._stop_event.set()
                        break

                now = time.monotonic()
                elapsed = now - report_started_at
                if elapsed >= self._status_interval:
                    with self._condition:
                        received = self._received_interval
                        self._received_interval = 0
                    latest_preview_fps = preview_interval / elapsed
                    summaries = {}
                    for name, samples in stage_samples.items():
                        summaries[name] = (
                            sum(samples) / len(samples) if samples else 0.0,
                            max(samples) if samples else 0.0,
                        )
                        samples.clear()
                    self.get_logger().info(
                        "FPS: input={:.1f}, preview={:.1f}/{:.1f} | "
                        "AVG/MAX ms: preprocess+H2D={:.3f}/{:.3f}, "
                        "pure-inference={:.3f}/{:.3f}, "
                        "postprocess+D2H={:.3f}/{:.3f} | skipped={}".format(
                            received / elapsed,
                            latest_preview_fps,
                            self._preview_fps,
                            summaries["preprocess"][0],
                            summaries["preprocess"][1],
                            summaries["inference"][0],
                            summaries["inference"][1],
                            summaries["postprocess"][0],
                            summaries["postprocess"][1],
                            self._skipped_total,
                        )
                    )
                    preview_interval = 0
                    report_started_at = now

                next_allowed_at += minimum_period
                if next_allowed_at < now - minimum_period:
                    next_allowed_at = now + minimum_period
        except Exception as exception:  # noqa: BLE001
            self._worker_error = exception
            self.get_logger().fatal(
                "Line detector preview worker failed: {}".format(exception)
            )
            self._stop_event.set()
        finally:
            if self._preview_enabled:
                try:
                    cv2.destroyWindow(self._window_name)
                except cv2.error:
                    pass

    def close(self):
        self._stop_event.set()
        if hasattr(self, "_condition"):
            with self._condition:
                self._condition.notify_all()
        if (
            hasattr(self, "_worker") and
            self._worker.is_alive() and
            self._worker is not threading.current_thread()
        ):
            self._worker.join(timeout=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LineDetactorNode()
        while rclpy.ok() and not node.stopped:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
