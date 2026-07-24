#!/usr/bin/env python3

from __future__ import annotations

import array
import os
import sys
import threading
import time
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class CameraDriverNode(Node):
    """Capture OAK frames independently from preview and ROS publishing."""

    def __init__(self) -> None:
        super().__init__("camera_driver_node")

        self.declare_parameter("enabled", True)
        self.declare_parameter("camera_socket", "CAM_A")
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("sensor_fps", 30.0)
        self.declare_parameter("resize_mode", "STRETCH")
        self.declare_parameter("undistort_enabled", False)
        self.declare_parameter("queue_size", 4)
        self.declare_parameter("queue_blocking", False)
        self.declare_parameter("frame_id", "camera_optical_frame")
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("publish_enabled", True)
        self.declare_parameter("publish_fps", 30.0)
        self.declare_parameter("preview_enabled", False)
        self.declare_parameter("preview_fps", 30.0)
        self.declare_parameter("preview_window_name", "OAK rectified image")
        self.declare_parameter("preview_max_width", 1280)
        self.declare_parameter("preview_max_height", 720)
        self.declare_parameter("camera_height_m", 0.25)
        self.declare_parameter("camera_downward_angle_deg", 15.0)
        self.declare_parameter("startup_timeout_sec", 5.0)
        self.declare_parameter("status_log_interval_sec", 5.0)

        self._enabled = bool(self.get_parameter("enabled").value)
        self._camera_socket = str(
            self.get_parameter("camera_socket").value
        ).upper()
        self._width = self._positive_int("width")
        self._height = self._positive_int("height")
        self._sensor_fps = self._positive_float("sensor_fps")
        self._resize_mode = str(
            self.get_parameter("resize_mode").value
        ).upper()
        self._undistort_enabled = bool(
            self.get_parameter("undistort_enabled").value
        )
        self._queue_size = self._positive_int("queue_size")
        self._queue_blocking = bool(
            self.get_parameter("queue_blocking").value
        )
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._image_topic = str(
            self.get_parameter("image_topic").value
        )
        self._publish_enabled = bool(
            self.get_parameter("publish_enabled").value
        )
        self._publish_fps = self._positive_float("publish_fps")
        self._preview_enabled = bool(
            self.get_parameter("preview_enabled").value
        )
        self._preview_fps = self._positive_float("preview_fps")
        self._preview_window_name = str(
            self.get_parameter("preview_window_name").value
        )
        self._preview_max_width = self._nonnegative_int(
            "preview_max_width"
        )
        self._preview_max_height = self._nonnegative_int(
            "preview_max_height"
        )
        self._camera_height_m = float(
            self.get_parameter("camera_height_m").value
        )
        self._camera_downward_angle_deg = float(
            self.get_parameter("camera_downward_angle_deg").value
        )
        self._startup_timeout_sec = self._positive_float(
            "startup_timeout_sec"
        )
        self._status_log_interval_sec = self._positive_float(
            "status_log_interval_sec"
        )

        if self._publish_enabled and not self._image_topic:
            raise ValueError("'image_topic' must not be empty")
        if self._preview_enabled and not self._preview_window_name:
            raise ValueError("'preview_window_name' must not be empty")

        self._publisher = (
            self.create_publisher(
                Image,
                self._image_topic,
                qos_profile_sensor_data,
            )
            if self._publish_enabled
            else None
        )
        self._pipeline: Any | None = None
        self._output_queue: Any | None = None
        self._capture_thread: threading.Thread | None = None
        self._publish_thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._frame_lock = threading.Lock()
        self._latest_frame: Any | None = None
        self._latest_stamp: Any | None = None
        self._received_sequence = 0
        self._published_sequence = 0
        self._previewed_sequence = 0
        self._received_total = 0
        self._published_total = 0
        self._previewed_total = 0
        self._received_status_count = 0
        self._published_status_count = 0
        self._previewed_status_count = 0
        self._first_frame_received = False
        self._startup_timeout_reported = False
        self._next_error_log_at = time.monotonic()
        self._started_at = time.monotonic()
        self._status_started_at = self._started_at
        self._cv2: Any | None = None
        self._preview_window_created = False
        self._preview_size: tuple[int, int] | None = None
        self._preview_timer = None
        self._status_timer = None

        if not self._enabled:
            self.get_logger().warn(
                "Camera is disabled by the 'enabled' parameter."
            )
            return

        if self._preview_enabled:
            self._initialize_preview()

        self._open_camera()
        self._capture_thread = threading.Thread(
            target=self._capture_frames,
            name="oak_capture",
            daemon=True,
        )
        self._capture_thread.start()

        if self._publish_enabled:
            self._publish_thread = threading.Thread(
                target=self._publish_frames,
                name="ros_image_publish",
                daemon=True,
            )
            self._publish_thread.start()
        if self._preview_enabled:
            self._preview_timer = self.create_timer(
                1.0 / self._preview_fps,
                self._preview_latest_frame,
            )
        self._status_timer = self.create_timer(
            self._status_log_interval_sec,
            self._log_status,
        )

        self.get_logger().info(
            "OAK camera pipeline started: "
            f"socket={self._camera_socket}, "
            f"output={self._width}x{self._height}, "
            f"sensor_fps={self._sensor_fps:g}, "
            f"depthai_undistortion={self._undistort_enabled}, "
            f"publish={'on' if self._publish_enabled else 'off'}"
            f"@{self._publish_fps:g}Hz, "
            f"topic={self._image_topic}, "
            f"direct_preview={'on' if self._preview_enabled else 'off'}"
            f"@{self._preview_fps:g}Hz"
        )
        self.get_logger().info(
            "Camera mounting information (metadata only): "
            f"height={self._camera_height_m:g}m, "
            f"downward_angle={self._camera_downward_angle_deg:g}deg"
        )

    def _positive_int(self, parameter_name: str) -> int:
        value = int(self.get_parameter(parameter_name).value)
        if value <= 0:
            raise ValueError(f"'{parameter_name}' must be greater than zero")
        return value

    def _nonnegative_int(self, parameter_name: str) -> int:
        value = int(self.get_parameter(parameter_name).value)
        if value < 0:
            raise ValueError(f"'{parameter_name}' must not be negative")
        return value

    def _positive_float(self, parameter_name: str) -> float:
        value = float(self.get_parameter(parameter_name).value)
        if value <= 0.0:
            raise ValueError(f"'{parameter_name}' must be greater than zero")
        return value

    def _initialize_preview(self) -> None:
        if (
            sys.platform.startswith("linux")
            and not os.environ.get("DISPLAY")
            and not os.environ.get("WAYLAND_DISPLAY")
        ):
            self.get_logger().warn(
                "Direct preview disabled because no graphical display "
                "is available."
            )
            self._preview_enabled = False
            return

        try:
            import cv2

            cv2.namedWindow(
                self._preview_window_name,
                cv2.WINDOW_NORMAL,
            )
            self._cv2 = cv2
            self._preview_window_created = True
        except Exception as exc:
            self.get_logger().warn(
                f"Direct preview disabled because its window could not "
                f"be created: {exc}"
            )
            self._preview_enabled = False
            self._cv2 = None
            self._preview_window_created = False

    def _open_camera(self) -> None:
        try:
            import depthai as dai
        except ImportError as exc:
            raise RuntimeError(
                "DepthAI is not installed. Install it with "
                "'python3 -m pip install depthai'."
            ) from exc

        camera_socket = getattr(
            dai.CameraBoardSocket,
            self._camera_socket,
            None,
        )
        if camera_socket is None:
            valid_sockets = self._enum_names(dai.CameraBoardSocket)
            raise ValueError(
                f"Unsupported camera_socket '{self._camera_socket}'. "
                f"Available values: {valid_sockets}"
            )

        resize_mode = getattr(dai.ImgResizeMode, self._resize_mode, None)
        if resize_mode is None:
            valid_modes = self._enum_names(dai.ImgResizeMode)
            raise ValueError(
                f"Unsupported resize_mode '{self._resize_mode}'. "
                f"Available values: {valid_modes}"
            )

        try:
            self._pipeline = dai.Pipeline()
            camera = self._pipeline.create(dai.node.Camera).build(
                camera_socket
            )
            output = camera.requestOutput(
                (self._width, self._height),
                type=dai.ImgFrame.Type.BGR888i,
                resizeMode=resize_mode,
                fps=self._sensor_fps,
                enableUndistortion=self._undistort_enabled,
            )
            self._output_queue = output.createOutputQueue(
                maxSize=self._queue_size,
                blocking=self._queue_blocking,
            )
            self._pipeline.start()
        except Exception as exc:
            self._close_camera()
            raise RuntimeError(
                "Failed to open the OAK camera or start its pipeline: "
                f"{exc}"
            ) from exc

    @staticmethod
    def _enum_names(enum_type: Any) -> str:
        names = [
            name
            for name in dir(enum_type)
            if name.isupper() and not name.startswith("_")
        ]
        return ", ".join(names) if names else "see the installed DepthAI API"

    def _capture_frames(self) -> None:
        while not self._stop_event.is_set():
            output_queue = self._output_queue
            if output_queue is None:
                return

            try:
                packet = output_queue.tryGet()
            except Exception as exc:
                if not self._stop_event.is_set():
                    self._log_receive_error(
                        f"Failed to read the camera queue: {exc}"
                    )
                time.sleep(0.001)
                continue

            if packet is None:
                time.sleep(0.0005)
                continue

            try:
                frame = packet.getCvFrame()
            except Exception as exc:
                self._log_receive_error(
                    "Received a camera packet but could not read its "
                    f"frame: {exc}"
                )
                continue

            if len(frame.shape) != 3 or frame.shape[2] != 3:
                self._log_receive_error(
                    "Expected an unprocessed BGR frame with 3 channels, "
                    f"but received shape={frame.shape}."
                )
                continue

            height, width, _ = frame.shape
            stamp = self.get_clock().now().to_msg()
            first_frame = False
            with self._frame_lock:
                self._latest_frame = frame
                self._latest_stamp = stamp
                self._received_sequence += 1
                self._received_total += 1
                self._received_status_count += 1
                if not self._first_frame_received:
                    self._first_frame_received = True
                    first_frame = True

            if first_frame:
                self.get_logger().info(
                    "Camera frame reception verified: "
                    f"{width}x{height}, encoding=bgr8, "
                    "DepthAI format=BGR888i"
                )

    def _latest_unconsumed_frame(
        self,
        consumed_sequence: int,
    ) -> tuple[Any, Any, int] | None:
        with self._frame_lock:
            if (
                self._latest_frame is None
                or self._latest_stamp is None
                or self._received_sequence == consumed_sequence
            ):
                return None
            return (
                self._latest_frame,
                self._latest_stamp,
                self._received_sequence,
            )

    def _publish_latest_frame(self) -> None:
        if self._publisher is None:
            return

        latest = self._latest_unconsumed_frame(
            self._published_sequence
        )
        if latest is None:
            return
        frame, stamp, sequence = latest
        height, width, _ = frame.shape

        message = Image()
        message.header.stamp = stamp
        message.header.frame_id = self._frame_id
        message.height = height
        message.width = width
        message.encoding = "bgr8"
        message.is_bigendian = False
        message.step = width * 3

        # Humble validates ordinary byte sequences element by element.
        # Passing array('B') uses the generated message's fast path.
        message.data = array.array("B", frame.tobytes())
        self._publisher.publish(message)

        with self._frame_lock:
            self._published_sequence = sequence
            self._published_total += 1
            self._published_status_count += 1

    def _publish_frames(self) -> None:
        publish_period = 1.0 / self._publish_fps
        next_publish_at = time.monotonic()

        while not self._stop_event.is_set():
            now = time.monotonic()
            remaining = next_publish_at - now
            if remaining > 0.0:
                self._stop_event.wait(remaining)
                continue

            try:
                self._publish_latest_frame()
            except Exception as exc:
                if not self._stop_event.is_set():
                    self._log_receive_error(
                        f"Failed to publish the latest ROS image: {exc}"
                    )

            next_publish_at += publish_period
            if next_publish_at < now - publish_period:
                next_publish_at = now + publish_period

    def _preview_latest_frame(self) -> None:
        if self._cv2 is None or not self._preview_window_created:
            return

        latest = self._latest_unconsumed_frame(
            self._previewed_sequence
        )
        if latest is not None:
            frame, _, sequence = latest
            self._resize_preview_window(frame)
            self._cv2.imshow(self._preview_window_name, frame)
            with self._frame_lock:
                self._previewed_sequence = sequence
                self._previewed_total += 1
                self._previewed_status_count += 1

        try:
            key = self._cv2.waitKey(1) & 0xFF
            visible = self._cv2.getWindowProperty(
                self._preview_window_name,
                self._cv2.WND_PROP_VISIBLE,
            )
        except Exception:
            self._disable_preview(
                "Direct preview window is no longer available."
            )
            return

        if key in (ord("q"), ord("Q"), 27):
            self._disable_preview(
                "Direct preview exit requested by keyboard."
            )
        elif visible < 1:
            self._disable_preview("Direct preview window was closed.")

    def _resize_preview_window(self, frame: Any) -> None:
        if self._cv2 is None:
            return

        height, width = frame.shape[:2]
        image_size = (width, height)
        if image_size == self._preview_size:
            return

        scale = 1.0
        if self._preview_max_width > 0:
            scale = min(scale, self._preview_max_width / width)
        if self._preview_max_height > 0:
            scale = min(scale, self._preview_max_height / height)

        window_width = max(1, int(round(width * scale)))
        window_height = max(1, int(round(height * scale)))
        self._cv2.resizeWindow(
            self._preview_window_name,
            window_width,
            window_height,
        )
        self._preview_size = image_size

    def _log_status(self) -> None:
        now = time.monotonic()
        elapsed = max(now - self._status_started_at, 1e-6)
        with self._frame_lock:
            receive_fps = self._received_status_count / elapsed
            publish_fps = self._published_status_count / elapsed
            preview_fps = self._previewed_status_count / elapsed
            received_total = self._received_total
            published_total = self._published_total
            previewed_total = self._previewed_total
            self._received_status_count = 0
            self._published_status_count = 0
            self._previewed_status_count = 0
        self._status_started_at = now

        if (
            not self._first_frame_received
            and not self._startup_timeout_reported
            and now - self._started_at >= self._startup_timeout_sec
        ):
            self._startup_timeout_reported = True
            self.get_logger().error(
                "Camera pipeline started, but no frame was received "
                f"within {self._startup_timeout_sec:g} seconds."
            )

        publish_detail = (
            f"{publish_fps:.1f}Hz/{published_total}"
            if self._publish_enabled
            else "off"
        )
        preview_detail = (
            f"{preview_fps:.1f}Hz/{previewed_total}"
            if self._preview_enabled
            else "off"
        )
        self.get_logger().info(
            "Camera active: "
            f"capture={receive_fps:.1f}Hz/{received_total}, "
            f"ROS_publish={publish_detail}, "
            f"direct_preview={preview_detail}"
        )

    def _log_receive_error(self, message: str) -> None:
        now = time.monotonic()
        if now >= self._next_error_log_at:
            self.get_logger().error(message)
            self._next_error_log_at = now + 1.0

    def _disable_preview(self, reason: str) -> None:
        if not self._preview_enabled:
            return
        self._preview_enabled = False
        if self._preview_timer is not None:
            self._preview_timer.cancel()
        self._close_preview()
        self.get_logger().info(reason)

    def _close_camera(self) -> None:
        self._stop_event.set()
        pipeline = self._pipeline
        self._output_queue = None
        self._pipeline = None
        if pipeline is not None and hasattr(pipeline, "stop"):
            try:
                pipeline.stop()
            except Exception as exc:
                self.get_logger().warn(
                    f"Failed to stop the camera pipeline cleanly: {exc}"
                )
        for worker_thread in (
            self._capture_thread,
            self._publish_thread,
        ):
            if (
                worker_thread is not None
                and worker_thread.is_alive()
                and worker_thread is not threading.current_thread()
            ):
                worker_thread.join(timeout=2.0)
        self._capture_thread = None
        self._publish_thread = None

    def _close_preview(self) -> None:
        if self._cv2 is not None and self._preview_window_created:
            try:
                self._cv2.destroyWindow(self._preview_window_name)
                self._cv2.waitKey(1)
            except Exception:
                pass
        self._preview_window_created = False

    def destroy_node(self) -> None:
        for timer in (
            self._preview_timer,
            self._status_timer,
        ):
            if timer is not None:
                timer.cancel()
        self._close_camera()
        self._close_preview()
        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: CameraDriverNode | None = None
    try:
        node = CameraDriverNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
