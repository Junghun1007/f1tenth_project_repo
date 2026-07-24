#!/usr/bin/env python3

from __future__ import annotations

import time
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class CameraDriverNode(Node):
    """Open an OAK camera and publish unprocessed RGB frames."""

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
        image_topic = str(self.get_parameter("image_topic").value)
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

        self._publisher = self.create_publisher(
            Image,
            image_topic,
            qos_profile_sensor_data,
        )
        self._pipeline: Any | None = None
        self._output_queue: Any | None = None
        self._timer = None
        self._started_at = time.monotonic()
        self._status_started_at = self._started_at
        self._next_status_log_at = (
            self._started_at + self._status_log_interval_sec
        )
        self._next_error_log_at = self._started_at
        self._frame_count = 0
        self._status_frame_count = 0
        self._first_frame_received = False
        self._startup_timeout_reported = False

        if not self._enabled:
            self.get_logger().warn(
                "Camera is disabled by the 'enabled' parameter."
            )
            return

        self._open_camera()
        # Poll slightly faster than the configured FPS so non-blocking queues
        # are drained without adding a full-frame period of latency.
        poll_rate_hz = max(60.0, self._sensor_fps * 2.0)
        self._timer = self.create_timer(
            1.0 / poll_rate_hz,
            self._receive_frame,
        )
        self.get_logger().info(
            "OAK camera pipeline started: "
            f"socket={self._camera_socket}, "
            f"output={self._width}x{self._height}, "
            f"sensor_fps={self._sensor_fps:g}, "
            f"depthai_undistortion={self._undistort_enabled}, "
            f"topic={image_topic}"
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

    def _positive_float(self, parameter_name: str) -> float:
        value = float(self.get_parameter(parameter_name).value)
        if value <= 0.0:
            raise ValueError(f"'{parameter_name}' must be greater than zero")
        return value

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

    def _receive_frame(self) -> None:
        if self._output_queue is None:
            return

        try:
            packet = self._output_queue.tryGet()
        except Exception as exc:
            self._log_receive_error(f"Failed to read the camera queue: {exc}")
            return

        now = time.monotonic()
        if packet is None:
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
            return

        try:
            frame = packet.getCvFrame()
        except Exception as exc:
            self._log_receive_error(
                f"Received a camera packet but could not read its frame: {exc}"
            )
            return

        if len(frame.shape) != 3 or frame.shape[2] != 3:
            self._log_receive_error(
                "Expected an unprocessed BGR frame with 3 channels, "
                f"but received shape={frame.shape}."
            )
            return

        height, width, _ = frame.shape
        if not self._first_frame_received:
            self._first_frame_received = True
            self.get_logger().info(
                "Camera frame reception verified: "
                f"{width}x{height}, encoding=bgr8, transport=uncompressed"
            )

        message = Image()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._frame_id
        message.height = height
        message.width = width
        message.encoding = "bgr8"
        message.is_bigendian = False
        message.step = width * 3
        message.data = frame.tobytes()
        self._publisher.publish(message)
        self._frame_count += 1
        self._status_frame_count += 1

        if now >= self._next_status_log_at:
            elapsed = max(now - self._status_started_at, 1e-6)
            measured_fps = self._status_frame_count / elapsed
            self.get_logger().info(
                "Camera is receiving and publishing every frame: "
                f"total={self._frame_count}, measured_fps={measured_fps:.1f}"
            )
            self._status_started_at = now
            self._status_frame_count = 0
            self._next_status_log_at = now + self._status_log_interval_sec

    def _log_receive_error(self, message: str) -> None:
        now = time.monotonic()
        if now >= self._next_error_log_at:
            self.get_logger().error(message)
            self._next_error_log_at = now + 1.0

    def _close_camera(self) -> None:
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

    def destroy_node(self) -> None:
        if self._timer is not None:
            self._timer.cancel()
        self._close_camera()
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
