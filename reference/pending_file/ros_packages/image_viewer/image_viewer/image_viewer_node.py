#!/usr/bin/env python3

from __future__ import annotations

import os
import sys
import threading
import time

import cv2
from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


class ImageViewerNode(Node):
    """Preview the latest image from one ROS topic at a configured rate."""

    def __init__(self) -> None:
        super().__init__("image_viewer_node")

        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("preview_fps", 15.0)
        self.declare_parameter("window_name", "")
        self.declare_parameter("fit_to_image", True)
        self.declare_parameter("window_width", 640)
        self.declare_parameter("window_height", 480)
        self.declare_parameter("max_window_width", 1280)
        self.declare_parameter("max_window_height", 720)
        self.declare_parameter("no_frame_timeout_sec", 3.0)

        self._image_topic = str(self.get_parameter("image_topic").value)
        self._preview_fps = self._positive_float("preview_fps")
        configured_window_name = str(
            self.get_parameter("window_name").value
        )
        self._window_name = configured_window_name or self.get_name()
        self._fit_to_image = bool(
            self.get_parameter("fit_to_image").value
        )
        self._window_width = self._nonnegative_int("window_width")
        self._window_height = self._nonnegative_int("window_height")
        self._max_window_width = self._nonnegative_int(
            "max_window_width"
        )
        self._max_window_height = self._nonnegative_int(
            "max_window_height"
        )
        self._no_frame_timeout_sec = self._positive_float(
            "no_frame_timeout_sec"
        )

        if not self._image_topic:
            raise ValueError("'image_topic' must not be empty")

        self._ensure_display_available()

        self._bridge = CvBridge()
        self._message_lock = threading.Lock()
        self._latest_message: Image | None = None
        self._received_sequence = 0
        self._displayed_sequence = 0
        self._started_at = time.monotonic()
        self._last_frame_received_at: float | None = None
        self._next_no_frame_warning_at = (
            self._started_at + self._no_frame_timeout_sec
        )
        self._last_image_size: tuple[int, int] | None = None
        self.exit_requested = False
        self._window_created = False

        image_qos = QoSProfile(depth=1)
        image_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        image_qos.durability = DurabilityPolicy.VOLATILE
        self._subscription = self.create_subscription(
            Image,
            self._image_topic,
            self._on_image,
            image_qos,
        )

        self._create_window()
        self._preview_timer = self.create_timer(
            1.0 / self._preview_fps,
            self._update_preview,
        )
        self.get_logger().info(
            "Image viewer started: "
            f"topic={self._image_topic}, "
            f"preview_fps={self._preview_fps:g}, "
            f"window='{self._window_name}'"
        )

    def _positive_float(self, parameter_name: str) -> float:
        value = float(self.get_parameter(parameter_name).value)
        if value <= 0.0:
            raise ValueError(f"'{parameter_name}' must be greater than zero")
        return value

    def _nonnegative_int(self, parameter_name: str) -> int:
        value = int(self.get_parameter(parameter_name).value)
        if value < 0:
            raise ValueError(f"'{parameter_name}' must not be negative")
        return value

    @staticmethod
    def _ensure_display_available() -> None:
        if (
            sys.platform.startswith("linux")
            and not os.environ.get("DISPLAY")
            and not os.environ.get("WAYLAND_DISPLAY")
        ):
            raise RuntimeError(
                "No graphical display is available. Set DISPLAY or "
                "WAYLAND_DISPLAY before starting image_viewer."
            )

    def _create_window(self) -> None:
        try:
            cv2.namedWindow(self._window_name, cv2.WINDOW_NORMAL)
            if (
                not self._fit_to_image
                and self._window_width > 0
                and self._window_height > 0
            ):
                cv2.resizeWindow(
                    self._window_name,
                    self._window_width,
                    self._window_height,
                )
            self._window_created = True
        except cv2.error as exc:
            raise RuntimeError(
                f"Failed to create preview window '{self._window_name}': {exc}"
            ) from exc

    def _on_image(self, message: Image) -> None:
        now = time.monotonic()
        with self._message_lock:
            self._latest_message = message
            self._received_sequence += 1
            self._last_frame_received_at = now
            self._next_no_frame_warning_at = (
                now + self._no_frame_timeout_sec
            )

    def _update_preview(self) -> None:
        now = time.monotonic()
        message = None
        sequence = self._displayed_sequence

        with self._message_lock:
            if self._received_sequence != self._displayed_sequence:
                message = self._latest_message
                sequence = self._received_sequence

        if message is not None:
            try:
                frame = self._bridge.imgmsg_to_cv2(
                    message,
                    desired_encoding="bgr8",
                )
                self._fit_window_to_image(
                    image_width=frame.shape[1],
                    image_height=frame.shape[0],
                )
                cv2.imshow(self._window_name, frame)
                self._displayed_sequence = sequence
            except (CvBridgeError, cv2.error) as exc:
                self.get_logger().error(
                    f"Failed to display an image from {self._image_topic}: {exc}"
                )

        self._warn_if_no_frames(now)
        self._handle_window_events()

    def _fit_window_to_image(
        self,
        image_width: int,
        image_height: int,
    ) -> None:
        if not self._fit_to_image:
            return

        image_size = (image_width, image_height)
        if image_size == self._last_image_size:
            return

        scale = 1.0
        if self._max_window_width > 0:
            scale = min(scale, self._max_window_width / image_width)
        if self._max_window_height > 0:
            scale = min(scale, self._max_window_height / image_height)

        window_width = max(1, int(round(image_width * scale)))
        window_height = max(1, int(round(image_height * scale)))
        cv2.resizeWindow(
            self._window_name,
            window_width,
            window_height,
        )
        self._last_image_size = image_size
        self.get_logger().info(
            "Preview window resized for image: "
            f"image={image_width}x{image_height}, "
            f"window={window_width}x{window_height}"
        )

    def _warn_if_no_frames(self, now: float) -> None:
        if now < self._next_no_frame_warning_at:
            return

        if self._last_frame_received_at is None:
            detail = "no image has been received"
        else:
            stale_for = now - self._last_frame_received_at
            detail = f"the last image was received {stale_for:.1f}s ago"

        self.get_logger().warn(
            f"Waiting for {self._image_topic}: {detail}."
        )
        self._next_no_frame_warning_at = now + self._no_frame_timeout_sec

    def _handle_window_events(self) -> None:
        try:
            key = cv2.waitKey(1) & 0xFF
            window_visible = cv2.getWindowProperty(
                self._window_name,
                cv2.WND_PROP_VISIBLE,
            )
        except cv2.error:
            self._request_exit("Preview window is no longer available.")
            return

        if key in (ord("q"), ord("Q"), 27):
            self._request_exit("Preview exit requested by keyboard.")
        elif window_visible < 1:
            self._request_exit("Preview window was closed.")

    def _request_exit(self, reason: str) -> None:
        if self.exit_requested:
            return
        self.exit_requested = True
        self._preview_timer.cancel()
        self.get_logger().info(reason)

    def destroy_node(self) -> None:
        preview_timer = getattr(self, "_preview_timer", None)
        if preview_timer is not None:
            preview_timer.cancel()
        if self._window_created:
            try:
                cv2.destroyWindow(self._window_name)
                cv2.waitKey(1)
            except cv2.error:
                pass
            self._window_created = False
        super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: ImageViewerNode | None = None
    try:
        node = ImageViewerNode()
        while rclpy.ok() and not node.exit_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
