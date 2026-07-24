#!/usr/bin/env python3

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


class NormalImagePublisherNode(Node):
    """Republish the camera image without host-side processing."""

    def __init__(self) -> None:
        super().__init__("normal_image_publisher_node")

        self.declare_parameter("input_topic", "/camera/image_raw")
        self.declare_parameter("output_topic", "/image/normal")
        self.declare_parameter("publish_rate_hz", 15.0)
        self.declare_parameter("status_log_interval_sec", 5.0)

        self._input_topic = str(self.get_parameter("input_topic").value)
        self._output_topic = str(self.get_parameter("output_topic").value)
        self._publish_rate_hz = float(
            self.get_parameter("publish_rate_hz").value
        )
        self._status_log_interval_sec = float(
            self.get_parameter("status_log_interval_sec").value
        )
        self._validate_parameters()

        image_qos = QoSProfile(depth=1)
        image_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        image_qos.durability = DurabilityPolicy.VOLATILE

        self._publisher = self.create_publisher(
            Image,
            self._output_topic,
            image_qos,
        )
        self._subscription = self.create_subscription(
            Image,
            self._input_topic,
            self._on_image,
            image_qos,
        )

        self._frame_count = 0
        self._received_frame_count = 0
        self._status_frame_count = 0
        self._status_started_at = time.monotonic()
        self._publish_period_sec = 1.0 / self._publish_rate_hz
        self._next_publish_at = self._status_started_at
        self._next_status_log_at = (
            self._status_started_at + self._status_log_interval_sec
        )

        self.get_logger().info(
            "Normal image publisher started without host processing: "
            f"{self._input_topic} -> {self._output_topic}, "
            f"publish_rate={self._publish_rate_hz:g}Hz"
        )

    def _validate_parameters(self) -> None:
        if not self._input_topic:
            raise ValueError("'input_topic' must not be empty")
        if not self._output_topic:
            raise ValueError("'output_topic' must not be empty")
        if self._input_topic == self._output_topic:
            raise ValueError(
                "'input_topic' and 'output_topic' must be different "
                "to prevent a publish loop"
            )
        if self._status_log_interval_sec <= 0.0:
            raise ValueError(
                "'status_log_interval_sec' must be greater than zero"
            )
        if self._publish_rate_hz <= 0.0:
            raise ValueError("'publish_rate_hz' must be greater than zero")

    def _on_image(self, message: Image) -> None:
        self._received_frame_count += 1
        now = time.monotonic()
        if now < self._next_publish_at:
            return

        self._publisher.publish(message)
        self._frame_count += 1
        self._status_frame_count += 1
        self._next_publish_at = now + self._publish_period_sec

        if self._frame_count == 1:
            self.get_logger().info(
                "First normal image republished unchanged: "
                f"{message.width}x{message.height}, "
                f"encoding={message.encoding}"
            )

        if now >= self._next_status_log_at:
            elapsed = max(now - self._status_started_at, 1e-6)
            measured_hz = self._status_frame_count / elapsed
            self.get_logger().info(
                "Normal image publisher is active: "
                f"received_total={self._received_frame_count}, "
                f"published_total={self._frame_count}, "
                f"measured_publish_hz={measured_hz:.1f}"
            )
            self._status_started_at = now
            self._status_frame_count = 0
            self._next_status_log_at = (
                now + self._status_log_interval_sec
            )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = NormalImagePublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
