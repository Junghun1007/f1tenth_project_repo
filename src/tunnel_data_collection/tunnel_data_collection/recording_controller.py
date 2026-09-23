"""Service-controlled AVI recorder for RGB, stereo IR, and stereo IR BEV."""

from datetime import datetime
import json
from pathlib import Path

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from .image_utils import decode_image
from .recording_utils import (
    FrameRateGate,
    VIDEO_STREAMS,
    new_session_path,
    validate_recording_settings,
)


class RecordingController(Node):
    def __init__(self):
        super().__init__("tunnel_recording_controller")
        self._output_root = Path(
            self.declare_parameter("output_root", "tunnel_recordings").value
        ).expanduser()
        self._recording_fps, self._codec = validate_recording_settings(
            self.declare_parameter("recording_fps", 30.0).value,
            self.declare_parameter("avi_codec", "MJPG").value,
        )
        self._output_root.mkdir(parents=True, exist_ok=True)

        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._recording_publisher = self.create_publisher(
            Bool, "~/recording", status_qos
        )
        self._session_publisher = self.create_publisher(
            String, "~/session_path", status_qos
        )
        self._subscriptions = []
        for topic in VIDEO_STREAMS:
            self._subscriptions.append(self.create_subscription(
                Image,
                topic,
                lambda message, stream_topic=topic: self._on_image(
                    stream_topic, message
                ),
                qos_profile_sensor_data,
            ))
        self.create_service(Trigger, "~/start", self._on_start)
        self.create_service(Trigger, "~/stop", self._on_stop)
        self.create_service(Trigger, "~/toggle", self._on_toggle)

        self._recording = False
        self._session_path = None
        self._started_at = None
        self._writers = {}
        self._gates = {}
        self._frame_counts = {}
        self._stream_sizes = {}
        self._publish_state()
        self.get_logger().info(
            f"AVI recorder ready but idle: {self._recording_fps:.3f} FPS, "
            f"codec={self._codec}, streams={len(VIDEO_STREAMS)}"
        )

    def _publish_state(self):
        self._recording_publisher.publish(Bool(data=self._recording))
        path = "" if self._session_path is None else str(self._session_path)
        self._session_publisher.publish(String(data=path))

    def _start(self):
        if self._recording:
            return False, f"already recording: {self._session_path}"
        self._session_path = new_session_path(self._output_root)
        self._session_path.mkdir(parents=True, exist_ok=False)
        self._started_at = datetime.now().astimezone()
        self._writers = {}
        self._gates = {
            topic: FrameRateGate(self._recording_fps) for topic in VIDEO_STREAMS
        }
        self._frame_counts = {topic: 0 for topic in VIDEO_STREAMS}
        self._stream_sizes = {}
        self._recording = True
        self._publish_state()
        self.get_logger().info(f"AVI RECORDING STARTED: {self._session_path}")
        return True, f"AVI recording started: {self._session_path}"

    def _message_stamp_ns(self, message):
        stamp = (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )
        return stamp if stamp > 0 else self.get_clock().now().nanoseconds

    def _open_writer(self, topic, frame):
        height, width = frame.shape[:2]
        path = self._session_path / VIDEO_STREAMS[topic]
        fourcc = cv2.VideoWriter_fourcc(*self._codec)
        writer = cv2.VideoWriter(
            str(path), fourcc, self._recording_fps, (width, height), True
        )
        if not writer.isOpened():
            writer.release()
            raise RuntimeError(
                f"OpenCV could not open {path} with AVI codec {self._codec}"
            )
        self._writers[topic] = writer
        self._stream_sizes[topic] = [width, height]
        self.get_logger().info(
            f"AVI stream opened: {path.name} {width}x{height} "
            f"@ {self._recording_fps:.3f} FPS"
        )
        return writer

    def _on_image(self, topic, message):
        if not self._recording:
            return
        stamp_ns = self._message_stamp_ns(message)
        if not self._gates[topic].accept(stamp_ns):
            return
        try:
            frame = decode_image(
                data=message.data,
                width=int(message.width),
                height=int(message.height),
                step=int(message.step),
                encoding=message.encoding,
            )
            writer = self._writers.get(topic)
            if writer is None:
                writer = self._open_writer(topic, frame)
            expected_size = tuple(self._stream_sizes[topic])
            if (frame.shape[1], frame.shape[0]) != expected_size:
                raise RuntimeError(
                    f"{topic} resolution changed during recording: "
                    f"{frame.shape[1]}x{frame.shape[0]} != "
                    f"{expected_size[0]}x{expected_size[1]}"
                )
            writer.write(frame)
            self._frame_counts[topic] += 1
        except (ValueError, RuntimeError, cv2.error) as error:
            self.get_logger().error(f"AVI frame rejected for {topic}: {error}")

    def _write_metadata(self, stopped_at):
        metadata = {
            "format": "AVI",
            "codec": self._codec,
            "recording_fps": self._recording_fps,
            "started_at": self._started_at.isoformat(),
            "stopped_at": stopped_at.isoformat(),
            "streams": {
                topic: {
                    "filename": filename,
                    "frames": self._frame_counts.get(topic, 0),
                    "resolution": self._stream_sizes.get(topic),
                }
                for topic, filename in VIDEO_STREAMS.items()
            },
        }
        (self._session_path / "recording_metadata.json").write_text(
            json.dumps(metadata, indent=2), encoding="utf-8"
        )

    def _stop(self):
        if not self._recording:
            return False, "recorder is already stopped"
        self._recording = False
        for writer in self._writers.values():
            writer.release()
        self._writers = {}
        stopped_at = datetime.now().astimezone()
        self._write_metadata(stopped_at)
        self._publish_state()
        counts = ", ".join(
            f"{VIDEO_STREAMS[topic]}={self._frame_counts[topic]}"
            for topic in VIDEO_STREAMS
        )
        self.get_logger().info(
            f"AVI RECORDING STOPPED: {self._session_path} ({counts})"
        )
        return True, f"AVI recording stopped: {self._session_path}"

    def _on_start(self, _request, response):
        try:
            response.success, response.message = self._start()
        except OSError as error:
            response.success, response.message = False, str(error)
        return response

    def _on_stop(self, _request, response):
        response.success, response.message = self._stop()
        return response

    def _on_toggle(self, _request, response):
        if self._recording:
            response.success, response.message = self._stop()
        else:
            try:
                response.success, response.message = self._start()
            except OSError as error:
                response.success, response.message = False, str(error)
        return response

    def destroy_node(self):
        if self._recording:
            self._stop()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RecordingController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
