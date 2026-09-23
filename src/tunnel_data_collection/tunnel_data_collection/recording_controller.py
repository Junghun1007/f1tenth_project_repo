"""Service-controlled MP4 recorder for RGB, stereo IR, and stereo IR BEV."""

from datetime import datetime
import json
from pathlib import Path
import shutil
import subprocess

import cv2
import numpy as np
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
    ffmpeg_mp4_command,
    new_session_path,
    validate_recording_settings,
)


class FfmpegVideoWriter:
    """Stream tightly packed BGR frames to one FFmpeg video encoder."""

    def __init__(self, command, output_path):
        self._output_path = Path(output_path)
        self._log_path = self._output_path.with_suffix(".ffmpeg.log")
        self._log_file = self._log_path.open("wb")
        try:
            self._process = subprocess.Popen(
                command,
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=self._log_file,
                bufsize=0,
            )
        except Exception:
            self._log_file.close()
            raise

    def _error_details(self):
        self._log_file.flush()
        try:
            details = self._log_path.read_text(
                encoding="utf-8", errors="replace"
            ).strip()
        except OSError:
            return ""
        return details[-2000:]

    def write(self, frame):
        return_code = self._process.poll()
        if return_code is not None:
            details = self._error_details()
            raise RuntimeError(
                f"FFmpeg exited with code {return_code}: {details}"
            )
        packed = np.ascontiguousarray(frame, dtype=np.uint8)
        if packed.ndim != 3 or packed.shape[2] != 3:
            raise ValueError("FFmpeg video input must be an HxWx3 BGR image")
        remaining = memoryview(packed).cast("B")
        try:
            while remaining:
                written = self._process.stdin.write(remaining)
                if not written:
                    raise BrokenPipeError("FFmpeg input pipe closed")
                remaining = remaining[written:]
        except (BrokenPipeError, OSError) as error:
            details = self._error_details()
            raise RuntimeError(f"FFmpeg write failed: {error}; {details}") from error

    def close(self):
        if self._process.stdin is not None and not self._process.stdin.closed:
            try:
                self._process.stdin.close()
            except (BrokenPipeError, OSError):
                pass
        try:
            return_code = self._process.wait(timeout=30.0)
        except subprocess.TimeoutExpired as error:
            self._process.kill()
            self._process.wait(timeout=5.0)
            self._log_file.close()
            raise RuntimeError(
                f"FFmpeg did not finish {self._output_path.name}"
            ) from error
        details = self._error_details()
        self._log_file.close()
        if return_code != 0:
            raise RuntimeError(
                f"FFmpeg exited with code {return_code} for "
                f"{self._output_path.name}: {details}"
            )
        if self._log_path.exists() and self._log_path.stat().st_size == 0:
            self._log_path.unlink()


class RecordingController(Node):
    def __init__(self):
        super().__init__("tunnel_recording_controller")
        self._output_root = Path(
            self.declare_parameter("output_root", "tunnel_recordings").value
        ).expanduser()
        self._recording_fps, self._h264_crf, self._h264_preset = (
            validate_recording_settings(
                self.declare_parameter("recording_fps", 30.0).value,
                self.declare_parameter("h264_crf", 18).value,
                self.declare_parameter("h264_preset", "ultrafast").value,
            )
        )
        configured_ffmpeg = str(
            self.declare_parameter("ffmpeg_binary", "ffmpeg").value
        ).strip()
        self._ffmpeg_binary = shutil.which(configured_ffmpeg)
        if self._ffmpeg_binary is None:
            raise RuntimeError(
                f"FFmpeg executable not found: {configured_ffmpeg}. "
                "Install it with: sudo apt install ffmpeg"
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
        self._encoder_errors = {}
        self._publish_state()
        self.get_logger().info(
            f"MP4 recorder ready but idle: {self._recording_fps:.3f} FPS, "
            f"H.264 CRF={self._h264_crf}, preset={self._h264_preset}, "
            f"streams={len(VIDEO_STREAMS)}"
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
        self._encoder_errors = {}
        self._recording = True
        self._publish_state()
        self.get_logger().info(f"MP4 RECORDING STARTED: {self._session_path}")
        return True, f"MP4 recording started: {self._session_path}"

    def _message_stamp_ns(self, message):
        stamp = (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )
        return stamp if stamp > 0 else self.get_clock().now().nanoseconds

    def _open_writer(self, topic, frame):
        height, width = frame.shape[:2]
        path = self._session_path / VIDEO_STREAMS[topic]
        command = ffmpeg_mp4_command(
            self._ffmpeg_binary,
            path,
            width,
            height,
            self._recording_fps,
            self._h264_crf,
            self._h264_preset,
        )
        writer = FfmpegVideoWriter(command, path)
        self._writers[topic] = writer
        self._stream_sizes[topic] = [width, height]
        self.get_logger().info(
            f"MP4 stream opened: {path.name} {width}x{height} "
            f"@ {self._recording_fps:.3f} FPS"
        )
        return writer

    def _on_image(self, topic, message):
        if not self._recording or topic in self._encoder_errors:
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
            self.get_logger().error(f"MP4 frame rejected for {topic}: {error}")
            self._encoder_errors[topic] = str(error)

    def _write_metadata(self, stopped_at):
        metadata = {
            "format": "MP4",
            "codec": "H.264/libx264",
            "h264_crf": self._h264_crf,
            "h264_preset": self._h264_preset,
            "encoder_backend": "ffmpeg",
            "recording_fps": self._recording_fps,
            "started_at": self._started_at.isoformat(),
            "stopped_at": stopped_at.isoformat(),
            "streams": {
                topic: {
                    "filename": filename,
                    "frames": self._frame_counts.get(topic, 0),
                    "resolution": self._stream_sizes.get(topic),
                    "file_size_bytes": self._file_size(filename),
                    "encoder_error": self._encoder_errors.get(topic),
                }
                for topic, filename in VIDEO_STREAMS.items()
            },
        }
        (self._session_path / "recording_metadata.json").write_text(
            json.dumps(metadata, indent=2), encoding="utf-8"
        )

    def _file_size(self, filename):
        path = self._session_path / filename
        return path.stat().st_size if path.exists() else 0

    def _stop(self):
        if not self._recording:
            return False, "recorder is already stopped"
        self._recording = False
        for topic, writer in self._writers.items():
            try:
                writer.close()
            except RuntimeError as error:
                self._encoder_errors[topic] = str(error)
                self.get_logger().error(
                    f"Could not finalize {VIDEO_STREAMS[topic]}: {error}"
                )
        self._writers = {}
        for topic, count in self._frame_counts.items():
            if count == 0 and topic not in self._encoder_errors:
                self._encoder_errors[topic] = "no frames received"
        stopped_at = datetime.now().astimezone()
        self._write_metadata(stopped_at)
        self._publish_state()
        counts = ", ".join(
            f"{VIDEO_STREAMS[topic]}={self._frame_counts[topic]}"
            for topic in VIDEO_STREAMS
        )
        self.get_logger().info(
            f"MP4 RECORDING STOPPED: {self._session_path} ({counts})"
        )
        if self._encoder_errors:
            failed = ", ".join(
                VIDEO_STREAMS[topic] for topic in self._encoder_errors
            )
            return False, f"MP4 recording stopped with errors: {failed}"
        return True, f"MP4 recording stopped: {self._session_path}"

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
