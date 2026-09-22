"""ROS service controlled rosbag recorder for tunnel runs."""

import os
from pathlib import Path
import signal
import subprocess

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from .recording_utils import new_session_path, rosbag_command


class RecordingController(Node):
    def __init__(self):
        super().__init__("tunnel_recording_controller")
        self._output_root = Path(
            self.declare_parameter("output_root", "tunnel_recordings").value
        ).expanduser()
        self._max_bag_size = int(
            self.declare_parameter("max_bag_size", 4294967296).value
        )
        rosbag_command(self._output_root / "validation", self._max_bag_size)
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
        self.create_service(Trigger, "~/start", self._on_start)
        self.create_service(Trigger, "~/stop", self._on_stop)
        self.create_service(Trigger, "~/toggle", self._on_toggle)
        self.create_timer(0.5, self._poll_process)

        self._process = None
        self._session_path = ""
        self._publish_state()
        self.get_logger().info(
            "Recorder ready but idle. Call ~/start to record only RGB and "
            "stereo-IR BEV."
        )

    def _publish_state(self):
        active = self._process is not None and self._process.poll() is None
        self._recording_publisher.publish(Bool(data=active))
        self._session_publisher.publish(String(data=self._session_path))

    def _start(self):
        if self._process is not None and self._process.poll() is None:
            return False, f"already recording: {self._session_path}"
        session = new_session_path(self._output_root)
        command = rosbag_command(session, self._max_bag_size)
        try:
            self._process = subprocess.Popen(command, start_new_session=True)
        except OSError as error:
            self.get_logger().error(f"could not start rosbag: {error}")
            return False, f"could not start rosbag: {error}"
        self._session_path = str(session)
        self._publish_state()
        self.get_logger().info(f"RECORDING STARTED: {session}")
        return True, f"recording started: {session}"

    def _stop(self):
        if self._process is None or self._process.poll() is not None:
            self._process = None
            self._publish_state()
            return False, "recorder is already stopped"
        process = self._process
        try:
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=15.0)
        except subprocess.TimeoutExpired:
            self.get_logger().warning("rosbag did not stop after SIGINT; terminating")
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5.0)
        self._process = None
        self._publish_state()
        self.get_logger().info(f"RECORDING STOPPED: {self._session_path}")
        return True, f"recording stopped: {self._session_path}"

    def _on_start(self, _request, response):
        response.success, response.message = self._start()
        return response

    def _on_stop(self, _request, response):
        response.success, response.message = self._stop()
        return response

    def _on_toggle(self, _request, response):
        if self._process is not None and self._process.poll() is None:
            response.success, response.message = self._stop()
        else:
            response.success, response.message = self._start()
        return response

    def _poll_process(self):
        if self._process is None:
            return
        return_code = self._process.poll()
        if return_code is None:
            return
        self.get_logger().error(
            f"rosbag exited unexpectedly with code {return_code}: {self._session_path}"
        )
        self._process = None
        self._publish_state()

    def destroy_node(self):
        if self._process is not None and self._process.poll() is None:
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
