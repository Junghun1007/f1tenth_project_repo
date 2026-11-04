"""Run the actual rcl YAML parser on a sourced ROS installation, without hardware."""
import pytest

rclpy = pytest.importorskip('rclpy', reason='Actual rcl parameter parser requires sourced ROS 2')
from rclpy.node import Node
from rclpy.parameter import Parameter
from longitudinal_autotune.storage import atomic_write


def test_humble_rcl_keeps_y_as_string(tmp_path,monkeypatch):
    # No vehicle processes or actuator topics are created by this test.
    monkeypatch.setenv('ROS_DOMAIN_ID','197')
    path = tmp_path/'ros_parameters.yaml'
    atomic_write(path, {'/**': {'ros__parameters': {'imu_yaw_axis':'y','enabled':False}}}, yaml_file=True)
    context = rclpy.context.Context()
    rclpy.init(context=context, args=['--ros-args','--params-file',str(path)])
    node = None
    try:
        node = Node('autotune_parameter_regression_probe',context=context)
        node.declare_parameter('imu_yaw_axis','y')
        node.declare_parameter('enabled',False)
        assert node.get_parameter('imu_yaw_axis').type_ == Parameter.Type.STRING
        assert node.get_parameter('imu_yaw_axis').value == 'y'
        assert node.get_parameter('enabled').type_ == Parameter.Type.BOOL
    finally:
        if node is not None: node.destroy_node()
        rclpy.shutdown(context=context)
