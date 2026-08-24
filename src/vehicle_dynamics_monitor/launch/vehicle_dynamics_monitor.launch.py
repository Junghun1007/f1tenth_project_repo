from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    input_mode = LaunchConfiguration("input_mode")
    can_interface = LaunchConfiguration("can_interface")
    can_controller_id = LaunchConfiguration("can_controller_id")

    default_params = PathJoinSubstitution(
        [
            FindPackageShare("vehicle_dynamics_monitor"),
            "config",
            "vehicle_dynamics.yaml",
        ]
    )

    dynamics_node = Node(
        package="vehicle_dynamics_monitor",
        executable="vehicle_dynamics_node",
        name="vehicle_dynamics_node",
        output="screen",
        parameters=[
            params_file,
            {
                "input_mode": input_mode,
                "can_interface": can_interface,
                "can_controller_id": ParameterValue(
                    can_controller_id, value_type=int
                ),
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Vehicle dynamics monitor parameter YAML.",
            ),
            DeclareLaunchArgument(
                "input_mode",
                default_value="ros_topic",
                description="ros_topic or receive-only socketcan.",
            ),
            DeclareLaunchArgument("can_interface", default_value="can0"),
            DeclareLaunchArgument("can_controller_id", default_value="0"),
            dynamics_node,
        ]
    )
