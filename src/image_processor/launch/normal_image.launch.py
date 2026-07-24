from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    camera_config = PathJoinSubstitution(
        [FindPackageShare("camera_driver"), "config", "camera_config.yaml"]
    )
    normal_image_config = PathJoinSubstitution(
        [FindPackageShare("image_processor"), "config", "normal_image.yaml"]
    )

    camera_driver_node = Node(
        package="camera_driver",
        executable="camera_driver_node",
        name="camera_driver_node",
        output="screen",
        # normal_image.yaml의 camera_driver_node 항목이 기본 카메라 설정을
        # 덮어써서 DepthAI 왜곡 보정과 /image/normal 직접 발행을 결정한다.
        parameters=[camera_config, normal_image_config],
    )

    return LaunchDescription([camera_driver_node])
