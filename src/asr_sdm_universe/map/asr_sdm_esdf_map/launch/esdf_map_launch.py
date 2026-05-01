from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rviz_config = get_package_share_directory("asr_sdm_esdf_map") + "/demo.rviz"
    sdf_map_config = (
        get_package_share_directory("asr_sdm_esdf_map") + "/config/sdf_map_config.yaml"
    )

    sdf_map_node = Node(
        package="asr_sdm_esdf_map",
        executable="sdf_map_node",
        name="sdf_map",
        output="screen",
        emulate_tty=True,
        parameters=[sdf_map_config],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rvizvisualisation",
        output="log",
        arguments=["-d", rviz_config],
    )

    return LaunchDescription([sdf_map_node, rviz_node])
