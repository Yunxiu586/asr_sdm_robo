from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('asr_sdm_local_path_modifier')
    default_config = os.path.join(pkg_share, 'config', 'local_path_modifier.yaml')
    default_rviz_config = os.path.join(pkg_share, 'rviz', 'local_path_modifier_test.rviz')

    config_arg = DeclareLaunchArgument(
        'config',
        default_value=default_config,
        description='Path to local path modifier test configuration file.'
    )

    # Default false: when guidance planner is already launched with RViz, this node reuses
    # the same RViz page through the shared /planning/* topics.
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Whether to start a standalone RViz2 window for local modifier testing.'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='Path to the RViz2 configuration file.'
    )

    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='world',
        description='Frame used by modifier outputs. Keep it the same as guidance planner and RViz Fixed Frame.'
    )

    modifier_node = Node(
        package='asr_sdm_local_path_modifier',
        executable='local_path_modifier_test',
        name='local_path_modifier_test',
        output='screen',
        parameters=[LaunchConfiguration('config'), {'frame_id': LaunchConfiguration('frame_id')}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_local_path_modifier',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    return LaunchDescription([
        config_arg,
        use_rviz_arg,
        rviz_config_arg,
        frame_id_arg,
        modifier_node,
        rviz_node,
    ])
