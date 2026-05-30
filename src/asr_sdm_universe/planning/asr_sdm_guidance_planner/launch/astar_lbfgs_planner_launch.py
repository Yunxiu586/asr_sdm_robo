from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('asr_sdm_guidance_planner')
    default_config = os.path.join(pkg_share, 'config', 'astar_lbfgs_planner.yaml')

    config_arg = DeclareLaunchArgument(
        'config',
        default_value=default_config,
        description='Path to the A* + L-BFGS planner configuration file.'
    )

    planner_node = Node(
        package='asr_sdm_guidance_planner',
        executable='rviz_astar_lbfgs_planner',
        name='astar_lbfgs_trajectory_planner',
        output='screen',
        parameters=[LaunchConfiguration('config')]
    )

    return LaunchDescription([
        config_arg,
        planner_node,
    ])
