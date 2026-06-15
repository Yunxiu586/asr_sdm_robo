from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('bspline')
    default_config = os.path.join(pkg_share, 'config', 'bspline_path_test.yaml')
    default_rviz_config = os.path.join(pkg_share, 'rviz', 'bspline_path_test.rviz')

    config_arg = DeclareLaunchArgument(
        'config',
        default_value=default_config,
        description='Path to B-spline path test configuration file.'
    )

    # Default false: in the full chain, guidance planner usually starts the shared RViz page.
    # Set use_rviz:=true only when testing this node independently.
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Whether to start a standalone RViz2 window for B-spline path testing.'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='Path to the RViz2 configuration file.'
    )

    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='world',
        description='Frame used by B-spline outputs. Keep it the same as guidance planner and RViz Fixed Frame.'
    )

    bspline_node = Node(
        package='bspline',
        executable='bspline_test_node',
        name='bspline_test_node',
        output='screen',
        parameters=[LaunchConfiguration('config'), {'frame_id': LaunchConfiguration('frame_id')}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_bspline_path_test',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    return LaunchDescription([
        config_arg,
        use_rviz_arg,
        rviz_config_arg,
        frame_id_arg,
        bspline_node,
        rviz_node,
    ])
