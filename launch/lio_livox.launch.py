from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('ig_lio')

    ig_lio_config = os.path.join(pkg_share, 'config', 'avia.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'ros2.rviz')

    # Single source of truth for sim time, shared by every node below. Set
    # use_sim_time:=true together with `ros2 bag play --clock` so the whole
    # system's "now" follows bag time (otherwise leave it false to avoid a
    # clock frozen at 0). Coerced to bool since launch args are strings.
    use_sim_time = ParameterValue(
        LaunchConfiguration('use_sim_time'), value_type=bool)

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use the /clock topic as the time source for all nodes '
                        '(pair with "ros2 bag play --clock").',
        ),

        # Launch RViz alongside the node when rviz:=true (default: off)
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Launch RViz2 with the bundled ros2.rviz config.',
        ),

        # ig_lio node
        #
        # NOTE: Livox support is compiled in only when the package was built with
        # livox_ros_driver2 present in the workspace. It is ported from upstream
        # but UNTESTED in this ROS 2 port -- see the README sensor-support table.
        Node(
            package='ig_lio',
            executable='ig_lio_node',
            name='ig_lio_node',
            output='screen',
            parameters=[
                ig_lio_config,
                # ig_lio timestamps everything from the sensor data, not the ROS
                # clock, so use_sim_time has no effect on it. Wired up anyway for
                # completeness and future-proofing (e.g. if a node that does read
                # the clock is added, or for bag playback with --clock).
                {'use_sim_time': use_sim_time},
            ],
        ),

        # RViz2 (only when rviz:=true)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
