from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ig_lio_config = os.path.join(
        get_package_share_directory('ig_lio'),
        'config',
        'ouster128.yaml'
    )

    return LaunchDescription([

        # Static transforms
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='ouster_broadcaster',
            arguments=['--x', '0', '--y', '0', '--z', '0',
                       '--roll', '0', '--pitch', '0', '--yaw', '0',
                       '--frame-id', 'base_link',
                       '--child-frame-id', 'os_sensor'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_broadcaster',
            arguments=['--x', '0', '--y', '0', '--z', '0',
                       '--roll', '1.5708', '--pitch', '0', '--yaw', '0',
                       '--frame-id', 'base_link',
                       '--child-frame-id', 'imu'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_broadcaster',
            arguments=['--x', '0', '--y', '0', '--z', '0',
                       '--roll', '0', '--pitch', '0', '--yaw', '0',
                       '--frame-id', 'base_link',
                       '--child-frame-id', 'camera_link'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='gps_broadcaster',
            arguments=['--x', '0', '--y', '0', '--z', '0',
                       '--roll', '0', '--pitch', '0', '--yaw', '0',
                       '--frame-id', 'base_link',
                       '--child-frame-id', 'gps'],
        ),

        # ig_lio node
        Node(
            package='ig_lio',
            executable='ig_lio_node',
            name='ig_lio_node',
            output='screen',
            parameters=[
                ig_lio_config,
                {'use_sim_time': True},
            ],
        ),
    ])