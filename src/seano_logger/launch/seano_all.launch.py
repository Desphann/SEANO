from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='seano_sensors',
            executable='gps_reader',
            name='gps_reader',
            output='screen'
        ),
        Node(
            package='seano_sensors',
            executable='imu_reader',
            name='imu_reader',
            output='screen'
        ),
        Node(
            package='seano_sensors',
            executable='ctd_reader',
            name='ctd_reader',
            output='screen'
        ),
        Node(
            package='seano_sensors',
            executable='adcp_reader',
            name='adcp_reader',
            output='screen'
        ),
        Node(
            package='seano_sensors',
            executable='battery_reader',
            name='battery_reader',
            output='screen'
        ),
        Node(
            package='seano_sensors',
            executable='sbes_reader',
            name='sbes_reader',
            output='screen'
        ),
        Node(
            package='seano_logger',
            executable='logger_node',
            name='logger_node',
            output='screen'
        ),
    ])