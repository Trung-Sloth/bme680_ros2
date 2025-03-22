from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bme680_sensor',
            executable='bme680_node',
            name='bme680_sensor_node',
            output='screen'
        ),
        Node(
            package='bme680_sensor',
            executable='google_sheets_logger',
            name='google_sheets_logger_node',
            output='screen'
        ),
    ])
