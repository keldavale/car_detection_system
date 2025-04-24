from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
            }]
        ),
        Node(
            package='car_detection_system',
            executable='detection_node',
            name='car_detection',
            parameters=[{
                'display_mode': 'local',
                'debug_mode': True,
                'show_fps': True
            }]
        )
    ]) 