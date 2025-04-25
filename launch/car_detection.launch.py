from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
import os

def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable(
            name='DISPLAY',
            value=':0'
        ),
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
            executable='python3',
            name='car_detection',
            arguments=[os.path.join(os.getenv('COLCON_PREFIX_PATH'), 'car_detection_system', 'scripts', 'detection_node.py')],
            parameters=[{
                'display_mode': 'local',
                'debug_mode': True,
                'show_fps': True
            }]
        )
    ]) 