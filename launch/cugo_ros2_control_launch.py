from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cugo_ros2_control',
            namespace='cugo_ros2_control',
            executable='cugo_ros2_control',
            output='screen',
            parameters=[
                {'timeout': 0.05},
                {'wheel_radius_l': 0.03858}, # default: CuGO V3
                {'wheel_radius_r': 0.03858}, # default: CuGO V3
                {'tread': 0.460}, # default: CuGO V3
                {'reduction_ratio': 1.0}, # default CuGO V3
                {'encoder_max': 2147483647}, # -2147483648 ~ 2147483647(Arduinoのlong intは32bit)
                {"encoder_resolution": 2048},
                {'arduino_addr': '192.168.11.216'},
                {'arduino_port': 8888},
                {'odom_frame_id': 'odom'},
                {'odom_child_frame_id': 'base_link'}
            ]
        ),
    ])
