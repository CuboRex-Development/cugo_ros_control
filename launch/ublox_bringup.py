from launch import LaunchDescription
from launch_ros.actions import Node

import os

from ament_index_python.packages import get_package_share_directory
	
def generate_launch_description():
    return LaunchDescription([
        # ublox node
        Node(
            package = 'ublox_gps', 
            executable = 'ublox_gps_node',
            output='both',
            parameters=[
                os.path.join(get_package_share_directory('cugo_ros2_control') , 'config/gnss/D9CX1.yaml')
            ]
        ),
    ])
