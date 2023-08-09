from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, TextSubstitution

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
	
def generate_launch_description():
    return LaunchDescription([        
        # imu
        Node(
            package = 'witmotion_ros',
            executable = 'witmotion_ros_node',
            parameters=[
                os.path.join(get_package_share_directory('cugo_ros2_control') , 'config/imu/wt901.yml')
            ],

            remappings = [
                ('scan','scan_raw'),
                ('scan_filtered','scan')
            ]
        ),
    ])
