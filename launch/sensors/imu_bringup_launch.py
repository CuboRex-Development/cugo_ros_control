from launch import LaunchDescription
from launch_ros.actions import Node

import os

from ament_index_python.packages import get_package_share_directory
	
def generate_launch_description():
    return LaunchDescription([    
        # static tf
        Node(
            package = 'tf2_ros', 
            executable = 'static_transform_publisher',
            name = 'baselink_to_imu',
            arguments = ['0','0','0.09','0','0','0','base_link','imu']
        ),

        # imu
        Node(
            package = 'witmotion_ros',
            executable = 'witmotion_ros_node',
            parameters=[
                os.path.join(get_package_share_directory('cugo_ros2_control') , 'config/sensors/wt901.yml')
            ]
        )
    ])
