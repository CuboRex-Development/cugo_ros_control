from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (EmitEvent , TimerAction)

import launch_ros.events
import launch_ros.events.lifecycle
import lifecycle_msgs.msg


import os

from ament_index_python.packages import get_package_share_directory
	
def generate_launch_description():
    return LaunchDescription([
        TimerAction(period=1.0,actions=[
            # static tf
            Node(
                package = 'tf2_ros', 
                executable = 'static_transform_publisher',
                name = 'baselink_to_imu',
                arguments = ['0','0','0.09','0','0','0','base_link','imu']
            ),
            
            # imu
            Node(
                package = 'rt_usb_9axisimu_driver',
                executable = 'rt_usb_9axisimu_driver',
                name = 'imu_rt',
                parameters=[
                    os.path.join(get_package_share_directory('cugo_ros2_control') , 'config/sensors/RT-USB-9axisIMU2.yml')
                ]
            ),
            
            # imu filter
            Node(
                package = 'imu_filter_madgwick',
                executable = 'imu_filter_madgwick_node',
                name = 'imu_filter',
                parameters=[
                    os.path.join(get_package_share_directory('cugo_ros2_control') , 'config/sensors/RT-USB-9axisIMU2.yml')
                ],
                remappings = [
                    ('imu/data','imu'),
                ]
            ),
            
            # Lifecycleの設定
            EmitEvent(
                event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch_ros.events.lifecycle.matches_node_name('/imu_rt'),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                )
            ),
            
            EmitEvent(
                event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch_ros.events.lifecycle.matches_node_name('/imu_rt'),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )
            )
        ])
    ])
