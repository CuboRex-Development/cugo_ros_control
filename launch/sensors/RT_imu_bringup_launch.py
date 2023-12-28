import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent , TimerAction, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.event_handlers.on_process_start import OnProcessStart

from launch_ros.actions import Node, LifecycleNode
import launch_ros.events
import launch_ros.events.lifecycle

import lifecycle_msgs.msg

from ament_index_python.packages import get_package_share_directory
	
def generate_launch_description():
    # Parameters
    imu_topic          = LaunchConfiguration('imu_topic'         , default='imu')
    imu_config_file    = LaunchConfiguration('imu_config_file'   , default='config/sensors/RT-USB-9axisIMU2.yml')
    filter_config_file = LaunchConfiguration('filter_config_file', default='config/sensors/RT-USB-9axisIMU2.yml')
    imu_config_fullpath    = LaunchConfiguration('imu_config_fullpath')
    filter_config_fullpath = LaunchConfiguration('filter_config_fullpath')


    # imu
    imu_node = LifecycleNode(
        package    = 'rt_usb_9axisimu_driver',
        executable = 'rt_usb_9axisimu_driver',
        name       = 'imu_rt',
        parameters = [
            imu_config_fullpath
            # os.path.join(get_package_share_directory('cugo_ros2_control') , imu_config_file)
        ]
    )
    return LaunchDescription([
        TimerAction(period=3.0,actions=[
            # Parameters
            DeclareLaunchArgument('imu_topic'         , default_value=imu_topic         , description='Topic name of Imu.msg'),
            DeclareLaunchArgument('imu_config_file'   , default_value=imu_config_file   , description='File name of imu config'),
            DeclareLaunchArgument('filter_config_file', default_value=filter_config_file, description='File name of imu filter config'),
            DeclareLaunchArgument(
                'imu_config_fullpath',
                default_value=[
                    TextSubstitution(text = get_package_share_directory('cugo_ros2_control')),
                    TextSubstitution(text = '/'),
                    imu_config_file
                ]
            ),
            DeclareLaunchArgument(
                'filter_config_fullpath',
                default_value=[
                    TextSubstitution(text = get_package_share_directory('cugo_ros2_control')),
                    TextSubstitution(text = '/'),
                    filter_config_file
                ]
            ),
            # static tf
            Node(
                package    = 'tf2_ros', 
                executable = 'static_transform_publisher',
                name       = 'baselink_to_imu',
                arguments  = ['0','0','0.09','0','0','0','base_link','imu']
            ),
            
            imu_node,
            
            # imu filter
            Node(
                package    = 'imu_filter_madgwick',
                executable = 'imu_filter_madgwick_node',
                name       = 'imu_filter',
                parameters = [
                    filter_config_fullpath
                ],
                remappings = [
                    ('imu/data',imu_topic),
                ]
            ),
            
            # Lifecycleの設定
            RegisterEventHandler(
                OnProcessStart(
                    target_action=imu_node,
                    on_start=[
                        EmitEvent(
                            event=launch_ros.events.lifecycle.ChangeState(
                                lifecycle_node_matcher=launch_ros.events.lifecycle.matches_node_name('/imu_rt'),
                                transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                            )
                        )
                    ]
                )
            ),
            
            RegisterEventHandler(
                launch_ros.event_handlers.OnStateTransition(
                    target_lifecycle_node=imu_node,
                    start_state='configuring', goal_state='inactive',
                    entities=[
                        EmitEvent(
                            event=launch_ros.events.lifecycle.ChangeState(
                                lifecycle_node_matcher=launch_ros.events.lifecycle.matches_node_name('/imu_rt'),
                                transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                            )
                        )
                    ]
                )
            )            
        ])
    ])
