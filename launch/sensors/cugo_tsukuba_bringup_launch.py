from launch import LaunchDescription
from launch_ros.actions import (Node, LifecycleNode)
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from launch.events import matches_action
from launch.actions import (DeclareLaunchArgument , RegisterEventHandler,LogInfo,EmitEvent)
from launch.substitutions import LaunchConfiguration

import lifecycle_msgs
import os

from ament_index_python.packages import get_package_share_directory
	
def generate_launch_description():
    # args for lidar
    channel_type     = LaunchConfiguration('channel_type', default='udp')
    udp_ip           = LaunchConfiguration('udp_ip', default='192.168.11.2')
    udp_port         = LaunchConfiguration('udp_port', default='8089') 
    frame_id         = LaunchConfiguration('frame_id', default='laser')
    inverted         = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode        = LaunchConfiguration('scan_mode', default='Sensitivity')
    scan_frequency   = LaunchConfiguration('scan_frequency', default='10')
    
    # imu
    imu_node = LifecycleNode(
        package = 'rt_usb_9axisimu_driver',
        executable = 'rt_usb_9axisimu_driver',
        name = 'imu_rt',
        parameters=[
            os.path.join(get_package_share_directory('cugo_ros2_control') , 'config/sensors/RT-USB-9axisIMU2.yml')
        ]
    )

    
    return LaunchDescription([
        # LiDAR arg
        DeclareLaunchArgument(
            'channel_type',
            default_value=channel_type,
            description='Specifying channel type of lidar'
        ),
        DeclareLaunchArgument(
            'udp_ip',
            default_value=udp_ip,
            description='Specifying udp ip to connected lidar'
        ),
        DeclareLaunchArgument(
            'udp_port',
            default_value=udp_port,
            description='Specifying udp port to connected lidar'
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='Specifying frame_id of lidar'
        ),
        DeclareLaunchArgument(
            'inverted',
            default_value=inverted,
            description='Specifying whether or not to invert scan data'
        ),
        DeclareLaunchArgument(
            'angle_compensate',
            default_value=angle_compensate,
            description='Specifying whether or not to enable angle_compensate of scan data'
        ),
        DeclareLaunchArgument(
            'scan_mode',
            default_value=scan_mode,
            description='Specifying scan mode of lidar'
        ),
        
        # static TF
        Node(
            package = 'tf2_ros', 
            executable = 'static_transform_publisher',
            name = 'baselink_to_lidar',
            arguments = ['0','0','0.16','3.14','0','0','base_link','laser']
        ),
        Node(
            package = 'tf2_ros', 
            executable = 'static_transform_publisher',
            name = 'baselink_to_imu',
            arguments = ['0','0','0.09','0','0','0','base_link','imu']
        ),
        Node(
            package = 'tf2_ros', 
            executable = 'static_transform_publisher',
            name = 'baselink_to_gps',
            arguments = ['0.29','0.20','0.80','0','0','0','base_link','gps']
        ),
        
        # ublox node
        Node(
            package = 'ublox_gps', 
            executable = 'ublox_gps_node',
            output='both',
            parameters=[
                os.path.join(get_package_share_directory('cugo_ros2_control') , 'config/sensors/D9CX1.yaml')
            ]
        ),
        
        # imu
        imu_node,
        
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
        
        # LiDAR
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{'channel_type': channel_type,
                         'udp_ip': udp_ip,
                         'udp_port': udp_port,
                         'frame_id': frame_id,
                         'inverted': inverted,
                         'angle_compensate': angle_compensate,
                         'scan_mode': scan_mode}],
            output='screen',
            remappings = [
                ('scan','scan_raw'),
            ]
        ),
        
        # laser filter
        Node(
            package = 'laser_filters', 
            executable = 'scan_to_scan_filter_chain',
            parameters=[
                os.path.join(get_package_share_directory('cugo_ros2_control') , 'config/sensors/v3ros_filter.yaml')
            ],
            remappings = [
                ('scan','scan_raw'),
                ('scan_filtered','scan')
            ]
        ),
        
        # imu node activation
        # unconfigured -> inactive
        EmitEvent(
            event=ChangeState(
                lifecycle_node_matcher=matches_action(imu_node),
                transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
            )
        ),
        
        # inactive -> active
        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=imu_node, 
                start_state = 'configuring',
                goal_state  = 'inactive',
                entities=[
                    LogInfo(msg="IMU Node activated"),
                    EmitEvent(
                        event=ChangeState(
                        lifecycle_node_matcher=matches_action(imu_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                        )
                    ),
                ],
            )
        )
    ])
