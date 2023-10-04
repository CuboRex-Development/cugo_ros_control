import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

	
def generate_launch_description():
    # Parameters
    channel_type      = LaunchConfiguration('channel_type'     , default='udp')
    udp_ip            = LaunchConfiguration('udp_ip'           , default='192.168.11.2')
    udp_port          = LaunchConfiguration('udp_port'         , default='8089') 
    frame_id          = LaunchConfiguration('frame_id'         , default='laser')
    inverted          = LaunchConfiguration('inverted'         , default='false')
    angle_compensate  = LaunchConfiguration('angle_compensate' , default='true')
    scan_mode         = LaunchConfiguration('scan_mode'        , default='Sensitivity')
    scan_topic        = LaunchConfiguration('scan_topic'       , default='scan')
    laser_filter_file = LaunchConfiguration('laser_filter_file', default='config/sensors/v3ros_filter.yaml')
    laser_filter_fullpath = LaunchConfiguration('laser_filter_fullpath')

    return LaunchDescription([
        # Parameters
        DeclareLaunchArgument('channel_type'     , default_value=channel_type     , description='Specifying channel type of lidar'),
        DeclareLaunchArgument('udp_ip'           , default_value=udp_ip           , description='Specifying udp ip to connected lidar'),
        DeclareLaunchArgument('udp_port'         , default_value=udp_port         , description='Specifying udp port to connected lidar'),
        DeclareLaunchArgument('frame_id'         , default_value=frame_id         , description='Specifying frame_id of lidar'),
        DeclareLaunchArgument('inverted'         , default_value=inverted         , description='Specifying whether or not to invert scan data'),
        DeclareLaunchArgument('angle_compensate' , default_value=angle_compensate , description='Specifying whether or not to enable angle_compensate of scan data'),
        DeclareLaunchArgument('scan_mode'        , default_value=scan_mode        , description='Specifying scan mode of lidar'),
        DeclareLaunchArgument('scan_topic'       , default_value=scan_topic       , description='Topic name of LaserScan.msg'),
        DeclareLaunchArgument('laser_filter_file', default_value=laser_filter_file, description='File name of laser filter'),
        DeclareLaunchArgument(
            'laser_filter_fullpath',
            default_value=[
                TextSubstitution(text = get_package_share_directory('cugo_ros2_control')),
                TextSubstitution(text = '/'),
                laser_filter_file
            ]
        ),

        
        # static TF
        Node(
            package    = 'tf2_ros', 
            executable = 'static_transform_publisher',
            name       = 'baselink_to_lidar',
            arguments  = ['0','0','0.16','3.14','0','0','base_link','laser']
        ),

        # LiDAR
        Node(
            package   ='rplidar_ros',
            executable='rplidar_node',
            name      ='rplidar_node',
            parameters=[{'channel_type'    : channel_type,
                         'udp_ip'          : udp_ip,
                         'udp_port'        : udp_port,
                         'frame_id'        : frame_id,
                         'inverted'        : inverted,
                         'angle_compensate': angle_compensate,
                         'scan_mode'       : scan_mode}],
            output='screen',
            remappings = [
                ('scan','scan_raw'),
            ]
        ),
        
        # laser filter
        Node(
            package   = 'laser_filters', 
            executable= 'scan_to_scan_filter_chain',
            parameters=[
                laser_filter_fullpath
                # os.path.join(get_package_share_directory('cugo_ros2_control') , laser_filter_file)
            ],
            remappings=[
                ('scan','scan_raw'),
                ('scan_filtered',scan_topic)
            ]
        )
    ])
