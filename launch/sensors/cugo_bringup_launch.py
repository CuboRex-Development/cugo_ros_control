from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.substitutions import FindPackageShare

import os

	
def generate_launch_description():
    # package directory
    pkg_share  = FindPackageShare(package='package_name').find('cugo_ros2_control')
    launch_dir = os.path.join(pkg_share, 'launch')
    
    # Parameters
    # LiDAR
    channel_type      = LaunchConfiguration('channel_type'     , default='udp')
    udp_ip            = LaunchConfiguration('udp_ip'           , default='192.168.11.2')
    udp_port          = LaunchConfiguration('udp_port'         , default='8089') 
    frame_id          = LaunchConfiguration('frame_id'         , default='laser')
    inverted          = LaunchConfiguration('inverted'         , default='false')
    angle_compensate  = LaunchConfiguration('angle_compensate' , default='true')
    scan_mode         = LaunchConfiguration('scan_mode'        , default='Sensitivity')
    scan_frequency    = LaunchConfiguration('scan_frequency'   , default='10')
    scan_topic        = LaunchConfiguration('scan_topic'       , default='scan')
    laser_filter_file = LaunchConfiguration('laser_filter_file', default='config/sensors/v3ros_filter.yaml')
    
    # imu
    imu_topic       = LaunchConfiguration('imu_topic'      , default='imu')
    imu_config_file = LaunchConfiguration('imu_config_file', default='config/sensors/wt901.yml')
    
    # GNSS
    fix_topic         = LaunchConfiguration('fix_topic'        , default='fix')
    ublox_config_file = LaunchConfiguration('ublox_config_file', default='config/sensors/D9CX1.yaml')

    return LaunchDescription([
        # Parameter
        # LiDAR
        DeclareLaunchArgument('channel_type'     , default_value=channel_type     , description='Specifying channel type of lidar'),
        DeclareLaunchArgument('udp_ip'           , default_value=udp_ip           , description='Specifying udp ip to connected lidar'),
        DeclareLaunchArgument('udp_port'         , default_value=udp_port         , description='Specifying udp port to connected lidar'),
        DeclareLaunchArgument('frame_id'         , default_value=frame_id         , description='Specifying frame_id of lidar'),
        DeclareLaunchArgument('inverted'         , default_value=inverted         , description='Specifying whether or not to invert scan data'),
        DeclareLaunchArgument('angle_compensate' , default_value=angle_compensate , description='Specifying whether or not to enable angle_compensate of scan data'),
        DeclareLaunchArgument('scan_mode'        , default_value=scan_mode        , description='Specifying scan mode of lidar'),
        DeclareLaunchArgument('scan_frequency'   , default_value=scan_frequency   , description='Specifying scan frequency of lidar'),
        DeclareLaunchArgument('scan_topic'       , default_value=scan_topic       , description='Topic name of LaserScan.msg'),
        DeclareLaunchArgument('laser_filter_file', default_value=laser_filter_file, description='File name of laser filter'),
        # imu
        DeclareLaunchArgument('imu_topic'      , default_value=imu_topic      , description='Topic name of Imu.msg'),
        DeclareLaunchArgument('imu_config_file', default_value=imu_config_file, description='File name of witmotion config'),
        # GNSS
        DeclareLaunchArgument('fix_topic'        , default_value=fix_topic        , description='Topic name of NavSatFix.msg'),
        DeclareLaunchArgument('ublox_config_file', default_value=ublox_config_file, description='File name of Ublox config'),

        # launch files
        # LiDAR
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'lidar_bringup_launch.py')),
            launch_arguments={
                'channel_type'     : channel_type,
                'udp_ip'           : udp_ip,
                'udp_port'         : udp_port,
                'frame_id'         : frame_id,
                'inverted'         : inverted,
                'angle_compensate' : angle_compensate,
                'scan_mode'        : scan_mode,
                'scan_frequency'   : scan_frequency,
                'scan_topic'       : scan_topic,
                'laser_filter_file': laser_filter_file
            }.items()
        ),
        
        # imu
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'imu_bringup_launch.py')),
            launch_arguments={
                'imu_topic'       : imu_topic,
                'imu_config_file' : imu_config_file
            }.items()
        ),
        
        # GNSS
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'imu_bringup_launch.py')),
            launch_arguments={
                'fix_topic'        : fix_topic,
                'ublox_config_file': ublox_config_file
            }.items()
        )
    ])
