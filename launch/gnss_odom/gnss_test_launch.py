import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument , IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node , SetRemap


def generate_launch_description():
    pkg_share         = FindPackageShare(package='cugo_ros2_control').find('cugo_ros2_control')
    
    launch_dir        = os.path.join(pkg_share       , 'launch')
    default_conf_dir  = os.path.join(pkg_share       , 'config/gnss_odom')
    default_ekf_conf  = os.path.join(default_conf_dir, 'navsat_transform.yaml')
    
    localization_conf = LaunchConfiguration('ekf_conf_file'      ,default=default_ekf_conf)
    use_sim_time      = LaunchConfiguration('use_sim_time'       ,default='false')
    wheel_odom_topic  = LaunchConfiguration('wheel_odom_topic'   ,default='wheel/odometry')
    global_odom_topic = LaunchConfiguration('global_odom_topic'  ,default='odometry/global')
    imu_topic         = LaunchConfiguration('imu_topic'          ,default='imu')
    gps_topic         = LaunchConfiguration('gps_topic'          ,default='fix')
    odom_child_frame  = LaunchConfiguration('odom_child_frame_id',default='base_footprint')
    
    
    return LaunchDescription([
        # Parameters Declare
        DeclareLaunchArgument('ekf_conf_file'      ,default_value = default_ekf_conf ,description = 'ekf parameter file'),
        DeclareLaunchArgument('use_sim_time'       ,default_value = use_sim_time     ,description = 'Use sim time'),
        DeclareLaunchArgument('global_odom_topic'  ,default_value = global_odom_topic,description = 'Topic name of odometry publishing EKF'),
        DeclareLaunchArgument('wheel_odom_topic'   ,default_value = wheel_odom_topic ,description = 'Topic name of wheel odometry published by robot'),
        DeclareLaunchArgument('imu_topic'          ,default_value = imu_topic        ,description = 'Topic name of imu'),
        DeclareLaunchArgument('gps_topic'          ,default_value = gps_topic        ,description = 'Topic name of NavSatFix published by gps '),        
        DeclareLaunchArgument('odom_child_frame_id',default_value = odom_child_frame ,description = 'Child Frame ID of wheel odometry published CuGo ROS control'),        
        
        # static tf
        # temp
        Node(
            package    = 'tf2_ros', 
            executable = 'static_transform_publisher',
            name       = 'base_footprint_to_base_link',
            arguments  = ['0','0','0.13','0','0','0','base_footprint','base_link']
        ),
        
        # Sensors
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'sensors/cugo_bringup_launch.py'))
        ),
        
        # gps odometry ekf 
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'gnss_odom/gnss_odom_launch.py')),
            launch_arguments={'ekf_conf_file'    :localization_conf ,
                              'use_sim_time'     :use_sim_time      ,
                              'global_odom_topic':global_odom_topic ,
                              'wheel_odom_topic' :wheel_odom_topic  ,
                              'imu_topic'        :imu_topic         ,
                              'gps_topic'        :gps_topic         }.items()
        ),
        
        # CuGo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'cugo_ros2_control_launch.py')),
            launch_arguments={'odom_child_frame_id':odom_child_frame}.items()
        ),
        
        # remap
        SetRemap(src='/odom',dst=wheel_odom_topic)
    ])
