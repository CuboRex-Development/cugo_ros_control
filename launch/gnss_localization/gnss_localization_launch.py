import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share         = FindPackageShare(package='cugo_ros2_control').find('cugo_ros2_control')
    default_conf_dir  = os.path.join(pkg_share       , 'config/gnss_localization')
    default_ekf_conf  = os.path.join(default_conf_dir, 'gnss_localization.yaml')
    
    
    localization_conf = LaunchConfiguration('ekf_conf_file'    ,default=default_ekf_conf)
    use_sim_time      = LaunchConfiguration('use_sim_time'     ,default='false')
    wheel_odom_topic  = LaunchConfiguration('wheel_odom_topic' ,default='wheel/odometry')
    global_odom_topic = LaunchConfiguration('global_odom_topic',default='odometry/global')
    imu_topic         = LaunchConfiguration('imu_topic'        ,default='imu')
    gps_topic         = LaunchConfiguration('gps_topic'        ,default='fix')
    
    
    return LaunchDescription([
        # Parameters
        DeclareLaunchArgument('ekf_conf_file'    ,default_value = default_ekf_conf ,description = 'ekf parameter file'),
        DeclareLaunchArgument('use_sim_time'     ,default_value = use_sim_time     ,description = 'Use sim time'),
        DeclareLaunchArgument('wheel_odom_topic' ,default_value = wheel_odom_topic ,description = 'Topic name of wheel odometry published by robot'),
        DeclareLaunchArgument('global_odom_topic',default_value = global_odom_topic,description = 'Topic name of odometry publishing EKF'),
        DeclareLaunchArgument('imu_topic'        ,default_value = imu_topic        ,description = 'Topic name of imu'),
        DeclareLaunchArgument('gps_topic'        ,default_value = gps_topic        ,description = 'Topic name of NavSatFix published by gps '),
        
        # EKF : TF/Map -> odom
        Node(
            package    ='robot_localization',
            executable ='ekf_node',
            name       ='ekf_map',
            output     ='screen',
            parameters =[
                localization_conf, 
                {'use_sim_time': use_sim_time}
            ],
            remappings =[
                ('odometry/filtered', global_odom_topic),
            ]
        ),

        # navsatfix to odometory
        Node(
            package    ='robot_localization',
            executable ='navsat_transform_node',
            name       ='navsat_transform',
            output     ='screen',
            parameters =[
                localization_conf, 
                {'use_sim_time': use_sim_time}
            ],
            remappings =[
                ('imu'              , imu_topic),
                ('gps/fix'          , gps_topic), 
                ('odometry/filtered', wheel_odom_topic)
            ]
        ),
        
        # GNSS Manager
        Node(
            package    ='GNSS_manager',
            executable ='GNSS_manager',
            name       ='gnss_manager'
        )
    ])
