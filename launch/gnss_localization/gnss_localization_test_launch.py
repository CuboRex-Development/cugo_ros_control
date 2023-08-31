import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument , IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node , SetRemap


def generate_launch_description():
    pkg_share         = FindPackageShare(package='cugo_ros2_control').find('cugo_ros2_control')
    
    launch_dir        = os.path.join(pkg_share       , 'launch')
    default_conf_dir  = os.path.join(pkg_share       , 'config/gnss_localization')
    default_ekf_conf  = os.path.join(default_conf_dir, 'gnss_localization.yaml')
    
    return LaunchDescription([
        GroupAction(
            actions = [
                # remap
                SetRemap(src='/wheel/odometry',dst='wheel/odometry'),
                SetRemap(src='/odom',dst='wheel/odometry'),

                # static tf
                # temp
                # Node(
                #     package    = 'tf2_ros', 
                #     executable = 'static_transform_publisher',
                #     name       = 'base_footprint_to_base_link',
                #     arguments  = ['0','0','0.13','0','0','0','base_footprint','base_link']
                # ),
                
                # Sensors
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(launch_dir, 'sensors/tsukuba_sensors_bringup_launch.py')),
                    launch_arguments={
                        'imu_topic'         :'imu' ,
                        'scan_topic'        :'scan',
                        'fix_topic'         :'fix' ,
                        
                        'imu_config_file'   :'config/sensors/RT-USB-9axisIMU2.yml',
                        'filter_config_file':'config/sensors/RT-USB-9axisIMU2.yml',
                        'laser_filter_file' :'config/sensors/v3ros_filter.yaml',
                        'ublox_config_file' :'config/sensors/D9CX1.yaml'
                    }.items()
                ),
                
                # gps odometry ekf 
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(launch_dir, 'gnss_localization/gnss_localization_launch.py')),
                    launch_arguments={
                        'use_sim_time'     :'false'                  ,
                        'imu_topic'        :'imu'                    ,
                        'gps_topic'        :'fix'                    ,
                        # 'global_odom_topic':'odometry/global'        ,
                        'global_odom_topic':'odom'                   ,
                        'wheel_odom_topic' :'cugo_ros2_control/wheel/odometry'         ,
                        'ekf_conf_file'    :'config/gnss_localization/gnss_localization.yaml'
                    }.items()
                ),
                
                # CuGo
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(launch_dir, 'cugo_ros2_control_launch.py')),
                    launch_arguments={
                        'wheel_radius_l'      : '0.03880',
                        'wheel_radius_r'      : '0.03858',
                        'tread'               : '0.474',
                        'odom_topic_name'     : 'cugo_ros2_control/wheel/odometry',
                        'odom_frame_id'       : 'odom',
                        'odom_child_frame_id' : 'base_link'
                    }.items()
                )
        ])
    ])
