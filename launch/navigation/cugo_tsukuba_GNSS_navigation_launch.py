from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    # package directory
    pkg_share  = FindPackageShare(package='package_name').find('cugo_ros2_control')
    launch_dir = os.path.join(pkg_share, 'launch')
    
    rviz_config_dir = os.path.join(get_package_share_directory('turtlebot3_cartographer'),
                                   'rviz', 'tb3_cartographer.rviz')
    
    
    return LaunchDescription([
        GroupAction(
            actions = [
                # remap
                SetRemap(src='/wheel/odometry',dst='wheel/odometry'),
                SetRemap(src='/odom',dst='wheel/odometry'),

                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', rviz_config_dir],
                    parameters=[{'use_sim_time': 'false'}],
                    output='screen'
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
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation/tsukuba_navigation_launch.py')),
                ),
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
