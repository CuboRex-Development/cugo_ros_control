from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare

import os

def generate_launch_description():
    # package directory
    pkg_share  = FindPackageShare(package='package_name').find('cugo_ros2_control')
    launch_dir = os.path.join(pkg_share, 'launch')
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'sensors/tsukuba_sensors_bringup_launch.py')),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'cugo_ros2_control_launch.py')),
            launch_arguments={
                        'wheel_radius_l'      : '0.03880',
                        'wheel_radius_r'      : '0.03858',
                        'tread'               : '0.470',
                        'odom_topic_name'     : 'cugo_ros2_control/wheel/odometry',
                        'odom_frame_id'       : 'odom',
                        'odom_child_frame_id' : 'base_link',
                        'pose_covariance'     : '[0.001, 0.001, 1000000.0, 1000000.0, 1000000.0, 1.0]',
                        'twist_covariance'    : '[0.001, 0.001, 1000000.0, 1000000.0, 1000000.0, 0.001]'            }.items()
        )
    ])
