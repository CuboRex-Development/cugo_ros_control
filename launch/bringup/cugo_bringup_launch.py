from launch import LaunchDescription
from launch.actions import  IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare

import os

	
def generate_launch_description():
    # package directory
    pkg_share  = FindPackageShare(package='package_name').find('cugo_ros2_control')
    launch_dir = os.path.join(pkg_share, 'launch')
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'sensors/sensors_bringup_launch.py')),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'cugov3_ros2_control_launch.py')),
        )
    ])
