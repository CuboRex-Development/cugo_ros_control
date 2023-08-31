import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
	
def generate_launch_description():
    # Parameters
    imu_topic       = LaunchConfiguration('imu_topic'      , default='imu')
    imu_config_file = LaunchConfiguration('imu_config_file', default='config/sensors/wt901.yml')
    imu_config_fullpath    = LaunchConfiguration('imu_config_fullpath')

    return LaunchDescription([    
        # Parameters
        DeclareLaunchArgument('imu_topic'      , default_value=imu_topic      , description='Topic name of Imu.msg'),
        DeclareLaunchArgument('imu_config_file', default_value=imu_config_file, description='File name of witmotion config'),
        DeclareLaunchArgument(
            'imu_config_fullpath',
            default_value=[
                TextSubstitution(text = get_package_share_directory('cugo_ros2_control')),
                TextSubstitution(text = '/'),
                imu_config_file
            ]
        ),

        # static tf
        Node(
            package    = 'tf2_ros', 
            executable = 'static_transform_publisher',
            name       = 'baselink_to_imu',
            arguments  = ['0','0','0.09','0','0','0','base_link','imu']
        ),

        # imu
        Node(
            package    = 'witmotion_ros',
            executable = 'witmotion_ros_node',
            parameters = [
                imu_config_fullpath
            ],
            remappings = [
                ('imu',imu_topic)
            ]
        )
    ])
