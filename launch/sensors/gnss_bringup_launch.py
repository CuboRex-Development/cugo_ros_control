import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
	
def generate_launch_description():
    # Parameters
    fix_topic         = LaunchConfiguration('fix_topic'        , default='fix')
    ublox_config_file = LaunchConfiguration('ublox_config_file', default='config/sensors/D9CX1.yaml')
    ublox_config_fullpath = LaunchConfiguration('ublox_config_fullpath')
    
    return LaunchDescription([
        # Parameters
        DeclareLaunchArgument('fix_topic'        , default_value=fix_topic        , description='Topic name of NavSatFix.msg'),
        DeclareLaunchArgument('ublox_config_file', default_value=ublox_config_file, description='File name of Ublox config'),
        DeclareLaunchArgument(
            'ublox_config_fullpath',
            default_value=[
                TextSubstitution(text = get_package_share_directory('cugo_ros2_control')),
                TextSubstitution(text = '/'),
                ublox_config_file
            ]
        ),

        # static tf
        Node(
            package    = 'tf2_ros', 
            executable = 'static_transform_publisher',
            name       = 'baselink_to_gps',
            arguments  = ['0.29','0.20','0.80','0','0','0','base_link','gps']
        ),

        # ublox node
        Node(
            package    = 'ublox_gps', 
            executable = 'ublox_gps_node',
            output     = 'both',
            parameters = [
                ublox_config_fullpath
                # os.path.join(get_package_share_directory('cugo_ros2_control') ,ublox_config_file)
            ],
            remappings = [
                ('fix',fix_topic)
            ]
        )
    ])
