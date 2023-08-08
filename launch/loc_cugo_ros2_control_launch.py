import os
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch_ros.actions import Node
import math

BASE_LINK      = 'base_link'
BASE_FOOTPRINT = 'base_footprint'


def generate_launch_description():

    
    # Parameter for Navigation
    # ファイルパス
    pkg_share = FindPackageShare(package='cugo_ros2_control').find('cugo_ros2_control')
    default_conf_dir = os.path.join(pkg_share, 'config/gnss_nav')
    robot_localization_file_path = os.path.join(default_conf_dir, 'ekf_with_gps.yaml')
    
    use_sim_time = LaunchConfiguration('use_sim_time')


    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='False',
        description='Use simulation clock if true'
    )


    
    Nodes = [
        # static tf
        # temp
        Node(
            package = 'tf2_ros', 
            executable = 'static_transform_publisher',
            name = 'base_footprint_to_base_link',
            arguments = ['0','0','0.13','0','0','0',BASE_FOOTPRINT, BASE_LINK]
        ),
        
        Node(
            package = 'tf2_ros', 
            executable = 'static_transform_publisher',
            name = 'gps_pos',
            arguments = ['0.29','0.20','0.80','0','0','0',BASE_LINK,'gps']
        ),

        Node(
            package = 'tf2_ros', 
            executable = 'static_transform_publisher',
            name = 'imu_pos',
            arguments = ['0','0','0.09','0','0','0',BASE_LINK,'imu']
        ),
        
        Node(
            package = "tf2_ros",
            executable = "static_transform_publisher",
            name = "swri84_transform",
            arguments=["0", "0", "0", "0", "0", "0", "map", "wgs84"]
        ),

        # cugo 
        Node(
            package='cugo_ros2_control',
            namespace='cugo_ros2_control',
            executable='cugo_ros2_control',
            output='screen',
            parameters=[
                {'ODOMETRY_DISPLAY': True},
                {'PARAMETERS_DISPLAY': False},
                {'TARGET_RPM_DISPLAY': True},
                {'SENT_PACKET_DISPLAY': False},
                {'RECV_PACKET_DISPLAY': True},
                {'READ_DATA_DISPLAY': True},

                {'timeout': 0.05},
                {'wheel_radius_l': 0.03858}, # default: CuGO V3
                {'wheel_radius_r': 0.03858}, # default: CuGO V3
                {'tread': 0.460}, # default: CuGO V3
                {'reduction_ratio': 1.0}, # default CuGO V3
                {'encoder_max': 2147483647}, # -2147483648 ~ 2147483647(Arduinoのlong intは32bit)
                {"encoder_resolution": 2048},
                {'arduino_addr': '192.168.11.216'},
                {'arduino_port': 8888},
                {'source_port': 8888},
                {'odom_frame_id': 'odom'},
                {'odom_child_frame_id': BASE_FOOTPRINT},
                {'abnormal_translation_acc_limit': 10.0}, # [m/s^2], default 1.0[m] translation for dt(0.1[s])
                {'abnormal_angular_acc_limit': 10.0*math.pi/4} # [rad/s^2], default pi/6[rad] rotation for dt(0.1[s])
            ],
            remappings=[
                ('/odom','/wheel/odometry')
            ]   
        ),
        
        # rviz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        ),
        

        
        # navsatfix to odometory
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[
                robot_localization_file_path, 
                {'use_sim_time': use_sim_time}
            ],
            remappings=[
                ('imu', 'imu'),
                ('gps/fix', 'fix'), 
                ('odometry/filtered', 'wheel/odometry')
            ]
        ),
        
        # TF : map to odom
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_map',
            output='screen',
            parameters=[
                robot_localization_file_path,   
                {'use_sim_time': use_sim_time}
            ],      
            remappings=[
                ('odometry/filtered', 'odometry/global'),
                ('/set_pose', '/initialpose')
            ]
        ),
        
    ]
    
    
    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)

    for node_action in Nodes:
        ld.add_action(node_action)
    
    return ld
    
    

