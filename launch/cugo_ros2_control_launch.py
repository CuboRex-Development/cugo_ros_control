import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import math

def generate_launch_description():
    # Parameters
    ODOMETRY_DISPLAY               = LaunchConfiguration('ODOMETRY_DISPLAY'              ,default=True),
    PARAMETERS_DISPLAY             = LaunchConfiguration('PARAMETERS_DISPLAY'            ,default=False),
    TARGET_RPM_DISPLAY             = LaunchConfiguration('TARGET_RPM_DISPLAY'            ,default=True),
    SENT_PACKET_DISPLAY            = LaunchConfiguration('SENT_PACKET_DISPLAY'           ,default=False),
    RECV_PACKET_DISPLAY            = LaunchConfiguration('RECV_PACKET_DISPLAY'           ,default=True),
    READ_DATA_DISPLAY              = LaunchConfiguration('READ_DATA_DISPLAY'             ,default=True),
    timeout                        = LaunchConfiguration('timeout'                       ,default=0.05),
    wheel_radius_l                 = LaunchConfiguration('wheel_radius_l'                ,default=0.03858),
    wheel_radius_r                 = LaunchConfiguration('wheel_radius_r'                ,default=0.03858),
    tread                          = LaunchConfiguration('tread'                         ,default=0.460),
    reduction_ratio                = LaunchConfiguration('reduction_ratio'               ,default=1.0),
    encoder_max                    = LaunchConfiguration('encoder_max'                   ,default=2147483647),
    encoder_resolution             = LaunchConfiguration("encoder_resolution"            ,default=2048),
    arduino_addr                   = LaunchConfiguration('arduino_addr'                  ,default='192.168.11.216'),
    arduino_port                   = LaunchConfiguration('arduino_port'                  ,default=8888),
    source_port                    = LaunchConfiguration('source_port'                   ,default=8888),
    odom_frame_id                  = LaunchConfiguration('odom_frame_id'                 ,default='odom'),
    odom_child_frame_id            = LaunchConfiguration('odom_child_frame_id'           ,default='base_link'),
    abnormal_translation_acc_limit = LaunchConfiguration('abnormal_translation_acc_limit',default=10.0),
    abnormal_angular_acc_limit     = LaunchConfiguration('abnormal_angular_acc_limit'    ,default=10.0*math.pi/4),

    return LaunchDescription([
        # Parameter
        DeclareLaunchArgument('ODOMETRY_DISPLAY'              ,default_value=True),
        DeclareLaunchArgument('PARAMETERS_DISPLAY'            ,default_value=False),
        DeclareLaunchArgument('TARGET_RPM_DISPLAY'            ,default_value=True),
        DeclareLaunchArgument('SENT_PACKET_DISPLAY'           ,default_value=False),
        DeclareLaunchArgument('RECV_PACKET_DISPLAY'           ,default_value=True),
        DeclareLaunchArgument('READ_DATA_DISPLAY'             ,default_value=True),
        DeclareLaunchArgument('timeout'                       ,default_value=0.05),
        DeclareLaunchArgument('wheel_radius_l'                ,default_value=0.03858),
        DeclareLaunchArgument('wheel_radius_r'                ,default_value=0.03858),
        DeclareLaunchArgument('tread'                         ,default_value=0.460),
        DeclareLaunchArgument('reduction_ratio'               ,default_value=1.0),
        DeclareLaunchArgument('encoder_max'                   ,default_value=2147483647),
        DeclareLaunchArgument("encoder_resolution"            ,default_value=2048),
        DeclareLaunchArgument('arduino_addr'                  ,default_value='192.168.11.216'),
        DeclareLaunchArgument('arduino_port'                  ,default_value=8888),
        DeclareLaunchArgument('source_port'                   ,default_value=8888),
        DeclareLaunchArgument('odom_frame_id'                 ,default_value='odom'),
        DeclareLaunchArgument('odom_child_frame_id'           ,default_value='base_link'),
        DeclareLaunchArgument('abnormal_translation_acc_limit',default_value=10.0),
        DeclareLaunchArgument('abnormal_angular_acc_limit'    ,default_value=10.0*math.pi/4),
        
        #Cugo 
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
                {'odom_child_frame_id': 'base_link'},
                {'abnormal_translation_acc_limit': 10.0}, # [m/s^2], default 1.0[m] translation for dt(0.1[s])
                {'abnormal_angular_acc_limit': 10.0*math.pi/4} # [rad/s^2], default pi/6[rad] rotation for dt(0.1[s])
            ]
        ),
    ])
