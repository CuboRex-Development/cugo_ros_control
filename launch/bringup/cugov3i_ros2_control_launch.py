from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import math

def generate_launch_description():
    # Parameters
    ODOMETRY_DISPLAY               = LaunchConfiguration('ODOMETRY_DISPLAY'              ,default=True)
    PARAMETERS_DISPLAY             = LaunchConfiguration('PARAMETERS_DISPLAY'            ,default=False)
    TARGET_RPM_DISPLAY             = LaunchConfiguration('TARGET_RPM_DISPLAY'            ,default=True)
    SENT_PACKET_DISPLAY            = LaunchConfiguration('SENT_PACKET_DISPLAY'           ,default=True)
    RECV_PACKET_DISPLAY            = LaunchConfiguration('RECV_PACKET_DISPLAY'           ,default=True)
    READ_DATA_DISPLAY              = LaunchConfiguration('READ_DATA_DISPLAY'             ,default=True)
    timeout                        = LaunchConfiguration('timeout'                       ,default=0.05)
    wheel_radius_l                 = LaunchConfiguration('wheel_radius_l'                ,default=0.03858)# default: CuGO V4
    wheel_radius_r                 = LaunchConfiguration('wheel_radius_r'                ,default=0.03858)# default: CuGO V4
    tread                          = LaunchConfiguration('tread'                         ,default=0.380)  # default: CuGO V4
    reduction_ratio                = LaunchConfiguration('reduction_ratio'               ,default=15.0)
    encoder_max                    = LaunchConfiguration('encoder_max'                   ,default=2147483647) # -2147483648 ~ 2147483647(Arduinoのlong intは32bit)
    encoder_resolution             = LaunchConfiguration('encoder_resolution'            ,default=24)
    arduino_addr                   = LaunchConfiguration('arduino_addr'                  ,default='192.168.11.216')
    arduino_port                   = LaunchConfiguration('arduino_port'                  ,default=8888)
    source_port                    = LaunchConfiguration('source_port'                   ,default=8888)
    odom_frame_id                  = LaunchConfiguration('odom_frame_id'                 ,default='odom')
    odom_child_frame_id            = LaunchConfiguration('odom_child_frame_id'           ,default='base_link')
    abnormal_translation_acc_limit = LaunchConfiguration('abnormal_translation_acc_limit',default=10.0)           # [m/s^2], default 1.0[m] translation for dt(0.1[s])
    abnormal_angular_acc_limit     = LaunchConfiguration('abnormal_angular_acc_limit'    ,default=10.0*math.pi/4) # [rad/s^2], default pi/6[rad] rotation for dt(0.1[s])
    # TODO: パラメータを変更してもトピック名が変わらないので修正
    odom_topic                     = LaunchConfiguration('odom_topic_name'               ,default='odom')

    pose_covariance                = LaunchConfiguration('pose_covariance'               ,default='[0.001, 0.001, 1000000.0, 1000000.0, 1000000.0, 1.0]')
    twist_covariance               = LaunchConfiguration('twist_covariance'              ,default='[0.001, 0.001, 1000000.0, 1000000.0, 1000000.0, 1.0]')

    # Select "UDP or USB"
    #comm_type                      = LaunchConfiguration('comm_type'                     ,default='UDP')
    comm_type                      = LaunchConfiguration('comm_type'                     ,default='USB')
    serial_port                    = LaunchConfiguration('serial_port'                   ,default='/dev/ttyACM0')
    serial_baudrate                = LaunchConfiguration('serial_baudrate'               ,default=115200)

    return LaunchDescription([
        # Parameter
        DeclareLaunchArgument('ODOMETRY_DISPLAY'              ,default_value=ODOMETRY_DISPLAY),
        DeclareLaunchArgument('PARAMETERS_DISPLAY'            ,default_value=PARAMETERS_DISPLAY),
        DeclareLaunchArgument('TARGET_RPM_DISPLAY'            ,default_value=TARGET_RPM_DISPLAY),
        DeclareLaunchArgument('SENT_PACKET_DISPLAY'           ,default_value=SENT_PACKET_DISPLAY),
        DeclareLaunchArgument('RECV_PACKET_DISPLAY'           ,default_value=RECV_PACKET_DISPLAY),
        DeclareLaunchArgument('READ_DATA_DISPLAY'             ,default_value=READ_DATA_DISPLAY),
        DeclareLaunchArgument('timeout'                       ,default_value=timeout),
        DeclareLaunchArgument('wheel_radius_l'                ,default_value=wheel_radius_l),
        DeclareLaunchArgument('wheel_radius_r'                ,default_value=wheel_radius_r),
        DeclareLaunchArgument('tread'                         ,default_value=tread),
        DeclareLaunchArgument('reduction_ratio'               ,default_value=reduction_ratio),
        DeclareLaunchArgument('encoder_max'                   ,default_value=encoder_max),
        DeclareLaunchArgument("encoder_resolution"            ,default_value=encoder_resolution),
        DeclareLaunchArgument('arduino_addr'                  ,default_value=arduino_addr),
        DeclareLaunchArgument('arduino_port'                  ,default_value=arduino_port),
        DeclareLaunchArgument('source_port'                   ,default_value=source_port),
        DeclareLaunchArgument('odom_frame_id'                 ,default_value=odom_frame_id),
        DeclareLaunchArgument('odom_child_frame_id'           ,default_value=odom_child_frame_id),
        DeclareLaunchArgument('abnormal_translation_acc_limit',default_value=abnormal_translation_acc_limit),
        DeclareLaunchArgument('abnormal_angular_acc_limit'    ,default_value=abnormal_angular_acc_limit),
        DeclareLaunchArgument('odom_topic_name'               ,default_value=odom_topic),
        DeclareLaunchArgument('pose_covariance'               ,default_value=pose_covariance),
        DeclareLaunchArgument('twist_covariance'              ,default_value=twist_covariance),
        DeclareLaunchArgument('comm_type'                     ,default_value=comm_type),
        DeclareLaunchArgument('serial_port'                   ,default_value=serial_port),
        DeclareLaunchArgument('serial_baudrate'               ,default_value=serial_baudrate),
        
        
        #Cugo 
        Node(
            package='cugo_ros2_control',
            namespace='cugo_ros2_control',
            executable='cugo_ros2_control',
            output='screen',
            parameters=[
                {'ODOMETRY_DISPLAY'              : ODOMETRY_DISPLAY},
                {'PARAMETERS_DISPLAY'            : PARAMETERS_DISPLAY},
                {'TARGET_RPM_DISPLAY'            : TARGET_RPM_DISPLAY},
                {'SENT_PACKET_DISPLAY'           : SENT_PACKET_DISPLAY},
                {'RECV_PACKET_DISPLAY'           : RECV_PACKET_DISPLAY},
                {'READ_DATA_DISPLAY'             : READ_DATA_DISPLAY},
                {'timeout'                       : timeout},
                {'wheel_radius_l'                : wheel_radius_l}, 
                {'wheel_radius_r'                : wheel_radius_r}, 
                {'tread'                         : tread},
                {'reduction_ratio'               : reduction_ratio},
                {'encoder_max'                   : encoder_max},
                {"encoder_resolution"            : encoder_resolution},
                {'arduino_addr'                  : arduino_addr},
                {'arduino_port'                  : arduino_port},
                {'source_port'                   : source_port},
                {'odom_frame_id'                 : odom_frame_id},
                {'odom_child_frame_id'           : odom_child_frame_id},
                {'abnormal_translation_acc_limit': abnormal_translation_acc_limit},
                {'abnormal_angular_acc_limit'    : abnormal_angular_acc_limit},
                {'pose_covariance'               : pose_covariance},
                {'twist_covariance'              : twist_covariance},
                {'comm_type'                     : comm_type},
                {'serial_port'                   : serial_port},
                {'serial_baudrate'               : serial_baudrate}
            ],
            remappings = [
                ('odom',odom_topic),
            ]
        ),
    ])
