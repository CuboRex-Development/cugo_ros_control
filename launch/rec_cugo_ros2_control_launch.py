from launch import LaunchDescription
from launch_ros.actions import Node
import math

# ----------------------------------------------------------
# 記録するtopic
# /cmd_vel /fix /imu /tf_static /wheel/odometry
# 
# 必須ではないが記録したほうがいいTopic
# /magnetometer /navpvt /navstatus /orientation /tf /scan
# 
# ----------------------------------------------------------


BASE_LINK      = 'base_link'
BASE_FOOTPRINT = 'base_footprint'

def generate_launch_description():
    return LaunchDescription([
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
                {'odom_frame_id': 'utm'},
                {'odom_child_frame_id': 'base_footprint'},
                {'abnormal_translation_acc_limit': 10.0}, # [m/s^2], default 1.0[m] translation for dt(0.1[s])
                {'abnormal_angular_acc_limit': 10.0*math.pi/4} # [rad/s^2], default pi/6[rad] rotation for dt(0.1[s])
            ],
            remappings=[
                ('/odom','/wheel/odometry')
            ]  
        ),
    ])
