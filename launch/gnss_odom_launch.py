import os
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import math

BASE_LINK      = 'base_link'
BASE_FOOTPRINT = 'base_footprint'


def generate_launch_description():
    # args for lidar
    channel_type     = LaunchConfiguration('channel_type', default='udp')
    udp_ip           = LaunchConfiguration('udp_ip', default='192.168.11.2')
    udp_port         = LaunchConfiguration('udp_port', default='8089') 
    frame_id         = LaunchConfiguration('frame_id', default='laser')
    inverted         = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode        = LaunchConfiguration('scan_mode', default='Sensitivity')
    scan_frequency   = LaunchConfiguration('scan_frequency', default='10')

    # args for laserfilter
    laser_filter_file = DeclareLaunchArgument(
        'laser_filter_file', default_value='laser_filters/v3ros_filter.yaml'
    )
    
    # Parameter for Localization
    pkg_share = FindPackageShare(package='cugo_ros2_control').find('cugo_ros2_control')
    default_conf_dir = os.path.join(pkg_share, 'config/gnss_odom')
    robot_localization_file_path = os.path.join(default_conf_dir, 'navsat_transform.yaml')
    
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
            package = 'tf2_ros', 
            executable = 'static_transform_publisher',
            name = 'lidar_pos',
            arguments = ['0','0','0.16','3.14','0','0',BASE_LINK,'laser']
        ),
        
        Node(
            package = "tf2_ros",
            executable = "static_transform_publisher",
            name = "swri84_transform",
            arguments=["0", "0", "0", "0", "0", "0", "map", "wgs84"]
        ),

        # LiDAR arg
        DeclareLaunchArgument(
            'channel_type',
            default_value=channel_type,
            description='Specifying channel type of lidar'
        ),
        DeclareLaunchArgument(
            'udp_ip',
            default_value=udp_ip,
            description='Specifying udp ip to connected lidar'
        ),
        DeclareLaunchArgument(
            'udp_port',
            default_value=udp_port,
            description='Specifying udp port to connected lidar'
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='Specifying frame_id of lidar'
        ),
        DeclareLaunchArgument(
            'inverted',
            default_value=inverted,
            description='Specifying whether or not to invert scan data'
        ),
        DeclareLaunchArgument(
            'angle_compensate',
            default_value=angle_compensate,
            description='Specifying whether or not to enable angle_compensate of scan data'
        ),
        DeclareLaunchArgument(
            'scan_mode',
            default_value=scan_mode,
            description='Specifying scan mode of lidar'
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
        
        # ublox node
        Node(
            package = 'ublox_gps', 
            executable = 'ublox_gps_node',
            output='both',
            parameters=[
                os.path.join(get_package_share_directory('cugo_ros2_control') , 'config/gnss/D9CX1.yaml')
            ]
        ),

        # imu
        Node(
            package = 'witmotion_ros',
            executable = 'witmotion_ros_node',
            parameters=[
                os.path.join(get_package_share_directory('cugo_ros2_control') , 'config/imu/wt901.yml')
            ],

            remappings = [
                ('scan','scan_raw'),
                ('scan_filtered','scan')
            ]
        ),

        # LiDAR
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{'channel_type': channel_type,
                         'udp_ip': udp_ip,
                         'udp_port': udp_port,
                         'frame_id': frame_id,
                         'inverted': inverted,
                         'angle_compensate': angle_compensate,
                         'scan_mode': scan_mode}],
            output='screen',
            remappings = [
                ('scan','scan_raw'),
            ]
        ),
        
        # laser filter
        Node(
            package = 'laser_filters', 
            executable = 'scan_to_scan_filter_chain',
            parameters=[
                os.path.join(get_package_share_directory('cugo_ros2_control') , 'config/laser_filters/v3ros_filter.yaml')
            ],
            remappings = [
                ('scan','scan_raw'),
                ('scan_filtered','scan')
            ]
        ),
        
        # EKF : TF/Map -> odom
        Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_map',
                output='screen',
                parameters=[
                    robot_localization_file_path, 
                    {'use_sim_time': use_sim_time}
                ],
                remappings=[
                    ('odometry/filtered', 'odometry/global'),
                ]
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
        
        # GNSS Manager
        Node(
            package='GNSS_manager',
            executable='GNSS_manager',
            name='gnss_manager',
        ),

        # rviz
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     output='screen',
        # ),
    ]
    
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)

    for node_action in Nodes:
        ld.add_action(node_action)
    
    return ld
