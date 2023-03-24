#ifndef CUGO_CONTROLLER_H
#define CUGO_CONTROLLER_H

// ros
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

// cpp
#include <iostream>
#include <string>
#include <math.h>
#include <time.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <arpa/inet.h>

class CugoController {
  private:
    struct UdpHeader
    {
      uint16_t sourcePort;
      uint16_t destinationPort;
      uint16_t length;
      uint16_t checksum;
    };

    const int UDP_BUFF_SIZE = 64;
    const int UDP_HEADER_SIZE = 8;

    // parameters
    std::string device_name = "/dev/Arduino";
    int baudrate       = 115200;
    float timeout        = 0.05;  // 10hzで回す場合
    float wheel_radius_l = 0.03858;   //初期値はCuGO V3の値
    float wheel_radius_r = 0.03858;   //初期値はCuGO V3の値
    float reduction_ratio = 1.0;      //初期値はCuGo V3の値
    float tread           = 0.635;     //CuGO MEGAの値
    int encoder_resolution = 2048;
    int encoder_max    = 2147483647; // -2147483648 ~ 2147483647(Arduinoのlong intは32bit)
    int stop_motor_time = 500; //NavigationやコントローラからSubscriberできなかったときにモータを>止めるまでの時間(ms)
    //std::string arduino_addr = "192.168.8.216";
    std::string arduino_addr = "127.0.0.1";
    int arduino_port = 8888;

    int not_recv_cnt   = 0;
    std::string send_str = "";
    std::string recv_str = "";

    float vector_v       = 0.0;
    float vector_omega   = 0.0;
    float target_rpm_l   = 0.0;
    float target_rpm_r   = 0.0;
    float recv_encoder_l = 0.0;
    float recv_encoder_r = 0.0;
    float last_recv_encoder_l = 0.0;
    float last_recv_encoder_r = 0.0;
    float odom_x = 0.0;
    float odom_y = 0.0;
    float odom_z = 0.0;
    float odom_roll = 0.0;
    float odom_pitch = 0.0;
    float odom_yaw = 0.0;
    float odom_twist_x = 0.0;
    float odom_twist_y = 0.0;
    float odom_twist_yaw = 0.0;
    float vx_dt = 0.0;
    float vy_dt = 0.0;
    float theta_dt = 0.0;
    bool start_serial_comm = false;

    // UDP
    int sock;
    struct sockaddr_in addr;

    ros::Time subscribe_time;
    ros::Time recv_time;
    ros::Time last_recv_time;
    ros::Time UDP_send_time;

    ros::Subscriber cmd_vel_sub;
    ros::Publisher odom_pub;

    void cmd_vel_callback();
    uint16_t calculate_checksum(const void*, size_t, size_t);
    float check_overflow(float, float);
    void calc_odom();
    void serial_send_cmd();
    void UDP_send_string_cmd();
    void UDP_send_cmd();
    void publish();

  public:
    CugoController(ros::NodeHandle);
    ~CugoController();
    void view_odom();
    void view_init();
    void init_serial();
    void init_time();
    void init_UDP();
    void close_UDP();
    void serial_reciev_state();
    void count2twist();
    void calc_count_to_vec();
    void twist2rpm();
    void check_stop_cmd_vel();
    void send_rpm_MCU();
    void recv_count_MCU();
    void odom_publish();
    void node_shutdown();
};

#endif
