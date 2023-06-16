#ifndef CUGO_CONTROLLER_H
#define CUGO_CONTROLLER_H

// cpp
#include <cstdio>
#include <cstring>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <sstream>
#include <errno.h>
#include <math.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <arpa/inet.h>

// ros2
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#define UDP_BUFF_SIZE 64
#define UDP_HEADER_SIZE 8

#define TARGET_RPM_L_PTR 0
#define TARGET_RPM_R_PTR 4

#define RECV_HEADER_CHECKSUM_PTR 6
#define RECV_ENCODER_L_PTR 0
#define RECV_ENCODER_R_PTR 4

using namespace std::chrono_literals;
using std::placeholders::_1;

class CugoController : public rclcpp::Node
{
  public:
    // ros2実装したもの
    CugoController();
    ~CugoController();
    void odom_publish();

    // 移植してくるもの
    void view_odom();
    void view_init();
    void view_parameters();
    void view_send_error();
    void view_recv_error();
    void view_recv_packet(unsigned char*, int);
    void view_sent_packet(unsigned char*, int);
    void view_target_rpm();
    void view_read_data();

    void init_time();
    void init_UDP();
    void close_UDP();

    // TODO 修正が必要なもの
    void count2twist();
    void twist2rpm();
    void check_failsafe();
    void check_stop_cmd_vel();
    void send_rpm_MCU();
    void recv_count_MCU();
    void node_shutdown();

    void reset_last_encoder();

  private:
    struct UdpHeader
    {
      uint16_t sourcePort;
      uint16_t destinationPort;
      uint16_t length;
      uint16_t checksum;
    };

    // display parameters
    bool ODOMETRY_DISPLAY = true;
    bool PARAMETERS_DISPLAY = true;
    bool RECV_PACKET_DISPLAY = true;
    bool SENT_PACKET_DISPLAY = true;
    bool TARGET_RPM_DISPLAY = true;
    bool READ_DATA_DISPLAY = true;

    // parameters
    float timeout;
    float wheel_radius_l;
    float wheel_radius_r;
    float reduction_ratio;
    float tread;
    float vx_dt_max;
    float theta_dt_max;
    int encoder_resolution;
    int encoder_max; // -2147483648 ~ 2147483647(Arduinoのlong intは32bit)
    std::string arduino_addr = "192.168.8.216";
    //std::string arduino_addr = "127.0.0.1"; // テスト用
    int arduino_port = 8888;
    int source_port = 8888;
    std::string odom_frame_id;
    std::string odom_child_frame_id;

    int stop_motor_time = 500; //NavigationやコントローラからSubscriberできなかったときにモータを>止めるまでの時間(ms)

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
    bool acc_limit_over_flag = false;
    bool first_recv_flag = false;

    int recv_err_count = 0;
    int checksum_err_count = 0;
    int diff_err_count = 0;

    // 故障代替値
    float alt_recv_encoder_l = 0.0;
    float alt_recv_encoder_r = 0.0;
    float alt_odom_x = 0.0;
    float alt_odom_y = 0.0;
    float alt_odom_yaw = 0.0;
    float alt_odom_twist_x = 0.0;
    float alt_odom_twist_yaw = 0.0;

    // UDP
    int sock;
    struct sockaddr_in local_addr; // 受信用
    struct sockaddr_in remote_addr; // 送信用

    // TODO 時間系
    rclcpp::Time subscribe_time;
    rclcpp::Time recv_time;
    rclcpp::Time last_recv_time;
    rclcpp::Time UDP_send_time;

    // ros2実装したもの
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // ros2実装したもの
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void publish();

    // 移植してくるもの
    uint16_t calculate_checksum(const void*, size_t, size_t);
    float check_overflow(float, float);
    void calc_odom();
    void create_UDP_packet(unsigned char*, CugoController::UdpHeader*, unsigned char*);
    void write_float_to_buf(unsigned char*, const int, float);
    void write_int_to_buf(unsigned char*, const int, int);
    void write_bool_to_buf(unsigned char*, const int, bool);
    float read_float_from_buf(unsigned char*, const int);
    int read_int_from_buf(unsigned char*, const int);
    bool read_bool_from_buf(unsigned char*, const int);
    uint16_t read_uint16_t_from_header(unsigned char*, const int);
    void UDP_send_cmd();
    void check_communication();


};

#endif
