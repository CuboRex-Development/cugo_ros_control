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
#include <errno.h>
#include <math.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <arpa/inet.h>

#define UDP_BUFF_SIZE 64
#define UDP_HEADER_SIZE 8

#define TARGET_RPM_L_PTR 0
#define TARGET_RPM_R_PTR 4

#define RECV_HEADER_CHECKSUM_PTR 6
#define RECV_ENCODER_L_PTR 0
#define RECV_ENCODER_R_PTR 4

class CugoController {
  private:
    struct UdpHeader
    {
      uint16_t sourcePort;
      uint16_t destinationPort;
      uint16_t length;
      uint16_t checksum;
    };

    // parameters
    //std::string device_name = "/dev/ttyUSB0";
    //int baudrate       = 115200;
    float timeout;
    float wheel_radius_l;
    float wheel_radius_r;
    float reduction_ratio;
    float tread;
    int encoder_resolution;
    int encoder_max; // -2147483648 ~ 2147483647(Arduinoのlong intは32bit)
    //std::string arduino_addr = "192.168.8.216";
    std::string arduino_addr = "127.0.0.1"; // テスト用
    int arduino_port = 8888;
    std::string odom_frame_id;
    std::string odom_child_frame_id;

    int stop_motor_time = 500; //NavigationやコントローラからSubscriberできなかったときにモータを>止めるまでの時間(ms)
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

    // serial
    //bool start_serial_comm = false;

    // UDP
    int sock;
    struct sockaddr_in addr;

    ros::Time subscribe_time;
    ros::Time recv_time;
    ros::Time last_recv_time;
    ros::Time UDP_send_time;

    ros::Subscriber cmd_vel_sub;
    ros::Publisher odom_pub;

    void cmd_vel_callback(const geometry_msgs::Twist&);
    uint16_t calculate_checksum(const void*, size_t, size_t);
    float check_overflow(float, float);
    void calc_odom();
    //void serial_send_cmd();
    void create_UDP_packet(unsigned char*, CugoController::UdpHeader*, unsigned char*);
    void write_float_to_buf(unsigned char*, const int, float);
    void write_int_to_buf(unsigned char*, const int, int);
    void write_bool_to_buf(unsigned char*, const int, bool);
    float read_float_from_buf(unsigned char*, const int);
    int read_int_from_buf(unsigned char*, const int);
    bool read_bool_from_buf(unsigned char*, const int);
    void UDP_send_string_cmd();
    void UDP_send_cmd();
    void publish();

  public:
    CugoController(ros::NodeHandle);
    ~CugoController();

    void view_odom();
    void view_init();

    //void init_serial();
    void init_time();
    void init_UDP();
    void close_UDP();
    //void serial_reciev_state();
    void count2twist();
    //void calc_count_to_vec();
    void twist2rpm();
    void check_stop_cmd_vel();
    void send_rpm_MCU();
    void recv_count_MCU();
    void odom_publish();
    void node_shutdown();
};

#endif
