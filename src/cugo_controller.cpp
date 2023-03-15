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
#include <string>
#include <math.h>
#include <time.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
//#include <unistd.h>
#include <arpa/inet.h>
//#include <netinet/in.h>

using namespace std;

// TODO 関数の引き数を上手く設定してグローバル変数を減らす
// parameters
string device_name = "/dev/Arduino";
int baudrate       = 115200;
float timeout        = 0.05;  // 10hzで回す場合
float wheel_radius_l = 0.03858;   //初期値はCuGO V3の値
float wheel_radius_r = 0.03858;   //初期値はCuGO V3の値
// float tread          = 0.380;     //初期値はCuGO V3の値
float reduction_ratio = 1.0;      //初期値はCuGo V3の値
// float wheel_radius_l  = 0.030375;  //CuGO MEGAの値
// float wheel_radius_r  = 0.030375;  //CuGO MEGAの値
float tread           = 0.635;     //CuGO MEGAの値
//float reduction_ratio = 0.3333;    //CuGO MEGAの値

int encoder_resolution = 2048;
int encoder_max    = 2147483647; // -2147483648 ~ 2147483647(Arduinoのlong intは32bit)

int not_recv_cnt   = 0;
int stop_motor_time = 500; //NavigationやコントローラからSubscriberできなかったときにモータを>止めるまでの時間(ms)
//self.arduino_addr = ('192.168.1.177', 8888)
//string arduino_addr = "192.168.8.216";
string arduino_addr = "127.0.0.1";
int arduino_port = 8888;
int UDP_BUFF = 256;
string send_str = "";
string recv_str = "";

// global variables
// TODO 関連の深いものを上手く構造体にまとめる
// odom系、vectorなどlr系、dt系
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
//static tf::TransformBroadcaster br;

void cmd_vel_callback(const geometry_msgs::Twist &msg)
{
  std::cout << "cmd_vel_callback\n";
  vector_v = msg.linear.x;
  vector_omega = msg.angular.z;
  // TODO 通信ロストの検知。異常状態の対応を決める
  // ex)ゼロ埋め、異常フラグ管理
  subscribe_time = ros::Time::now();
}

void publish()
{
  // odom_quatの計算
  // sendTransform
  // odom_msg作成
  // view_odom();
  // publish(odom)
}

void view_odom()
{
  std::cout << "in_twist: " << vector_v << ", " << vector_omega << std::endl;
  std::cout << "out_twist: " << odom_twist_x << ", " << odom_twist_yaw << std::endl;
  std::cout << "odom: " << odom_x << ", " << odom_y << ", " << odom_yaw << std::endl;
}

void view_init()
{
  std::cout << "veiw_init" << std::endl;
}

// 後で実装するかも
void init_serial()
{
}

void init_time()
{
  recv_time = ros::Time::now();
  last_recv_time = ros::Time::now();
  subscribe_time = ros::Time::now();
}

void bind_UDP()
{
  bind(sock, (const struct sockaddr *)&addr, sizeof(addr));
}

void init_UDP()
{
  sock = socket(AF_INET, SOCK_DGRAM, 0);
  addr.sin_family = AF_INET;
  addr.sin_port = htons(arduino_port);

  bind_UDP();
  int val = 1;
  ioctl(sock, FIONBIO, &val);
}

void close_UDP()
{
  std::cout << "UDP port close..." << std::endl;
  close(sock);
}

// 後で実装するかも
void serial_send_cmd()
{
}

void UDP_send_string_cmd()
{
  std::cout << "UDP_send_string_cmd" << std::endl;
  std::cout << "send cmd: " << send_str << std::endl;
  UDP_send_time = ros::Time::now();
  string sn1 = to_string(target_rpm_l);
  string sn2 = to_string(target_rpm_r);
  string send_data = sn1 + "," + sn2 + "\n";
  sendto(sock, send_data.c_str(), send_data.length(), 0, (struct sockaddr *)&addr, sizeof(addr));
}

void UDP_send_cmd()
{
  float data[] = {target_rpm_l, target_rpm_r};
  UDP_send_time = ros::Time::now();
  int send_len = sendto(sock, (unsigned char*) data, sizeof data, 0, (struct sockaddr *)&addr, sizeof(addr));
  //sendto(sock, (unsigned char*) data, sizeof data, 0, (struct sockaddr *)&addr, sizeof(addr));
  std::cout << "send_len: " << send_len << std::endl;
}

// 後で実装するかも
void serial_reciev_state()
{
}

float check_overflow(float diff_, float max_)
{
  std::cout << "check_overflow\n";
  // upper limit
  if (diff_ > max_ * 0.9) {
    diff_ = diff_ - max_ * 2;
    std::cout << "Overflow: " << diff_ << std::endl;
    std::cout << "Overflow max!" << std::endl;
    return diff_;
  }
  // lower limit
  if (diff_ < -max_ * 0.9) {
    diff_ = diff_ + max_ * 2;
    std::cout << "Overflow: " << diff_ << std::endl;
    std::cout << "Overflow min!" << std::endl;
  }
  return diff_;
}

void count2twist()
{
  std::cout << "\ncount2twist" << std::endl;
  float diff_time = (recv_time - last_recv_time).toSec();
  std::cout << "diff_time: " << diff_time << std::endl;
  if (diff_time != 0.0) {
    std::cout << "calc twist" << std::endl;
    int count_diff_l = recv_encoder_l - last_recv_encoder_l;
    int count_diff_r = recv_encoder_r - last_recv_encoder_r;

    last_recv_encoder_l = recv_encoder_l;
    last_recv_encoder_r = recv_encoder_r;

    // マイコン側のオーバーフロー対策
    count_diff_l = check_overflow(count_diff_l,  encoder_max);
    count_diff_r = check_overflow(count_diff_r,  encoder_max);

    float vl = count_diff_l / (encoder_resolution * reduction_ratio) * 2 * wheel_radius_l * M_PI;
    float vr = count_diff_r / (encoder_resolution * reduction_ratio) * 2 * wheel_radius_r * M_PI;

    vx_dt = (vl + vr) / 2;
    theta_dt = (vr - vl) / tread;

    odom_twist_x = vx_dt / diff_time;
    odom_twist_yaw = theta_dt / diff_time;
  }
  else {
    std::cout << "diff_time == 0.0" << std::endl;
  }
}

void calc_odom()
{
  odom_x += vx_dt * cos(odom_yaw) - vy_dt * sin(odom_yaw);
  odom_y += vx_dt * sin(odom_yaw) - vy_dt * cos(odom_yaw);
  odom_yaw += theta_dt;
}

// serial only
// 後で実装するかも
void calc_count_to_vec()
{
}

void twist2rpm()
{
  std::cout << "twist2rpm\n";
  float omega_l = vector_v / wheel_radius_l - tread * vector_omega / (2 * wheel_radius_l);
  float omega_r = vector_v / wheel_radius_r + tread * vector_omega / (2 * wheel_radius_r);

  target_rpm_l = omega_l * 60 / (2 * M_PI);
  target_rpm_r = omega_r * 60 / (2 * M_PI);

  std::cout << target_rpm_l << ", " << target_rpm_r << std::endl;
}

void check_stop_cmd_vel()
{
  std::cout << "\ncheck_stop_cmdvel" << std::endl;
  //ros::Duration subscribe_duration = ros::Time::now() - subscribe_time;
  float subscribe_duration = (ros::Time::now() - subscribe_time).toSec();
  if(subscribe_duration > ((float)stop_motor_time / 1000)) {
    std::cout << "/cmd_vel disconnect...\nset target rpm 0.0" << std::endl;
    vector_v = 0.0;
    vector_omega = 0.0;
  }
}

void send_rpm_MCU()
{
  std::cout << "\nsend_rpm_MCU" << std::endl;
  // create send data

  //UDP_send_cmd();
  UDP_send_string_cmd();
}

void recv_count_MCU()
{
  char buf[UDP_BUFF];
  memset(buf, 0, sizeof(buf));
  int n = recv(sock, buf, sizeof(buf), 0);
  if(n < 1) {
    if (errno == EAGAIN) {
      printf("data does not achieved yet\n");
    }
    else{
      perror("recv");
    }
  }else{
    printf("-------received data-------\n");
    // TODO floatでの受信ができない
    //std::cout << atof(buf) << std::endl;
    std::cout << string(buf) << std::endl;
  }
}

void odom_publish()
{
  std::cout << "odom_publish\n";
  calc_odom();
  publish();
}

void node_shutdown()
{
}

void read_params(ros::NodeHandle nh)
{
  nh.param("device", device_name, std::string("/dev/Arduino"));
  nh.param("baudrate", baudrate, 115200);
  nh.param("timeout", timeout, (float)0.05); // default: 10Hz
  nh.param("wheel_radius_l", wheel_radius_l, (float)0.03858); // default: CuGO V3
  nh.param("wheel_radius_r", wheel_radius_r, (float)0.03858); // default: CuGO V3
  nh.param("tread", tread, (float)0.380); // default: CuGO V3
  nh.param("reduction_ratio", reduction_ratio, (float)1.0); // default: CuGO V3
  nh.param("encoder_resolution", encoder_resolution, 2048);
  nh.param("encoder_max", encoder_max, 2147483647); // -2147483648 ~ 2147483647(Arduinoのlong intは32bit) nh.param("arduino_addr", arduino_addr, string("192.168.8.216"));
  nh.param("arduino_port", arduino_port, 8888);
}

int main(int argc, char **argv)
{
  std::cout << "cugo_controller start!" << std::endl;
  ros::init(argc, argv, "cugo_controller");
  ros::NodeHandle nh;

  odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);

  cmd_vel_sub = nh.subscribe("/cmd_vel", 10, cmd_vel_callback);

  read_params(nh);

  ros::Rate loop_rate(10);

  init_time();
  init_UDP();

  while (ros::ok())
  {
    check_stop_cmd_vel();
    twist2rpm();
    send_rpm_MCU();
    recv_count_MCU();
    count2twist();
    odom_publish();
    ros::spinOnce();
    loop_rate.sleep();
  }

  close_UDP();
  //std::cout << std::endl;
  //cmd_vel_sub.shutdown();
  //ros::shutdown();

  return 0;
}
