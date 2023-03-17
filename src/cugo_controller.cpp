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
#include <unistd.h>
#include <arpa/inet.h>

using namespace std;

struct UdpHeader
{
  uint16_t sourcePort;
  uint16_t destinationPort;
  uint16_t length;
  uint16_t checksum;
};

// TODO 関数の引き数を上手く設定してグローバル変数を減らす
// parameters
string device_name = "/dev/Arduino";
int baudrate       = 115200;
float timeout        = 0.05;  // 10hzで回す場合
float wheel_radius_l = 0.03858;   //初期値はCuGO V3の値
float wheel_radius_r = 0.03858;   //初期値はCuGO V3の値
float reduction_ratio = 1.0;      //初期値はCuGo V3の値
float tread           = 0.635;     //CuGO MEGAの値

int encoder_resolution = 2048;
int encoder_max    = 2147483647; // -2147483648 ~ 2147483647(Arduinoのlong intは32bit)

int not_recv_cnt   = 0;
int stop_motor_time = 500; //NavigationやコントローラからSubscriberできなかったときにモータを>止めるまでの時間(ms)
//string arduino_addr = "192.168.8.216";
string arduino_addr = "127.0.0.1";
int arduino_port = 8888;
const int UDP_BUFF_SIZE = 64;
const int UDP_HEADER_SIZE = 8;
string send_str = "";
string recv_str = "";

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
struct timeval tv;

ros::Time subscribe_time;
ros::Time recv_time;
ros::Time last_recv_time;
ros::Time UDP_send_time;

ros::Subscriber cmd_vel_sub;
ros::Publisher odom_pub;

void cmd_vel_callback(const geometry_msgs::Twist &msg)
{
  std::cout << "cmd_vel_callback\n";
  vector_v = msg.linear.x;
  vector_omega = msg.angular.z;
  // TODO 通信ロストの検知。異常状態の対応を決める
  // ex)ゼロ埋め、異常フラグ管理
  subscribe_time = ros::Time::now();
}

void view_odom()
{
  std::cout << "in_twist: " << vector_v << ", " << vector_omega << std::endl;
  std::cout << "out_twist: " << odom_twist_x << ", " << odom_twist_yaw << std::endl;
  std::cout << "odom: " << odom_x << ", " << odom_y << ", " << odom_yaw << std::endl;
}

void publish()
{
  static tf::TransformBroadcaster odom_broadcaster;
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odom_yaw);

  // TODO roslaunch
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = recv_time;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";

  odom_trans.transform.translation.x = odom_x;
  odom_trans.transform.translation.y = odom_y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;

  // send the transform
  odom_broadcaster.sendTransform(odom_trans);

  // odom_msg作成
  nav_msgs::Odometry odom;
  odom.header.stamp = recv_time;
  odom.header.frame_id = "odom";

  // set the position
  odom.pose.pose.position.x = odom_x;
  odom.pose.pose.position.y = odom_y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  // set the velocity
  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = odom_twist_x;
  odom.twist.twist.linear.y = odom_twist_y;
  odom.twist.twist.angular.z = odom_twist_yaw;

  view_odom();
  odom_pub.publish(odom);
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

  tv.tv_sec = timeout;
  tv.tv_usec = 0;

  if(setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv, sizeof(tv)) < 0) {
    perror("setsockopt");
    return;
  }

  bind_UDP();
  int val = 1;
  ioctl(sock, FIONBIO, &val);
}

void close_UDP()
{
  std::cout << "UDP port close..." << std::endl;
  close(sock);
}

uint16_t calculateChecksum(const void* data, size_t size, size_t start = 0)
{
  uint16_t checksum = 0;
  const uint8_t* bytes = static_cast<const uint8_t*>(data);

  // バイト列を2バイトずつ加算
  for (size_t i = start; i < size; i += 2)
  {
    checksum += (bytes[i] << 8) | bytes[i+1];
  }

  // キャリーがあった場合は回収
  checksum = (checksum & 0xFFFF) + (checksum >> 16);

  // チェックサムを反転
  return ~checksum;
}

// 後で実装するかも
void serial_send_cmd()
{
}

void UDP_send_string_cmd()
{
  std::cout << "UDP_send_string_cmd" << std::endl;
  UDP_send_time = ros::Time::now();
  string send_data = to_string(target_rpm_l) + "," + to_string(target_rpm_r) + "\n";
  sendto(sock, send_data.c_str(), send_data.length(), 0, (struct sockaddr *)&addr, sizeof(addr));
}

void UDP_send_cmd()
{
  unsigned char byte_data[UDP_BUFF_SIZE];
  memset(byte_data, 0x00, sizeof(byte_data));

  const unsigned char HEAD = 0xFF;
  std::memcpy(byte_data, &HEAD, sizeof(HEAD));

  unsigned char* target_rpm_l_ptr = reinterpret_cast<unsigned char*>(&target_rpm_l);
  unsigned char* target_rpm_r_ptr = reinterpret_cast<unsigned char*>(&target_rpm_r);

  std::memcpy(byte_data + 1, target_rpm_l_ptr, sizeof(float));
  std::memcpy(byte_data + 5, target_rpm_l_ptr, sizeof(float));

  // チェックサムを計算
  uint16_t checksum = calculateChecksum(byte_data, UDP_BUFF_SIZE);
  printf("send calc_checksum: %x\n", checksum);

  // UPDパケットを作成
  UdpHeader header;
  header.sourcePort = 0;
  header.destinationPort = htons(arduino_port);
  header.length = sizeof(UdpHeader) + sizeof(byte_data);
  header.checksum = checksum;

  unsigned char* packet = new unsigned char[header.length];
  size_t offset = 0;

  memcpy(packet + offset, &header.sourcePort, sizeof(header.sourcePort));
  offset += sizeof(header.sourcePort);

  memcpy(packet + offset, &header.destinationPort, sizeof(header.destinationPort));
  offset += sizeof(header.destinationPort);

  memcpy(packet + offset, &header.length, sizeof(header.length));
  offset += sizeof(header.length);

  memcpy(packet + offset, &header.checksum, sizeof(header.checksum));
  offset += sizeof(header.checksum);

  memcpy(packet + offset, (unsigned char*) byte_data, sizeof(byte_data));
  std::cout << "offset: " << offset << std::endl;

  UDP_send_time = ros::Time::now();
  // UDPパケットを送信
  int send_len = sendto(sock, (unsigned char*) packet, header.length, 0, (struct sockaddr *)&addr, sizeof(addr));
  std::cout << "send_len: " << send_len << std::endl;
  std::cout << "header.length: " << header.length << std::endl;
  delete[] packet;
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
  // binary
  UDP_send_cmd();
  // string
  //UDP_send_string_cmd();
}

void recv_count_MCU()
{
  char buf[UDP_HEADER_SIZE + UDP_BUFF_SIZE];
  memset(buf, 0, sizeof(buf));
  int recv_len = recv(sock, buf, sizeof(buf), 0);
  std::cout << "recv_len: " << recv_len << std::endl;
  if(recv_len < 0) {
    if (errno == EAGAIN) {
      printf("data does not achieved yet\n");
      perror("Timeout");
    }
    else{
      perror("recv");
    }
  }else{
    printf("-------received data-------\n");
    printf("header\n");
    for(int i=0;i<UDP_HEADER_SIZE;i++){
      printf("%3hhu ", buf[i]);
    }
    printf("\n");
    printf("data\n");
    for(int i=UDP_HEADER_SIZE;i<UDP_BUFF_SIZE;i++){
      printf("%3hhu ", buf[i]);
      if((i+1)%8==0) printf("\n");
    }
    printf("\n");

    uint16_t recv_checksum = (*(uint16_t*)(buf+6));
    uint16_t calc_checksum = calculateChecksum(buf, recv_len, UDP_HEADER_SIZE);

    if(recv_checksum != calc_checksum) {
      std::cerr << "Packet integrity check failed" << std::endl;
      return;
    }

    // ベクトル計算用の時間を計測
    last_recv_time = recv_time;
    recv_time = ros::Time::now();

    // floatで読み出し
    recv_encoder_l = *reinterpret_cast<float*>(buf + UDP_HEADER_SIZE + 1);
    recv_encoder_r = *reinterpret_cast<float*>(buf + UDP_HEADER_SIZE + 5);

    std::cout << "recv_encoder: " << recv_encoder_l << ", " << recv_encoder_r << std::endl;
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
  close_UDP();
  cmd_vel_sub.shutdown();
  ros::shutdown();
  std::cout << "node_shutdown" << std::endl;
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
  nh.param("encoder_max", encoder_max, 2147483647); // -2147483648 ~ 2147483647(Arduinoのlong intは32bit)
  nh.param("arduino_addr", arduino_addr, string("192.168.8.216"));
  nh.param("arduino_port", arduino_port, 8888);
}

int main(int argc, char **argv)
{
  std::cout << "cugo_controller start!" << std::endl;
  ros::init(argc, argv, "cugo_controller");
  ros::NodeHandle nh;

  odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);
  cmd_vel_sub = nh.subscribe("/cmd_vel", 10, cmd_vel_callback);

  ros::Rate loop_rate(10);

  try {
    read_params(nh);

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

    node_shutdown();
  }
  catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
    node_shutdown();
  }
  catch (const ros::Exception &e) {
    ROS_ERROR("Error occured: %s ", e.what());
    node_shutdown();
  }

  return 0;
}
