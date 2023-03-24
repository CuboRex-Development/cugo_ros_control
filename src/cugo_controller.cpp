#include "cugo_controller/cugo_controller.hpp"

CugoController::CugoController(ros::NodeHandle nh) {
  odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);
  cmd_vel_sub = nh.subscribe("/cmd_vel", 10, &CugoController::cmd_vel_callback, this);

  // grab the parameters
  //nh.param("device", device_name, std::string("/dev/Arduino"));
  //nh.param("baudrate", baudrate, 115200);
  nh.param("timeout", timeout, (float)0.05); // default: 10Hz
  nh.param("wheel_radius_l", wheel_radius_l, (float)0.03858); // default: CuGO V3
  nh.param("wheel_radius_r", wheel_radius_r, (float)0.03858); // default: CuGO V3
  nh.param("tread", tread, (float)0.380); // default: CuGO V3
  nh.param("reduction_ratio", reduction_ratio, (float)1.0); // default: CuGO V3
  nh.param("encoder_resolution", encoder_resolution, 2048);
  nh.param("encoder_max", encoder_max, 2147483647); // -2147483648 ~ 2147483647(Arduinoのlong intは32bit)
  nh.param("arduino_addr", arduino_addr, std::string("192.168.8.216"));
  nh.param("arduino_port", arduino_port, 8888);
  nh.param("odom_frame_id", odom_frame_id, std::string("odom"));
  nh.param("odom_child_frame_id", odom_child_frame_id, std::string("base_link"));

  std::cout << "timeout: " << timeout << std::endl;
  std::cout << "wheel_radius_l: " << wheel_radius_l << std::endl;
  std::cout << "wheel_radius_r: " << wheel_radius_r << std::endl;
  std::cout << "tread: " << tread << std::endl;
  std::cout << "reduction_ratio: " << reduction_ratio << std::endl;
  std::cout << "encoder_resolution: " << encoder_resolution << std::endl;
  std::cout << "encoder_max: " << encoder_max << std::endl;
  std::cout << "arduino_port: " << arduino_port << std::endl;
  std::cout << "arduino_addr: " << arduino_addr << std::endl;
  std::cout << "odom_frame_id: " << odom_frame_id << std::endl;
  std::cout << "odom_child_frame_id: " << odom_child_frame_id << std::endl;

  view_init();
}

CugoController::~CugoController() {}

void CugoController::cmd_vel_callback(const geometry_msgs::Twist &msg)
{
  std::cout << "cmd_vel_callback" << std::endl;
  vector_v = msg.linear.x;
  vector_omega = msg.angular.z;
  // TODO 通信ロストの検知。異常状態の対応を決める
  // ex)ゼロ埋め、異常フラグ管理
  subscribe_time = ros::Time::now();
}

uint16_t CugoController::calculate_checksum(const void* data, size_t size, size_t start = 0)
{
  uint16_t checksum = 0;
  const uint8_t* bytes = static_cast<const uint8_t*>(data);
  // バイト列を2バイトずつ加算
  for (size_t i = start; i < size; i += 2)
  {
    checksum += (bytes[i] << 8) | bytes[i+1];
  }
  // 桁あふれがあった場合は回収
  checksum = (checksum & 0xFFFF) + (checksum >> 16);
  // チェックサムを反転
  return ~checksum;
}

float CugoController::check_overflow(float diff_, float max_)
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

void CugoController::calc_odom()
{
  odom_x += vx_dt * cos(odom_yaw) - vy_dt * sin(odom_yaw);
  odom_y += vx_dt * sin(odom_yaw) - vy_dt * cos(odom_yaw);
  odom_yaw += theta_dt;
}

//void CugoController::serial_send_cmd()
//{
//}

void CugoController::UDP_send_string_cmd()
{
  std::cout << "UDP_send_string_cmd" << std::endl;
  UDP_send_time = ros::Time::now();
  std::string send_data = std::to_string(target_rpm_l) + "," + std::to_string(target_rpm_r) + "\n";
  sendto(sock, send_data.c_str(), send_data.length(), 0, (struct sockaddr *)&addr, sizeof(addr));
}

void CugoController::UDP_send_cmd()
{
  // bodyデータ用バッファの作成
  unsigned char body[UDP_BUFF_SIZE];
  memset(body, 0x00, sizeof(body));

  unsigned char* target_rpm_l_ptr = reinterpret_cast<unsigned char*>(&target_rpm_l);
  unsigned char* target_rpm_r_ptr = reinterpret_cast<unsigned char*>(&target_rpm_r);

  // バッファへのbodyデータの書き込み
  std::memcpy(body + 1, target_rpm_l_ptr, sizeof(float));
  std::memcpy(body + 5, target_rpm_l_ptr, sizeof(float));

  // チェックサムを計算
  uint16_t checksum = calculate_checksum(body, UDP_BUFF_SIZE);
  printf("send calc_checksum: %x\n", checksum);

  // UPDパケットを作成
  UdpHeader header;
  header.sourcePort = 0;
  header.destinationPort = htons(arduino_port);
  header.length = sizeof(UdpHeader) + sizeof(body);
  header.checksum = checksum;

  unsigned char* packet = new unsigned char[header.length];
  size_t offset = 0;
  // パケットへheaderの書き込み
  memcpy(packet + offset, &header.sourcePort, sizeof(header.sourcePort));
  offset += sizeof(header.sourcePort);
  memcpy(packet + offset, &header.destinationPort, sizeof(header.destinationPort));
  offset += sizeof(header.destinationPort);
  memcpy(packet + offset, &header.length, sizeof(header.length));
  offset += sizeof(header.length);
  memcpy(packet + offset, &header.checksum, sizeof(header.checksum));
  offset += sizeof(header.checksum);

  // パケットへ送信データ本体の書き込み
  memcpy(packet + offset, (unsigned char*) body, sizeof(body));
  std::cout << "offset: " << offset << std::endl;

  UDP_send_time = ros::Time::now();
  // send UDP packet
  int send_len = sendto(sock, (unsigned char*) packet, header.length, 0, (struct sockaddr *)&addr, sizeof(addr));
  std::cout << "send_len: " << send_len << std::endl;
  std::cout << "header.length: " << header.length << std::endl;
  delete[] packet;
}

void CugoController::publish()
{
  static tf::TransformBroadcaster odom_broadcaster;
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(odom_yaw);

  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = recv_time;
  odom_trans.header.frame_id = odom_frame_id;
  odom_trans.child_frame_id = odom_child_frame_id;
  odom_trans.transform.translation.x = odom_x;
  odom_trans.transform.translation.y = odom_y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;

  // send the transform
  odom_broadcaster.sendTransform(odom_trans);

  // odom_msg作成
  nav_msgs::Odometry odom;
  odom.header.stamp = recv_time;
  odom.header.frame_id = odom_frame_id;

  // set the position
  odom.pose.pose.position.x = odom_x;
  odom.pose.pose.position.y = odom_y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  // set the velocity
  odom.child_frame_id = odom_child_frame_id;
  odom.twist.twist.linear.x = odom_twist_x;
  odom.twist.twist.linear.y = odom_twist_y;
  odom.twist.twist.angular.z = odom_twist_yaw;

  view_odom();
  odom_pub.publish(odom);
}

void CugoController::view_odom()
{
  std::cout << "in_twist: " << vector_v << ", " << vector_omega << std::endl;
  std::cout << "out_twist: " << odom_twist_x << ", " << odom_twist_yaw << std::endl;
  std::cout << "odom: " << odom_x << ", " << odom_y << ", " << odom_yaw << std::endl;
}

void CugoController::view_init()
{
  std::cout << "view_init" << std::endl;
}

// TODO
//void CugoController::init_serial()
//{
//}

void CugoController::init_time()
{
  recv_time = ros::Time::now();
  last_recv_time = ros::Time::now();
  subscribe_time = ros::Time::now();
}

void CugoController::init_UDP()
{
  sock = socket(AF_INET, SOCK_DGRAM, 0);
  addr.sin_family = AF_INET;
  addr.sin_port = htons(arduino_port);

  struct timeval tv;
  tv.tv_sec = timeout;
  tv.tv_usec = 0;

  if(setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv, sizeof(tv)) < 0) {
    perror("setsockopt");
    return;
  }

  bind(sock, (const struct sockaddr *)&addr, sizeof(addr));
  int val = 1;
  ioctl(sock, FIONBIO, &val);
}

void CugoController::close_UDP()
{
  std::cout << "UDP port close..." << std::endl;
  close(sock);
}

// TODO
//void CugoController::serial_reciev_state()
//{
//}

void CugoController::count2twist()
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
    // TODO 故障代替値の検討
  }
}

// TODO serial用
//void CugoController::calc_count_to_vec()
//{
//}

void CugoController::twist2rpm()
{
  std::cout << "twist2rpm\n";
  float omega_l = vector_v / wheel_radius_l - tread * vector_omega / (2 * wheel_radius_l);
  float omega_r = vector_v / wheel_radius_r + tread * vector_omega / (2 * wheel_radius_r);
  target_rpm_l = omega_l * 60 / (2 * M_PI);
  target_rpm_r = omega_r * 60 / (2 * M_PI);
  std::cout << target_rpm_l << ", " << target_rpm_r << std::endl;
}

void CugoController::check_stop_cmd_vel()
{
  std::cout << "\ncheck_stop_cmdvel" << std::endl;
  float subscribe_duration = (ros::Time::now() - subscribe_time).toSec();
  if(subscribe_duration > ((float)stop_motor_time / 1000)) {
    std::cout << "/cmd_vel disconnect...\nset target rpm 0.0" << std::endl;
    vector_v = 0.0;
    vector_omega = 0.0;
  }
}

void CugoController::send_rpm_MCU()
{
  std::cout << "\nsend_rpm_MCU" << std::endl;
  UDP_send_cmd(); // binary
  //UDP_send_string_cmd(); // string
}

void CugoController::recv_count_MCU()
{
  char buf[UDP_HEADER_SIZE + UDP_BUFF_SIZE];
  memset(buf, 0, sizeof(buf));

  // 受信
  int recv_len = recv(sock, buf, sizeof(buf), 0);
  std::cout << "recv_len: " << recv_len << std::endl;
  if(recv_len <= 0) {
    if (errno == EAGAIN) {
      printf("data does not achieved yet\n");
      perror("Timeout");
    }
    else{
      perror("recv");
    }
    // TODO 異常値の格納
  }
  else{
    // 受信したパケットの表示
    printf("-------received data-------\n");
    // ヘッダーの表示
    printf("header\n");
    for(int i=0;i<UDP_HEADER_SIZE;i++){
      printf("%3hhu ", buf[i]);
    }
    printf("\n");
    // ボディデータの表示
    printf("data\n");
    for(int i=UDP_HEADER_SIZE;i<UDP_BUFF_SIZE;i++){
      printf("%3hhu ", buf[i]);
      if((i+1)%8==0) printf("\n");
    }
    printf("\n");

    uint16_t recv_checksum = (*(uint16_t*)(buf+6));
    uint16_t calc_checksum = calculate_checksum(buf, recv_len, UDP_HEADER_SIZE);

    if(recv_checksum != calc_checksum) {
      std::cerr << "Packet integrity check failed" << std::endl;
      // TODO 処理の要検討
      return;
    }

    // ベクトル計算用の時間を計測
    last_recv_time = recv_time;
    recv_time = ros::Time::now();

    // floatで読み出し
    // TODO 即値を消す
    recv_encoder_l = *reinterpret_cast<float*>(buf + UDP_HEADER_SIZE + 1);
    recv_encoder_r = *reinterpret_cast<float*>(buf + UDP_HEADER_SIZE + 5);

    std::cout << "recv_encoder: " << recv_encoder_l << ", " << recv_encoder_r << std::endl;
  }
}

void CugoController::odom_publish()
{
  std::cout << "odom_publish\n";
  calc_odom();
  publish();
}

void CugoController::node_shutdown()
{
  close_UDP();
  cmd_vel_sub.shutdown();
  ros::shutdown();
  std::cout << "node_shutdown" << std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cugo_controller");
  ros::NodeHandle nh("~");
  std::cout << "cugo_controller start!" << std::endl;

  ros::Rate loop_rate(10);
  CugoController node(nh);

  try {
    node.init_time();
    node.init_UDP();

    while (ros::ok())
    {
      node.check_stop_cmd_vel();
      node.twist2rpm();
      node.send_rpm_MCU();
      node.recv_count_MCU();
      node.count2twist();
      node.odom_publish();
      ros::spinOnce();
      loop_rate.sleep();
    }

    node.node_shutdown();
  }
  catch (const std::exception &e) {
    std::cerr << "std::exception: " <<  e.what() << std::endl;
    node.node_shutdown();
  }
  catch (const ros::Exception &e) {
    // TODO test(rosmasterのkill)
    ROS_ERROR("Error occured: %s ", e.what());
    node.node_shutdown();
  }

  return 0;
}
