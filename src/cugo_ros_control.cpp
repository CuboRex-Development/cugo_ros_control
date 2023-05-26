#include "cugo_ros_control/cugo_ros_control.hpp"

CugoController::CugoController(ros::NodeHandle nh)
{
  odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);
  cmd_vel_sub = nh.subscribe("/cmd_vel", 10, &CugoController::cmd_vel_callback, this);

  // grab the parameters
  //nh.param("device", device_name, std::string("/dev/Arduino"));
  //nh.param("baudrate", baudrate, 115200);
  nh.param("timeout", timeout, (float)0.05); // default: 10Hz
  nh.param("wheel_radius_l", wheel_radius_l, (float)0.03858); // default: CuGO V3
  nh.param("wheel_radius_r", wheel_radius_r, (float)0.03858); // default: CuGO V3
  nh.param("tread", tread, (float)0.460); // default: CuGO V3
  nh.param("reduction_ratio", reduction_ratio, (float)1.0); // default: CuGO V3
  nh.param("encoder_max", encoder_max, 2147483647); // -2147483648 ~ 2147483647(Arduinoのlong intは32bit)
  nh.param("encoder_resolution", encoder_resolution, 2048);
  nh.param("arduino_addr", arduino_addr, std::string("192.168.11.216"));
  nh.param("arduino_port", arduino_port, 8888);
  nh.param("source_port", source_port, 8888);
  nh.param("odom_frame_id", odom_frame_id, std::string("odom"));
  nh.param("odom_child_frame_id", odom_child_frame_id, std::string("base_link"));

  view_parameters();
  view_init();
}

CugoController::~CugoController()
{
}

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
  //std::cout << "check_overflow" << std::endl;;
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
    return diff_;
  }
  return diff_;
}

void CugoController::calc_odom()
{
  odom_x += vx_dt * cos(odom_yaw) - vy_dt * sin(odom_yaw);
  odom_y += vx_dt * sin(odom_yaw) - vy_dt * cos(odom_yaw);
  odom_yaw += theta_dt;

  // 故障代替値の更新
  alt_odom_x = odom_x;
  alt_odom_y = odom_y;
  alt_odom_yaw = odom_yaw;
}

//void CugoController::serial_send_cmd()
//{
//}

void CugoController::UDP_send_string_cmd()
{
  std::cout << "UDP_send_string_cmd" << std::endl;
  UDP_send_time = ros::Time::now();
  std::string send_data = std::to_string(target_rpm_l) + "," + std::to_string(target_rpm_r) + "\n";
  sendto(sock, send_data.c_str(), send_data.length(), 0, (struct sockaddr *)&remote_addr, sizeof(remote_addr));
}

void CugoController::create_UDP_packet(unsigned char* packet, CugoController::UdpHeader* header, unsigned char* body)
{
  size_t offset = 0;

  // パケットへヘッダの書き込み
  memcpy(packet + offset, &header->sourcePort, sizeof(header->sourcePort));
  offset += sizeof(header->sourcePort);
  memcpy(packet + offset, &header->destinationPort, sizeof(header->destinationPort));
  offset += sizeof(header->destinationPort);
  memcpy(packet + offset, &header->length, sizeof(header->length));
  offset += sizeof(header->length);
  memcpy(packet + offset, &header->checksum, sizeof(header->checksum));
  offset += sizeof(header->checksum);

  // パケットへボディデータの書き込み
  memcpy(packet + offset, body, sizeof(unsigned char)*UDP_BUFF_SIZE);
}

void CugoController::write_float_to_buf(unsigned char* buf, const int TARGET, float val)
{
  unsigned char* val_ptr = reinterpret_cast<unsigned char*>(&val);
  std::memcpy(buf + TARGET, val_ptr, sizeof(float));
}

void CugoController::write_int_to_buf(unsigned char* buf, const int TARGET, int val)
{
  unsigned char* val_ptr = reinterpret_cast<unsigned char*>(&val);
  std::memcpy(buf + TARGET, val_ptr, sizeof(int));
}

void CugoController::write_bool_to_buf(unsigned char* buf, const int TARGET, bool val)
{
  unsigned char* val_ptr = reinterpret_cast<unsigned char*>(&val);
  std::memcpy(buf + TARGET, val_ptr, sizeof(bool));
}

float CugoController::read_float_from_buf(unsigned char* buf, const int TARGET)
{
  float val = *reinterpret_cast<float*>(buf + UDP_HEADER_SIZE + TARGET);
  return val;
}

int CugoController::read_int_from_buf(unsigned char* buf, const int TARGET)
{
  int val = *reinterpret_cast<int*>(buf + UDP_HEADER_SIZE + TARGET);
  return val;
}

bool CugoController::read_bool_from_buf(unsigned char* buf, const int TARGET)
{
  bool val = *reinterpret_cast<bool*>(buf + UDP_HEADER_SIZE + TARGET);
  return val;
}

uint16_t CugoController::read_uint16_t_from_header(unsigned char* buf, const int TARGET)
{
  if (TARGET >= UDP_HEADER_SIZE-1) return 0;
  uint16_t val = *reinterpret_cast<uint16_t*>(buf + TARGET);
  return val;
}


void CugoController::UDP_send_cmd()
{
  // ボディデータ用バッファの作成
  unsigned char body[UDP_BUFF_SIZE];
  // バッファの初期化
  memset(body, 0x00, sizeof(body));

  // バッファへのbodyデータの書き込み
  write_float_to_buf(body, TARGET_RPM_L_PTR, target_rpm_l);
  write_float_to_buf(body, TARGET_RPM_R_PTR, target_rpm_r);

  // チェックサムを計算
  uint16_t checksum = calculate_checksum(body, UDP_BUFF_SIZE);
  printf("send calc_checksum: %x\n", checksum);

  // UDPヘッダの作成
  UdpHeader header;
  header.sourcePort = htons(source_port);
  header.destinationPort = htons(arduino_port);
  header.length = sizeof(UdpHeader) + sizeof(body);
  header.checksum = checksum;

  // UDPパケットの作成
  unsigned char* packet = new unsigned char[header.length];
  create_UDP_packet(packet, &header, body);

  // UDPパケットの送信
  UDP_send_time = ros::Time::now();
  int send_len = sendto(sock, (unsigned char*) packet, header.length, 0, (struct sockaddr *)&remote_addr, sizeof(remote_addr));

  // 送信失敗時
  if (send_len <= 0)
  {
    // errnoによるエラー表示
    view_send_error();
  }

  std::cout << "send_len: " << send_len << std::endl;
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

void CugoController::view_parameters()
{
  std::cout << "arduino_addr: " << arduino_addr << std::endl;
  std::cout << "arduino_port: " << arduino_port << std::endl;
  std::cout << "encoder_max: " << encoder_max << std::endl;
  std::cout << "encoder_resolution: " << encoder_resolution << std::endl;
  std::cout << "odom_child_frame_id: " << odom_child_frame_id << std::endl;
  std::cout << "odom_frame_id: " << odom_frame_id << std::endl;
  std::cout << "reduction_ratio: " << reduction_ratio << std::endl;
  std::cout << "timeout: " << timeout << std::endl;
  std::cout << "tread: " << tread << std::endl;
  std::cout << "wheel_radius_l: " << wheel_radius_l << std::endl;
  std::cout << "wheel_radius_r: " << wheel_radius_r << std::endl;
}

void CugoController::view_send_error()
{
  int errsv = errno;
  if (errsv == EAGAIN || errsv == EWOULDBLOCK)
  {
    perror("EAGAIN or EWOULDBLOCK");
    ROS_ERROR("send_err EAGAIN or EWOULDBLOCK, errno: %i", errsv);
  }
  else if (errsv == EALREADY)
  {
    perror("EALREADY");
    ROS_ERROR("send_err EALREADY, errno: %i", errsv);
  }
  else if (errsv == EBADF)
  {
    perror("EBADF");
    ROS_ERROR("send_err EBADF, errno: %i", errsv);
  }
  else if (errsv == ECONNRESET)
  {
    perror("ECONNRESET");
    ROS_ERROR("send_err ECONNRESET, errno: %i", errsv);
  }
  else if (errsv == EDESTADDRREQ)
  {
    perror("EDESTADDRREQ");
    ROS_ERROR("send_err EDESTADDRREQ, errno: %i", errsv);
  }
  else if (errsv == EFAULT)
  {
    perror("EFAULT");
    ROS_ERROR("send_err EFAULT, errno: %i", errsv);
  }
  else if (errsv == EINTR)
  {
    perror("EINTR");
    ROS_ERROR("send_err EINTR, errno: %i", errsv);
  }
  else if (errsv == EINVAL)
  {
    perror("EINVAL");
    ROS_ERROR("send_err EINVAL, errno: %i", errsv);
  }
  else if (errsv == EISCONN)
  {
    perror("EISCONN");
    ROS_ERROR("send_err EISCONN, errno: %i", errsv);
  }
  else if (errsv == EMSGSIZE)
  {
    perror("EMSGSIZE");
    ROS_ERROR("send_err EMSGSIZE, errno: %i", errsv);
  }
  else if (errsv == ENOBUFS)
  {
    perror("ENOBUFFS");
    ROS_ERROR("send_err ENOBUFS, errno: %i", errsv);
  }
  else if (errsv == ENOMEM)
  {
    perror("ENOMEM");
    ROS_ERROR("send_err ENOMEM, errno: %i", errsv);
  }
  else if (errsv == ENOTCONN)
  {
    perror("ENOTCONN");
    ROS_ERROR("send_err ENOTCONN, errno: %i", errsv);
  }
  else if (errsv == ENOTSOCK)
  {
    perror("ENOTSOCK");
    ROS_ERROR("send_err ENOTSOCK, errno: %i", errsv);
  }
  else if (errsv == EOPNOTSUPP)
  {
    perror("EOPNOTSUPP");
    ROS_ERROR("send_err EOPNOTSUPP, errno: %i", errsv);
  }
  else if (errsv == EPIPE)
  {
    perror("EPIPE");
    ROS_ERROR("send_err EPIPE, errno: %i", errsv);
  }
  else
  {
    perror("other error");
    ROS_ERROR("send_err errno: %i", errsv);
  }
}

void CugoController::view_recv_error()
{
  int errsv = errno;
  if (errsv == EAGAIN || errsv == EWOULDBLOCK)
  {
    perror("EAGAIN or EWOULDBLOCK");
    ROS_ERROR("recv_err EAGAIN or EWOULDBLOCK, errno: %i", errsv);
  }
  else if (errsv == EBADF)
  {
    perror("Invalid socket");
    ROS_ERROR("recv_err EBADF, errno: %i", errsv);
  }
  else if (errsv == ECONNREFUSED)
  {
    perror("Refused Network Connection");
    ROS_ERROR("recv_err ECONNREFUSED, errno: %i", errsv);
  }
  else if (errsv == EFAULT)
  {
    perror("EFAULT");
    ROS_ERROR("recv_err EFAULT, errno: %i", errsv);
  }
  else if (errsv == EINTR)
  {
    perror("EINTR");
    ROS_ERROR("recv_err EINTR, errno: %i", errsv);
  }
  else if (errsv == EINVAL)
  {
    perror("EINVAL: Invalid Argument");
    ROS_ERROR("recv_err EINVAL, errno: %i", errsv);
  }
  else if (errsv == ENOMEM)
  {
    perror("ENOMEM");
    ROS_ERROR("recv_err ENOMEM, errno: %i", errsv);
  }
  else if (errsv == ENOTCONN)
  {
    perror("ENOTCONN: socket is not connected");
    ROS_ERROR("recv_err ENOTCONN, errno: %i", errsv);
  }
  else if (errsv == ENOTSOCK)
  {
    perror("ENOTSOCK");
    ROS_ERROR("recv_err ENOTSOCK, errno: %i", errsv);
  }
  else
  {
    perror("other recv error");
    ROS_ERROR("recv_err errno: %i", errsv);
  }
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

  // 自身の受信用ipアドレス、ポート設定
  local_addr.sin_family = AF_INET; // IPv4
  local_addr.sin_port = htons(source_port);
  local_addr.sin_addr.s_addr = INADDR_ANY; // INADDR_ANYの場合すべてのアドレスからパケットを受信する

  // 送信先arduinoのipアドレス、ポート設定
  remote_addr.sin_family = AF_INET; // IPv4
  remote_addr.sin_port = htons(arduino_port);
  remote_addr.sin_addr.s_addr = inet_addr(arduino_addr.c_str());

  // 受信タイムアウトの設定
  struct timeval tv;
  tv.tv_sec = timeout;
  tv.tv_usec = 0;
  int setsockopt_status;
  setsockopt_status = setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv, sizeof(tv));
  if (setsockopt_status < 0)
  {
    perror("setsockopt failed");
    ROS_ERROR("setsockopt failed");
    throw std::logic_error("an exception occured: setsockopt failed");
    return;
  }

  // 受信ポート設定
  int bind_status;
  bind_status = bind(sock, (const struct sockaddr *)&local_addr, sizeof(local_addr));
  if (bind_status < 0)
  {
    perror("bind failed");
    ROS_ERROR("bind failed");
    throw std::logic_error("an exception occured: bind failed");
    return;
  }

  // ノンブロッキングモードの設定
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
  if (diff_time != 0.0)
  {
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

    // 故障代替値の更新
    alt_odom_twist_x = odom_twist_x;
    alt_odom_twist_yaw = odom_twist_yaw;

    diff_err_count = 0;
  }
  else
  {
    std::cout << "diff_time == 0.0" << std::endl;
  }
}

// TODO serial用
//void CugoController::calc_count_to_vec()
//{
//}

void CugoController::twist2rpm()
{
  std::cout << "\ntwist2rpm" << std::endl;
  float omega_l = vector_v / wheel_radius_l - tread * vector_omega / (2 * wheel_radius_l);
  float omega_r = vector_v / wheel_radius_r + tread * vector_omega / (2 * wheel_radius_r);
  target_rpm_l = omega_l * 60 / (2 * M_PI);
  target_rpm_r = omega_r * 60 / (2 * M_PI);
  std::cout << target_rpm_l << ", " << target_rpm_r << std::endl;
}

void CugoController::check_failsafe()
{
  check_communication();
}

void CugoController::check_communication()
{
  recv_err_count++;
  checksum_err_count++;
  diff_err_count++;
  // エラーが一定回数連続して確認された場合、故障代替値を格納する
  // パケットの受信がなかった場合
  if (recv_err_count > 5)
  {
    vector_v = 0.0;
    vector_omega = 0.0;
    recv_encoder_l = alt_recv_encoder_l;
    recv_encoder_r = alt_recv_encoder_r;
    ROS_ERROR("UDP recv error: Could not receive packet");
    // エラーカウントのリセット
    recv_err_count = 0;
  }
  // 受信時にチェックサムが一致しなかった場合
  if (checksum_err_count > 5)
  {
    vector_v = 0.0;
    vector_omega = 0.0;
    recv_encoder_l = alt_recv_encoder_l;
    recv_encoder_r = alt_recv_encoder_r;
    ROS_ERROR("UDP recv error: Packet integrity check failed");
    // エラーカウントのリセット
    checksum_err_count = 0;
  }
  // 同一時刻のデータ処理が続いた場合
  if (diff_err_count > 5)
  {
    vector_v = 0.0;
    vector_omega = 0.0;
    odom_twist_x = alt_odom_twist_x;
    odom_twist_yaw = alt_odom_twist_yaw;
    ROS_ERROR("UDP recv error: diff_time zero division error");
    // エラーカウントのリセット
    diff_err_count = 0;
  }
}

void CugoController::check_stop_cmd_vel()
{
  float subscribe_duration = (ros::Time::now() - subscribe_time).toSec();
  if (subscribe_duration > ((float)stop_motor_time / 1000))
  {
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
  std::cout << "\nrecv_count_MCU" << std::endl;
  unsigned char buf[UDP_HEADER_SIZE + UDP_BUFF_SIZE];
  // バッファの初期化
  memset(buf, 0x00, sizeof(buf));

  // 受信
  int recv_len = recv(sock, buf, sizeof(buf), 0);
  std::cout << "recv_len: " << recv_len << std::endl;
  // 受信バッファがない場合
  if (recv_len <= 0)
  {
    view_recv_error();
  }
  // 受信バッファがある場合
  else
  {
    // 受信したパケットの表示
    printf("-------received data-------\n");
    // ヘッダの表示
    printf("header\n");
    for(int i=0;i<UDP_HEADER_SIZE;i++)
    {
      printf("%3hhu ", buf[i]);
    }
    printf("\n");
    // ボディデータの表示
    printf("data\n");
    for(int i=UDP_HEADER_SIZE;i<UDP_BUFF_SIZE;i++)
    {
      printf("%3hhu ", buf[i]);
      if ((i+1)%8==0) printf("\n");
    }
    printf("\n");

    // エラーカウントのリセット
    recv_err_count = 0;

    uint16_t recv_checksum = read_uint16_t_from_header(buf, RECV_HEADER_CHECKSUM_PTR);
    uint16_t calc_checksum = calculate_checksum(buf, recv_len, UDP_HEADER_SIZE);
    // 異常時のフロー
    if (recv_checksum != calc_checksum)
    {
      ROS_ERROR("Packet integrity check failed");
    }
    // 正常時のフロー
    else
    {
      // エラーカウントのリセット
      checksum_err_count = 0;

      // ベクトル計算用の時間を計測
      last_recv_time = recv_time;
      recv_time = ros::Time::now();
      // UDPの一往復の時間を測定
      std::cout << "UDP time:" << (recv_time - UDP_send_time) << std::endl;

      // エンコーダ値の読み出し
      recv_encoder_l = read_float_from_buf(buf, RECV_ENCODER_L_PTR);
      recv_encoder_r = read_float_from_buf(buf, RECV_ENCODER_R_PTR);

      // 故障代替値の更新
      alt_recv_encoder_l = recv_encoder_l;
      alt_recv_encoder_r = recv_encoder_r;

      std::cout << "recv_encoder: " << recv_encoder_l << ", " << recv_encoder_r << std::endl;
    }
  }
}

void CugoController::odom_publish()
{
  std::cout << "\nodom_publish" << std::endl;;
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
  ros::init(argc, argv, "cugo_ros_control");
  ros::NodeHandle nh("~");
  std::cout << "cugo_ros_control start!" << std::endl;

  ros::Rate loop_rate(10);
  CugoController node(nh);

  try
  {
    node.init_time();
    node.init_UDP();

    while (ros::ok())
    {
      node.check_failsafe();
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

  catch (const ros::Exception &e)
  {
    ROS_ERROR("ros::Exception error occured: %s ", e.what());
    node.node_shutdown();
  }

  catch (const std::exception &e)
  {
    ROS_ERROR("std::exception error occured: %s ", e.what());
    node.node_shutdown();
  }

  return 0;
}
