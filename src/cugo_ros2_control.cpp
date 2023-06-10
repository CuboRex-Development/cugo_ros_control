#include "cugo_ros2_control/cugo_ros2_control.hpp"

CugoController::CugoController()
: Node("cugo_ros2_control")
{
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&CugoController::cmd_vel_callback, this, _1));

  // Initialize the transform broadcaster
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

  // パラメータ読み込み
  this->declare_parameter("ODOMETRY_DISPLAY", true);
  ODOMETRY_DISPLAY = this->get_parameter("ODOMETRY_DISPLAY").as_bool();
  this->declare_parameter("PARAMETERS_DISPLAY", false);
  PARAMETERS_DISPLAY = this->get_parameter("PARAMETERS_DISPLAY").as_bool();
  this->declare_parameter("RECV_PACKET_DISPLAY", true);
  RECV_PACKET_DISPLAY = this->get_parameter("RECV_PACKET_DISPLAY").as_bool();
  this->declare_parameter("SENT_PACKET_DISPLAY", false);
  SENT_PACKET_DISPLAY = this->get_parameter("SENT_PACKET_DISPLAY").as_bool();
  this->declare_parameter("TARGET_RPM_DISPLAY", true);
  TARGET_RPM_DISPLAY = this->get_parameter("TARGET_RPM_DISPLAY").as_bool();
  this->declare_parameter("READ_DATA_DISPLAY", true);
  READ_DATA_DISPLAY = this->get_parameter("READ_DATA_DISPLAY").as_bool();

  this->declare_parameter("timeout", (float)0.05); // default: 10Hz
  timeout = this->get_parameter("timeout").as_double();
  this->declare_parameter("wheel_radius_l", (float)0.03858); // default: CuGO V3
  wheel_radius_l = this->get_parameter("wheel_radius_l").as_double();
  this->declare_parameter("wheel_radius_r", (float)0.03858); // default: CuGO V3
  wheel_radius_r = this->get_parameter("wheel_radius_r").as_double();
  this->declare_parameter("tread", (float)0.460); // default: CuGO V3
  tread = this->get_parameter("tread").as_double();
  this->declare_parameter("reduction_ratio", (float)1.0); // default CuGO V3
  reduction_ratio = this->get_parameter("reduction_ratio").as_double();
  this->declare_parameter("encoder_max", 2147483647); // -2147483648 ~ 2147483647(Arduinoのlong intは32bit)
  encoder_max = this->get_parameter("encoder_max").as_int();
  this->declare_parameter("encoder_resolution", 2048);
  encoder_resolution = this->get_parameter("encoder_resolution").as_int();
  this->declare_parameter("arduino_addr", std::string("192.168.11.216"));
  arduino_addr = this->get_parameter("arduino_addr").as_string();
  this->declare_parameter("arduino_port", 8888);
  arduino_port = this->get_parameter("arduino_port").as_int();
  this->declare_parameter("source_port", 8888);
  source_port = this->get_parameter("source_port").as_int();
  this->declare_parameter("odom_frame_id", std::string("odom"));
  odom_frame_id = this->get_parameter("odom_frame_id").as_string();
  this->declare_parameter("odom_child_frame_id", std::string("base_link"));
  odom_child_frame_id = this->get_parameter("odom_child_frame_id").as_string();

  this->declare_parameter("vx_dt_max", (float)3.3); // default CuGO V3 [km/h]
  vx_dt_max = this->get_parameter("vx_dt_max").as_double() * 1000 / 3600; // [m/s]
  theta_dt_max = vx_dt_max * 2 / tread; // [m/s]

  view_parameters();
  //view_init();
}

CugoController::~CugoController()
{
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
  // upper limit
  if (diff_ > max_ * 0.9) {
    diff_ = diff_ - max_ * 2;
    RCLCPP_WARN(this->get_logger(), "Overflow max!: %f", diff_);
    return diff_;
  }
  // lower limit
  if (diff_ < -max_ * 0.9) {
    diff_ = diff_ + max_ * 2;
    RCLCPP_WARN(this->get_logger(), "Overflow min!: %f", diff_);
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

void CugoController::create_UDP_packet(unsigned char* packet, CugoController::UdpHeader* header, unsigned char* body)
{
  size_t offset = 0;

  // パケットへヘッダの書き込み
  std::memcpy(packet + offset, &header->sourcePort, sizeof(header->sourcePort));
  offset += sizeof(header->sourcePort);
  std::memcpy(packet + offset, &header->destinationPort, sizeof(header->destinationPort));
  offset += sizeof(header->destinationPort);
  std::memcpy(packet + offset, &header->length, sizeof(header->length));
  offset += sizeof(header->length);
  std::memcpy(packet + offset, &header->checksum, sizeof(header->checksum));
  offset += sizeof(header->checksum);

  // パケットへボディデータの書き込み
  std::memcpy(packet + offset, body, sizeof(unsigned char)*UDP_BUFF_SIZE);
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
  //printf("send calc_checksum: %x\n", checksum);

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
  UDP_send_time = this->get_clock()->now();
  int send_len = sendto(sock, (unsigned char*) packet, header.length, 0, (struct sockaddr *)&remote_addr, sizeof(remote_addr));

  // 送信失敗時
  if (send_len <= 0)
  {
    // errnoによるエラー表示
    view_send_error();
  }

  view_sent_packet(packet, send_len);
  delete[] packet;
}

void CugoController::view_odom()
{
  if (ODOMETRY_DISPLAY == true)
  {
    std::cout << "DISPLAY TWIST & ODOMETRY" << std::endl;
    std::cout << "in_twist: " << vector_v << ", " << vector_omega << std::endl;
    std::cout << "out_twist: " << odom_twist_x << ", " << odom_twist_yaw << std::endl;
    std::cout << "odom: " << odom_x << ", " << odom_y << ", " << odom_yaw << std::endl;
    std::cout << std::endl;
  }
}

void CugoController::view_init()
{
  std::cout << "view_init" << std::endl;
}

void CugoController::view_parameters()
{
  if (PARAMETERS_DISPLAY == true)
  {
    std::cout << "DISPLAY PARAMETERS" << std::endl;
    std::cout << "arduino_addr: " << arduino_addr << std::endl;
    std::cout << "arduino_port: " << arduino_port << std::endl;
    std::cout << "source_port: " << source_port << std::endl;
    std::cout << "encoder_max: " << encoder_max << std::endl;
    std::cout << "encoder_resolution: " << encoder_resolution << std::endl;
    std::cout << "odom_child_frame_id: " << odom_child_frame_id << std::endl;
    std::cout << "odom_frame_id: " << odom_frame_id << std::endl;
    std::cout << "reduction_ratio: " << reduction_ratio << std::endl;
    std::cout << "timeout: " << timeout << std::endl;
    std::cout << "tread: " << tread << std::endl;
    std::cout << "wheel_radius_l: " << wheel_radius_l << std::endl;
    std::cout << "wheel_radius_r: " << wheel_radius_r << std::endl;
    std::cout << std::endl;
  }
}

void CugoController::view_recv_packet(unsigned char* buf, int recv_len)
{
  if (RECV_PACKET_DISPLAY == true)
  {
    std::cout << "DISPLAY RECIEVED PACKET" << std::endl;
    // 受信したパケットの大きさ
    std::cout << "recv_len: " << recv_len << std::endl;
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
  }
}

void CugoController::view_sent_packet(unsigned char* buf, int send_len)
{
  if (SENT_PACKET_DISPLAY == true)
  {
    std::cout << "DISPLAY SENT PACKET" << std::endl;
    // 送信したパケットの大きさ
    std::cout << "send_len: " << send_len << std::endl;

    // 送信したパケットの表示
    printf("---------sent data---------\n");
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

  }
}

void CugoController::view_target_rpm()
{
  if (TARGET_RPM_DISPLAY == true)
  {
    std::cout << "DISPLAY TARGET_RPM" << std::endl;
    std::cout << "Target rpm(L/R): " << target_rpm_l << ", " << target_rpm_r << std::endl;
    std::cout << std::endl;
  }
}

void CugoController::view_read_data()
{
  if (READ_DATA_DISPLAY == true)
  {
    std::cout << "DISPLAY RECIEVED DATA" << std::endl;
    std::cout << "Recieved encoder(L/R): " << recv_encoder_l << ", " << recv_encoder_r << std::endl;
    std::cout << std::endl;
  }
}

void CugoController::init_UDP()
{
  sock = socket(AF_INET, SOCK_DGRAM, 0);

  // 自身の受信用ipアドレス、ポート設定
  local_addr.sin_family = AF_INET; // IPv4
  local_addr.sin_port = htons(source_port);
  local_addr.sin_addr.s_addr = INADDR_ANY; // INADDR_ANYの場合すべてのアドレスからのパケットを受信する

  // 送信先arduinoのipアドレス、ポート設定
  remote_addr.sin_family = AF_INET; // IPv4
  remote_addr.sin_port = htons(arduino_port);
  remote_addr.sin_addr.s_addr = inet_addr(arduino_addr.c_str());

  // 受信タイムアウトの設定
  struct timeval tv;
  tv.tv_sec = timeout;
  tv.tv_usec = 0;
  int setsockopt_status = setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv, sizeof(tv));
  if (setsockopt_status < 0)
  {
    perror("setsockopt");
    RCLCPP_ERROR(this->get_logger(), "setsockopt failed");
    throw std::logic_error("an exception occured: setsockopt failed");
    return;
  }

  // 受信ポート設定
  int bind_status;
  bind_status = bind(sock, (const struct sockaddr *)&local_addr, sizeof(local_addr));
  if (bind_status < 0)
  {
    perror("bind failed");
    RCLCPP_ERROR(this->get_logger(), "bind failed");
    throw std::logic_error("an exception occured: bind failed");
    return;
  }

  // ノンブロッキングモードの設定
  int val = 1;
  ioctl(sock, FIONBIO, &val);
}

bool CugoController::get_first_recv_flag()
{
  return first_recv_flag;
}

void CugoController::close_UDP()
{
  std::cout << "UDP port close..." << std::endl;
  close(sock);
}

void CugoController::count2twist()
{
  float diff_time = (recv_time - last_recv_time).seconds();
  //std::cout << "diff_time: " << diff_time << std::endl;
  if (diff_time != 0.0)
  {
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

    // 加速度上限を超えていないか確認
    if(fabs(vx_dt) > fabs(vx_dt_max))
    {
      acc_limit_over_flag = true;
      RCLCPP_ERROR(this->get_logger(), "over vx_dt_max, did not update odometry");
    }
    else if(fabs(theta_dt) > fabs(theta_dt_max))
    {
      acc_limit_over_flag = true;
      RCLCPP_ERROR(this->get_logger(), "over theta_dt_max, did not update odometry");
    }
    else
    {
      odom_twist_x = vx_dt / diff_time;
      odom_twist_yaw = theta_dt / diff_time;

      // 故障代替値の更新
      alt_odom_twist_x = odom_twist_x;
      alt_odom_twist_yaw = odom_twist_yaw;

      acc_limit_over_flag = false;
    }

    diff_err_count = 0;
  }
  else
  {
    //std::cout << "diff_time == 0.0" << std::endl;
  }
}

void CugoController::twist2rpm()
{
  float omega_l = vector_v / wheel_radius_l - tread * vector_omega / (2 * wheel_radius_l);
  float omega_r = vector_v / wheel_radius_r + tread * vector_omega / (2 * wheel_radius_r);
  target_rpm_l = omega_l * 60 / (2 * M_PI);
  target_rpm_r = omega_r * 60 / (2 * M_PI);

  view_target_rpm();
}

void CugoController::check_failsafe()
{
  check_communication();
}

void CugoController::send_rpm_MCU()
{
  UDP_send_cmd(); // binary
  //UDP_send_string_cmd(); // string
}

void CugoController::recv_count_MCU()
{
  unsigned char buf[UDP_HEADER_SIZE + UDP_BUFF_SIZE];
  // バッファの初期化
  memset(buf, 0x00, sizeof(buf));

  // 受信
  int recv_len = recv(sock, buf, sizeof(buf), 0);
  //std::cout << "recv_len: " << recv_len << std::endl;
  // 受信バッファがない場合
  if (recv_len <= 0)
  {
    view_recv_error();
  }
  // 受信バッファがある場合
  else
  {
    view_recv_packet(buf, recv_len);

    // エラーカウントのリセット
    recv_err_count = 0;

    uint16_t recv_checksum = read_uint16_t_from_header(buf, RECV_HEADER_CHECKSUM_PTR);
    uint16_t calc_checksum = calculate_checksum(buf, recv_len, UDP_HEADER_SIZE);
    // 異常時のフロー
    if (recv_checksum != calc_checksum)
    {
      RCLCPP_ERROR(this->get_logger(), "Packet integrity check failed");
    }
    // 正常時のフロー
    else
    {
      // エラーカウントのリセット
      checksum_err_count = 0;

      // ベクトル計算用の時間を計測
      last_recv_time = recv_time;
      recv_time = this->get_clock()->now();
      // UDPの一往復の時間を測定
      //std::cout << "UDP time:" << (recv_time - UDP_send_time).seconds() << std::endl;

      // エンコーダ値の読み出し
      recv_encoder_l = read_float_from_buf(buf, RECV_ENCODER_L_PTR);
      recv_encoder_r = read_float_from_buf(buf, RECV_ENCODER_R_PTR);

      // 故障代替値の更新
      alt_recv_encoder_l = recv_encoder_l;
      alt_recv_encoder_r = recv_encoder_r;

      if (!first_recv_flag)
      {
        first_recv_flag = true;
        last_recv_encoder_l = recv_encoder_l;
        last_recv_encoder_r = recv_encoder_r;
      }

      view_read_data();
    }
  }
}

void CugoController::init_time()
{
  recv_time = this->get_clock()->now();
  last_recv_time = this->get_clock()->now();
  subscribe_time = this->get_clock()->now();
}

void CugoController::view_send_error()
{
  int errsv = errno;
  if (errsv == EAGAIN || errsv == EWOULDBLOCK)
  {
    perror("EAGAIN or EWOULDBLOCK");
    RCLCPP_ERROR(this->get_logger(), "send_err EAGAIN or EWOULDBLOCK, errno: %i", errsv);
  }
  else if (errsv == EALREADY)
  {
    perror("EALREADY");
    RCLCPP_ERROR(this->get_logger(), "send_err EALREADY, errno: %i", errsv);
  }
  else if (errsv == EBADF)
  {
    perror("EBADF");
    RCLCPP_ERROR(this->get_logger(), "send_err EBADF, errno: %i", errsv);
  }
  else if (errsv == ECONNRESET)
  {
    perror("ECONNRESET");
    RCLCPP_ERROR(this->get_logger(), "send_err ECONNRESET, errno: %i", errsv);
  }
  else if (errsv == EDESTADDRREQ)
  {
    perror("EDESTADDRREQ");
    RCLCPP_ERROR(this->get_logger(), "send_err EDESTADDRREQ, errno: %i", errsv);
  }
  else if (errsv == EFAULT)
  {
    perror("EFAULT");
    RCLCPP_ERROR(this->get_logger(), "send_err EFAULT, errno: %i", errsv);
  }
  else if (errsv == EINTR)
  {
    perror("EINTR");
    RCLCPP_ERROR(this->get_logger(), "send_err EINTR, errno: %i", errsv);
  }
  else if (errsv == EINVAL)
  {
    perror("EINVAL");
    RCLCPP_ERROR(this->get_logger(), "send_err EINVAL, errno: %i", errsv);
  }
  else if (errsv == EISCONN)
  {
    perror("EISCONN");
    RCLCPP_ERROR(this->get_logger(), "send_err EISCONN, errno: %i", errsv);
  }
  else if (errsv == EMSGSIZE)
  {
    perror("EMSGSIZE");
    RCLCPP_ERROR(this->get_logger(), "send_err EMSGSIZE, errno: %i", errsv);
  }
  else if (errsv == ENOBUFS)
  {
    perror("ENOBUFFS");
    RCLCPP_ERROR(this->get_logger(), "send_err ENOBUFS, errno: %i", errsv);
  }
  else if (errsv == ENOMEM)
  {
    perror("ENOMEM");
    RCLCPP_ERROR(this->get_logger(), "send_err ENOMEM, errno: %i", errsv);
  }
  else if (errsv == ENOTCONN)
  {
    perror("ENOTCONN");
    RCLCPP_ERROR(this->get_logger(), "send_err ENOTCONN, errno: %i", errsv);
  }
  else if (errsv == ENOTSOCK)
  {
    perror("ENOTSOCK");
    RCLCPP_ERROR(this->get_logger(), "send_err ENOTSOCK, errno: %i", errsv);
  }
  else if (errsv == EOPNOTSUPP)
  {
    perror("EOPNOTSUPP");
    RCLCPP_ERROR(this->get_logger(), "send_err EOPNOTSUPP, errno: %i", errsv);
  }
  else if (errsv == EPIPE)
  {
    perror("EPIPE");
    RCLCPP_ERROR(this->get_logger(), "send_err EPIPE, errno: %i", errsv);
  }
  else
  {
    perror("other error");
    RCLCPP_ERROR(this->get_logger(), "send_err errno: %i", errsv);
  }
}

void CugoController::view_recv_error()
{
  int errsv = errno;
  if (errsv == EAGAIN || errsv == EWOULDBLOCK)
  {
    perror("EAGAIN or EWOULDBLOCK");
    RCLCPP_ERROR(this->get_logger(), "recv_err EAGAIN or EWOULDBLOCK, errno: %i", errsv);
  }
  else if (errsv == EBADF)
  {
    perror("Invalid socket");
    RCLCPP_ERROR(this->get_logger(), "recv_err EBADF, errno: %i", errsv);
  }
  else if (errsv == ECONNREFUSED)
  {
    perror("Refused Network Connection");
    RCLCPP_ERROR(this->get_logger(), "recv_err ECONNREFUSED, errno: %i", errsv);
  }
  else if (errsv == EFAULT)
  {
    perror("EFAULT");
    RCLCPP_ERROR(this->get_logger(), "recv_err EFAULT, errno: %i", errsv);
  }
  else if (errsv == EINTR)
  {
    perror("EINTR");
    RCLCPP_ERROR(this->get_logger(), "recv_err EINTR, errno: %i", errsv);
  }
  else if (errsv == EINVAL)
  {
    perror("EINVAL: Invalid Argument");
    RCLCPP_ERROR(this->get_logger(), "recv_err EINVAL, errno: %i", errsv);
  }
  else if (errsv == ENOMEM)
  {
    perror("ENOMEM");
    RCLCPP_ERROR(this->get_logger(), "recv_err ENOMEM, errno: %i", errsv);
  }
  else if (errsv == ENOTCONN)
  {
    perror("ENOTCONN: socket is not connected");
    RCLCPP_ERROR(this->get_logger(), "recv_err ENOTCONN, errno: %i", errsv);
  }
  else if (errsv == ENOTSOCK)
  {
    perror("ENOTSOCK");
    RCLCPP_ERROR(this->get_logger(), "recv_err ENOTSOCK, errno: %i", errsv);
  }
  else
  {
    perror("other recv error");
    RCLCPP_ERROR(this->get_logger(), "recv_err errno: %i", errsv);
  }
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
    RCLCPP_ERROR(this->get_logger(), "UDP recv error: Could not receive packet");
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
    RCLCPP_ERROR(this->get_logger(), "UDP recv error: Packet integrity check failed");
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
    RCLCPP_ERROR(this->get_logger(), "UDP recv error: diff_time zero division error");
    // エラーカウントのリセット
    diff_err_count = 0;
  }
}

void CugoController::check_stop_cmd_vel()
{
  float subscribe_duration = (this->get_clock()->now() - subscribe_time).seconds();
  if (subscribe_duration > ((float)stop_motor_time / 1000))
  {
    RCLCPP_WARN(this->get_logger(), "/cmd_vel disconnect...\nset target rpm 0.0");
    vector_v = 0.0;
    vector_omega = 0.0;
  }
}

void CugoController::node_shutdown()
{
  close_UDP();
  cmd_vel_sub_.reset();
  rclcpp::shutdown();
}


void CugoController::odom_publish()
{
  //RCLCPP_INFO(this->get_logger(), "Publishing odometry" );
  if (!acc_limit_over_flag)
  {
    calc_odom();
  }
  publish();
}

void CugoController::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  vector_v = msg->linear.x;
  vector_omega = msg->angular.z;
  subscribe_time = this->get_clock()->now();
}

void CugoController::publish()
{
  geometry_msgs::msg::TransformStamped t;

  // Read message content and assign it to
  // corresponding tf variables
  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = odom_frame_id;
  t.child_frame_id = odom_child_frame_id;

  // Turtle only exists in 2D, thus we get x and y translation
  // coordinates from the message and set the z coordinate to 0
  t.transform.translation.x = 0.0; //msg->x
  t.transform.translation.y = 0.0; //msg->y
  t.transform.translation.z = 0.0;

  // For the same reason, turtle can only rotate around one axis
  // and this why we set rotation in x and y to 0 and obtain
  // rotation in z axis from the message
  tf2::Quaternion q;
  q.setRPY(0, 0, odom_yaw);
  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();

  // Send the transformation
  tf_broadcaster_->sendTransform(t);

  geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg(q);

  nav_msgs::msg::Odometry odom;
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

  odom_pub_->publish(odom);
}

int main(int argc, char * argv[])
{
  // rclcppの初期化
  rclcpp::init(argc, argv);
  std::shared_ptr<CugoController> node = std::make_shared<CugoController>();
  //std::cout << "cugo_ros2_control start!" << std::endl;

  rclcpp::WallRate loop_rate(10); // 10Hz

  try
  {
    node->init_time();
    node->init_UDP();
    while(!node->get_first_recv_flag())
    {
      node->send_rpm_MCU();
      node->recv_count_MCU();
    }

    while (rclcpp::ok())
    {
      node->check_failsafe();
      node->check_stop_cmd_vel();
      node->twist2rpm();
      node->send_rpm_MCU();
      node->recv_count_MCU();
      node->count2twist();
      node->odom_publish();
      rclcpp::spin_some(node);
      loop_rate.sleep();
    }
    rclcpp::shutdown();
  }
  catch (const rclcpp::exceptions::RCLError &e)
  {
    RCLCPP_ERROR(node->get_logger(), "unexpectedly failed with %s ", e.what());
    rclcpp::shutdown();
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(node->get_logger(), "std::exception error occured: %s ", e.what());
    rclcpp::shutdown();
  }
  return 0;
}

