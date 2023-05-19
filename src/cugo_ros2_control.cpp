#include "cugo_ros2_control/cugo_ros2_control.hpp"

CugoController::CugoController()
: Node("cugo_ros2_control")
{
  twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&CugoController::twist_callback, this, _1));

  // Initialize the transform broadcaster
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
}

CugoController::~CugoController()
{
}

void CugoController::view()
{
  RCLCPP_INFO(this->get_logger(), "Viewing...");
}

void CugoController::publish()
{
  odom_publish();
  RCLCPP_INFO(this->get_logger(), "Publishing odometry" );
}

void CugoController::twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
{
  //RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->linear);
  RCLCPP_INFO(this->get_logger(), "I heard: twist");
}

void CugoController::odom_publish()
{
  geometry_msgs::msg::TransformStamped t;

  // Read message content and assign it to
  // corresponding tf variables
  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = "odom";
  t.child_frame_id = "base_link";

  // Turtle only exists in 2D, thus we get x and y translation
  // coordinates from the message and set the z coordinate to 0
  t.transform.translation.x = 0.0; //msg->x
  t.transform.translation.y = 0.0; //msg->y
  t.transform.translation.z = 0.0;

  // For the same reason, turtle can only rotate around one axis
  // and this why we set rotation in x and y to 0 and obtain
  // rotation in z axis from the message
  tf2::Quaternion q;
  q.setRPY(0, 0, 0.0/* theta */);
  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();

  // Send the transformation
  tf_broadcaster_->sendTransform(t);

  geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg(q);

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = this->get_clock()->now(); //recv_time;
  odom.header.frame_id = "odom"; //odom_frame_id;

  // set the position
  odom.pose.pose.position.x = 0.0; //odom_x
  odom.pose.pose.position.y = 0.0; //odom_y
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  // set the velocity
  odom.child_frame_id = "base_link"; //odom_child_frame_id
  odom.twist.twist.linear.x = 0.0; //odom_twist_x;
  odom.twist.twist.linear.y = 0.0; //odom_twist_y
  odom.twist.twist.angular.z = 0.0; // odom_twist_yaw

  odom_pub_->publish(odom);
}

int main(int argc, char * argv[])
{
  // rclcppの初期化
  rclcpp::init(argc, argv);
  std::shared_ptr<CugoController> node = std::make_shared<CugoController>();

  rclcpp::WallRate loop_rate(10); // 10Hz
  while (rclcpp::ok())
  {
    node->view();
    node->publish();
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}

