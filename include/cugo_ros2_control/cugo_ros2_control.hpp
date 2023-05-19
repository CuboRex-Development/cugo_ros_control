#ifndef CUGO_CONTROLLER_H
#define CUGO_CONTROLLER_H

#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

class CugoController : public rclcpp::Node
{
  public:
    CugoController();
    ~CugoController();
    void view();
    void publish();

  private:
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const;
    void odom_publish();
};

#endif
