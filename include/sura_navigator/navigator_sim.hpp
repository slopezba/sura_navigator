#pragma once

#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "sura_msgs/msg/navigator.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace sura_navigator
{

class NavigatorSim : public rclcpp::Node
{
public:
  NavigatorSim();

private:
  void publishFromTf();
  void altitudeCallback(const sensor_msgs::msg::Range::SharedPtr msg);
  static double wrapAngle(double angle);

  rclcpp::Publisher<sura_msgs::msg::Navigator>::SharedPtr navigator_pub_;
  rclcpp::Publisher<sura_msgs::msg::Navigator>::SharedPtr legacy_navigator_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr altitude_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string parent_frame_;
  std::string child_frame_;
  float altitude_{0.0F};

  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  bool has_previous_transform_{false};
  bool has_previous_velocity_{false};
  geometry_msgs::msg::TransformStamped previous_transform_;
  double previous_roll_{0.0};
  double previous_pitch_{0.0};
  double previous_yaw_{0.0};
  geometry_msgs::msg::Twist previous_body_velocity_;
  geometry_msgs::msg::Twist previous_ned_velocity_;
};

}  // namespace sura_navigator
