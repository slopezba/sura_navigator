#pragma once

#include <memory>
#include <string>

#include "geometry_msgs/msg/accel.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "sura_msgs/msg/navigator.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace sura_navigator
{

class NavigatorNode : public rclcpp::Node
{
public:
  NavigatorNode();

private:
  void handleOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);
  void handleTwistOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);
  void handleAltitude(const sensor_msgs::msg::Range::SharedPtr msg);
  void publishFromTf();
  void publishTfFromOdometry(const nav_msgs::msg::Odometry & odom_msg);
  sura_msgs::msg::Navigator buildNavigatorFromOdometry(
    const nav_msgs::msg::Odometry & odom_msg,
    const rclcpp::Time & stamp);
  void publishNavigator(const sura_msgs::msg::Navigator & navigator_msg);
  void updateAccelerations(
    sura_msgs::msg::Navigator & navigator_msg,
    const rclcpp::Time & stamp);
  static tf2::Matrix3x3 poseOrientationToRotation(const geometry_msgs::msg::Pose & pose);
  static tf2::Vector3 lowPass(
    const tf2::Vector3 & previous,
    const tf2::Vector3 & current,
    double alpha);
  static geometry_msgs::msg::Vector3 toVector3(const tf2::Vector3 & vector);
  static geometry_msgs::msg::Accel toAccel(
    const tf2::Vector3 & linear,
    const tf2::Vector3 & angular);
  static double wrapAngle(double angle);

  rclcpp::Publisher<sura_msgs::msg::Navigator>::SharedPtr navigator_pub_;
  rclcpp::Publisher<sura_msgs::msg::Navigator>::SharedPtr legacy_navigator_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr twist_odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr altitude_sub_;
  rclcpp::TimerBase::SharedPtr tf_timer_;

  std::string parent_frame_;
  std::string child_frame_;
  double linear_lpf_alpha_{0.2};
  bool use_tf_fallback_{false};
  bool publish_tf_{false};
  double odom_timeout_{0.5};
  rclcpp::Time last_odom_stamp_{0, 0, RCL_ROS_TIME};
  bool has_odom_{false};
  float altitude_{0.0F};

  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  bool has_previous_transform_{false};
  geometry_msgs::msg::TransformStamped previous_transform_;
  bool has_previous_velocity_{false};
  rclcpp::Time previous_state_stamp_{0, 0, RCL_ROS_TIME};
  double previous_roll_{0.0};
  double previous_pitch_{0.0};
  double previous_yaw_{0.0};
  bool has_linear_filter_state_{false};
  nav_msgs::msg::Odometry::SharedPtr latest_twist_odom_;
  tf2::Vector3 previous_body_linear_{0.0, 0.0, 0.0};
  tf2::Vector3 previous_body_angular_{0.0, 0.0, 0.0};
  geometry_msgs::msg::Twist previous_body_velocity_;
  geometry_msgs::msg::Twist previous_ned_velocity_;
};

}  // namespace sura_navigator
