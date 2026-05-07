#include "sura_navigator/navigator_node.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/time.h"

namespace sura_navigator
{

NavigatorNode::NavigatorNode()
: Node("sura_navigator"),
  tf_buffer_(this->get_clock())
{
  this->declare_parameter<std::string>("odom_topic", "/cirtesub/localization/odometry");
  this->declare_parameter<std::string>("altitude_topic", "/cirtesub/sensors/dvl/altitude");
  this->declare_parameter<std::string>("navigator_topic", "/cirtesub/navigator/navigation");
  this->declare_parameter<std::string>("parent_frame", "world_ned");
  this->declare_parameter<std::string>("child_frame", "cirtesub/base_link");
  this->declare_parameter<bool>("use_tf_fallback", false);
  this->declare_parameter<double>("publish_rate", 50.0);
  this->declare_parameter<double>("odom_timeout", 0.5);

  const std::string odom_topic = this->get_parameter("odom_topic").as_string();
  const std::string altitude_topic = this->get_parameter("altitude_topic").as_string();
  const std::string navigator_topic = this->get_parameter("navigator_topic").as_string();
  parent_frame_ = this->get_parameter("parent_frame").as_string();
  child_frame_ = this->get_parameter("child_frame").as_string();
  use_tf_fallback_ = this->get_parameter("use_tf_fallback").as_bool();
  const double publish_rate = std::max(1.0, this->get_parameter("publish_rate").as_double());
  odom_timeout_ = std::max(0.0, this->get_parameter("odom_timeout").as_double());

  navigator_pub_ = this->create_publisher<sura_msgs::msg::Navigator>(navigator_topic, 10);
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic,
    10,
    std::bind(&NavigatorNode::handleOdometry, this, std::placeholders::_1));
  altitude_sub_ = this->create_subscription<sensor_msgs::msg::Range>(
    altitude_topic,
    10,
    std::bind(&NavigatorNode::handleAltitude, this, std::placeholders::_1));

  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_, this, false);

  if (use_tf_fallback_) {
    tf_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate),
      std::bind(&NavigatorNode::publishFromTf, this));
  }

  RCLCPP_INFO(this->get_logger(), "SURA navigator started");
  RCLCPP_INFO(this->get_logger(), "Reading odometry: %s", odom_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "Reading altitude: %s", altitude_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "Publishing navigator: %s", navigator_topic.c_str());
  if (use_tf_fallback_) {
    RCLCPP_INFO(
      this->get_logger(),
      "TF fallback enabled: %s -> %s",
      parent_frame_.c_str(),
      child_frame_.c_str());
  }
}

double NavigatorNode::wrapAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

void NavigatorNode::handleOdometry(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const rclcpp::Time stamp =
    msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0 ?
    this->now() :
    rclcpp::Time(msg->header.stamp);

  sura_msgs::msg::Navigator navigator_msg = buildNavigatorFromOdometry(*msg, stamp);
  updateAccelerations(navigator_msg, stamp);
  navigator_pub_->publish(navigator_msg);

  last_odom_stamp_ = stamp;
  has_odom_ = true;
}

void NavigatorNode::handleAltitude(const sensor_msgs::msg::Range::SharedPtr msg)
{
  altitude_ = msg->range;
}

sura_msgs::msg::Navigator NavigatorNode::buildNavigatorFromOdometry(
  const nav_msgs::msg::Odometry & odom_msg,
  const rclcpp::Time & stamp)
{
  (void)stamp;

  sura_msgs::msg::Navigator navigator_msg;
  navigator_msg.position = odom_msg.pose.pose;
  navigator_msg.altitude = altitude_;

  tf2::Quaternion q(
    odom_msg.pose.pose.orientation.x,
    odom_msg.pose.pose.orientation.y,
    odom_msg.pose.pose.orientation.z,
    odom_msg.pose.pose.orientation.w);

  if (q.length2() < 1e-12) {
    q.setValue(0.0, 0.0, 0.0, 1.0);
  } else {
    q.normalize();
  }

  navigator_msg.position.orientation.x = q.x();
  navigator_msg.position.orientation.y = q.y();
  navigator_msg.position.orientation.z = q.z();
  navigator_msg.position.orientation.w = q.w();

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  navigator_msg.rpy.x = roll;
  navigator_msg.rpy.y = pitch;
  navigator_msg.rpy.z = yaw;

  navigator_msg.body_velocity = odom_msg.twist.twist;

  const tf2::Vector3 body_linear(
    odom_msg.twist.twist.linear.x,
    odom_msg.twist.twist.linear.y,
    odom_msg.twist.twist.linear.z);
  const tf2::Vector3 ned_linear = tf2::quatRotate(q, body_linear);

  navigator_msg.ned_velocity.linear.x = ned_linear.x();
  navigator_msg.ned_velocity.linear.y = ned_linear.y();
  navigator_msg.ned_velocity.linear.z = ned_linear.z();

  navigator_msg.ned_velocity.angular = odom_msg.twist.twist.angular;

  return navigator_msg;
}

void NavigatorNode::updateAccelerations(
  sura_msgs::msg::Navigator & navigator_msg,
  const rclcpp::Time & stamp)
{
  if (!has_previous_velocity_) {
    previous_state_stamp_ = stamp;
    previous_body_velocity_ = navigator_msg.body_velocity;
    previous_ned_velocity_ = navigator_msg.ned_velocity;
    previous_roll_ = navigator_msg.rpy.x;
    previous_pitch_ = navigator_msg.rpy.y;
    previous_yaw_ = navigator_msg.rpy.z;
    has_previous_velocity_ = true;
    return;
  }

  const double dt = (stamp - previous_state_stamp_).seconds();
  if (dt <= 0.0) {
    return;
  }

  navigator_msg.body_acceleration.linear.x =
    (navigator_msg.body_velocity.linear.x - previous_body_velocity_.linear.x) / dt;
  navigator_msg.body_acceleration.linear.y =
    (navigator_msg.body_velocity.linear.y - previous_body_velocity_.linear.y) / dt;
  navigator_msg.body_acceleration.linear.z =
    (navigator_msg.body_velocity.linear.z - previous_body_velocity_.linear.z) / dt;

  navigator_msg.body_acceleration.angular.x =
    (navigator_msg.body_velocity.angular.x - previous_body_velocity_.angular.x) / dt;
  navigator_msg.body_acceleration.angular.y =
    (navigator_msg.body_velocity.angular.y - previous_body_velocity_.angular.y) / dt;
  navigator_msg.body_acceleration.angular.z =
    (navigator_msg.body_velocity.angular.z - previous_body_velocity_.angular.z) / dt;

  navigator_msg.ned_acceleration.linear.x =
    (navigator_msg.ned_velocity.linear.x - previous_ned_velocity_.linear.x) / dt;
  navigator_msg.ned_acceleration.linear.y =
    (navigator_msg.ned_velocity.linear.y - previous_ned_velocity_.linear.y) / dt;
  navigator_msg.ned_acceleration.linear.z =
    (navigator_msg.ned_velocity.linear.z - previous_ned_velocity_.linear.z) / dt;

  navigator_msg.ned_acceleration.angular.x =
    wrapAngle(navigator_msg.rpy.x - previous_roll_) / dt;
  navigator_msg.ned_acceleration.angular.y =
    wrapAngle(navigator_msg.rpy.y - previous_pitch_) / dt;
  navigator_msg.ned_acceleration.angular.z =
    wrapAngle(navigator_msg.rpy.z - previous_yaw_) / dt;

  previous_state_stamp_ = stamp;
  previous_body_velocity_ = navigator_msg.body_velocity;
  previous_ned_velocity_ = navigator_msg.ned_velocity;
  previous_roll_ = navigator_msg.rpy.x;
  previous_pitch_ = navigator_msg.rpy.y;
  previous_yaw_ = navigator_msg.rpy.z;
}

void NavigatorNode::publishFromTf()
{
  if (has_odom_) {
    const double time_since_odom = (this->now() - last_odom_stamp_).seconds();
    if (time_since_odom <= odom_timeout_) {
      return;
    }
  }

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_.lookupTransform(parent_frame_, child_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Could not lookup TF %s -> %s: %s",
      parent_frame_.c_str(),
      child_frame_.c_str(),
      ex.what());
    return;
  }

  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header = transform.header;
  odom_msg.child_frame_id = child_frame_;
  odom_msg.pose.pose.position.x = transform.transform.translation.x;
  odom_msg.pose.pose.position.y = transform.transform.translation.y;
  odom_msg.pose.pose.position.z = transform.transform.translation.z;
  odom_msg.pose.pose.orientation = transform.transform.rotation;

  if (has_previous_transform_) {
    const rclcpp::Time current_time(transform.header.stamp);
    const rclcpp::Time previous_time(previous_transform_.header.stamp);
    const double dt = (current_time - previous_time).seconds();

    if (dt > 0.0) {
      const double vx_ned =
        (transform.transform.translation.x - previous_transform_.transform.translation.x) / dt;
      const double vy_ned =
        (transform.transform.translation.y - previous_transform_.transform.translation.y) / dt;
      const double vz_ned =
        (transform.transform.translation.z - previous_transform_.transform.translation.z) / dt;

      tf2::Quaternion q(
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z,
        transform.transform.rotation.w);
      q.normalize();

      const tf2::Vector3 v_ned(vx_ned, vy_ned, vz_ned);
      const tf2::Vector3 v_body = tf2::quatRotate(q.inverse(), v_ned);
      odom_msg.twist.twist.linear.x = v_body.x();
      odom_msg.twist.twist.linear.y = v_body.y();
      odom_msg.twist.twist.linear.z = v_body.z();

      double roll = 0.0;
      double pitch = 0.0;
      double yaw = 0.0;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

      odom_msg.twist.twist.angular.x = wrapAngle(roll - previous_roll_) / dt;
      odom_msg.twist.twist.angular.y = wrapAngle(pitch - previous_pitch_) / dt;
      odom_msg.twist.twist.angular.z = wrapAngle(yaw - previous_yaw_) / dt;
    }
  }

  const rclcpp::Time stamp =
    transform.header.stamp.sec == 0 && transform.header.stamp.nanosec == 0 ?
    this->now() :
    rclcpp::Time(transform.header.stamp);
  sura_msgs::msg::Navigator navigator_msg = buildNavigatorFromOdometry(odom_msg, stamp);
  updateAccelerations(navigator_msg, stamp);
  navigator_pub_->publish(navigator_msg);

  previous_transform_ = transform;
  has_previous_transform_ = true;
}

}  // namespace sura_navigator

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sura_navigator::NavigatorNode>());
  rclcpp::shutdown();
  return 0;
}
