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

geometry_msgs::msg::Vector3 NavigatorNode::toVector3(const tf2::Vector3 & vector)
{
  geometry_msgs::msg::Vector3 out;
  out.x = vector.x();
  out.y = vector.y();
  out.z = vector.z();
  return out;
}

geometry_msgs::msg::Accel NavigatorNode::toAccel(
  const tf2::Vector3 & linear,
  const tf2::Vector3 & angular)
{
  geometry_msgs::msg::Accel out;
  out.linear = toVector3(linear);
  out.angular = toVector3(angular);
  return out;
}

tf2::Matrix3x3 NavigatorNode::poseOrientationToRotation(const geometry_msgs::msg::Pose & pose)
{
  tf2::Quaternion q(
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z,
    pose.orientation.w);
  return tf2::Matrix3x3(q);
}

tf2::Vector3 NavigatorNode::lowPass(
  const tf2::Vector3 & previous,
  const tf2::Vector3 & current,
  double alpha)
{
  const double clamped_alpha = std::clamp(alpha, 0.0, 1.0);
  return previous + clamped_alpha * (current - previous);
}

NavigatorNode::NavigatorNode()
: Node("sura_navigator"),
  tf_buffer_(this->get_clock())
{
  this->declare_parameter<std::string>("odom_topic", "localization/odometry");
  this->declare_parameter<std::string>("twist_odom_topic", "");
  this->declare_parameter<std::string>("altitude_topic", "sensors/dvl/altitude");
  this->declare_parameter<std::string>("navigator_topic", "navigator/navigation");
  this->declare_parameter<std::string>("legacy_navigator_topic", "");
  this->declare_parameter<std::string>("parent_frame", "world_ned");
  this->declare_parameter<std::string>("child_frame", "base_link");
  this->declare_parameter<bool>("odom_twist_in_body_frame", false);
  this->declare_parameter<bool>("odom_invert_angular_z", true);
  this->declare_parameter<bool>("twist_odom_twist_in_body_frame", true);
  this->declare_parameter<bool>("twist_odom_invert_angular_z", false);
  this->declare_parameter<double>("linear_lpf_alpha", 0.2);
  this->declare_parameter<bool>("use_tf_fallback", false);
  this->declare_parameter<bool>("publish_tf", false);
  this->declare_parameter<double>("publish_rate", 50.0);
  this->declare_parameter<double>("odom_timeout", 0.5);

  const std::string odom_topic = this->get_parameter("odom_topic").as_string();
  const std::string twist_odom_topic =
    this->get_parameter("twist_odom_topic").as_string();
  const std::string altitude_topic = this->get_parameter("altitude_topic").as_string();
  const std::string navigator_topic = this->get_parameter("navigator_topic").as_string();
  const std::string legacy_navigator_topic =
    this->get_parameter("legacy_navigator_topic").as_string();
  parent_frame_ = this->get_parameter("parent_frame").as_string();
  child_frame_ = this->get_parameter("child_frame").as_string();
  use_tf_fallback_ = this->get_parameter("use_tf_fallback").as_bool();
  publish_tf_ = this->get_parameter("publish_tf").as_bool();
  linear_lpf_alpha_ = std::max(
    0.0, this->get_parameter("linear_lpf_alpha").as_double());
  const double publish_rate = std::max(1.0, this->get_parameter("publish_rate").as_double());
  odom_timeout_ = std::max(0.0, this->get_parameter("odom_timeout").as_double());

  navigator_pub_ = this->create_publisher<sura_msgs::msg::Navigator>(navigator_topic, 10);
  if (!legacy_navigator_topic.empty()) {
    legacy_navigator_pub_ = this->create_publisher<sura_msgs::msg::Navigator>(
      legacy_navigator_topic, 10);
  }
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic,
    10,
    std::bind(&NavigatorNode::handleOdometry, this, std::placeholders::_1));
  if (!twist_odom_topic.empty()) {
    twist_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      twist_odom_topic,
      10,
      std::bind(&NavigatorNode::handleTwistOdometry, this, std::placeholders::_1));
  }
  altitude_sub_ = this->create_subscription<sensor_msgs::msg::Range>(
    altitude_topic,
    10,
    std::bind(&NavigatorNode::handleAltitude, this, std::placeholders::_1));

  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_, this, false);
  if (publish_tf_) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

  if (use_tf_fallback_) {
    tf_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate),
      std::bind(&NavigatorNode::publishFromTf, this));
  }

  RCLCPP_INFO(this->get_logger(), "SURA navigator started");
  RCLCPP_INFO(this->get_logger(), "Reading odometry: %s", odom_topic.c_str());
  if (!twist_odom_topic.empty()) {
    RCLCPP_INFO(this->get_logger(), "Using external twist from: %s", twist_odom_topic.c_str());
    RCLCPP_INFO(
      this->get_logger(), "External odometry twist already in body frame: %s",
      this->get_parameter("twist_odom_twist_in_body_frame").as_bool() ? "true" : "false");
  }
  RCLCPP_INFO(this->get_logger(), "Reading altitude: %s", altitude_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "Publishing navigator: %s", navigator_topic.c_str());
  if (!legacy_navigator_topic.empty()) {
    RCLCPP_INFO(
      this->get_logger(), "Publishing legacy navigator: %s", legacy_navigator_topic.c_str());
  }
  RCLCPP_INFO(
    this->get_logger(), "Odometry twist already in body frame: %s",
    this->get_parameter("odom_twist_in_body_frame").as_bool() ? "true" : "false");
  RCLCPP_INFO(
    this->get_logger(), "Invert odometry angular.z: %s",
    this->get_parameter("odom_invert_angular_z").as_bool() ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "Linear LPF alpha: %.3f", linear_lpf_alpha_);
  if (use_tf_fallback_) {
    RCLCPP_INFO(
      this->get_logger(),
      "TF fallback enabled: %s -> %s",
      parent_frame_.c_str(),
      child_frame_.c_str());
  }
  RCLCPP_INFO(this->get_logger(), "Publish odometry TF: %s", publish_tf_ ? "true" : "false");
}

double NavigatorNode::wrapAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

void NavigatorNode::handleTwistOdometry(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  latest_twist_odom_ = msg;
}

void NavigatorNode::handleOdometry(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const rclcpp::Time stamp =
    msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0 ?
    this->now() :
    rclcpp::Time(msg->header.stamp);

  publishTfFromOdometry(*msg);

  sura_msgs::msg::Navigator navigator_msg = buildNavigatorFromOdometry(*msg, stamp);
  updateAccelerations(navigator_msg, stamp);
  publishNavigator(navigator_msg);

  last_odom_stamp_ = stamp;
  has_odom_ = true;
}

void NavigatorNode::handleAltitude(const sensor_msgs::msg::Range::SharedPtr msg)
{
  altitude_ = msg->range;
}

void NavigatorNode::publishTfFromOdometry(const nav_msgs::msg::Odometry & odom_msg)
{
  if (!publish_tf_ || !tf_broadcaster_) {
    return;
  }

  geometry_msgs::msg::TransformStamped transform;
  transform.header = odom_msg.header;
  if (transform.header.stamp.sec == 0 && transform.header.stamp.nanosec == 0) {
    transform.header.stamp = this->now();
  }
  transform.header.frame_id = parent_frame_;
  transform.child_frame_id = child_frame_;
  transform.transform.translation.x = odom_msg.pose.pose.position.x;
  transform.transform.translation.y = odom_msg.pose.pose.position.y;
  transform.transform.translation.z = odom_msg.pose.pose.position.z;
  transform.transform.rotation = odom_msg.pose.pose.orientation;

  tf_broadcaster_->sendTransform(transform);
}

sura_msgs::msg::Navigator NavigatorNode::buildNavigatorFromOdometry(
  const nav_msgs::msg::Odometry & odom_msg,
  const rclcpp::Time & stamp)
{
  (void)stamp;

  sura_msgs::msg::Navigator navigator_msg;
  navigator_msg.position = odom_msg.pose.pose;
  navigator_msg.altitude = altitude_;

  const tf2::Matrix3x3 rotation_matrix = poseOrientationToRotation(odom_msg.pose.pose);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  rotation_matrix.getRPY(roll, pitch, yaw);
  navigator_msg.rpy.x = roll;
  navigator_msg.rpy.y = pitch;
  navigator_msg.rpy.z = yaw;

  const auto twist_source_msg = latest_twist_odom_ ? latest_twist_odom_ : nav_msgs::msg::Odometry::SharedPtr(
    new nav_msgs::msg::Odometry(odom_msg));
  const bool twist_in_body_frame = latest_twist_odom_
    ? this->get_parameter("twist_odom_twist_in_body_frame").as_bool()
    : this->get_parameter("odom_twist_in_body_frame").as_bool();
  const bool invert_angular_z = latest_twist_odom_
    ? this->get_parameter("twist_odom_invert_angular_z").as_bool()
    : this->get_parameter("odom_invert_angular_z").as_bool();

  const tf2::Vector3 body_linear = twist_in_body_frame
    ? tf2::Vector3(
        twist_source_msg->twist.twist.linear.x,
        twist_source_msg->twist.twist.linear.y,
        twist_source_msg->twist.twist.linear.z)
    : rotation_matrix.transpose() * tf2::Vector3(
        twist_source_msg->twist.twist.linear.x,
        twist_source_msg->twist.twist.linear.y,
        twist_source_msg->twist.twist.linear.z);

  tf2::Vector3 filtered_body_linear = body_linear;
  if (!has_linear_filter_state_) {
    has_linear_filter_state_ = true;
  } else {
    filtered_body_linear = lowPass(previous_body_linear_, body_linear, linear_lpf_alpha_);
  }

  const tf2::Vector3 body_angular(
    twist_source_msg->twist.twist.angular.x,
    twist_source_msg->twist.twist.angular.y,
    invert_angular_z ?
      -twist_source_msg->twist.twist.angular.z :
      twist_source_msg->twist.twist.angular.z);

  navigator_msg.body_velocity.linear.x = filtered_body_linear.x();
  navigator_msg.body_velocity.linear.y = filtered_body_linear.y();
  navigator_msg.body_velocity.linear.z = filtered_body_linear.z();
  navigator_msg.body_velocity.angular.x = body_angular.x();
  navigator_msg.body_velocity.angular.y = body_angular.y();
  navigator_msg.body_velocity.angular.z = body_angular.z();

  const tf2::Vector3 ned_linear = rotation_matrix * filtered_body_linear;
  navigator_msg.ned_velocity.linear.x = ned_linear.x();
  navigator_msg.ned_velocity.linear.y = ned_linear.y();
  navigator_msg.ned_velocity.linear.z = ned_linear.z();
  navigator_msg.ned_velocity.angular.x = body_angular.x();
  navigator_msg.ned_velocity.angular.y = body_angular.y();
  navigator_msg.ned_velocity.angular.z = body_angular.z();

  return navigator_msg;
}

void NavigatorNode::publishNavigator(const sura_msgs::msg::Navigator & navigator_msg)
{
  navigator_pub_->publish(navigator_msg);
  if (legacy_navigator_pub_) {
    legacy_navigator_pub_->publish(navigator_msg);
  }
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

  const tf2::Vector3 current_body_linear(
    navigator_msg.body_velocity.linear.x,
    navigator_msg.body_velocity.linear.y,
    navigator_msg.body_velocity.linear.z);
  const tf2::Vector3 current_body_angular(
    navigator_msg.body_velocity.angular.x,
    navigator_msg.body_velocity.angular.y,
    navigator_msg.body_velocity.angular.z);
  const tf2::Vector3 body_linear_accel =
    (current_body_linear - previous_body_linear_) / dt;
  const tf2::Vector3 body_angular_accel =
    (current_body_angular - previous_body_angular_) / dt;

  const tf2::Matrix3x3 rotation_matrix = poseOrientationToRotation(navigator_msg.position);
  const tf2::Vector3 ned_linear_accel = rotation_matrix * body_linear_accel;
  const tf2::Vector3 ned_angular_accel = rotation_matrix * body_angular_accel;

  navigator_msg.body_acceleration = toAccel(body_linear_accel, body_angular_accel);
  navigator_msg.ned_acceleration = toAccel(ned_linear_accel, ned_angular_accel);

  previous_state_stamp_ = stamp;
  previous_body_linear_ = current_body_linear;
  previous_body_angular_ = current_body_angular;
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
  publishNavigator(navigator_msg);

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
