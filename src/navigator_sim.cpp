#include "sura_navigator/navigator_sim.hpp"

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

NavigatorSim::NavigatorSim()
: Node("navigator_sim"),
  tf_buffer_(this->get_clock())
{
  this->declare_parameter<std::string>("parent_frame", "world_ned");
  this->declare_parameter<std::string>("child_frame", "sura/base_link");
  this->declare_parameter<std::string>("navigator_topic", "/sura/navigator/navigation");
  this->declare_parameter<std::string>("altitude_topic", "/sura/sensors/dvl/altitude");
  this->declare_parameter<double>("publish_rate", 50.0);

  parent_frame_ = this->get_parameter("parent_frame").as_string();
  child_frame_ = this->get_parameter("child_frame").as_string();
  const std::string navigator_topic =
    this->get_parameter("navigator_topic").as_string();
  const std::string altitude_topic =
    this->get_parameter("altitude_topic").as_string();
  const double publish_rate =
    std::max(1.0, this->get_parameter("publish_rate").as_double());

  navigator_pub_ = this->create_publisher<sura_msgs::msg::Navigator>(
    navigator_topic, 10);
  altitude_sub_ = this->create_subscription<sensor_msgs::msg::Range>(
    altitude_topic, 10, std::bind(&NavigatorSim::altitudeCallback, this, std::placeholders::_1));

  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_, this, false);

  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / publish_rate),
    std::bind(&NavigatorSim::publishFromTf, this));

  RCLCPP_INFO(this->get_logger(), "SURA navigator simulation adapter started");
  RCLCPP_INFO(
    this->get_logger(), "Reading TF: %s -> %s", parent_frame_.c_str(), child_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "Publishing: %s", navigator_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "Reading altitude: %s", altitude_topic.c_str());
}

double NavigatorSim::wrapAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

void NavigatorSim::publishFromTf()
{
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

  sura_msgs::msg::Navigator navigator_msg;

  navigator_msg.position.position.x = transform.transform.translation.x;
  navigator_msg.position.position.y = transform.transform.translation.y;
  navigator_msg.position.position.z = transform.transform.translation.z;
  navigator_msg.position.orientation = transform.transform.rotation;
  navigator_msg.altitude = altitude_;

  tf2::Quaternion q(
    transform.transform.rotation.x,
    transform.transform.rotation.y,
    transform.transform.rotation.z,
    transform.transform.rotation.w);

  tf2::Matrix3x3 rotation_matrix(q);

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  rotation_matrix.getRPY(roll, pitch, yaw);

  navigator_msg.rpy.x = roll;
  navigator_msg.rpy.y = pitch;
  navigator_msg.rpy.z = yaw;

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

      navigator_msg.ned_velocity.linear.x = vx_ned;
      navigator_msg.ned_velocity.linear.y = vy_ned;
      navigator_msg.ned_velocity.linear.z = vz_ned;

      tf2::Vector3 v_ned(vx_ned, vy_ned, vz_ned);
      tf2::Vector3 v_body = rotation_matrix.transpose() * v_ned;

      navigator_msg.body_velocity.linear.x = v_body.x();
      navigator_msg.body_velocity.linear.y = v_body.y();
      navigator_msg.body_velocity.linear.z = v_body.z();

      const double roll_rate = wrapAngle(roll - previous_roll_) / dt;
      const double pitch_rate = wrapAngle(pitch - previous_pitch_) / dt;
      const double yaw_rate = wrapAngle(yaw - previous_yaw_) / dt;

      navigator_msg.body_velocity.angular.x = roll_rate;
      navigator_msg.body_velocity.angular.y = pitch_rate;
      navigator_msg.body_velocity.angular.z = yaw_rate;

      navigator_msg.ned_velocity.angular.x = roll_rate;
      navigator_msg.ned_velocity.angular.y = pitch_rate;
      navigator_msg.ned_velocity.angular.z = yaw_rate;

      if (has_previous_velocity_) {
        navigator_msg.body_acceleration.linear.x =
          (navigator_msg.body_velocity.linear.x - previous_body_velocity_.linear.x) / dt;
        navigator_msg.body_acceleration.linear.y =
          (navigator_msg.body_velocity.linear.y - previous_body_velocity_.linear.y) / dt;
        navigator_msg.body_acceleration.linear.z =
          (navigator_msg.body_velocity.linear.z - previous_body_velocity_.linear.z) / dt;

        navigator_msg.ned_acceleration.linear.x =
          (navigator_msg.ned_velocity.linear.x - previous_ned_velocity_.linear.x) / dt;
        navigator_msg.ned_acceleration.linear.y =
          (navigator_msg.ned_velocity.linear.y - previous_ned_velocity_.linear.y) / dt;
        navigator_msg.ned_acceleration.linear.z =
          (navigator_msg.ned_velocity.linear.z - previous_ned_velocity_.linear.z) / dt;

        navigator_msg.body_acceleration.angular.x =
          (navigator_msg.body_velocity.angular.x - previous_body_velocity_.angular.x) / dt;
        navigator_msg.body_acceleration.angular.y =
          (navigator_msg.body_velocity.angular.y - previous_body_velocity_.angular.y) / dt;
        navigator_msg.body_acceleration.angular.z =
          (navigator_msg.body_velocity.angular.z - previous_body_velocity_.angular.z) / dt;

        navigator_msg.ned_acceleration.angular.x =
          (navigator_msg.ned_velocity.angular.x - previous_ned_velocity_.angular.x) / dt;
        navigator_msg.ned_acceleration.angular.y =
          (navigator_msg.ned_velocity.angular.y - previous_ned_velocity_.angular.y) / dt;
        navigator_msg.ned_acceleration.angular.z =
          (navigator_msg.ned_velocity.angular.z - previous_ned_velocity_.angular.z) / dt;
      }
    }
  }

  previous_transform_ = transform;
  previous_roll_ = roll;
  previous_pitch_ = pitch;
  previous_yaw_ = yaw;
  previous_body_velocity_ = navigator_msg.body_velocity;
  previous_ned_velocity_ = navigator_msg.ned_velocity;
  has_previous_transform_ = true;
  has_previous_velocity_ = true;

  navigator_pub_->publish(navigator_msg);
}

void NavigatorSim::altitudeCallback(const sensor_msgs::msg::Range::SharedPtr msg)
{
  altitude_ = msg->range;
}

}  // namespace sura_navigator

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sura_navigator::NavigatorSim>());
  rclcpp::shutdown();
  return 0;
}
