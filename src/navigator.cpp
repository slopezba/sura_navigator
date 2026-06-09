#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/accel.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "sura_msgs/msg/navigator.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_ros/transform_broadcaster.h"

namespace sura_navigator
{

class Navigator : public rclcpp::Node
{
public:
  Navigator()
  : Node("sura_navigator")
  {
    declare_parameter<std::string>("odom_topic", "/cirtesub/stonefish/odometry");
    declare_parameter<std::string>("altitude_topic", "/cirtesub/sensors/dvl/altitude");
    declare_parameter<std::string>("navigator_topic", "/cirtesub/navigator/navigation");
    declare_parameter<std::string>("parent_frame", "world_ned");
    declare_parameter<std::string>("child_frame", "cirtesub/base_link");
    declare_parameter<bool>("publish_tf", true);

    odom_topic_ = get_parameter("odom_topic").as_string();
    altitude_topic_ = get_parameter("altitude_topic").as_string();
    navigator_topic_ = get_parameter("navigator_topic").as_string();
    parent_frame_ = get_parameter("parent_frame").as_string();
    child_frame_ = get_parameter("child_frame").as_string();
    publish_tf_ = get_parameter("publish_tf").as_bool();

    navigator_pub_ = create_publisher<sura_msgs::msg::Navigator>(navigator_topic_, 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10, std::bind(&Navigator::handleOdometry, this, std::placeholders::_1));
    altitude_sub_ = create_subscription<sensor_msgs::msg::Range>(
      altitude_topic_, 10, std::bind(&Navigator::handleAltitude, this, std::placeholders::_1));

    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    RCLCPP_INFO(get_logger(), "SURA navigator started");
    RCLCPP_INFO(get_logger(), "Reading odometry from: %s", odom_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Reading DVL altitude from: %s", altitude_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Publishing navigator to: %s", navigator_topic_.c_str());
    if (publish_tf_) {
      RCLCPP_INFO(
        get_logger(), "Publishing TF: %s -> %s", parent_frame_.c_str(), child_frame_.c_str());
    }
  }

private:
  static double wrapAngle(double angle)
  {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  static geometry_msgs::msg::Vector3 toVector3(const tf2::Vector3 & vector)
  {
    geometry_msgs::msg::Vector3 out;
    out.x = vector.x();
    out.y = vector.y();
    out.z = vector.z();
    return out;
  }

  static geometry_msgs::msg::Accel toAccel(
    const tf2::Vector3 & linear,
    const tf2::Vector3 & angular)
  {
    geometry_msgs::msg::Accel out;
    out.linear = toVector3(linear);
    out.angular = toVector3(angular);
    return out;
  }

  void handleAltitude(const sensor_msgs::msg::Range::SharedPtr msg)
  {
    altitude_ = msg->range;
  }

  void handleOdometry(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const rclcpp::Time stamp =
      msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0 ?
      now() :
      rclcpp::Time(msg->header.stamp);

    sura_msgs::msg::Navigator navigator_msg = buildNavigator(*msg);
    updateAccelerations(navigator_msg, stamp);
    publishTf(*msg, stamp);
    navigator_pub_->publish(navigator_msg);
  }

  sura_msgs::msg::Navigator buildNavigator(const nav_msgs::msg::Odometry & odom_msg)
  {
    sura_msgs::msg::Navigator navigator_msg;
    navigator_msg.position = odom_msg.pose.pose;
    navigator_msg.altitude = altitude_;

    tf2::Quaternion orientation(
      odom_msg.pose.pose.orientation.x,
      odom_msg.pose.pose.orientation.y,
      odom_msg.pose.pose.orientation.z,
      odom_msg.pose.pose.orientation.w);
    if (orientation.length2() > 0.0) {
      orientation.normalize();
    } else {
      orientation.setRPY(0.0, 0.0, 0.0);
    }

    const tf2::Matrix3x3 rotation_matrix(orientation);
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    rotation_matrix.getRPY(roll, pitch, yaw);
    navigator_msg.rpy.x = roll;
    navigator_msg.rpy.y = pitch;
    navigator_msg.rpy.z = yaw;

    const tf2::Vector3 ned_linear(
      odom_msg.twist.twist.linear.x,
      odom_msg.twist.twist.linear.y,
      odom_msg.twist.twist.linear.z);
    const tf2::Vector3 body_linear = rotation_matrix.transpose() * ned_linear;

    const tf2::Vector3 angular(
      odom_msg.twist.twist.angular.x,
      odom_msg.twist.twist.angular.y,
      odom_msg.twist.twist.angular.z);

    navigator_msg.ned_velocity.linear = toVector3(ned_linear);
    navigator_msg.ned_velocity.angular = toVector3(angular);
    navigator_msg.body_velocity.linear = toVector3(body_linear);
    navigator_msg.body_velocity.angular = toVector3(angular);

    return navigator_msg;
  }

  void updateAccelerations(sura_msgs::msg::Navigator & navigator_msg, const rclcpp::Time & stamp)
  {
    if (!has_previous_velocity_) {
      previous_stamp_ = stamp;
      previous_body_velocity_ = navigator_msg.body_velocity;
      previous_ned_velocity_ = navigator_msg.ned_velocity;
      has_previous_velocity_ = true;
      return;
    }

    const double dt = (stamp - previous_stamp_).seconds();
    if (dt <= 0.0) {
      return;
    }

    const tf2::Vector3 body_linear_accel(
      (navigator_msg.body_velocity.linear.x - previous_body_velocity_.linear.x) / dt,
      (navigator_msg.body_velocity.linear.y - previous_body_velocity_.linear.y) / dt,
      (navigator_msg.body_velocity.linear.z - previous_body_velocity_.linear.z) / dt);
    const tf2::Vector3 body_angular_accel(
      (navigator_msg.body_velocity.angular.x - previous_body_velocity_.angular.x) / dt,
      (navigator_msg.body_velocity.angular.y - previous_body_velocity_.angular.y) / dt,
      (navigator_msg.body_velocity.angular.z - previous_body_velocity_.angular.z) / dt);

    const tf2::Vector3 ned_linear_accel(
      (navigator_msg.ned_velocity.linear.x - previous_ned_velocity_.linear.x) / dt,
      (navigator_msg.ned_velocity.linear.y - previous_ned_velocity_.linear.y) / dt,
      (navigator_msg.ned_velocity.linear.z - previous_ned_velocity_.linear.z) / dt);
    const tf2::Vector3 ned_angular_accel(
      (navigator_msg.ned_velocity.angular.x - previous_ned_velocity_.angular.x) / dt,
      (navigator_msg.ned_velocity.angular.y - previous_ned_velocity_.angular.y) / dt,
      (navigator_msg.ned_velocity.angular.z - previous_ned_velocity_.angular.z) / dt);

    navigator_msg.body_acceleration = toAccel(body_linear_accel, body_angular_accel);
    navigator_msg.ned_acceleration = toAccel(ned_linear_accel, ned_angular_accel);

    previous_stamp_ = stamp;
    previous_body_velocity_ = navigator_msg.body_velocity;
    previous_ned_velocity_ = navigator_msg.ned_velocity;
  }

  void publishTf(const nav_msgs::msg::Odometry & odom_msg, const rclcpp::Time & stamp)
  {
    if (!publish_tf_ || !tf_broadcaster_) {
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = parent_frame_;
    transform.child_frame_id = child_frame_;
    transform.transform.translation.x = odom_msg.pose.pose.position.x;
    transform.transform.translation.y = odom_msg.pose.pose.position.y;
    transform.transform.translation.z = odom_msg.pose.pose.position.z;
    transform.transform.rotation = odom_msg.pose.pose.orientation;

    tf_broadcaster_->sendTransform(transform);
  }

  std::string odom_topic_;
  std::string altitude_topic_;
  std::string navigator_topic_;
  std::string parent_frame_;
  std::string child_frame_;
  bool publish_tf_{false};
  float altitude_{0.0F};

  rclcpp::Publisher<sura_msgs::msg::Navigator>::SharedPtr navigator_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr altitude_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  bool has_previous_velocity_{false};
  rclcpp::Time previous_stamp_{0, 0, RCL_ROS_TIME};
  geometry_msgs::msg::Twist previous_body_velocity_;
  geometry_msgs::msg::Twist previous_ned_velocity_;
};

}  // namespace sura_navigator

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sura_navigator::Navigator>());
  rclcpp::shutdown();
  return 0;
}
