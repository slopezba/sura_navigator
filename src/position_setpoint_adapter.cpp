#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_listener.h"

namespace sura_navigator
{

class PositionSetpointAdapter : public rclcpp::Node
{
public:
  PositionSetpointAdapter()
  : Node("position_setpoint_adapter"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    this->declare_parameter<std::string>(
      "input_topic", "body_position/setpoint_world");
    this->declare_parameter<std::string>(
      "output_topic", "controller/body_position/setpoint");
    this->declare_parameter<std::string>("target_frame", "blueboat/map");
    this->declare_parameter<double>("transform_timeout", 0.2);

    input_topic_ = this->get_parameter("input_topic").as_string();
    output_topic_ = this->get_parameter("output_topic").as_string();
    target_frame_ = this->get_parameter("target_frame").as_string();
    transform_timeout_ = this->get_parameter("transform_timeout").as_double();

    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(),
      this->get_node_timers_interface());
    tf_buffer_.setCreateTimerInterface(timer_interface);

    publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(output_topic_, 10);
    subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      input_topic_,
      10,
      std::bind(&PositionSetpointAdapter::onSetpoint, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "Position setpoint adapter started. input=%s output=%s target_frame=%s",
      input_topic_.c_str(),
      output_topic_.c_str(),
      target_frame_.c_str());
  }

private:
  void onSetpoint(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    geometry_msgs::msg::PoseStamped outgoing = *msg;

    if (outgoing.header.frame_id.empty()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Received setpoint without frame_id. Assuming target frame '%s'.",
        target_frame_.c_str());
      outgoing.header.frame_id = target_frame_;
    }

    if (outgoing.header.frame_id == target_frame_) {
      publisher_->publish(outgoing);
      return;
    }

    try {
      geometry_msgs::msg::PoseStamped transformed;
      tf_buffer_.transform(
        outgoing,
        transformed,
        target_frame_,
        tf2::durationFromSec(transform_timeout_));
      publisher_->publish(transformed);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Failed to transform setpoint from '%s' to '%s': %s",
        outgoing.header.frame_id.c_str(),
        target_frame_.c_str(),
        ex.what());
    }
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string target_frame_;
  double transform_timeout_{0.2};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
};

}  // namespace sura_navigator

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sura_navigator::PositionSetpointAdapter>());
  rclcpp::shutdown();
  return 0;
}
