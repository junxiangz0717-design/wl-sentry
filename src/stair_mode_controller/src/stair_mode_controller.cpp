#include <algorithm>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <msg_process/msg/keyboard_control.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

namespace
{

constexpr double kPi = 3.14159265358979323846;

double normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double radToDeg(double angle)
{
  return angle * 180.0 / kPi;
}

double degToRad(double angle)
{
  return angle * kPi / 180.0;
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & quaternion_msg)
{
  tf2::Quaternion quaternion(
    quaternion_msg.x,
    quaternion_msg.y,
    quaternion_msg.z,
    quaternion_msg.w);

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
  return yaw;
}

class StairModeController : public rclcpp::Node
{
public:
  StairModeController()
  : Node("stair_mode_controller"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    map_frame_ = declare_parameter("map_frame", "map");
    base_frame_ = declare_parameter("base_frame", "base_link");
    control_topic_ = declare_parameter("control_topic", "/keyboard_control");
    check_rate_ = declare_parameter("check_rate", 10.0);

    x_min_ = declare_parameter("x_min", 0.0);
    x_max_ = declare_parameter("x_max", 1.0);
    y_min_ = declare_parameter("y_min", 0.0);
    y_max_ = declare_parameter("y_max", 1.0);
    target_yaw_deg_ = declare_parameter("target_yaw_deg", 0.0);
    yaw_tolerance_deg_ = declare_parameter("yaw_tolerance_deg", 10.0);

    flat_length_leg_ = declare_parameter("flat_length_leg", 1);
    stair_length_leg_ = declare_parameter("stair_length_leg", 2);
    spin_mode_ = declare_parameter("spin_mode", 0);

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    control_publisher_ =
      create_publisher<msg_process::msg::KeyboardControl>(control_topic_, qos);

    publishCurrentMode(flat_length_leg_, "startup");

    const auto period = std::chrono::duration<double>(1.0 / std::max(check_rate_, 1.0));
    timer_ = create_wall_timer(period, std::bind(&StairModeController::updateMode, this));
  }

private:
  void updateMode()
  {
    geometry_msgs::msg::TransformStamped transform;

    try {
      transform = tf_buffer_.lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Failed to lookup transform %s -> %s: %s",
        map_frame_.c_str(), base_frame_.c_str(), ex.what());
      publishCurrentMode(flat_length_leg_, "tf_unavailable");
      return;
    }

    const double x = transform.transform.translation.x;
    const double y = transform.transform.translation.y;
    const double yaw = yawFromQuaternion(transform.transform.rotation);

    const bool inside_region =
      x >= x_min_ && x <= x_max_ &&
      y >= y_min_ && y <= y_max_;

    const double yaw_error =
      normalizeAngle(yaw - degToRad(target_yaw_deg_));
    const bool heading_match =
      std::abs(radToDeg(yaw_error)) <= yaw_tolerance_deg_;

    const int desired_length_leg =
      inside_region && heading_match ? stair_length_leg_ : flat_length_leg_;

    publishCurrentMode(desired_length_leg, inside_region, heading_match, x, y, yaw, yaw_error);
  }

  void publishCurrentMode(
    int desired_length_leg,
    bool inside_region,
    bool heading_match,
    double x,
    double y,
    double yaw,
    double yaw_error)
  {
    msg_process::msg::KeyboardControl control_msg;
    control_msg.spin_mode = static_cast<uint8_t>(spin_mode_);
    control_msg.length_leg = static_cast<uint8_t>(desired_length_leg);
    control_publisher_->publish(control_msg);

    if (desired_length_leg == last_length_leg_) {
      return;
    }

    last_length_leg_ = desired_length_leg;
    RCLCPP_INFO(
      get_logger(),
      "length_leg switched to %d, position=(%.3f, %.3f), yaw=%.1f deg, yaw_error=%.1f deg, inside_region=%s, heading_match=%s",
      desired_length_leg,
      x,
      y,
      radToDeg(yaw),
      radToDeg(yaw_error),
      inside_region ? "true" : "false",
      heading_match ? "true" : "false");
  }

  void publishCurrentMode(int desired_length_leg, const std::string & reason)
  {
    msg_process::msg::KeyboardControl control_msg;
    control_msg.spin_mode = static_cast<uint8_t>(spin_mode_);
    control_msg.length_leg = static_cast<uint8_t>(desired_length_leg);
    control_publisher_->publish(control_msg);

    if (desired_length_leg == last_length_leg_ && reason == last_reason_) {
      return;
    }

    last_length_leg_ = desired_length_leg;
    last_reason_ = reason;
    RCLCPP_INFO(
      get_logger(),
      "length_leg set to %d (%s)",
      desired_length_leg,
      reason.c_str());
  }

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Publisher<msg_process::msg::KeyboardControl>::SharedPtr control_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string map_frame_;
  std::string base_frame_;
  std::string control_topic_;

  double check_rate_{10.0};
  double x_min_{0.0};
  double x_max_{1.0};
  double y_min_{0.0};
  double y_max_{1.0};
  double target_yaw_deg_{0.0};
  double yaw_tolerance_deg_{10.0};

  int flat_length_leg_{1};
  int stair_length_leg_{2};
  int spin_mode_{0};
  int last_length_leg_{-1};
  std::string last_reason_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StairModeController>());
  rclcpp::shutdown();
  return 0;
}
