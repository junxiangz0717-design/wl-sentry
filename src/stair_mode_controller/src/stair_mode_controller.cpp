#include <msg_process/msg/keyboard_control.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

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
  : Node("stair_mode_controller")
  {
    odom_topic_ = declare_parameter("odom_topic", "/odin/odometry_high");
    control_topic_ = declare_parameter("control_topic", "/keyboard_control");

    map_to_odom_x_ = declare_parameter("map_to_odom_x", 0.0);
    map_to_odom_y_ = declare_parameter("map_to_odom_y", 0.0);
    map_to_odom_yaw_deg_ = declare_parameter("map_to_odom_yaw_deg", 0.0);

    x_min_ = declare_parameter("x_min", 0.0);
    x_max_ = declare_parameter("x_max", 1.0);
    y_min_ = declare_parameter("y_min", 0.0);
    y_max_ = declare_parameter("y_max", 1.0);
    target_yaw_deg_ = declare_parameter("target_yaw_deg", 0.0);
    yaw_tolerance_deg_ = declare_parameter("yaw_tolerance_deg", 10.0);

    flat_length_leg_ = declare_parameter("flat_length_leg", 1);
    stair_length_leg_ = declare_parameter("stair_length_leg", 2);
    spin_mode_ = declare_parameter("spin_mode", 0);

    const auto control_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    control_publisher_ =
      create_publisher<msg_process::msg::KeyboardControl>(control_topic_, control_qos);

    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&StairModeController::odometryCallback, this, std::placeholders::_1));

    publishCurrentMode(flat_length_leg_, "startup");
  }

private:
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const double odom_x = msg->pose.pose.position.x;
    const double odom_y = msg->pose.pose.position.y;
    const double odom_yaw = yawFromQuaternion(msg->pose.pose.orientation);

    const double map_to_odom_yaw = degToRad(map_to_odom_yaw_deg_);
    const double cos_yaw = std::cos(map_to_odom_yaw);
    const double sin_yaw = std::sin(map_to_odom_yaw);

    const double map_x = map_to_odom_x_ + cos_yaw * odom_x - sin_yaw * odom_y;
    const double map_y = map_to_odom_y_ + sin_yaw * odom_x + cos_yaw * odom_y;
    const double map_yaw = normalizeAngle(map_to_odom_yaw + odom_yaw);

    const bool inside_region =
      map_x >= x_min_ && map_x <= x_max_ &&
      map_y >= y_min_ && map_y <= y_max_;

    const double yaw_error =
      normalizeAngle(map_yaw - degToRad(target_yaw_deg_));
    const bool heading_match =
      std::abs(radToDeg(yaw_error)) <= yaw_tolerance_deg_;

    const int desired_length_leg =
      inside_region && heading_match ? stair_length_leg_ : flat_length_leg_;

    publishCurrentMode(
      desired_length_leg,
      inside_region,
      heading_match,
      odom_x,
      odom_y,
      map_x,
      map_y,
      map_yaw,
      yaw_error);
  }

  void publishCurrentMode(
    int desired_length_leg,
    bool inside_region,
    bool heading_match,
    double odom_x,
    double odom_y,
    double map_x,
    double map_y,
    double map_yaw,
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
      "length_leg switched to %d, odom_position=(%.3f, %.3f), map_position=(%.3f, %.3f), map_yaw=%.1f deg, yaw_error=%.1f deg, inside_region=%s, heading_match=%s",
      desired_length_leg,
      odom_x,
      odom_y,
      map_x,
      map_y,
      radToDeg(map_yaw),
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

  rclcpp::Publisher<msg_process::msg::KeyboardControl>::SharedPtr control_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;

  std::string odom_topic_;
  std::string control_topic_;

  double map_to_odom_x_{0.0};
  double map_to_odom_y_{0.0};
  double map_to_odom_yaw_deg_{0.0};
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
