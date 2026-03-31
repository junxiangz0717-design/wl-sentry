#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>

#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

namespace
{

enum class MotionMode
{
  kNone,
  kForward,
  kBackward,
  kLeft,
  kRight,
  kRotateLeft,
  kRotateRight
};

class TerminalGuard
{
public:
  TerminalGuard()
  {
    terminal_fd_ = resolveTerminalFd(owns_terminal_fd_);

    if (tcgetattr(terminal_fd_, &original_termios_) == -1) {
      throw std::runtime_error("Failed to read terminal attributes: " + lastErrorString());
    }

    termios raw_termios = original_termios_;
    raw_termios.c_lflag &= ~(ICANON | ECHO);
    raw_termios.c_cc[VMIN] = 0;
    raw_termios.c_cc[VTIME] = 0;

    if (tcsetattr(terminal_fd_, TCSANOW, &raw_termios) == -1) {
      throw std::runtime_error("Failed to configure terminal for raw input: " + lastErrorString());
    }
  }

  ~TerminalGuard()
  {
    tcsetattr(terminal_fd_, TCSANOW, &original_termios_);
    if (owns_terminal_fd_) {
      close(terminal_fd_);
    }
  }

  TerminalGuard(const TerminalGuard &) = delete;
  TerminalGuard & operator=(const TerminalGuard &) = delete;

  int fd() const
  {
    return terminal_fd_;
  }

private:
  static int resolveTerminalFd(bool & owns_terminal_fd)
  {
    if (isatty(STDIN_FILENO)) {
      owns_terminal_fd = false;
      return STDIN_FILENO;
    }

    const int tty_fd = open("/dev/tty", O_RDWR);
    if (tty_fd == -1) {
      throw std::runtime_error(
        "Failed to access /dev/tty. Please run this node from an interactive terminal.");
    }

    owns_terminal_fd = true;
    return tty_fd;
  }

  static std::string lastErrorString()
  {
    return std::strerror(errno);
  }

  termios original_termios_{};
  int terminal_fd_{STDIN_FILENO};
  bool owns_terminal_fd_{false};
};

class KeyboardCmdVelController : public rclcpp::Node
{
public:
  KeyboardCmdVelController()
  : Node("keyboard_cmd_vel_controller")
  {
    linear_step_ = declare_parameter("linear_step", 0.1);
    angular_step_ = declare_parameter("angular_step", 0.1);
    max_linear_speed_ = declare_parameter("max_linear_speed", 1.0);
    max_angular_speed_ = declare_parameter("max_angular_speed", 2.0);
    publish_rate_ = declare_parameter("publish_rate", 10.0);

    publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    publish_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_),
      [this]() { publishCurrentTwist(); });

    printHelp();
    publishCurrentTwist();
  }

  void handleKey(char key)
  {
    switch (key) {
      case 'w':
      case 'W':
        updateLinearX(+linear_step_);
        break;
      case 's':
      case 'S':
        updateLinearX(-linear_step_);
        break;
      case 'a':
      case 'A':
        updateLinearY(+linear_step_);
        break;
      case 'd':
      case 'D':
        updateLinearY(-linear_step_);
        break;
      case 'q':
      case 'Q':
        updateAngularZ(+angular_step_);
        break;
      case 'e':
      case 'E':
        updateAngularZ(-angular_step_);
        break;
      case ' ':
        brake();
        break;
      default:
        return;
    }

    publishCurrentTwist();
    printCurrentState(key);
  }

  void brake()
  {
    current_twist_ = geometry_msgs::msg::Twist();
    active_mode_ = MotionMode::kNone;
  }

  void stopAndPublish()
  {
    brake();
    publishCurrentTwist();
  }

private:
  void updateLinearX(double delta)
  {
    zeroAll();
    const double base_speed =
      active_mode_ == (delta > 0.0 ? MotionMode::kForward : MotionMode::kBackward) ?
      last_speed_before_reset_ : 0.0;
    current_twist_.linear.x = clamp(base_speed + delta);
    active_mode_ = delta > 0.0 ? MotionMode::kForward : MotionMode::kBackward;
  }

  void updateLinearY(double delta)
  {
    zeroAll();
    const double base_speed =
      active_mode_ == (delta > 0.0 ? MotionMode::kLeft : MotionMode::kRight) ?
      last_speed_before_reset_ : 0.0;
    current_twist_.linear.y = clamp(base_speed + delta);
    active_mode_ = delta > 0.0 ? MotionMode::kLeft : MotionMode::kRight;
  }

  void updateAngularZ(double delta)
  {
    zeroAll();
    const double base_speed =
      active_mode_ == (delta > 0.0 ? MotionMode::kRotateLeft : MotionMode::kRotateRight) ?
      last_speed_before_reset_ : 0.0;
    current_twist_.angular.z = clamp(base_speed + delta, max_angular_speed_);
    active_mode_ = delta > 0.0 ? MotionMode::kRotateLeft : MotionMode::kRotateRight;
  }

  void zeroAll()
  {
    last_speed_before_reset_ = 0.0;
    if (active_mode_ == MotionMode::kForward || active_mode_ == MotionMode::kBackward) {
      last_speed_before_reset_ = current_twist_.linear.x;
    } else if (active_mode_ == MotionMode::kLeft || active_mode_ == MotionMode::kRight) {
      last_speed_before_reset_ = current_twist_.linear.y;
    } else if (
      active_mode_ == MotionMode::kRotateLeft || active_mode_ == MotionMode::kRotateRight)
    {
      last_speed_before_reset_ = current_twist_.angular.z;
    }

    current_twist_ = geometry_msgs::msg::Twist();
  }

  double clamp(double value, double limit = 0.0) const
  {
    const double speed_limit = limit > 0.0 ? limit : max_linear_speed_;
    return std::clamp(value, -speed_limit, speed_limit);
  }

  void publishCurrentTwist()
  {
    publisher_->publish(current_twist_);
  }

  void printHelp() const
  {
    std::cout << "\nKeyboard /cmd_vel controller started.\n"
              << "Controls:\n"
              << "  w/s : forward/backward, step " << linear_step_ << " m/s\n"
              << "  a/d : left/right translation, step " << linear_step_ << " m/s\n"
              << "  q/e : left/right rotation, step " << angular_step_ << " rad/s\n"
              << "  space: brake\n"
              << "  Ctrl+C: exit\n\n"
              << "Only one direction is active at a time.\n" << std::endl;
  }

  void printCurrentState(char key) const
  {
    std::cout << "key[" << (key == ' ' ? "space" : std::string(1, key))
              << "] -> linear.x: " << current_twist_.linear.x
              << ", linear.y: " << current_twist_.linear.y
              << ", angular.z: " << current_twist_.angular.z << std::endl;
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  geometry_msgs::msg::Twist current_twist_;

  double linear_step_{0.1};
  double angular_step_{0.1};
  double max_linear_speed_{1.0};
  double max_angular_speed_{2.0};
  double publish_rate_{10.0};
  double last_speed_before_reset_{0.0};
  MotionMode active_mode_{MotionMode::kNone};
};

bool readKey(int terminal_fd, char & key)
{
  fd_set read_set;
  FD_ZERO(&read_set);
  FD_SET(terminal_fd, &read_set);

  timeval timeout{};
  timeout.tv_sec = 0;
  timeout.tv_usec = 100000;

  const int ready = select(terminal_fd + 1, &read_set, nullptr, nullptr, &timeout);
  if (ready > 0 && FD_ISSET(terminal_fd, &read_set)) {
    return ::read(terminal_fd, &key, 1) > 0;
  }

  if (ready < 0 && errno != EINTR) {
    std::cerr << "select() failed: " << std::strerror(errno) << std::endl;
  }

  return false;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    TerminalGuard terminal_guard;
    auto node = std::make_shared<KeyboardCmdVelController>();

    while (rclcpp::ok()) {
      rclcpp::spin_some(node);

      char key = '\0';
      if (readKey(terminal_guard.fd(), key)) {
        node->handleKey(key);
      }
    }

    node->stopAndPublish();
  } catch (const std::exception & error) {
    std::cerr << error.what() << std::endl;
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
