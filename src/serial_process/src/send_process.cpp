#include "serial_data.h"
#include "geometry_msgs/msg/twist.hpp"
#include "msg_process/msg/keyboard_control.hpp"
#include <rclcpp/rclcpp.hpp> 
#include "log.h"

class SendProcess : public rclcpp::Node
{
private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<msg_process::msg::KeyboardControl>::SharedPtr keyboard_control_sub_;
    SendData serial_send_data_;

public:
    SendProcess() : Node("send_process")
    {
        this->cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 1, 
            std::bind(&SendProcess::CmdVelCallback, this, std::placeholders::_1));

        const auto control_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
        this->keyboard_control_sub_ =
            this->create_subscription<msg_process::msg::KeyboardControl>(
                "/keyboard_control", control_qos,
                std::bind(&SendProcess::KeyboardControlCallback, this, std::placeholders::_1));

        // 线速度
        this->serial_send_data_.vx = 0.0f;
        // this->serial_send_data_.vy = 0.0f;
        this->serial_send_data_.wz = 0.0f;
        this->serial_send_data_.spin_mode = 0;
        this->serial_send_data_.length_leg = 0;
    }

    ~SendProcess() = default;

    void CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        this->serial_send_data_.vx = msg->linear.x; 
        // this->serial_send_data_.vy = msg->linear.y; 
        this->serial_send_data_.wz = msg->angular.z/3.1416*180/1000.0f; // 转换为度每秒，并缩小100倍以适应协议范围
        this->WriteSerialData();
    }

    void KeyboardControlCallback(const msg_process::msg::KeyboardControl::SharedPtr msg)
    {
        this->serial_send_data_.spin_mode = msg->spin_mode;
        this->serial_send_data_.length_leg = msg->length_leg;
        this->WriteSerialData();
    }

    void WriteSerialData()
    {
        RCLCPP_INFO(
            this->get_logger(),
            "发送串口数据: vx=%.2f, wz=%.2f, spin_mode=%u, length_leg=%u",
            this->serial_send_data_.vx,
            this->serial_send_data_.wz,
            static_cast<unsigned int>(this->serial_send_data_.spin_mode),
            static_cast<unsigned int>(this->serial_send_data_.length_leg));
        serialdata.zh_write(this->serial_send_data_);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SendProcess>());
    rclcpp::shutdown();
    return 0;
}
