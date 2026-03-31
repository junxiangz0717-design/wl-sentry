/**
 * @file car_control.cpp
 * @author 张俊翔(junxiangz0717@gmail.com)
 * @brief  读取速度及决策信息，发送给串口话题控制实车的状态
 * @date 2026-03-07
 * @copyright Copyright SCUT RobotLab(c) 2026
 */

#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int8.hpp"

#include "msg_process/msg/decison_send_data.hpp"
#include "msg_process/msg/send_data.hpp"

using namespace std::chrono_literals;

/**
 * @class CarControlNode
 * @brief 车辆控制节点类，负责订阅各方决策与速度指令，并以固定频率向串口发送打包数据
 */
class CarControlNode : public rclcpp::Node
{
public:
    /**
     * @brief 构造函数，初始化发布者、订阅者和定时器
     */
    CarControlNode() : Node("car_control")
    {
        // 1. 初始化发布者 (发布给串口)
        serial_pub_ = this->create_publisher<msg_process::msg::SendData>("/serial_send_data", 10);

        // 2. 初始化订阅者
        sub_decision_ = this->create_subscription<msg_process::msg::DecisonSendData>(
            "/decision_send_data", 10, 
            std::bind(&CarControlNode::decision_callback, this, std::placeholders::_1));

        sub_speed_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_virtual", 1, 
            std::bind(&CarControlNode::speed_callback, this, std::placeholders::_1));

        // 3. 初始化定时器，90Hz 频率触发发布
        timer_ = this->create_wall_timer(
            11ms, std::bind(&CarControlNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Car control node has been started.");
    }

private:
    /**
     * @brief 裁判系统及自主决策数据回调函数
     * @param msg 接收到的决策数据
     */
    void decision_callback(const msg_process::msg::DecisonSendData::SharedPtr msg)
    {
        serial_send_datas_.tripod_spin = msg->tripod_spin;
        serial_send_datas_.spin_mode = msg -> spin_mode;
        serial_send_datas_.decision_yaw_az = msg->decision_yaw_az;
        serial_send_datas_.style = msg->style;
        serial_send_datas_.pitch_mode = msg->pitch_mode;
        serial_send_datas_.attack_rune = msg->attack_rune;
        
        // 哨兵裁判系统自主决策数据
        serial_send_datas_.confirm_respawn = msg->confirm_respawn;
        serial_send_datas_.buy_respawn = msg->buy_respawn;
        serial_send_datas_.home_buy_ammo = msg->home_buy_ammo;
        serial_send_datas_.remote_buy_ammo_times = msg->remote_buy_ammo_times;
        serial_send_datas_.remote_buy_hp_times = msg->remote_buy_hp_times;
        serial_send_datas_.posture = msg->posture;
    }

    /**
     * @brief 虚拟速度指令回调函数
     * @param msg 接收到的Twist速度指令
     */
    void speed_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        serial_send_datas_.vx = msg->linear.x;
        serial_send_datas_.vy = msg->linear.y;
    }

    /**
     * @brief 路径规划旋转模式回调函数
     * @param msg 接收到的旋转模式指令
     */
    void spin_mode_callback(const std_msgs::msg::Int8::SharedPtr msg)
    {
        serial_send_datas_.spin_mode = msg->data;
    }

    /**
     * @brief 定时器回调函数，以固定频率将打包好的数据发布出去
     */
    void timer_callback()
    {
        serial_pub_->publish(serial_send_datas_);
    }

    // ROS2 对象声明
    rclcpp::Publisher<msg_process::msg::SendData>::SharedPtr serial_pub_;
    rclcpp::Subscription<msg_process::msg::DecisonSendData>::SharedPtr sub_decision_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_speed_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr spin_mode_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // 内部缓存的串口发送数据
    msg_process::msg::SendData serial_send_datas_;
};

int main(int argc, char **argv)
{
    // 初始化 ROS2 节点环境
    rclcpp::init(argc, argv);
    
    // 创建节点并开始自旋处理回调
    auto node = std::make_shared<CarControlNode>();
    rclcpp::spin(node);
    
    // 退出前清理
    rclcpp::shutdown();
    return 0;
}