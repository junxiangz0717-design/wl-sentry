#!/bin/bash

# 如果遇到错误则停止执行
set -e

echo ">>> 正在更新软件源..."
sudo apt update

echo ">>> 正在安装 ROS 2 Humble 相关软件包..."

# 1. 安装仿真环境 (Gazebo)
sudo apt install -y ros-humble-gazebo-*

# 2. 安装 SLAM 建图组件 (Cartographer)
sudo apt install -y \
    ros-humble-cartographer \
    ros-humble-cartographer-ros

# 3. 安装导航框架 (Nav2)
sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup

# 4. 安装机器人描述与状态发布工具
sudo apt install -y \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    ros-humble-rviz2

echo ">>> 安装完成！"
