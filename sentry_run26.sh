#!/bin/bash

# 获取当前工作路径，确保脚本能找到 setup.bash
ODIN_WS_SETUP="$HOME/odin_ws/install/setup.bash"
BOT_WS_SETUP="$HOME/wl-sentry/install/setup.bash"

echo "正在启动机器人组件，请稍候..."

# 1. 启动 odin_ros_driver
gnome-terminal --tab --title="Odin Driver" -- bash -c "source $ODIN_WS_SETUP; ros2 launch odin_ros_driver odin1_ros2.launch.py; exec bash"

sleep 5  # 等待驱动初始化

# 2. 启动 navigation2
gnome-terminal --tab --title="Navigation2" -- bash -c "source $BOT_WS_SETUP; ros2 launch bot_navigation2 navigation2.launch.py; exec bash"

sleep 2

# 3. 启动 stair_mode_controller
gnome-terminal --tab --title="Stair Mode Controller" -- bash -c "source $BOT_WS_SETUP; ros2 launch stair_mode_controller stair_mode_controller.launch.py; exec bash"

sleep 1

# 4. 启动 serial_process
gnome-terminal --tab --title="Serial Process" -- bash -c "source $BOT_WS_SETUP; ros2 launch serial_process serial.launch.py; exec bash"

echo "所有任务已在独立窗口中启动。"