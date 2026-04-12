#!/bin/bash

# 获取当前工作路径，确保脚本能找到 setup.bash
ODIN_WS_SETUP="$HOME/Odin/install/setup.bash"
BOT_WS_SETUP="$HOME/26-vision-sentry/Navagation2/install/setup.bash"
Serial_WS_SETUP="$HOME/26-vision-sentry/Communication/install/setup.bash"

echo "正在启动机器人组件，请稍候..."

# 1. 启动 odin_ros_driver 
gnome-terminal --window --title="Odin Driver" -- bash -c "source $ODIN_WS_SETUP;
ros2 launch odin_ros_driver odin1_ros2.launch.py; exec bash"

sleep 5  # 等待驱动初始化

# 2. 启动 virtual_head_node
gnome-terminal --tab --title="Virtual Head" -- bash -c "source $BOT_WS_SETUP;
source $Serial_WS_SETUP;
ros2 run virtual_head_pkg virtual_head_node; exec bash"

sleep 1

# # 3. 启动 terrain_analysis
# gnome-terminal --tab --title="Terrain Analysis" -- bash -c "source $BOT_WS_SETUP;
# ros2 launch terrain_analysis terrain_analysis.launch.py; exec bash"
# 3. 启动 terrain_analysis
gnome-terminal --tab --title="Terrain Analysis" -- bash -c "source $BOT_WS_SETUP;
ros2 launch cloud_filter_pkg cloud_filter.launch.py ; exec bash"

# 4. 启动 pointcloud_to_laserscan
gnome-terminal --tab --title="PointCloud to LaserScan" -- bash -c "source $BOT_WS_SETUP;
ros2 launch pointcloud_to_laserscan sample_pointcloud_to_laserscan_launch.py; exec bash"

sleep 1

# 5. 启动 navigation2
gnome-terminal --tab --title="Navigation2" -- bash -c "source $BOT_WS_SETUP;
ros2 launch bot_navigation2 navigation2.launch.py; exec bash"

sleep 2

# 4. 启动 serial_process
# gnome-terminal --tab --title="Serial Process" -- bash -c "source $Serial_WS_SETUP; 
# ros2 launch serial_process serial.launch.py; exec bash"

echo "所有任务已在独立窗口中启动。"
