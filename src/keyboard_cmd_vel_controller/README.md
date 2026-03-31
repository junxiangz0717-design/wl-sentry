# keyboard_cmd_vel_controller

一个基于 ROS2 Humble 和 C++ 的键盘控制功能包，用于向 `/cmd_vel` 发布 `geometry_msgs/msg/Twist` 控制小车运动。

## 功能说明

- `w` / `s`：前进 / 后退
- `a` / `d`：左移 / 右移
- `q` / `e`：左旋 / 右旋
- `space`：立即刹车，所有速度清零
- 同一个方向连续按键会持续累加速度
- 任意时刻只会有一个方向生效
- 切换到其他方向时，之前的线速度和角速度全部清零

默认步进参数：

- 线速度步进：`0.1 m/s`
- 角速度步进：`0.1 rad/s`
- 最大发布线速度：`1.0 m/s`
- 最大发布角速度：`2.0 rad/s`

## 编译

```bash
cd ~/test_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select keyboard_cmd_vel_controller
source install/setup.bash
```

## 运行

### 方式一：直接运行节点

```bash
ros2 run keyboard_cmd_vel_controller keyboard_cmd_vel_node
```

### 方式二：使用 launch 启动

默认参数已经直接写在 launch 文件的 `parameters` 里：

```bash
ros2 launch keyboard_cmd_vel_controller keyboard_cmd_vel_controller.launch.py
```

这个节点会优先从当前终端读取键盘；如果标准输入不可用，会自动回退到 `/dev/tty`。
因此从交互式终端执行 `ros2 launch` 也可以正常控制。
