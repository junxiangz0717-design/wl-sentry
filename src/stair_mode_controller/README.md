# stair_mode_controller

直接订阅高频里程计 `/odin/odometry_high`，使用静态 `map -> odom` 补偿将里程计位姿换算到 `map` 坐标系，再判断机器人是否进入指定矩形区域且朝向满足要求。

- 满足区域和朝向条件时，发布 `length_leg = 2`，进入上台阶模式
- 不满足条件时，发布 `length_leg = 1`，保持平地运动模式
- 不再依赖 TF 查询、定时器轮询或速度前瞻补偿

发布话题默认使用 `/keyboard_control`，消息类型为 `msg_process/msg/KeyboardControl`。

## 参数

- `odom_topic`：高频里程计话题，默认 `/odin/odometry_high`
- `control_topic`：底盘控制话题，默认 `/keyboard_control`
- `map_to_odom_x` / `map_to_odom_y` / `map_to_odom_yaw_deg`：静态 `map -> odom` 补偿，语义与静态 TF 发布一致，用于把 odom 位姿换算到 map
- `x_min` / `x_max` / `y_min` / `y_max`：矩形区域边界，单位米，定义在 `map` 坐标系
- `target_yaw_deg`：目标朝向，单位度，定义在 `map` 坐标系
- `yaw_tolerance_deg`：允许朝向误差，单位度
- `flat_length_leg`：平地模式腿长，默认 `1`
- `stair_length_leg`：上台阶模式腿长，默认 `2`
- `spin_mode`：透传到 `KeyboardControl` 的 `spin_mode`，默认 `0`

位姿换算公式如下：

```text
map_position = T_map_odom * odom_position
map_yaw = map_to_odom_yaw + odom_yaw
```

其中 `T_map_odom` 由 `map_to_odom_x`、`map_to_odom_y`、`map_to_odom_yaw_deg` 给出。

## 运行

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to stair_mode_controller
source install/setup.bash
ros2 launch stair_mode_controller stair_mode_controller.launch.py
```
