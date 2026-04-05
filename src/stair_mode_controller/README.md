# stair_mode_controller

根据 `map -> base_link` 的位姿判断机器人是否进入指定矩形区域，且当前朝向是否落在目标角度容差内。

- 满足区域和朝向条件时，发布 `length_leg = 2`，进入上台阶模式
- 不满足条件时，发布 `length_leg = 1`，保持平地运动模式

发布话题默认使用 `/keyboard_control`，消息类型为 `msg_process/msg/KeyboardControl`。

## 参数

- `map_frame` / `base_frame`：TF 查询的父子坐标系，默认 `map` 和 `base_link`
- `x_min` / `x_max` / `y_min` / `y_max`：矩形区域边界
- `target_yaw_deg`：目标朝向，单位度
- `yaw_tolerance_deg`：允许朝向误差，单位度
- `flat_length_leg`：平地模式腿长，默认 `1`
- `stair_length_leg`：上台阶模式腿长，默认 `2`
- `spin_mode`：透传到 `KeyboardControl` 的 `spin_mode`，默认 `0`

## 运行

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to stair_mode_controller
source install/setup.bash
ros2 launch stair_mode_controller stair_mode_controller.launch.py
```
