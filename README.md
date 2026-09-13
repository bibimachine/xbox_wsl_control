# xbox_control — Xbox 手柄 ROS 2 工作区

三个独立包：

- **`src/gamepad_msgs`** — 语义化手柄状态消息包（`gamepad_msgs/GamepadState`），定义"设备原始值 -> 通用语义"映射后的标准状态。
- **`src/xbox_reader`** — 手柄输入读取：`joy_node` 驱动手柄发布 `/joy`，`joy_reader` 打印可读日志，并把原始 Joy 映射成语义状态发布到 `/gamepad/state`（摇杆 Y 前推为正、扳机归一化为按下深度）。手柄连接（WSL2 + xboxdrv）、轴/按钮映射表见 [`src/xbox_reader/README.md`](src/xbox_reader/README.md)。
- **`src/ee_teleop`** — 末端速度遥控前端：`ee_teleop` 订阅 `/gamepad/state`，处理成平滑的 `TwistStamped` 末端速度指令（死区/限幅/EMA/加速度限幅/看门狗）+ 夹爪开合，后端可接 MoveIt Servo。控制映射、yaml 参数配置、Servo remap、自检用法见 [`src/ee_teleop/README.md`](src/ee_teleop/README.md)。

数据链路：`/joy`（sensor_msgs/Joy）→ `xbox_reader` → `/gamepad/state`（gamepad_msgs/GamepadState）→ `ee_teleop` → `/ee_teleop/twist`（TwistStamped）+ 夹爪指令。

## 构建

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select gamepad_msgs xbox_reader ee_teleop
source install/setup.bash
```
