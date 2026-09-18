# ee_teleop — 机械臂末端速度遥控前端（SO101）

把语义化手柄状态变成平滑的末端速度指令。本节点订阅 `xbox_reader` 发布的 `/gamepad/state`（手柄事件流频率不均，节点缓存最新状态），用固定频率（默认 100 Hz）做死区/归一化、限幅、EMA 滤波、加速度限幅后发布，后端可接 MoveIt Servo。

## 节点

`ee_teleop`

### 输入

| Topic | 类型 | 说明 |
|---|---|---|
| `/gamepad/state` | `gamepad_msgs/GamepadState` | 语义化手柄状态（由 `xbox_reader` 发布；摇杆 Y 前推已为正，扳机已归一化为深度） |

### 输出

| Topic | 类型 | 说明 |
|---|---|---|
| `/ee_teleop/twist` | `geometry_msgs/TwistStamped` | 末端线速度指令，`header.frame_id = base_frame`（默认 base_link），每个周期必发（含全零），只用 linear 三分量 |
| `/servo_node/delta_joint_cmds` | `control_msgs/JointJog` | 单关节旋转指令（Y 键 → `wrist_roll`），每个周期必发（含全零），**必须带非零 stamp**（Servo 对 stamp=0 的 JointJog 不更新新鲜时间戳，会被判 stale 永久忽略） |
| `/so101_gripper_controller/commands` | `std_msgs/Float64MultiArray` | 夹爪指令，仅 X 键上升沿切换时发一条，`data` 长度 1 |

## 控制映射（语义键，写死在代码里）

| 输入（GamepadState 字段） | 输出 | 说明 |
|---|---|---|
| `left_stick_y` 前推/后拉 | twist.linear.x | 满偏 ±0.1 m/s；摇杆向量先经 `control_yaw` 在 base xy 平面旋转（定义"人的前方"相对 base +X 的方向），再输出 |
| `left_stick_x` 左/右 | twist.linear.y | 满偏 ±0.1 m/s，同上旋转 |
| `button_a` 按住 | twist.linear.z | +0.1 m/s 上升（base 竖直安装时即垂直地面） |
| `button_b` 按住 | twist.linear.z | −0.1 m/s 下降；A、B 同按则 vz=0 |
| `button_y` 按住 | JointJog `wrist_roll` | **单关节旋转** ±1.5 rad/s，JointJog 通道直发，纯 wrist_roll、不经过雅可比。Servo 2.5.9 双通道互斥、Twist 优先，**约定不平移与转腕同时操作**（摇杆有输入时 Y 被忽略）。旋向不对调 `wz_sign` |
| `button_x` 按一下 | 夹爪开合切换 | 上升沿切换开/合目标，发一条 `/so101_gripper_controller/commands` |

为什么 Y 走 JointJog 而不是 Twist：绕 wrist_roll 关节轴的 Twist 经雅可比伪逆后**不是单关节运动**——EE 不在关节轴上，单关节转动天然带动 EE 位移，伪逆会派其他关节补偿，结果多关节联动。只有 JointJog 能保证"只有 wrist_roll 动"。另：URDF 中 wrist_roll 关节轴 ⊥ 工具系 z（绕工具 z 转是夹爪侧摆，不是腕滚）。

## 处理链（每个控制周期，每通道独立）

1. **死区 + 归一化**（仅摇杆通道）：`|x| < deadzone(0.05)` → 0；否则 `(x − sign(x)·deadzone)/(1 − deadzone)`，过死区后速度从 0 连续起步，无阶跃。
2. **限幅缩放**（摇杆）：`vx = left_stick_y × v_max_xy`，`vy = left_stick_x × v_max_xy`（`v_max_xy` 默认 0.1 m/s；GamepadState 已约定前推为正，无需取负），再经 `control_yaw` 旋转。按钮通道不做死区和摇杆缩放，直接给恒定值。
3. **EMA 低通滤波**：`v_f = α·v_target + (1−α)·v_prev`，`alpha` 默认 0.12，vx/vy/vz/w_joint 四通道各自独立。
4. **加速度限幅**（梯形速度规划）：`|v_f − v_prev| ≤ accel_max·dt`，线速度 `accel_max` 默认 0.5 m/s²，角速度 `angular_accel_max` 默认 2.0 rad/s²。**EMA 在前、限幅在后**，顺序不能反。
5. **看门狗**：距上次收到 `/gamepad/state` 超过 `joy_timeout`（0.3 s，按收到消息的接收时间判断）→ 本周期目标速度清零，但输出仍走滤波和加速度限幅**缓慢停下**，不急停。
6. **输出端微死区**：`|v| < vel_epsilon(0.002 m/s)`、`|w| < omega_epsilon(0.01 rad/s)` 直接置精确的 0。EMA 是指数衰减、渐近逼近 0 而永不归零，不截断的话静止时输出会长期显示 e-12 ~ e-174 量级的 denormal 残值。

发布频率由 `publish_rate`（默认 100 Hz）决定，`dt = 1/publish_rate`。

## 参数（config/ee_teleop.yaml）

所有数值参数在 **`config/ee_teleop.yaml`** 里改，launch 通过 `config_file` 参数加载：

| 参数 | 默认值 | 含义 |
|---|---|---|
| `publish_rate` | 100.0 | 控制/发布频率 (Hz)，与 Servo/JTC 同频，避免命令量化台阶 |
| `deadzone` | 0.05 | 摇杆死区 |
| `v_max_xy` | 0.1 | 摇杆满偏线速度 (m/s) |
| `v_max_z` | 0.1 | 按钮 Z 向速度 (m/s) |
| `omega_max` | 1.5 | Y 键 wrist_roll 转速 (rad/s)，嫌慢继续加，URDF 上限 10 |
| `wz_sign` | −1.0 | Y 键旋向符号 |
| `alpha` | 0.12 | EMA 滤波系数；100 Hz 下 ≈80 ms 时间常数（手感等价旧 30 Hz+0.3），改频率要同步换算 |
| `accel_max` | 0.5 | 线加速度限幅 (m/s²) |
| `angular_accel_max` | 2.0 | 角加速度限幅 (rad/s²) |
| `joy_timeout` | 0.3 | 看门狗超时 (s) |
| `base_frame` | 'base_link' | 输出 TwistStamped 的 frame_id（base 竖直安装时其 xy 即平行地面） |
| `control_yaw` | 0.0 | 摇杆"前"方向相对 base +X 的偏航角 (rad，右手为正)，方向不对就试 ±1.5708 |
| `wrist_joint` | 'wrist_roll' | Y 键单关节旋转的关节名 |
| `joint_command_topic` | '/servo_node/delta_joint_cmds' | Servo 的 JointJog 输入话题 |
| `enable_gripper` | true | 是否发夹爪指令 |
| `gripper_open_value` | 0.1 | 夹爪开指令值 |
| `gripper_close_value` | −0.1 | 夹爪合指令值 |
| `vel_epsilon` | 0.002 | 输出端微死区：线速度低于此值置 0（截断 EMA 指数衰减尾巴，否则静止时会以 denormal 小数无限逼近 0） |
| `omega_epsilon` | 0.01 | 输出端微死区：角速度低于此值置 0 |

按钮功能映射（A=Z升 B=Z降 Y=手腕关节旋转 X=夹爪）和摇杆选择是语义键、写死在代码里，不是参数。

## 夹爪指令值说明

默认值 **0.1 / −0.1 是 effort（力矩）量纲**，沿用 `so101_moveit_gazebo` 的 `manual_control.py`（`GRIP_OPEN_EFFORT = 0.1`、`GRIP_CLOSE_EFFORT = −0.1` N·m），用于 Gazebo 的 effort 夹爪控制器。

⚠️ **换到 MuJoCo 等 position 控制器时这两个值必须改**：position 控制器要的是关节限位内的位置值（SO101 夹爪 lower=−0.174533, upper=1.74533），直接发 0.1/−0.1 没有意义。在自己的 yaml 里改成限位内数值，如 `gripper_open_value: 1.5`、`gripper_close_value: -0.1`。

## 使用

```bash
cd xbox_control
colcon build --packages-select ee_teleop
source install/setup.bash

# 手柄方式（joy_node + teleop，用包内默认 yaml）
ros2 launch ee_teleop ee_teleop.launch.py

# 用自己的参数文件覆盖
ros2 launch ee_teleop ee_teleop.launch.py \
  config_file:=/absolute/path/to/my_teleop.yaml

# 不带手柄自检（假 /gamepad/state 数据跑 7 个用例）
ros2 launch ee_teleop self_check.launch.py
```

单独跑节点（不加 yaml 时用代码内置默认值）：

```bash
ros2 run ee_teleop ee_teleop --ros-args -p publish_rate:=5.0
```

## 接 MoveIt Servo

同时占用 Servo 的两个输入通道（在 `servo_parameters.yaml` 中确认话题名一致）：

| ee_teleop 输出 | Servo 参数 | 用途 |
|---|---|---|
| `/ee_teleop/twist` (TwistStamped) | `cartesian_command_in_topic` | 摇杆+A/B 的末端线速度 |
| `/servo_node/delta_joint_cmds` (JointJog) | `joint_command_in_topic` | Y 键 wrist_roll 单关节旋转 |

注意：

- Servo 按 TwistStamped 的 `frame_id` 经 TF 变换到规划系（`base_link`/`world`/`ee_frame` 均可，默认 `base_frame='base_link'` 正好匹配）。
- **Servo 2.5.9 双通道互斥、Cartesian 优先**（`servo_calcs.cpp` else-if）：Twist 非零时 JointJog 被忽略。约定不平移与转腕同时操作；若日后需要边移动边转腕，可在 `ws_moveit2` 源码把互斥改为叠加（本地补丁，升级时需同步维护）。
- JointJog 必须带非零 `header.stamp`（用仿真时钟），否则 Servo 永不更新其新鲜时间戳，命令被当 stale 忽略。

## 自检

`self_check` 节点自己发布 `/gamepad/state`（不依赖真实手柄，**不要同时起 joy_node**，因为它自己占这个 topic），按时间轴发 7 组输入并断言输出，任一 FAIL 时退出码非零：

```bash
ros2 launch ee_teleop self_check.launch.py   # teleop + self_check 一起
```
