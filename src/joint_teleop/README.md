# joint_teleop — 关节级遥控前端（SO101，C++）

把语义化手柄状态变成平滑的单关节速度指令，直发 MoveIt Servo 的 JointJog 通道。
本节点订阅 `xbox_reader` 发布的 `/gamepad/state`（手柄事件流频率不均，节点缓存最新状态），
用固定频率（默认 100 Hz）做死区/归一化、限幅、EMA 滤波、加速度限幅后发布。

本包是原 `ee_teleop`（Python，TwistStamped 末端速度 → Servo 笛卡尔通道 → 实时 IK）的
**关节级重写版**：不经过逆解算，不存在 IK 解跳变 / 姿态不连续问题。
原 Python 版 `ee_teleop` 保留作为历史版本与参考，两包**互斥使用**
（都发 JointJog/Twist 到 Servo，不要同时运行）。

## 为什么用关节级而不是末端位姿遥操

旧版路径是"手柄 → TwistStamped 末端速度 → Servo 笛卡尔通道 → 每周期实时 IK"，主要问题：

1. **IK 解跳变**：SO-101 是 5 自由度臂（雅可比 6×5 天然病态），同一末端速度需求在不同周期可能
   收敛到不同解分支，表现为肘/腕姿态突变、轨迹不连续；越靠近奇异位形越严重。
2. **姿态不连续**：姿态增量经伪逆映射到关节空间后多关节联动补偿 EE 位移，末端姿态与操作意图对不上。
3. **手柄输入分辨率低**：摇杆 8~16 bit 量化 + 人手微颤，经 IK 放大为关节空间抖动。

手柄遥操采集数据时，人需要的是"指定某个关节往某个方向转"的确定性。关节级方案把左摇杆 X 轴直接
映射为**选中关节的速度**（JointJog 直发 Servo 关节通道），没有逆解算、没有解分支选择，方向与速度
完全由操作者决定。这也是低成本主从遥操（ALOHA、SO-100 主从）在数据采集上普遍采用关节空间映射的原因。

代价：笛卡尔空间的直线运动能力丢失（不能"让末端水平走直线"）。5 自由度臂本身也做不到任意 6 维
位姿——位姿跟随类方案的上限受此约束（见末节"关于位姿跟随"）。

## 节点

`joint_teleop`（C++ / rclcpp）

### 输入

| Topic | 类型 | 说明 |
|---|---|---|
| `/gamepad/state` | `gamepad_msgs/GamepadState` | 语义化手柄状态（由 `xbox_reader` 发布；摇杆 X 右为正） |

### 输出

| Topic | 类型 | 说明 |
|---|---|---|
| `/servo_node/delta_joint_cmds` | `control_msgs/JointJog` | 当前选中关节的速度指令，每个周期必发（含全零），**必须带非零 stamp**（Servo 对 stamp=0 的 JointJog 不更新新鲜时间戳，会被判 stale 永久忽略） |
| `/so101_gripper_controller/commands` | `std_msgs/Float64MultiArray` | 夹爪指令，仅 X 键上升沿切换时发一条，`data` 长度 1 |

## 控制映射（语义键，写死在代码里）

| 输入（GamepadState 字段） | 输出 | 说明 |
|---|---|---|
| `left_stick_x` 左/右（**只用 X 轴**） | JointJog 选中关节速度 | 死区+归一化后 × `omega_max`，摇杆正负即关节正负方向（SO101 臂关节均为单轴 revolute）；哪个关节手感反了改 `joint_signs` 对应项 |
| `button_rb` 按一下 | 选中关节 +1 | 在 `arm_joints` 列表内循环（到尾回头），切换时打印当前关节名 |
| `button_lb` 按一下 | 选中关节 −1 | 同上 |
| `button_x` 按一下 | 夹爪开合切换 | 上升沿切换开/合目标，发一条 `/so101_gripper_controller/commands` |

其余键（A/B/Y、右摇杆、扳机、十字键）不使用。

## 处理链（每个控制周期）

1. **死区 + 归一化**：`|x| < deadzone(0.05)` → 0；否则 `(x − sign(x)·deadzone)/(1 − deadzone)`，过死区后速度从 0 连续起步，无阶跃。
2. **限幅缩放**：`ω_target = left_stick_x × omega_max × joint_signs[selected]`。
3. **EMA 低通滤波**：`ω_f = α·ω_target + (1−α)·ω_prev`，`alpha` 默认 0.12，100 Hz 下 ≈80 ms 时间常数。
4. **角加速度限幅**（梯形速度规划）：`|ω_f − ω_prev| ≤ angular_accel_max·dt`，默认 2.0 rad/s²。**EMA 在前、限幅在后**，顺序不能反。
5. **看门狗**：距上次收到 `/gamepad/state` 超过 `joy_timeout`（0.3 s，按收到消息的接收时间判断）→ 本周期目标速度清零，但输出仍走滤波和加速度限幅**缓慢停下**，不急停。
6. **输出端微死区**：`|ω| < omega_epsilon(0.01 rad/s)` 直接置精确的 0，截断 EMA 指数衰减尾巴。

发布频率由 `publish_rate`（默认 100 Hz）决定，`dt = 1/publish_rate`。

## 参数（config/joint_teleop.yaml）

所有数值参数在 **`config/joint_teleop.yaml`** 里改，launch 通过 `config_file` 参数加载：

| 参数 | 默认值 | 含义 |
|---|---|---|
| `publish_rate` | 100.0 | 控制/发布频率 (Hz)，与 Servo/JTC 同频，避免命令量化台阶 |
| `deadzone` | 0.05 | 摇杆死区 |
| `omega_max` | 1.5 | 摇杆满偏关节速度 (rad/s)，嫌慢继续加，URDF 上限 10 |
| `alpha` | 0.12 | EMA 滤波系数；改频率要同步换算 |
| `angular_accel_max` | 2.0 | 角加速度限幅 (rad/s²) |
| `joy_timeout` | 0.3 | 看门狗超时 (s) |
| `base_frame` | 'base_link' | 输出 JointJog 的 frame_id |
| `arm_joints` | [shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll] | RB/LB 循环切换的遥操关节（夹爪不在此列） |
| `joint_signs` | [1.0 × 5] | 与 `arm_joints` 对齐的方向符号，手感反了改对应项为 −1.0 |
| `joint_command_topic` | '/servo_node/delta_joint_cmds' | Servo 的 JointJog 输入话题 |
| `enable_gripper` | true | 是否发夹爪指令 |
| `gripper_open_value` | 0.1 | 夹爪开指令值 |
| `gripper_close_value` | −0.1 | 夹爪合指令值 |
| `omega_epsilon` | 0.01 | 输出端微死区 (rad/s) |

按钮功能映射（RB/LB 切关节、X 键夹爪）和摇杆选择是语义键、写死在代码里，不是参数。

## 夹爪指令值说明

默认值 **0.1 / −0.1 是 effort（力矩）量纲**，沿用 `so101_moveit_gazebo` 的 `manual_control.py`，用于 Gazebo 的 effort 夹爪控制器。

⚠️ **换到 MuJoCo 等 position 控制器时这两个值必须改**：position 控制器要的是关节限位内的位置值（SO101 夹爪 lower=−0.174533, upper=1.74533），直接发 0.1/−0.1 没有意义。在自己的 yaml 里改成限位内数值，如 `gripper_open_value: 1.5`、`gripper_close_value: -0.1`。

## 使用

```bash
cd xbox_control
colcon build --packages-select joint_teleop
source install/setup.bash

# 手柄方式（joy_node + teleop，用包内默认 yaml）
ros2 launch joint_teleop joint_teleop.launch.py

# 用自己的参数文件覆盖
ros2 launch joint_teleop joint_teleop.launch.py \
  config_file:=/absolute/path/to/my_teleop.yaml
```

单独跑节点（不加 yaml 时用代码内置默认值）：

```bash
ros2 run joint_teleop joint_teleop --ros-args -p publish_rate:=5.0
```

## 接 MoveIt Servo

只占用 Servo 的 JointJog 关节通道（在 `so101_moveit_gazebo` 的 `servo_parameters.yaml` 中确认话题名一致）：

| joint_teleop 输出 | Servo 参数 | 用途 |
|---|---|---|
| `/servo_node/delta_joint_cmds` (JointJog) | `joint_command_in_topic` | 选中关节速度 |

注意：

- JointJog 必须带非零 `header.stamp`（节点用 `now()`，配合 launch 中的 `use_sim_time:=True` 即为仿真时钟），否则 Servo 永不更新其新鲜时间戳，命令被当 stale 忽略。
- `servo_parameters.yaml` 的 `cartesian_command_in_topic: /ee_teleop/twist` 供旧 Python 版 `ee_teleop` 使用；只跑 joint_teleop 时该通道闲置无害。`halt_all_joints_in_joint_mode: true` 保证单关节 jog 时其余关节停止。
- Servo 侧关节速度缩放由 `command_in_type: "speed_units"` 决定：本节点发的是 rad/s 实际速度，`scale.joint` 在 speed_units 模式下被忽略。

## 关于位姿跟随（pose-tracking）方案

曾评估过"RViz 式目标位姿跟随"（手柄挪动虚拟目标点，阻尼最小二乘跟踪）：

- **比 RViz 拖拽好**：RViz 的近似 IK 是"解得出来才动"，5 自由度下经常出现整只 ghost 不动；
  阻尼最小二乘永远给出最小二乘方向，能跟的分量照跟。
- **但上限一样**：5 个关节最多满足 5 维约束。位置（3 维）+ 固定姿态（3 维）= 6 维需求天然过约束，
  位置目标与锁定姿态冲突时只能靠增益配比牺牲姿态。RViz 上"只有蓝色箭头好用"正是这个约束的体现，
  不是 IK 求解器的问题。所以该方案未实现——要 RViz 级灵活度，换 6+ 自由度臂或主从臂才是正解。
