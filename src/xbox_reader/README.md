# xbox_reader — Xbox 手柄输入读取包

在 WSL2 中使用雷神 G30S 手柄（有线模式），通过 `joy_node` 将手柄输入发布为 `/joy` 话题，`joy_reader` 节点订阅并打印可读日志。

## 环境

- 手柄：雷神 G30S，有线连接（USB 透传进 WSL 后枚举为 `045e:028e Microsoft Corp. Xbox360 Controller`）
- 系统：Windows 11 + WSL2（Ubuntu 22.04，内核 `6.6.x-microsoft-standard-WSL2`）
- ROS：Humble
- 驱动：`xboxdrv`（用户态驱动）

## 为什么需要 xboxdrv

WSL2 官方内核**没有编译 xpad 手柄驱动**，设备透传进来后 `lsusb` 能看到，但没有驱动绑定、`/dev/input/` 下没有节点，手柄灯也不亮。

`xboxdrv` 是用户态驱动：通过 libusb 直接和手柄通信，并用 uinput 创建虚拟输入设备（节点名 `Xbox Gamepad (userspace driver)`），同时会点亮手柄灯。

> 注意：WSL2 内核也没有 `joydev`，所以只会出现 `/dev/input/event0`，**不会有 `/dev/input/js0`**，不要按网上标准 Linux 教程找 js 节点。

## 一次性配置

### 1. Windows 侧安装 usbipd 并绑定手柄

PowerShell（管理员）：

```powershell
winget install usbipd
usbipd list                              # 找到手柄的 BUSID
usbipd bind --busid <BUSID>
usbipd attach --wsl --busid <BUSID>      # 每次重新插拔或 wsl --shutdown 后要重新执行
```

### 2. WSL 内安装 xboxdrv

```bash
sudo apt install xboxdrv
```

### 3. 把手柄设备权限放给当前用户

`xboxdrv` 创建的 `/dev/input/event0` 属于 `root:input`，普通用户读不到，需要加入 `input` 组：

```bash
sudo usermod -aG input $USER
```

然后**必须完全重启 WSL**（组权限在登录时生效）：

```powershell
wsl --shutdown
```

## 每次使用流程

```bash
# Windows PowerShell：透传手柄（如果还没 attach） 多半得 --force
usbipd attach --wsl --busid <BUSID>

# WSL 内：启动用户态驱动（灯应亮起，生成 /dev/input/event0）
sudo xboxdrv --silent --detach

# 编译并启动（首次或改动后）
cd xbox_control
colcon build --packages-select xbox_reader --symlink-install
source install/setup.bash
ros2 launch xbox_reader xbox_joy.launch.py

# 另开终端看原始数据
ros2 topic echo /joy
```

## 查看设备名称（evtest）

launch 文件里 `device_name` 要填 SDL 识别的设备名，就是 evdev 设备名。查询方法：

```bash
sudo apt install evtest
sudo evtest
```

列出设备中选择 `Xbox Gamepad (userspace driver)` 对应的编号，即可看到设备名、按键事件流。也可以直接看 sysfs：

```bash
cat /sys/class/input/event0/device/name
# 输出: Xbox Gamepad (userspace driver)
```

> Humble 的 `joy_node` 基于 SDL2，`device_name` 匹配的是**设备名字符串**（如上面的输出），不是 `/dev/input/event0` 这样的路径。设备序号参数叫 `device_id`。
> 如果系统里只有这一个手柄，也可以不传参数，默认 `device_id: 0` 自动选中。

## G30S 按键映射（xboxdrv 默认布局）

`ros2 topic echo /joy` 的原始数组索引：

| axes 索引 | 含义 | 静止值 | 备注 |
|---|---|---|---|
| 0 | 左摇杆 X | 0 | 左负右正 |
| 1 | 左摇杆 Y | 0 | 上负下正 |
| 2 | 右摇杆 X | 0 | |
| 3 | 右摇杆 Y | 0 | |
| 4 | RT（右扳机） | **+1.0** | 按下时向 -1.0 变化，按满为 -1.0（与原版 Xbox 方向相反） |
| 5 | LT（左扳机） | **+1.0** | 同上 |
| 6 | 十字键 X | 0 | -1 左 / +1 右 |
| 7 | 十字键 Y | 0 | -1 上 / +1 下 |

| buttons 索引 | 含义 |
|---|---|
| 0 | A |
| 1 | B |
| 2 | X |
| 3 | Y |
| 4 | LB（左肩键） |
| 5 | RB（右肩键） |
| 6 | Back |
| 7 | Start |
| 8 | Guide（Xbox 键） |
| 9 | 左摇杆按下 |
| 10 | 右摇杆按下 |

注意 G30S 这款兼容手柄的扳机轴方向与原版 Xbox 相反（原版静止 -1、按满 +1）。本包的 `joy_reader` 已做归一化处理：`depth = (1.0 - v) / 2.0`，得到 0（未按）~ 1（按满）。

## joy_reader 用法

单独运行（不启动 joy_node，仅打印已存在的 /joy）：

```bash
ros2 run xbox_reader joy_reader
```

launch 一起起 joy_node + reader：

```bash
ros2 launch xbox_reader xbox_joy.launch.py
```

打印内容：摇杆偏转、扳机深度、当前按住按钮、按钮按下/松开事件。

## /gamepad/state 语义状态发布

`joy_reader` 同时把原始 `sensor_msgs/Joy` 映射为 **`gamepad_msgs/GamepadState`** 发布到 `/gamepad/state`（10 Hz 节流），完成"设备原始值 -> 通用语义"的转换，下游（如 ee_teleop）拿到直接用、不再关心具体轴号/按钮号：

| GamepadState 字段 | 来源 | 转换规则 |
|---|---|---|
| `left_stick_x` / `right_stick_x` | axes[0] / axes[2] | 原值（右为正） |
| `left_stick_y` / `right_stick_y` | axes[1] / axes[3] | **取负**（前推为正） |
| `right_trigger` | axes[4] (RT) | `(1 − v) / 2`，静止 +1 → 0（未按），按满 −1 → 1 |
| `left_trigger` | axes[5] (LT) | 同上 |
| `button_a/b/x/y` | buttons[0/1/2/3] | == 1 |
| `button_lb/rb` | buttons[4/5] | == 1 |
| `button_back/start/guide` | buttons[6/7/8] | == 1 |
| `stick_left_press` / `stick_right_press` | buttons[9/10] | == 1 |
| `dpad_left/right` | axes[6] | < −0.5 / > +0.5 |
| `dpad_up/down` | axes[7] | < −0.5 / > +0.5 |
| `raw` | 整个 Joy 消息 | 原样附带，调试用 |
| `header.stamp` | 当前时间 | frame_id = `gamepad` |

查看：`ros2 topic echo /gamepad/state`。消息定义在 `gamepad_msgs` 包 `msg/GamepadState.msg`。

## 常见问题

**手柄灯不亮 / lsusb 能看到但没反应**
内核没有 xpad，属正常现象。确认 `xboxdrv` 在跑：`ps aux | grep xboxdrv`。注意只能起一个实例，起多了抢不到设备，先 `sudo pkill xboxdrv` 再启动一个。

**joy_node 启动报找不到设备 / 按键没反应**
八成是权限问题：确认 `groups` 输出里有 `input`，没有就回"一次性配置"第 3 步（改完必须 `wsl --shutdown` 重启）。

**`/dev/input/` 下没有 js0**
WSL2 内核没有 joydev，只有 event 节点是正常的，joy_node 直接用 event 节点，不影响使用。

**换了别的手柄**
用 `evtest` 重新查设备名，确认按键映射（尤其扳机方向），更新 launch 的 `dev` 参数和 `joy_reader.py` 里的映射表。
