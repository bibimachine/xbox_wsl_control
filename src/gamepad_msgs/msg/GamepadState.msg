# 语义化手柄状态：由 reader 完成"设备原始值 -> 通用语义"的映射
# 约定：摇杆 Y 轴前推为正、X 轴右为正；扳机为按下深度；连续量均已归一化

std_msgs/Header header

# 摇杆（-1.0 ~ +1.0，Y 前为正 —— reader 已做符号翻转，teleop 拿到直接用）
float64 left_stick_x
float64 left_stick_y
float64 right_stick_x
float64 right_stick_y

# 扳机（0.0 未按 ~ 1.0 按满 —— reader 已把"静止 +1"归一化成深度）
float64 left_trigger
float64 right_trigger

# 面键
bool button_a
bool button_b
bool button_x
bool button_y

# 肩键
bool button_lb
bool button_rb

# 十字键
bool dpad_up
bool dpad_down
bool dpad_left
bool dpad_right

# 摇杆按下
bool stick_left_press
bool stick_right_press

# 功能键
bool button_back
bool button_start
bool button_guide

# 调试用原始数据（teleop 不读）
sensor_msgs/Joy raw
