import time

import rclpy
from gamepad_msgs.msg import GamepadState
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy

# Xbox 手柄在 Linux joy 设备上的标准映射
AXES_NAME = {
    0: '左摇杆X', 1: '左摇杆Y',
    2: '右摇杆X', 3: '右摇杆Y', 4: 'RT',
    5: 'LT', 6: '十字键X', 7: '十字键Y',
}
BUTTONS_NAME = {
    0: 'A', 1: 'B', 2: 'X', 3: 'Y',
    4: 'LB', 5: 'RB', 6: 'Back', 7: 'Start',
    8: 'Guide', 9: '左摇杆按下', 10: '右摇杆按下',
}
DEADZONE = 0.1
# 扳机轴是单向轴，不是摇杆那种居中轴：这款手柄静止为 +1.0，按到底为 -1.0
TRIGGER_AXES = {4: 'RT', 5: 'LT'}
# /joy 事件频率不均且偏快，语义状态发布按 10 Hz 节流
PUBLISH_PERIOD = 0.1


class JoyReader(Node):
    def __init__(self):
        super().__init__('joy_reader')
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.state_pub = self.create_publisher(GamepadState, '/gamepad/state', 10)
        self._prev_buttons = []
        self._last_pub = 0.0

    def _axis(self, msg: Joy, i):
        return msg.axes[i] if i < len(msg.axes) else 0.0

    def _button(self, msg: Joy, i):
        return i < len(msg.buttons) and msg.buttons[i] == 1

    def _to_state(self, msg: Joy):
        # 原始值 -> 语义：Y 轴翻转（前推为正），扳机归一化为按下深度
        state = GamepadState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.header.frame_id = 'gamepad'
        state.left_stick_x = self._axis(msg, 0)
        state.left_stick_y = -self._axis(msg, 1)
        state.right_stick_x = self._axis(msg, 2)
        state.right_stick_y = -self._axis(msg, 3)
        state.right_trigger = (1.0 - self._axis(msg, 4)) / 2.0
        state.left_trigger = (1.0 - self._axis(msg, 5)) / 2.0
        state.button_a = self._button(msg, 0)
        state.button_b = self._button(msg, 1)
        state.button_x = self._button(msg, 2)
        state.button_y = self._button(msg, 3)
        state.button_lb = self._button(msg, 4)
        state.button_rb = self._button(msg, 5)
        state.button_back = self._button(msg, 6)
        state.button_start = self._button(msg, 7)
        state.button_guide = self._button(msg, 8)
        state.stick_left_press = self._button(msg, 9)
        state.stick_right_press = self._button(msg, 10)
        dpad_x = self._axis(msg, 6)
        dpad_y = self._axis(msg, 7)
        state.dpad_left = dpad_x < -0.5
        state.dpad_right = dpad_x > 0.5
        state.dpad_up = dpad_y < -0.5
        state.dpad_down = dpad_y > 0.5
        state.raw = msg
        return state

    def joy_callback(self, msg: Joy):
        now = time.monotonic()
        if now - self._last_pub >= PUBLISH_PERIOD:
            self._last_pub = now
            self.state_pub.publish(self._to_state(msg))

        sticks = []
        triggers = []
        for i, v in enumerate(msg.axes):
            if i in TRIGGER_AXES:
                depth = (1.0 - v) / 2.0  # 归一化成 0(未按) ~ 1(按满)
                if depth > DEADZONE:
                    triggers.append(f'{TRIGGER_AXES[i]}={depth:.2f}')
            elif abs(v) > DEADZONE:
                sticks.append(f'{AXES_NAME.get(i, f"轴{i}")}={v:+.2f}')
        pressed = [BUTTONS_NAME.get(i, f'键{i}') for i, v in enumerate(msg.buttons) if v == 1]

        events = []
        for i in range(max(len(msg.buttons), len(self._prev_buttons))):
            cur = msg.buttons[i] if i < len(msg.buttons) else 0
            prev = self._prev_buttons[i] if i < len(self._prev_buttons) else 0
            if cur != prev:
                events.append(f'{BUTTONS_NAME.get(i, f"键{i}")}{"按下" if cur else "松开"}')
        self._prev_buttons = list(msg.buttons)

        parts = []
        if sticks:
            parts.append('摇杆: ' + ' '.join(sticks))
        if triggers:
            parts.append('扳机: ' + ' '.join(triggers))
        if pressed:
            parts.append('按住: ' + ' '.join(pressed))
        if events:
            parts.append('事件: ' + ' '.join(events))
        if parts:
            self.get_logger().info(' | '.join(parts))


def main(args=None):
    rclpy.init(args=args)
    node = JoyReader()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
