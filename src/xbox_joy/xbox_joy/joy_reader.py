import rclpy
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


class JoyReader(Node):
    def __init__(self):
        super().__init__('joy_reader')
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self._prev_buttons = []

    def joy_callback(self, msg: Joy):
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
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
