import sys
import time

import rclpy
from gamepad_msgs.msg import GamepadState
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

# 用例时间轴（秒，monotonic）：(结束时刻, left_stick_y, 按钮字段名元组, 是否发状态)
# GamepadState 约定摇杆 Y 前推为正：+1.0 = 满偏前推
TIMELINE = [
    (1.0, 0.0, (), True),                 # 1 静默
    (1.5, 0.05, (), True),                # 2 死区内
    (2.5, 1.0, (), True),                 # 3 满偏前推
    (3.3, 0.0, ('button_a',), True),      # 4a A 按住
    (4.0, 0.0, ('button_a', 'button_b'), True),  # 4b A+B 同按
    (4.8, 0.0, ('button_y',), True),      # 5 Y 按住
    (5.0, 1.0, ('button_x',), True),      # 6a X 按下
    (5.3, 1.0, (), True),
    (5.5, 1.0, ('button_x',), True),      # 6b 再按一次 X
    (5.7, 1.0, (), True),
    (7.8, 0.0, (), False),                # 7 看门狗：停发 /gamepad/state
]
PUB_PERIOD = 0.02


class SelfCheck(Node):
    def __init__(self):
        super().__init__('ee_teleop_self_check')
        self.state_pub = self.create_publisher(GamepadState, '/gamepad/state', 10)
        self.create_subscription(
            TwistStamped, '/ee_teleop/twist', self.twist_callback, 50)
        self.create_subscription(
            Float64MultiArray, '/so101_gripper_controller/commands',
            self.gripper_callback, 10)
        self._twist_samples = []   # (t, vx, vy, vz, wz)
        self._gripper_msgs = []    # (t, data0)
        self._t0 = time.monotonic()
        self.results = []
        self.finished = False
        self.create_timer(PUB_PERIOD, self.tick)

    def _now(self):
        return time.monotonic() - self._t0

    def twist_callback(self, msg: TwistStamped):
        v = msg.twist
        self._twist_samples.append(
            (self._now(), v.linear.x, v.linear.y, v.linear.z, v.angular.z))

    def gripper_callback(self, msg: Float64MultiArray):
        if msg.data:
            self._gripper_msgs.append((self._now(), msg.data[0]))

    def _segment(self, t):
        for end, stick_y, buttons, publish in TIMELINE:
            if t < end:
                return stick_y, buttons, publish
        return 0.0, (), False

    def tick(self):
        t = self._now()
        stick_y, buttons, publish = self._segment(t)
        if publish:
            msg = GamepadState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.left_stick_y = stick_y
            for name in buttons:
                setattr(msg, name, True)
            self.state_pub.publish(msg)
        if t > TIMELINE[-1][0] and not self.finished:
            self.finished = True
            self.evaluate()

    def _twists(self, t_a, t_b):
        return [s for s in self._twist_samples if t_a <= s[0] < t_b]

    def _check(self, name, ok, detail=''):
        self.results.append((name, ok))
        self.get_logger().info(f'{"PASS" if ok else "FAIL"}  {name}'
                               + (f'  ({detail})' if detail else ''))

    def evaluate(self):
        def all_zero(samples):
            return all(abs(v) < 1e-9 for s in samples for v in s[1:])

        s = self._twists(0.05, 0.95)
        self._check('1 静默输出全零', len(s) > 0 and all_zero(s),
                    f'{len(s)} 个样本')

        s = self._twists(1.05, 1.45)
        self._check('2 死区内 vx=0', len(s) > 0 and all_zero(s),
                    f'{len(s)} 个样本')

        s_end = self._twists(2.3, 2.45)
        vx_max = max((x[1] for x in s_end), default=0.0)
        s_ramp = self._twists(1.6, 2.4)
        has_mid = any(0.02 < x[1] < 0.085 for x in s_ramp)
        vy_ok = all(abs(x[2]) < 1e-6 for x in s_ramp)
        self._check('3 满偏 vx≈0.1 且斜坡非跳变',
                    vx_max >= 0.09 and has_mid and vy_ok,
                    f'末段 vmax={vx_max:.4f}, 中间值={has_mid}, vy=0={vy_ok}')

        s = self._twists(3.0, 3.25)
        vz = max((x[3] for x in s), default=0.0)
        s = self._twists(3.7, 3.95)
        vz_ab = max((abs(x[3]) for x in s), default=1.0)
        self._check('4 A 升 vz≈+0.05，A+B 同按 vz≈0',
                    0.04 <= vz <= 0.06 and vz_ab < 0.01,
                    f'A: {vz:.4f}, A+B: |vz|max={vz_ab:.5f}')

        s = self._twists(4.5, 4.75)
        wz = min((x[4] for x in s), default=0.0)
        self._check('5 Y 按住 angular.z≈-0.6', -0.62 <= wz <= -0.58,
                    f'wz={wz:.4f}')

        g = [m for m in self._gripper_msgs if 4.8 <= m[0] < 5.7]
        signs_ok = (len(g) == 2 and g[0][1] * g[1][1] < 0
                    and abs(abs(g[0][1]) - 0.1) < 1e-6
                    and abs(abs(g[1][1]) - 0.1) < 1e-6)
        self._check('6 X 键两次切换夹爪(0.1/-0.1)', signs_ok,
                    f'收到 {len(g)} 条: {[round(m[1], 3) for m in g]}')

        stop = 5.7
        s_early = self._twists(stop + 0.05, stop + 0.2)
        early_ok = any(x[1] > 0.01 for x in s_early)
        # EMA 指数衰减是渐近的：目标清零后 v≈0.1×0.7^n，
        # 看门狗 0.3s 后才切目标，再衰减 ~1.6s(48 周期)约 3e-8，阈值取 1e-5
        s_late = self._twists(stop + 1.7, stop + 1.9)
        late_ok = len(s_late) > 0 and all(
            abs(v) < 1e-5 for s in s_late for v in s[1:])
        self._check('7 看门狗缓停: 停发瞬间非零, 1.9s 后≈0',
                    early_ok and late_ok,
                    f'停发后~0.1s vmax={max((x[1] for x in s_early), default=0):.4f},'
                    f' 停发后~1.8s |v|max='
                    f'{max((abs(v) for s in s_late for v in s[1:]), default=1):.2e}')

        passed = sum(1 for _, ok in self.results if ok)
        self.get_logger().info(
            f'== 总结: {passed}/{len(self.results)} 通过 ==')
        self.exit_code = 0 if passed == len(self.results) else 1


def main(args=None):
    rclpy.init(args=args)
    node = SelfCheck()
    exit_code = 1
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.05)
        exit_code = getattr(node, 'exit_code', 1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(exit_code)
