import math

import rclpy
from gamepad_msgs.msg import GamepadState
from geometry_msgs.msg import TwistStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class EETeleop(Node):
    def __init__(self):
        super().__init__('ee_teleop')

        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('deadzone', 0.1)
        self.declare_parameter('v_max_xy', 0.1)
        self.declare_parameter('v_max_z', 0.05)
        self.declare_parameter('omega_max', 0.6)
        self.declare_parameter('wz_sign', -1.0)
        self.declare_parameter('alpha', 0.3)
        self.declare_parameter('accel_max', 0.5)
        self.declare_parameter('angular_accel_max', 2.0)
        self.declare_parameter('joy_timeout', 0.3)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('enable_gripper', True)
        self.declare_parameter('gripper_open_value', 0.1)
        self.declare_parameter('gripper_close_value', -0.1)
        self.declare_parameter('vel_epsilon', 0.002)
        self.declare_parameter('omega_epsilon', 0.01)

        publish_rate = self.get_parameter('publish_rate').value
        self.deadzone = self.get_parameter('deadzone').value
        self.v_max_xy = self.get_parameter('v_max_xy').value
        self.v_max_z = self.get_parameter('v_max_z').value
        self.omega_max = self.get_parameter('omega_max').value
        self.wz_sign = self.get_parameter('wz_sign').value
        self.alpha = self.get_parameter('alpha').value
        self.accel_max = self.get_parameter('accel_max').value
        self.angular_accel_max = self.get_parameter('angular_accel_max').value
        self.joy_timeout = self.get_parameter('joy_timeout').value
        self.base_frame = self.get_parameter('base_frame').value
        self.enable_gripper = self.get_parameter('enable_gripper').value
        self.gripper_open_value = self.get_parameter('gripper_open_value').value
        self.gripper_close_value = self.get_parameter('gripper_close_value').value
        self.vel_epsilon = self.get_parameter('vel_epsilon').value
        self.omega_epsilon = self.get_parameter('omega_epsilon').value

        # 语义键写死：A=Z升 B=Z降 Y=旋转 X=夹爪切换；摇杆用 left_stick_x/y
        self.create_subscription(
            GamepadState, '/gamepad/state', self.state_callback, 10)
        self.twist_pub = self.create_publisher(TwistStamped, '/ee_teleop/twist', 10)
        self.gripper_pub = self.create_publisher(
            Float64MultiArray, '/so101_gripper_controller/commands', 1)

        self._pad = None
        self._last_pad_time = None
        self._prev_gripper_btn = False
        self._gripper_open = True
        self._vel = [0.0, 0.0, 0.0, 0.0]  # vx, vy, vz, wz

        self._dt = 1.0 / publish_rate
        self.create_timer(self._dt, self.control_cycle)

    def state_callback(self, msg: GamepadState):
        self._pad = msg
        self._last_pad_time = self.get_clock().now()

    def _apply_deadzone(self, x):
        dz = self.deadzone
        if abs(x) < dz:
            return 0.0
        return (x - math.copysign(dz, x)) / (1.0 - dz)

    def _compute_targets(self, state):
        # 摇杆限幅缩放 + 按钮恒定值，返回目标速度 (vx, vy, vz, wz)
        vx = self._apply_deadzone(state.left_stick_y) * self.v_max_xy
        vy = self._apply_deadzone(state.left_stick_x) * self.v_max_xy
        vz = 0.0
        if state.button_a != state.button_b:
            vz = self.v_max_z if state.button_a else -self.v_max_z
        wz = self.wz_sign * self.omega_max if state.button_y else 0.0
        return (vx, vy, vz, wz)

    def _ema_step(self, targets):
        # 四通道独立 EMA 低通：v_f = alpha*target + (1-alpha)*v_prev
        return [self.alpha * t + (1.0 - self.alpha) * v
                for t, v in zip(targets, self._vel)]

    def _accel_limit(self, v_f, dt):
        # 限幅在后：|v_f - v_prev| <= accel_max*dt（角速度用独立限幅），就地更新 self._vel
        for i in range(4):
            accel = self.accel_max if i < 3 else self.angular_accel_max
            dv = max(-accel * dt, min(accel * dt, v_f[i] - self._vel[i]))
            self._vel[i] += dv

    def _watchdog_expired(self, now):
        return (self._pad is None
                or (now - self._last_pad_time).nanoseconds * 1e-9 > self.joy_timeout)

    def _update_gripper(self, state):
        # X 键上升沿切换开/合目标，发一条夹爪指令
        btn = state.button_x
        if btn and not self._prev_gripper_btn and self.enable_gripper:
            self._gripper_open = not self._gripper_open
            msg = Float64MultiArray()
            msg.data = [self.gripper_open_value if self._gripper_open
                        else self.gripper_close_value]
            self.gripper_pub.publish(msg)
        self._prev_gripper_btn = btn

    def _make_twist(self, vx, vy, vz, wz):
        # 输出端微死区：截断 EMA 指数衰减尾巴（否则会以 denormal 形式无限逼近 0）
        vx = 0.0 if abs(vx) < self.vel_epsilon else vx
        vy = 0.0 if abs(vy) < self.vel_epsilon else vy
        vz = 0.0 if abs(vz) < self.vel_epsilon else vz
        wz = 0.0 if abs(wz) < self.omega_epsilon else wz
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = self.base_frame
        twist.twist.linear.x = vx
        twist.twist.linear.y = vy
        twist.twist.linear.z = vz
        twist.twist.angular.z = wz
        return twist

    def _publish_twist(self, twist):
        self.twist_pub.publish(twist)

    def control_cycle(self):
        now = self.get_clock().now()
        if self._watchdog_expired(now):
            targets = (0.0, 0.0, 0.0, 0.0)
        else:
            targets = self._compute_targets(self._pad)
            self._update_gripper(self._pad)

        v_f = self._ema_step(targets)
        self._accel_limit(v_f, self._dt)
        self._publish_twist(self._make_twist(*self._vel))


def main(args=None):
    rclpy.init(args=args)
    node = EETeleop()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
