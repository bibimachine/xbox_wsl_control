#include <algorithm>
#include <cmath>
#include <optional>
#include <string>
#include <vector>

#include "control_msgs/msg/joint_jog.hpp"
#include "gamepad_msgs/msg/gamepad_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace
{
double applyDeadzone(double x, double dz)
{
  if (std::abs(x) < dz) {
    return 0.0;
  }
  return (x - std::copysign(dz, x)) / (1.0 - dz);
}
}  // namespace

class JointTeleop : public rclcpp::Node
{
public:
  JointTeleop() : Node("joint_teleop")
  {
    declare_parameter<double>("publish_rate", 100.0);
    declare_parameter<double>("deadzone", 0.05);
    declare_parameter<double>("omega_max", 1.5);
    declare_parameter<double>("alpha", 0.12);
    declare_parameter<double>("angular_accel_max", 2.0);
    declare_parameter<double>("joy_timeout", 0.3);
    declare_parameter<std::string>("base_frame", "base_link");
    declare_parameter<std::vector<std::string>>(
      "arm_joints",
      {"shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"});
    declare_parameter<std::vector<double>>("joint_signs", {1.0, 1.0, 1.0, 1.0, 1.0});
    declare_parameter<std::string>("joint_command_topic", "/servo_node/delta_joint_cmds");
    declare_parameter<bool>("enable_gripper", true);
    declare_parameter<double>("gripper_open_value", 0.1);
    declare_parameter<double>("gripper_close_value", -0.1);
    declare_parameter<double>("omega_epsilon", 0.01);

    const double publish_rate = get_parameter("publish_rate").as_double();
    deadzone_ = get_parameter("deadzone").as_double();
    omega_max_ = get_parameter("omega_max").as_double();
    alpha_ = get_parameter("alpha").as_double();
    angular_accel_max_ = get_parameter("angular_accel_max").as_double();
    joy_timeout_ = get_parameter("joy_timeout").as_double();
    base_frame_ = get_parameter("base_frame").as_string();
    arm_joints_ = get_parameter("arm_joints").as_string_array();
    joint_signs_ = get_parameter("joint_signs").as_double_array();
    enable_gripper_ = get_parameter("enable_gripper").as_bool();
    gripper_open_value_ = get_parameter("gripper_open_value").as_double();
    gripper_close_value_ = get_parameter("gripper_close_value").as_double();
    omega_epsilon_ = get_parameter("omega_epsilon").as_double();

    if (arm_joints_.empty()) {
      throw std::runtime_error("arm_joints 不能为空");
    }
    joint_signs_.resize(arm_joints_.size(), 1.0);

    state_sub_ = create_subscription<gamepad_msgs::msg::GamepadState>(
      "/gamepad/state", 10,
      [this](gamepad_msgs::msg::GamepadState::SharedPtr msg) {
        pad_ = *msg;
        last_pad_time_ = now();
      });

    joint_pub_ = create_publisher<control_msgs::msg::JointJog>(
      get_parameter("joint_command_topic").as_string(), 10);
    gripper_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/so101_gripper_controller/commands", 1);

    dt_ = 1.0 / publish_rate;
    timer_ = create_wall_timer(
      std::chrono::duration<double>(dt_), std::bind(&JointTeleop::controlCycle, this));

    RCLCPP_INFO(get_logger(), "joint teleop ready, %zu joints, selected: %s",
                arm_joints_.size(), arm_joints_[selected_].c_str());
  }

private:
  void controlCycle()
  {
    const bool stale = !last_pad_time_.has_value() ||
                       (now() - last_pad_time_.value()).seconds() > joy_timeout_;

    double target = 0.0;
    if (!stale) {
      target = applyDeadzone(pad_.left_stick_x, deadzone_) * omega_max_ *
               joint_signs_[selected_];
      handleJointSwitch();
      handleGripper();
    }

    // EMA 低通 + 角加速度限幅（EMA 在前、限幅在后）
    double omega = alpha_ * target + (1.0 - alpha_) * omega_;
    const double dw = std::clamp(
      omega - omega_, -angular_accel_max_ * dt_, angular_accel_max_ * dt_);
    omega_ += dw;

    control_msgs::msg::JointJog jog;
    // JointJog 必须带非零 stamp：servo_calcs.cpp 对 stamp=0 的消息不更新新鲜
    // 时间戳，会被判 stale 永久忽略（use_sim_time 下 now() 即为仿真时钟）
    jog.header.stamp = now();
    jog.header.frame_id = base_frame_;
    jog.joint_names = {arm_joints_[selected_]};
    jog.velocities = {std::abs(omega_) < omega_epsilon_ ? 0.0 : omega_};
    joint_pub_->publish(jog);
  }

  void handleJointSwitch()
  {
    const std::size_t n = arm_joints_.size();
    if (pad_.button_rb && !prev_rb_) {
      selected_ = (selected_ + 1) % n;
      RCLCPP_INFO(get_logger(), "selected joint: %s (%zu/%zu)",
                  arm_joints_[selected_].c_str(), selected_ + 1, n);
    }
    if (pad_.button_lb && !prev_lb_) {
      selected_ = (selected_ + n - 1) % n;
      RCLCPP_INFO(get_logger(), "selected joint: %s (%zu/%zu)",
                  arm_joints_[selected_].c_str(), selected_ + 1, n);
    }
    prev_rb_ = pad_.button_rb;
    prev_lb_ = pad_.button_lb;
  }

  void handleGripper()
  {
    if (pad_.button_x && !prev_gripper_btn_ && enable_gripper_) {
      gripper_open_ = !gripper_open_;
      std_msgs::msg::Float64MultiArray msg;
      msg.data = {gripper_open_ ? gripper_open_value_ : gripper_close_value_};
      gripper_pub_->publish(msg);
    }
    prev_gripper_btn_ = pad_.button_x;
  }

  // 参数
  double deadzone_;
  double omega_max_;
  double alpha_;
  double angular_accel_max_;
  double joy_timeout_;
  std::string base_frame_;
  std::vector<std::string> arm_joints_;
  std::vector<double> joint_signs_;
  bool enable_gripper_;
  double gripper_open_value_;
  double gripper_close_value_;
  double omega_epsilon_;

  // 状态
  gamepad_msgs::msg::GamepadState pad_{};
  std::optional<rclcpp::Time> last_pad_time_;
  bool prev_rb_ = false;
  bool prev_lb_ = false;
  bool prev_gripper_btn_ = false;
  bool gripper_open_ = true;
  std::size_t selected_ = 0;
  double omega_ = 0.0;
  double dt_;

  rclcpp::Subscription<gamepad_msgs::msg::GamepadState>::SharedPtr state_sub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointTeleop>());
  rclcpp::shutdown();
  return 0;
}
