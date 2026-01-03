#include "joy_controller/joy_controller_base.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

void JoyTeleopBase::declareCommonParameters()
{
  declare_parameter("deadzone", 0.1);
}

CallbackReturn JoyTeleopBase::on_configure(const rclcpp_lifecycle::State &)
{
  joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
    "joy", 5,
    std::bind(&JoyTeleopBase::joyCallback, this, std::placeholders::_1));

  return CallbackReturn::SUCCESS;
}

CallbackReturn JoyTeleopBase::on_activate(const rclcpp_lifecycle::State &)
{
  is_activated_ = true;
  onTeleopActivated();
  return CallbackReturn::SUCCESS;
}

CallbackReturn JoyTeleopBase::on_deactivate(const rclcpp_lifecycle::State &)
{
  is_activated_ = false;
  onTeleopDeactivated();
  return CallbackReturn::SUCCESS;
}

CallbackReturn JoyTeleopBase::on_cleanup(const rclcpp_lifecycle::State &)
{
  joy_sub_.reset();
  last_buttons_.clear();
  return CallbackReturn::SUCCESS;
}

CallbackReturn JoyTeleopBase::on_shutdown(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

void JoyTeleopBase::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  if (!is_activated_) return;

  // 初始化按钮状态
  if (last_buttons_.empty()) {
    last_buttons_ = msg->buttons;
  }
  handleAxes(*msg);
  for (size_t i = 0; i < msg->buttons.size(); ++i) {
    if (msg->buttons[i] == 1 && last_buttons_[i] == 0) {
      handleButtonPressed(i);//按下事件
    } else if (msg->buttons[i] == 0 && last_buttons_[i] == 1) {
      handleButtonReleased(i);//释放事件
    }
  }

  last_buttons_ = msg->buttons;
}
