#pragma once

#include "joy_controller/joy_controller_base.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

class DiffDriveJoyTeleop : public JoyTeleopBase
{
public:
  explicit DiffDriveJoyTeleop(const rclcpp::NodeOptions & options);

protected:
  // lifecycle overrides (扩展而不是替换)
  CallbackReturn on_configure(
    const rclcpp_lifecycle::State & state) override;

  CallbackReturn on_activate(
    const rclcpp_lifecycle::State & state) override;

  CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & state) override;

  CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & state) override;

  // Joy semantic handlers
  void handleAxes(const sensor_msgs::msg::Joy & joy) override;
  void handleButtonPressed(size_t button) override;

private:
  // ROS interfaces
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr
    cmd_vel_pub_;

  // Parameters
  int axis_linear_{1};
  int axis_angular_{0};
  double scale_linear_{0.5};
  double scale_angular_{1.0};
};
