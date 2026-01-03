
#include "joy_controller/diff_drive_joy_teleop.hpp"
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

DiffDriveJoyTeleop::DiffDriveJoyTeleop(
  const rclcpp::NodeOptions & options)
: JoyTeleopBase("diff_drive_joy_teleop", options)
{
  this->declare_parameter<int>("axis_linear", axis_linear_);
  this->declare_parameter<int>("axis_angular", axis_angular_);
  this->declare_parameter<double>("scale_linear", scale_linear_);
  this->declare_parameter<double>("scale_angular", scale_angular_);
}

CallbackReturn DiffDriveJoyTeleop::on_configure(
  const rclcpp_lifecycle::State & state)
{
  auto ret = JoyTeleopBase::on_configure(state);
  if (ret != CallbackReturn::SUCCESS) {
    return ret;
  }

  axis_linear_  = this->get_parameter("axis_linear").as_int();
  axis_angular_ = this->get_parameter("axis_angular").as_int();
  scale_linear_ = this->get_parameter("scale_linear").as_double();
  scale_angular_= this->get_parameter("scale_angular").as_double();

  RCLCPP_INFO(
    this->get_logger(),
    "DiffDrive configured: lin_axis=%d ang_axis=%d lin_scale=%.2f ang_scale=%.2f",
    axis_linear_, axis_angular_, scale_linear_, scale_angular_);

  cmd_vel_pub_ =
    this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  return CallbackReturn::SUCCESS;
}

CallbackReturn DiffDriveJoyTeleop::on_activate(
  const rclcpp_lifecycle::State & state)
{
  if (cmd_vel_pub_) {
    cmd_vel_pub_->on_activate();
  }

  return JoyTeleopBase::on_activate(state);//先激活子类on_activate，再激活基类on_activate
}

CallbackReturn DiffDriveJoyTeleop::on_deactivate(
  const rclcpp_lifecycle::State & state)
{
  //先让基类失效 joy 处理
  auto ret = JoyTeleopBase::on_deactivate(state);

  if (cmd_vel_pub_ && cmd_vel_pub_->is_activated()) {
    geometry_msgs::msg::Twist stop;
    cmd_vel_pub_->publish(stop);
    cmd_vel_pub_->on_deactivate();
  }
  return ret;
}

CallbackReturn DiffDriveJoyTeleop::on_cleanup(const rclcpp_lifecycle::State & state)
{
  cmd_vel_pub_.reset();
  return JoyTeleopBase::on_cleanup(state);
}

/* ================= Joy handlers ================= */

void DiffDriveJoyTeleop::handleAxes(
  const sensor_msgs::msg::Joy & joy)
{
  if (!cmd_vel_pub_ || !cmd_vel_pub_->is_activated()) {
    return;
  }
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x  = joy.axes[axis_linear_]  * scale_linear_;
  cmd.angular.z = joy.axes[axis_angular_] * scale_angular_;
  cmd_vel_pub_->publish(cmd);
}

void DiffDriveJoyTeleop::handleButtonPressed(size_t button)
{
  // （button 0）急停
  if (button == 0 && cmd_vel_pub_ && cmd_vel_pub_->is_activated()) {
    RCLCPP_WARN(this->get_logger(), "Emergency stop!");

    geometry_msgs::msg::Twist stop;
    cmd_vel_pub_->publish(stop);
  }
    switch (button) {
        case 1:
            RCLCPP_INFO(this->get_logger(), "button 1 process");
            break;
        case 2:
            RCLCPP_INFO(this->get_logger(), "button 2 process");
            break;
        case 3:
            RCLCPP_INFO(this->get_logger(), "button 3 process");
            break;
        case 4:
            RCLCPP_INFO(this->get_logger(), "button 4 process");
            break;
        case 5:
            RCLCPP_INFO(this->get_logger(), "button 5 process");
            break;
        case 6:
            RCLCPP_INFO(this->get_logger(), "button 6 process");
            break;
        case 7:
            RCLCPP_INFO(this->get_logger(), "button 7 process");
            break;
        case 8:
            RCLCPP_INFO(this->get_logger(), "button 8 process");
            break;
        case 9:
            RCLCPP_INFO(this->get_logger(), "button 9 process");
            break;
        case 10:
            RCLCPP_INFO(this->get_logger(), "button 10 process");
            break;
        default:
            RCLCPP_WARN(this->get_logger(), "no such button !!!!!!!!");
            break;
    }

}



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<DiffDriveJoyTeleop>(
    rclcpp::NodeOptions().use_intra_process_comms(true));

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
