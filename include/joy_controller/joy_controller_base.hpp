#ifndef JOY_CONTROLLER_BASE_HPP_
#define JOY_CONTROLLER_BASE_HPP_

/* std_msgs/Header header
builtin_interfaces/Time stamp
    int32 sec
    uint32 nanosec
string frame_id

# The axes measurements from a joystick.
float32[] axes

# The buttons measurements from a joystick.
int32[] buttons
*/

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "rclcpp/rclcpp.hpp"
class JoyTeleopBase : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit JoyTeleopBase(const std::string & node_name,const rclcpp::NodeOptions & options)
  : LifecycleNode(node_name, options)
  {
    declareCommonParameters();
  }

protected:
  //lifecycle callbacks
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

  // ===== 子类必须实现的接口 =====
  virtual void handleAxes(const sensor_msgs::msg::Joy & msg) = 0;
  virtual void handleButtonPressed(size_t button) {}
  virtual void handleButtonReleased(size_t button) {}

  /// lifecycle hook
  virtual void onTeleopActivated() {}
  virtual void onTeleopDeactivated() {}

protected:
  bool isActive() const { return is_activated_; }

private:
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void declareCommonParameters();

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  bool is_activated_{false};
  std::vector<int32_t> last_buttons_;
};


#endif