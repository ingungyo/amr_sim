#ifndef SWERVE_CONTROLLER__SWERVE_CONTROLLER_NODE_HPP_
#define SWERVE_CONTROLLER__SWERVE_CONTROLLER_NODE_HPP_

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "swerve_controller/control_model.hpp"
#include "swerve_controller/drive_module.hpp"
#include "swerve_controller/states.hpp"

namespace swerve_controller
{

class SwerveControllerNode : public rclcpp::Node
{
public:
  SwerveControllerNode();

private:
  void declare_parameters();
  std::vector<DriveModule> get_drive_modules();
  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg);
  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg);
  void on_timer();
  void update_odometry(double dt);
  void publish_odometry();

  std::string robot_base_frame_;
  std::string twist_topic_;
  std::string position_controller_name_;
  std::string velocity_controller_name_;
  std::string odom_topic_;
  std::string joint_state_topic_;
  int cycle_frequency_hz_{50};
  double steering_angle_limit_degrees_{100.0};
  double steering_angle_limit_rad_{100.0 * M_PI / 180.0};

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr steering_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::optional<geometry_msgs::msg::Twist> last_cmd_;
  rclcpp::Time last_cmd_time_;
  rclcpp::Time last_state_update_time_;

  std::vector<DriveModule> drive_modules_;
  std::unique_ptr<ControlModelBase> control_model_;

  BodyState body_state_;
  std::vector<DriveModuleMeasuredValues> module_states_;
};

}  // namespace hamr_swerve_controller

#endif  // HAMR_SWERVE_CONTROLLER__SWERVE_CONTROLLER_NODE_HPP_

