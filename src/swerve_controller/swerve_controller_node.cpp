#include <memory>
#include <vector>
#include <string>
#include <chrono>
#include <cmath>
#include <optional>
#include <algorithm>
#include <limits>

#include "swerve_controller/swerve_controller_node.hpp"

namespace swerve_controller
{

SwerveControllerNode::SwerveControllerNode()
  : rclcpp::Node("swerve_controller_cpp")
{
  declare_parameters();

  robot_base_frame_ = get_parameter("robot_base_frame").as_string();
  twist_topic_ = get_parameter("twist_topic").as_string();
  position_controller_name_ = get_parameter("position_controller_name").as_string();
  velocity_controller_name_ = get_parameter("velocity_controller_name").as_string();
  odom_topic_ = get_parameter("odom_topic").as_string();
  joint_state_topic_ = get_parameter("joint_state_topic").as_string();
  cycle_frequency_hz_ = get_parameter("cycle_fequency").as_int();
  steering_angle_limit_degrees_ = get_parameter("steering_angle_limit_degrees").as_double();
  steering_angle_limit_rad_ = steering_angle_limit_degrees_ * M_PI / 180.0;

  const std::string steering_topic = "/" + position_controller_name_ + "/commands";
  const std::string velocity_topic = "/" + velocity_controller_name_ + "/commands";

  steering_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(steering_topic, rclcpp::QoS(10));
  velocity_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(velocity_topic, rclcpp::QoS(10));
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, rclcpp::QoS(10));

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    joint_state_topic_, rclcpp::QoS(10),
    std::bind(&SwerveControllerNode::on_joint_state, this, std::placeholders::_1));

  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    twist_topic_, rclcpp::QoS(10),
    std::bind(&SwerveControllerNode::on_cmd_vel, this, std::placeholders::_1));

  drive_modules_ = get_drive_modules();
  control_model_ = std::make_unique<SimpleFourWheelSteeringControlModel>(drive_modules_);

  body_state_ = BodyState();
  module_states_.resize(drive_modules_.size());
  for (size_t i = 0; i < drive_modules_.size(); ++i) {
    module_states_[i].name = drive_modules_[i].name;
    module_states_[i].position_in_body_coordinates = drive_modules_[i].steering_axis_xy_position;
  }

  last_cmd_time_ = now();
  last_state_update_time_ = now();

  timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / std::max(1, cycle_frequency_hz_)),
    std::bind(&SwerveControllerNode::on_timer, this));

  RCLCPP_INFO(
    get_logger(), "SwerveControllerNode started. pub: %s, %s; sub: %s, %s",
    steering_topic.c_str(), velocity_topic.c_str(), joint_state_topic_.c_str(), twist_topic_.c_str());
}

void SwerveControllerNode::declare_parameters()
{
  declare_parameter("robot_base_frame", "base_footprint");
  declare_parameter("twist_topic", "cmd_vel");
  declare_parameter("position_controller_name", "position_controller");
  declare_parameter("velocity_controller_name", "velocity_controller");
  declare_parameter("odom_topic", "/odom");
  declare_parameter("joint_state_topic", "joint_states");
  declare_parameter("cycle_fequency", 50);
  declare_parameter("steering_angle_limit_degrees", 100.0);

  declare_parameter("robot_geometry.wheel_base", 0.46);
  declare_parameter("robot_geometry.wheel_track", 0.42);
  declare_parameter("robot_geometry.wheel_radius", 0.0865);
  declare_parameter("robot_geometry.wheel_width", 0.1);

  declare_parameter<std::vector<std::string>>("steering_joints", std::vector<std::string>{});
  declare_parameter<std::vector<std::string>>("drive_joints", std::vector<std::string>{});

  std::vector<std::string> module_names = {"left_front", "left_rear", "right_rear", "right_front"};
  for (const auto & name : module_names) {
    declare_parameter("drive_modules." + name + ".steering_motor_maximum_velocity", 3.14);
    declare_parameter("drive_modules." + name + ".steering_motor_minimum_acceleration", 0.05);
    declare_parameter("drive_modules." + name + ".steering_motor_maximum_acceleration", 0.5);
    declare_parameter("drive_modules." + name + ".drive_motor_maximum_velocity", 2.0);
    declare_parameter("drive_modules." + name + ".drive_motor_minimum_acceleration", 0.05);
    declare_parameter("drive_modules." + name + ".drive_motor_maximum_acceleration", 0.5);
  }
}

std::vector<DriveModule> SwerveControllerNode::get_drive_modules()
{
  std::vector<DriveModule> modules;

  double wheel_base = get_parameter("robot_geometry.wheel_base").as_double();
  double wheel_track = get_parameter("robot_geometry.wheel_track").as_double();
  double wheel_radius = get_parameter("robot_geometry.wheel_radius").as_double();
  double wheel_width = get_parameter("robot_geometry.wheel_width").as_double();

  auto steering_joint_names = get_parameter("steering_joints").as_string_array();
  auto drive_joint_names = get_parameter("drive_joints").as_string_array();

  struct ModuleConfig { std::string name; double x; double y; };
  std::vector<ModuleConfig> configs = {
    // {"left_front", 0.5 * wheel_base, 0.5 * wheel_track},
    // {"left_rear", -0.5 * wheel_base, 0.5 * wheel_track},
    // {"right_rear", -0.5 * wheel_base, -0.5 * wheel_track},
    // {"right_front", 0.5 * wheel_base, -0.5 * wheel_track}
    {"right_front", 0.5 * wheel_base, -0.5 * wheel_track},
    {"left_rear", -0.5 * wheel_base, 0.5 * wheel_track}
  };

  for (const auto & cfg : configs) {
    std::string steering_joint;
    for (const auto & j : steering_joint_names) {
      if (j.find(cfg.name) != std::string::npos) {
        steering_joint = j;
      }
    }
    if (steering_joint.empty()) {
      steering_joint = "joint_steering_" + cfg.name;
    }

    std::string drive_joint;
    for (const auto & j : drive_joint_names) {
      if (j.find(cfg.name) != std::string::npos) {
        drive_joint = j;
      }
    }
    if (drive_joint.empty()) {
      drive_joint = "joint_drive_" + cfg.name;
    }

    modules.emplace_back(
      cfg.name, steering_joint, drive_joint, Point{cfg.x, cfg.y, 0.0},
      wheel_radius, wheel_width,
      get_parameter("drive_modules." + cfg.name + ".steering_motor_maximum_velocity").as_double(),
      get_parameter("drive_modules." + cfg.name + ".steering_motor_minimum_acceleration").as_double(),
      get_parameter("drive_modules." + cfg.name + ".steering_motor_maximum_acceleration").as_double(),
      get_parameter("drive_modules." + cfg.name + ".drive_motor_maximum_velocity").as_double(),
      get_parameter("drive_modules." + cfg.name + ".drive_motor_minimum_acceleration").as_double(),
      get_parameter("drive_modules." + cfg.name + ".drive_motor_maximum_acceleration").as_double()
    );
  }
  return modules;
}

void SwerveControllerNode::on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  for (size_t i = 0; i < drive_modules_.size(); ++i) {
    for (size_t j = 0; j < msg->name.size(); ++j) {
      if (msg->name[j] == drive_modules_[i].steering_link_name) {
        module_states_[i].orientation_in_body_coordinates.z = msg->position[j];
        module_states_[i].orientation_velocity_in_body_coordinates.z = msg->velocity[j];
        
      }
      if (msg->name[j] == drive_modules_[i].driving_link_name) {
        module_states_[i].drive_velocity_in_module_coordinates.x = msg->velocity[j] * drive_modules_[i].wheel_radius;
      }
    }
  }
}

void SwerveControllerNode::on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  last_cmd_ = *msg;
  last_cmd_time_ = now();
}

void SwerveControllerNode::on_timer()
{
  rclcpp::Time current_time = now();
  double dt = (current_time - last_state_update_time_).seconds();
  if (dt <= 0.0) {
    dt = 0.001;
  }

  update_odometry(dt);
  publish_odometry();
  last_state_update_time_ = current_time;

  if (last_cmd_) {
    BodyMotion desired_motion;
    desired_motion.linear_velocity = {last_cmd_->linear.x, last_cmd_->linear.y, 0.0};
    desired_motion.angular_velocity = {0.0, 0.0, last_cmd_->angular.z};

    auto drive_module_potential_states = control_model_->state_of_wheel_modules_from_body_motion(desired_motion);

    auto is_within_angle_limit = [this](double angle) {
      if (std::isinf(angle)) {
        return false;
      }
      double normalized = normalize_angle(angle);
      return std::abs(normalized) <= steering_angle_limit_rad_;
    };

    std_msgs::msg::Float64MultiArray steering_msg;
    std_msgs::msg::Float64MultiArray velocity_msg;

    for (size_t i = 0; i < drive_modules_.size(); ++i) {
      auto potential_states = drive_module_potential_states[i];

      DriveModuleDesiredValues chosen_state;

      double current_angle = module_states_[i].orientation_in_body_coordinates.z;
      double forward_angle = potential_states.first.steering_angle_in_radians;
      double reverse_angle = potential_states.second.steering_angle_in_radians;

      bool forward_valid = is_within_angle_limit(forward_angle);
      bool reverse_valid = is_within_angle_limit(reverse_angle);

      if (std::isinf(forward_angle)) {
        chosen_state = potential_states.first;
        chosen_state.steering_angle_in_radians = current_angle;
        chosen_state.drive_velocity_in_meters_per_second = 0.0;
      } else if (forward_valid && reverse_valid) {
        double diff_forward = std::abs(difference_between_angles(current_angle, forward_angle));
        double diff_reverse = std::abs(difference_between_angles(current_angle, reverse_angle));
        chosen_state = (diff_forward <= diff_reverse) ? potential_states.first : potential_states.second;
      } else if (forward_valid) {
        chosen_state = potential_states.first;
      } else if (reverse_valid) {
        chosen_state = potential_states.second;
      } else {
        double diff_forward = std::abs(difference_between_angles(current_angle, forward_angle));
        double diff_reverse = std::abs(difference_between_angles(current_angle, reverse_angle));
        chosen_state = (diff_forward <= diff_reverse) ? potential_states.first : potential_states.second;
      }

      steering_msg.data.push_back(chosen_state.steering_angle_in_radians);
      double wheel_omega = chosen_state.drive_velocity_in_meters_per_second / drive_modules_[i].wheel_radius;
      velocity_msg.data.push_back(wheel_omega);
    }

    steering_pub_->publish(steering_msg);
    velocity_pub_->publish(velocity_msg);
  }
}

void SwerveControllerNode::update_odometry(double dt)
{
  BodyMotion body_motion = control_model_->body_motion_from_wheel_module_states(module_states_);

  double local_dx = (body_state_.motion_in_body_coordinates.linear_velocity.x + body_motion.linear_velocity.x) * 0.5 * dt;
  double local_dy = (body_state_.motion_in_body_coordinates.linear_velocity.y + body_motion.linear_velocity.y) * 0.5 * dt;
  double d_theta = (body_state_.motion_in_body_coordinates.angular_velocity.z + body_motion.angular_velocity.z) * 0.5 * dt;

  double avg_theta = body_state_.orientation_in_world_coordinates.z + d_theta * 0.5;

  double global_dx = local_dx * std::cos(avg_theta) - local_dy * std::sin(avg_theta);
  double global_dy = local_dx * std::sin(avg_theta) + local_dy * std::cos(avg_theta);

  body_state_.position_in_world_coordinates.x += global_dx;
  body_state_.position_in_world_coordinates.y += global_dy;
  body_state_.orientation_in_world_coordinates.z += d_theta;

  body_state_.orientation_in_world_coordinates.z = std::atan2(std::sin(body_state_.orientation_in_world_coordinates.z), std::cos(body_state_.orientation_in_world_coordinates.z));

  body_state_.motion_in_body_coordinates = body_motion;
}

void SwerveControllerNode::publish_odometry()
{
  auto msg = nav_msgs::msg::Odometry();
  msg.header.stamp = now();
  msg.header.frame_id = "odom";
  msg.child_frame_id = robot_base_frame_;

  msg.pose.pose.position.x = body_state_.position_in_world_coordinates.x;
  msg.pose.pose.position.y = body_state_.position_in_world_coordinates.y;
  msg.pose.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0, 0, body_state_.orientation_in_world_coordinates.z);
  msg.pose.pose.orientation = tf2::toMsg(q);

  msg.twist.twist.linear.x = body_state_.motion_in_body_coordinates.linear_velocity.x;
  msg.twist.twist.linear.y = body_state_.motion_in_body_coordinates.linear_velocity.y;
  msg.twist.twist.angular.z = body_state_.motion_in_body_coordinates.angular_velocity.z;

  odom_pub_->publish(msg);

  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = msg.header.stamp;
  tf.header.frame_id = "odom";
  tf.child_frame_id = robot_base_frame_;
  tf.transform.translation.x = msg.pose.pose.position.x;
  tf.transform.translation.y = msg.pose.pose.position.y;
  tf.transform.translation.z = 0.0;
  tf.transform.rotation = msg.pose.pose.orientation;
  tf_broadcaster_->sendTransform(tf);
}

}  // namespace hamr_swerve_controller

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<swerve_controller::SwerveControllerNode>());
  rclcpp::shutdown();
  return 0;
}
