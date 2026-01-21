#pragma once

#include <vector>
#include <utility>
#include "swerve_controller/states.hpp"
#include "swerve_controller/control_model.hpp"

namespace swerve_controller {

class MotionCommandBase {
public:
  virtual ~MotionCommandBase() = default;
  
  virtual double time_for_motion() const = 0;

  virtual BodyMotion to_body_state(ControlModelBase& model) const = 0;
  
  virtual std::pair<std::vector<DriveModuleDesiredValues>, std::vector<DriveModuleDesiredValues>>
  to_drive_module_state(ControlModelBase& model) const = 0;
};

class BodyMotionCommand : public MotionCommandBase {
public:
  double time_span;
  Vector3 linear_velocity;
  Vector3 angular_velocity;

  BodyMotionCommand(
    double time_span,
    double linear_x_velocity_in_meters_per_second,
    double linear_y_velocity_in_meters_per_second,
    double angular_z_velocity_in_radians_per_second): 
    time_span(time_span),
    linear_velocity(linear_x_velocity_in_meters_per_second, linear_y_velocity_in_meters_per_second, 0.0),
    angular_velocity(0.0, 0.0, angular_z_velocity_in_radians_per_second) {}

  double time_for_motion() const override { 
    return time_span; 
  }

  BodyMotion to_body_state(ControlModelBase& model) const override {
    return BodyMotion(
      linear_velocity.x,
      linear_velocity.y,
      angular_velocity.z,
      0.0, 0.0, 0.0,
      0.0, 0.0, 0.0
    );
  }

  std::pair<std::vector<DriveModuleDesiredValues>, std::vector<DriveModuleDesiredValues>>
  to_drive_module_state(ControlModelBase& model) const override {
    const auto drive_module_potential_states = model.state_of_wheel_modules_from_body_motion(to_body_state(model));
    std::vector<DriveModuleDesiredValues> forward_states; forward_states.reserve(drive_module_potential_states.size());
    std::vector<DriveModuleDesiredValues> reverse_states; reverse_states.reserve(drive_module_potential_states.size());
    for (const auto& x : drive_module_potential_states) {
      forward_states.push_back(x.first); 
      reverse_states.push_back(x.second); 
    }
    return {forward_states, reverse_states};
  }
};

class DriveModuleMotionCommand : public MotionCommandBase {
public:
  double time_span;
  std::vector<DriveModuleDesiredValues> desired_states;

  DriveModuleMotionCommand(
    double time_span, 
    const std::vector<DriveModuleDesiredValues>& desired_states): 
    time_span(time_span), 
    desired_states(desired_states) {}

  double time_for_motion() const override {
    return time_span; 
  }

  BodyMotion to_body_state(ControlModelBase& model) const override {
    return model.body_motion_from_wheel_module_states(desired_states);
  }

  std::pair<std::vector<DriveModuleDesiredValues>, std::vector<DriveModuleDesiredValues>>
  to_drive_module_state(ControlModelBase& model) const override {
    return { desired_states, {} };
  }
};

} // namespace swerve_controller


