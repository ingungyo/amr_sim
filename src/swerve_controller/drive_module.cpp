#include "swerve_controller/drive_module.hpp"

namespace swerve_controller
{

DriveModule::DriveModule(
  const std::string & name,
  const std::string & steering_link,
  const std::string & drive_link,
  const Point & steering_axis_xy_position,
  double wheel_radius,
  double wheel_width,
  double steering_motor_maximum_velocity,
  double steering_motor_minimum_acceleration,
  double steering_motor_maximum_acceleration,
  double drive_motor_maximum_velocity,
  double drive_motor_minimum_acceleration,
  double drive_motor_maximum_acceleration)
: name(name),
  steering_link_name(steering_link),
  driving_link_name(drive_link),
  steering_axis_xy_position(steering_axis_xy_position),
  wheel_radius(wheel_radius),
  wheel_width(wheel_width),
  steering_motor_maximum_velocity(steering_motor_maximum_velocity),
  steering_motor_minimum_acceleration(steering_motor_minimum_acceleration),
  steering_motor_maximum_acceleration(steering_motor_maximum_acceleration),
  drive_motor_maximum_velocity(drive_motor_maximum_velocity),
  drive_motor_minimum_acceleration(drive_motor_minimum_acceleration),
  drive_motor_maximum_acceleration(drive_motor_maximum_acceleration)
{
}

}  // namespace hamr_swerve_controller

