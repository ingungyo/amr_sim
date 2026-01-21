#ifndef SWERVE_CONTROLLER__DRIVE_MODULE_HPP_
#define SWERVE_CONTROLLER__DRIVE_MODULE_HPP_

#include "swerve_controller/geometry.hpp"
#include <string>

namespace swerve_controller
{

class DriveModule
{
public:
  DriveModule(
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
    double drive_motor_maximum_acceleration);
  
  std::string name;
  std::string steering_link_name;
  std::string driving_link_name;
  Point steering_axis_xy_position;
  double wheel_radius;
  double wheel_width;
  double steering_motor_maximum_velocity;
  double steering_motor_minimum_acceleration;
  double steering_motor_maximum_acceleration;
  double drive_motor_maximum_velocity;
  double drive_motor_minimum_acceleration;
  double drive_motor_maximum_acceleration;
};

}  // namespace hamr_swerve_controller

#endif  // HAMR_SWERVE_CONTROLLER__DRIVE_MODULE_HPP_
