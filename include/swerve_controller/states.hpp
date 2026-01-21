#ifndef SWERVE_CONTROLLER__STATES_HPP_
#define SWERVE_CONTROLLER__STATES_HPP_

#include "swerve_controller/geometry.hpp"
#include <string>
#include <cmath>
#include <tuple>

namespace swerve_controller
{

struct BodyMotion
{
    Vector3 linear_velocity;
    Vector3 angular_velocity;
    Vector3 linear_acceleration;
    Vector3 angular_acceleration;
    Vector3 linear_jerk;
    Vector3 angular_jerk;
};

struct BodyState
{
    Point position_in_world_coordinates;
    Orientation orientation_in_world_coordinates;
    BodyMotion motion_in_body_coordinates;
};

struct DriveModuleDesiredValues
{
    std::string name;
    double steering_angle_in_radians = 0.0;
    double drive_velocity_in_meters_per_second = 0.0;
};

struct DriveModuleMeasuredValues
{
    std::string name;
    Point position_in_body_coordinates;
    Orientation orientation_in_body_coordinates;

    Vector3 drive_velocity_in_module_coordinates;
    Vector3 orientation_velocity_in_body_coordinates;

    Vector3 drive_acceleration_in_module_coordinates;
    Vector3 orientation_acceleration_in_body_coordinates;

    Vector3 drive_jerk_in_module_coordinates;
    Vector3 orientation_jerk_in_body_coordinates;

    // Python의 xy_drive_velocity 메서드
    std::tuple<double, double> xy_drive_velocity() const
    {
        double v_x = drive_velocity_in_module_coordinates.x * std::cos(orientation_in_body_coordinates.z);
        double v_y = drive_velocity_in_module_coordinates.x * std::sin(orientation_in_body_coordinates.z);
        return {v_x, v_y};
    }
};

}  // namespace hamr_swerve_controller

#endif  // HAMR_SWERVE_CONTROLLER__STATES_HPP_
