#ifndef SWERVE_CONTROLLER__GEOMETRY_HPP_
#define SWERVE_CONTROLLER__GEOMETRY_HPP_

namespace swerve_controller
{

struct Point
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct Vector3
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct Orientation
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

}  // namespace hamr_swerve_controller

#endif  // HAMR_SWERVE_CONTROLLER__GEOMETRY_HPP_
