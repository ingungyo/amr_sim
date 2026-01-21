#ifndef HAMR_SWERVE_CONTROLLER__CONTROL_MODEL_HPP_
#define HAMR_SWERVE_CONTROLLER__CONTROL_MODEL_HPP_

#include "swerve_controller/drive_module.hpp"
#include "swerve_controller/states.hpp"

#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <utility>

namespace swerve_controller
{

// Normalize an angle to the range [-pi, pi]
inline double normalize_angle(double angle_in_radians)
{
    // atan2 returns values in [-pi, pi]
    return std::atan2(std::sin(angle_in_radians), std::cos(angle_in_radians));
}

inline double difference_between_angles(double starting_angle_in_radians, double ending_angle_in_radians)
{
    double start = normalize_angle(starting_angle_in_radians);
    double end = normalize_angle(ending_angle_in_radians);
    double diff = end - start;

    if (diff > M_PI) {
        diff -= 2 * M_PI;
    } else if (diff < -M_PI) {
        diff += 2 * M_PI;
    }
    return diff;
}

class ControlModelBase
{
public:
    virtual ~ControlModelBase() = default;
    virtual BodyMotion body_motion_from_wheel_module_states(const std::vector<DriveModuleMeasuredValues>& states) = 0;
    virtual std::vector<std::pair<DriveModuleDesiredValues, DriveModuleDesiredValues>> state_of_wheel_modules_from_body_motion(const BodyMotion& state) = 0;
};

class SimpleFourWheelSteeringControlModel : public ControlModelBase
{
public:
    explicit SimpleFourWheelSteeringControlModel(const std::vector<DriveModule>& drive_modules);

    BodyMotion body_motion_from_wheel_module_states(const std::vector<DriveModuleMeasuredValues>& states) override;
    std::vector<std::pair<DriveModuleDesiredValues, DriveModuleDesiredValues>> state_of_wheel_modules_from_body_motion(const BodyMotion& state) override;

private:
    std::vector<DriveModule> modules_;
    Eigen::MatrixXd inverse_kinematics_matrix_;
    Eigen::MatrixXd forward_kinematics_matrix_;
};

} // namespace hamr_swerve_controller

#endif // HAMR_SWERVE_CONTROLLER__CONTROL_MODEL_HPP_
