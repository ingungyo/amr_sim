#include "swerve_controller/control_model.hpp"
#include <iostream>
#include <algorithm>
#include <limits>

namespace swerve_controller
{

SimpleFourWheelSteeringControlModel::SimpleFourWheelSteeringControlModel(const std::vector<DriveModule>& drive_modules)
: modules_(drive_modules)
{
    inverse_kinematics_matrix_.resize(2 * modules_.size(), 3);
    for (size_t i = 0; i < modules_.size(); ++i) {
        inverse_kinematics_matrix_.row(2 * i) << 1.0, 0.0, -modules_[i].steering_axis_xy_position.y;
        inverse_kinematics_matrix_.row(2 * i + 1) << 0.0, 1.0, modules_[i].steering_axis_xy_position.x;
    }
    
    // Moore-Penrose pseudo-inverse for the forward kinematics
    forward_kinematics_matrix_ = inverse_kinematics_matrix_.completeOrthogonalDecomposition().pseudoInverse();
}

BodyMotion SimpleFourWheelSteeringControlModel::body_motion_from_wheel_module_states(const std::vector<DriveModuleMeasuredValues>& states)
{
    Eigen::VectorXd drive_state_vector(2 * states.size());
    for (size_t i = 0; i < states.size(); ++i) {
        auto [v_x, v_y] = states[i].xy_drive_velocity();
        drive_state_vector(2 * i) = v_x;
        drive_state_vector(2 * i + 1) = v_y;
    }

    Eigen::Vector3d body_state_vector = forward_kinematics_matrix_ * drive_state_vector;

    BodyMotion motion;
    motion.linear_velocity = {body_state_vector(0), body_state_vector(1), 0.0};
    motion.angular_velocity = {0.0, 0.0, body_state_vector(2)};
    return motion;
}

std::vector<std::pair<DriveModuleDesiredValues, DriveModuleDesiredValues>> SimpleFourWheelSteeringControlModel::state_of_wheel_modules_from_body_motion(const BodyMotion& state)
{
    Eigen::Vector3d body_state_vector;
    body_state_vector << state.linear_velocity.x, state.linear_velocity.y, state.angular_velocity.z;

    Eigen::VectorXd drive_state_vector = inverse_kinematics_matrix_ * body_state_vector;

    std::vector<double> drive_velocities;
    double max_velocity = 0.0;

    for (size_t i = 0; i < modules_.size(); ++i) {
        double v_x = drive_state_vector(2 * i);
        double v_y = drive_state_vector(2 * i + 1);
        double velocity = std::hypot(v_x, v_y);
        drive_velocities.push_back(velocity);
        if (velocity > max_velocity) {
            max_velocity = velocity;
        }
    }
    
    double normalization_factor = 1.0;
    if (modules_.size() > 0 && max_velocity > modules_[0].drive_motor_maximum_velocity) { 
        // Assuming all modules have same max velocity, or at least normalizing to the first one
        normalization_factor = modules_[0].drive_motor_maximum_velocity / max_velocity;
    }

    std::vector<std::pair<DriveModuleDesiredValues, DriveModuleDesiredValues>> result;
    for (size_t i = 0; i < modules_.size(); ++i) {
        double v_x = drive_state_vector(2 * i);
        double v_y = drive_state_vector(2 * i + 1);
        double drive_velocity = drive_velocities[i];

        double forward_steering_angle;
        if (std::abs(drive_velocity) < 1e-9) {
            forward_steering_angle = std::numeric_limits<double>::infinity();
        } else {
            forward_steering_angle = std::atan2(v_y, v_x);
        }
        
        double reverse_steering_angle = std::numeric_limits<double>::infinity();
        if (!std::isinf(forward_steering_angle)) {
           reverse_steering_angle = normalize_angle(forward_steering_angle + M_PI);
        }

        DriveModuleDesiredValues forward_state;
        forward_state.name = modules_[i].name;
        forward_state.steering_angle_in_radians = forward_steering_angle;
        forward_state.drive_velocity_in_meters_per_second = drive_velocity * normalization_factor;

        DriveModuleDesiredValues reverse_state;
        reverse_state.name = modules_[i].name;
        reverse_state.steering_angle_in_radians = reverse_steering_angle;
        reverse_state.drive_velocity_in_meters_per_second = -drive_velocity * normalization_factor;

        result.push_back({forward_state, reverse_state});
    }

    return result;
}

} // namespace hamr_swerve_controller

