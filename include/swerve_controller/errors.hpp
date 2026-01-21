#pragma once
#include <stdexcept>

namespace swerve_controller {

  struct IncompleteTrajectoryException : public std::runtime_error {
    using std::runtime_error::runtime_error;
  };

  struct InvalidTimeFractionException : public std::runtime_error {
    using std::runtime_error::runtime_error;
  };

} // namespace swerve_controller
