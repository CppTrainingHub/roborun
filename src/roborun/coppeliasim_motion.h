#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "roborun/types.h"

namespace roborun {

inline int ComputeCoppeliaSimMotionSteps(const JointPositions& initial,
                                         const JointPositions& target,
                                         double joint_speed_radians_per_second,
                                         double simulation_time_step_seconds) {
  if (!std::isfinite(joint_speed_radians_per_second) || joint_speed_radians_per_second <= 0.0 ||
      !std::isfinite(simulation_time_step_seconds) || simulation_time_step_seconds <= 0.0) {
    throw std::runtime_error("CoppeliaSim motion timing requires positive finite values");
  }
  double maximum_distance = 0.0;
  for (std::size_t index = 0; index < kJointCount; ++index) {
    const double distance = std::abs(target[index] - initial[index]);
    if (!std::isfinite(distance)) {
      throw std::runtime_error("CoppeliaSim motion distance must be finite");
    }
    maximum_distance = std::max(maximum_distance, distance);
  }
  const double required_steps =
      std::ceil(maximum_distance / (joint_speed_radians_per_second * simulation_time_step_seconds));
  if (required_steps > static_cast<double>(std::numeric_limits<int>::max())) {
    throw std::runtime_error("CoppeliaSim motion requires too many simulation steps");
  }
  return std::max(1, static_cast<int>(required_steps));
}

}  // namespace roborun
