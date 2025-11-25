#pragma once
#include "spg/Init.hpp"

namespace spg {
namespace setpoint {

/**
 * @brief Humanoid-aware setpoint update function
 * 
 * This function generates setpoints for humanoid robots that have directional
 * velocity and acceleration constraints, and orientation-dependent movement efficiency.
 * 
 * @param d SPG state containing robot state, parameters, and targets
 * @return Updated SPG state with humanoid-aware setpoint
 */
SPGState SetHumanoid(SPGState& d);

} // namespace setpoint
} // namespace spg