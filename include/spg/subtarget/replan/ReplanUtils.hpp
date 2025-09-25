#pragma once
#include <Eigen/Dense>
#include <vector>
#include "spg/Init.hpp"

namespace spg {
namespace subtarget {
namespace replan {

Subtarget determineSetpointLimits(const SPGState& d, const Subtarget& subtarget);
double getDistanceInsidePenaltyArea(const SPGState& d, const Eigen::Vector3d& pos);
bool newSubtargetDesired(const SPGState& d);
Subtarget newSubtarget(const SPGState& d, const Subtarget& subtarget);
bool notTouchingObstacle(const Eigen::Vector2d& p_robot, const Eigen::MatrixXd& p_obstacles, double obstacle_margin);
bool quickstopDesired(const SPGState& d);
Subtarget quickstop(const SPGState& d, const Subtarget& subtarget);
Subtarget toTarget(const SPGState& d);
Subtarget updateBest(const Subtarget& best, const Subtarget& candidate, const Eigen::Vector3d& target);

} // namespace replan
} // namespace subtarget
} // namespace spg
