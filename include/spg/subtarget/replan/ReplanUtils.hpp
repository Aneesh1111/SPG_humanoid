#pragma once
#include <Eigen/Dense>
#include <vector>
#include "spg/Init.hpp"

namespace spg {
namespace subtarget {
namespace replan {

struct CheckResults {
    bool age_threshold_reached;
    bool hasball;
    bool is_colliding;
    bool start_braking;
    bool small_angle_diff;
    bool rotate_slow_with_large_angle_diff;
    bool rotate_fast_with_small_angle_diff;
    bool is_close_to_subtarget;
    bool is_at_subtarget_soon;
    bool subtarget_at_target;
    bool finished_fast_rotation;
    bool fast_rotation_needed;
    bool move_to_worse_position;
    bool move_slowly_to_better_position;
    bool large_angle_diff_with_ball;
    bool starts_with_violation;
    bool target_outside_penalty_area_while_subtarget_inside;
};

Subtarget determineSetpointLimits(const SPGState& d, const Subtarget& subtarget);
double getDistanceInsidePenaltyArea(const SPGState& d, const Eigen::Vector3d& pos);
CheckResults getChecks(const SPGState& d);
bool newSubtargetDesired(const SPGState& d);
Subtarget newSubtarget(SPGState& d, const Subtarget& subtarget);
bool notTouchingObstacle(const Eigen::Vector2d& p_robot, const Eigen::MatrixXd& p_obstacles, double obstacle_margin);
bool quickstopDesired(const SPGState& d);
Subtarget quickstop(const SPGState& d, const Subtarget& subtarget);
Subtarget toTarget(SPGState& d);
Subtarget updateBest(const Subtarget& best, const Subtarget& candidate, const Eigen::Vector3d& target);

} // namespace replan
} // namespace subtarget
} // namespace spg
