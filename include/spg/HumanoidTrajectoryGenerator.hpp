// Example integration of humanoid-aware SPG system
// This shows how to modify the main trajectory generation to use humanoid constraints

#include "spg/Init.hpp" 
#include "spg/HumanoidUtils.hpp"
#include "spg/setpoint/BalanceXY.hpp"
#include "spg/setpoint/Segment.hpp"

namespace spg {
namespace humanoid {

/**
 * @brief Enhanced trajectory generation considering humanoid robot constraints
 * @param d SPG state containing robot parameters and current state
 * @param target_pos Target position in global coordinates
 * @param target_vel Target velocity in global coordinates  
 * @return Generated trajectory segments
 */
std::vector<setpoint::Segment> generateHumanoidTrajectory(SPGState& d, 
                                               const Eigen::Vector3d& target_pos,
                                               const Eigen::Vector3d& target_vel) {
    
    // Get current robot state
    Eigen::Vector3d current_pos = d.input.robot.p;
    Eigen::Vector3d current_vel = d.input.robot.v;
    double robot_orientation = d.input.robot.IMU_orientation(2); // Assuming yaw is in z-component
    
    // Calculate movement direction for efficiency assessment
    Eigen::Vector3d movement_direction = target_pos - current_pos;
    
    // Get effective velocity and acceleration limits considering orientation
    Eigen::Vector3d effective_vmax = calculateEffectiveVelocityLimits(d.par, robot_orientation, movement_direction);
    Eigen::Vector3d effective_amax = calculateEffectiveAccelerationLimits(d.par, robot_orientation, movement_direction);
    Eigen::Vector3d effective_dmax = effective_amax; // Use same for deceleration
    
    // Initialize trajectory segments
    std::vector<setpoint::Segment> segments(4); // acceleration, const vel, deceleration, stop
    
    // Use humanoid-aware balancing
    Eigen::Vector3d balanced_vmax, balanced_amax, balanced_dmax;
    setpoint::balanceXY_humanoid(segments, current_pos, current_vel, target_pos, target_vel,
                                d.par, robot_orientation, 
                                balanced_vmax, balanced_amax, balanced_dmax);
    
    return segments;
}

/**
 * @brief Calculate rotation strategy that minimizes movement penalty
 * @param d SPG state
 * @param target_pos Target position
 * @param current_orientation Current robot orientation [rad]
 * @return Optimal orientation strategy
 */
struct OrientationStrategy {
    double target_orientation;      // Optimal orientation to face [rad]
    bool should_rotate_first;      // Whether to rotate before moving
    double rotation_movement_blend; // How much to blend rotation with movement (0=sequential, 1=simultaneous)
};

OrientationStrategy calculateOptimalOrientation(const SPGState& d,
                                              const Eigen::Vector3d& target_pos,
                                              double current_orientation) {
    OrientationStrategy strategy;
    
    // Calculate direction to target
    Eigen::Vector2d direction_to_target = (target_pos.head<2>() - d.input.robot.p.head<2>());
    double distance_to_target = direction_to_target.norm();
    
    if (distance_to_target < 1e-6) {
        // Already at target, no rotation needed
        strategy.target_orientation = current_orientation;
        strategy.should_rotate_first = false;
        strategy.rotation_movement_blend = 1.0;
        return strategy;
    }
    
    // Calculate optimal orientation (facing target)
    strategy.target_orientation = std::atan2(direction_to_target(1), direction_to_target(0));
    
    // Calculate angle difference
    double angle_diff = strategy.target_orientation - current_orientation;
    // Normalize to [-pi, pi]
    while (angle_diff > M_PI) angle_diff -= 2*M_PI;
    while (angle_diff < -M_PI) angle_diff += 2*M_PI;
    
    // Decision logic for rotation strategy
    double abs_angle_diff = std::abs(angle_diff);
    
    if (abs_angle_diff < M_PI/6) { // < 30 degrees
        // Small angle difference - move while rotating
        strategy.should_rotate_first = false;
        strategy.rotation_movement_blend = 1.0; // Simultaneous
    } else if (abs_angle_diff < M_PI/2) { // 30-90 degrees  
        // Medium angle difference - blend rotation and movement
        strategy.should_rotate_first = false;
        strategy.rotation_movement_blend = 0.7; // Mostly simultaneous
    } else { // > 90 degrees
        // Large angle difference - rotate first for efficiency
        strategy.should_rotate_first = true;
        strategy.rotation_movement_blend = 0.2; // Mostly sequential
    }
    
    // Consider distance - for short distances, rotation penalty is less important
    if (distance_to_target < 1.0) {
        strategy.should_rotate_first = false;
        strategy.rotation_movement_blend = std::min(1.0, strategy.rotation_movement_blend + 0.3);
    }
    
    return strategy;
}

/**
 * @brief Generate movement command that considers humanoid constraints
 * @param d SPG state
 * @param target_pos Target position  
 * @param dt Time step
 * @return Optimal movement command [vx, vy, vtheta] in global coordinates
 */
Eigen::Vector3d generateHumanoidMovementCommand(const SPGState& d,
                                               const Eigen::Vector3d& target_pos,
                                               double dt) {
    
    double current_orientation = d.input.robot.IMU_orientation(2);
    Eigen::Vector3d current_pos = d.input.robot.p;
    Eigen::Vector3d current_vel = d.input.robot.v;
    
    // Get orientation strategy
    OrientationStrategy orientation_strategy = calculateOptimalOrientation(d, target_pos, current_orientation);
    
    // Calculate desired translational movement
    Eigen::Vector3d pos_error = target_pos - current_pos;
    Eigen::Vector3d desired_global_vel = pos_error / dt; // Simple proportional control
    
    // Transform to robot-local frame to apply constraints
    Eigen::Vector3d desired_local_vel = globalToLocalVelocity(desired_global_vel, current_orientation);
    
    // Apply humanoid velocity limits in local frame
    desired_local_vel(0) = std::max(-d.par.vmax_move_x, std::min(d.par.vmax_move_x, desired_local_vel(0))); // sideways (left/right)
    desired_local_vel(1) = std::max(-d.par.vmax_move_y, std::min(d.par.vmax_move_y, desired_local_vel(1))); // forward/backward (front/back)
    
    // Apply efficiency factors
    desired_local_vel(0) *= d.par.sideways_efficiency;  // sideways efficiency
    desired_local_vel(1) *= d.par.forward_efficiency;   // forward efficiency
    
    // Calculate rotational command
    double angle_error = orientation_strategy.target_orientation - current_orientation;
    while (angle_error > M_PI) angle_error -= 2*M_PI;
    while (angle_error < -M_PI) angle_error += 2*M_PI;
    
    double desired_rotation = angle_error / dt; // Proportional rotational control
    desired_rotation = std::max(-d.par.vmax_rotate, std::min(d.par.vmax_rotate, desired_rotation));
    
    // Apply rotation penalty if moving
    if (desired_local_vel.head<2>().norm() > 0.1) {
        desired_rotation *= (1.0 - d.par.rotation_while_moving_penalty);
    }
    
    // Blend rotation with movement based on strategy
    desired_rotation *= orientation_strategy.rotation_movement_blend;
    
    // Transform constrained local velocity back to global frame
    Eigen::Vector3d constrained_local_vel(desired_local_vel(0), desired_local_vel(1), desired_rotation);
    Eigen::Vector3d final_global_vel = localToGlobalVelocity(constrained_local_vel, current_orientation);
    
    return final_global_vel;
}

} // namespace humanoid
} // namespace spg