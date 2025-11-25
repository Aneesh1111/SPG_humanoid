#pragma once
#include <Eigen/Dense>
#include "spg/Init.hpp"

namespace spg {
namespace humanoid {

/**
 * @brief Transform global velocities to robot-local frame considering orientation
 * @param global_velocity Desired velocity in global coordinates [vx, vy, vtheta]
 * @param robot_orientation Current robot orientation [rad]
 * @return Robot-local velocity [v_sideways, v_forward, v_rotation]
 * Note: Forward/backward = Y-direction, Sideways = X-direction
 */
inline Eigen::Vector3d globalToLocalVelocity(const Eigen::Vector3d& global_velocity, double robot_orientation) {
    double cos_theta = std::cos(robot_orientation);
    double sin_theta = std::sin(robot_orientation);
    
    // Transform translational velocities to robot frame
    // Forward/backward movement is in Y-direction (robot's front/back)
    double v_forward = -global_velocity(0) * sin_theta + global_velocity(1) * cos_theta;
    // Sideways movement is in X-direction (robot's left/right)
    double v_sideways = global_velocity(0) * cos_theta + global_velocity(1) * sin_theta;
    double v_rotation = global_velocity(2);
    
    return Eigen::Vector3d(v_sideways, v_forward, v_rotation);
}

/**
 * @brief Transform robot-local velocities back to global frame
 * @param local_velocity Robot-local velocity [v_sideways, v_forward, v_rotation]
 * @param robot_orientation Current robot orientation [rad]
 * @return Global velocity [vx, vy, vtheta]
 * Note: Forward/backward = Y-direction, Sideways = X-direction
 */
inline Eigen::Vector3d localToGlobalVelocity(const Eigen::Vector3d& local_velocity, double robot_orientation) {
    double cos_theta = std::cos(robot_orientation);
    double sin_theta = std::sin(robot_orientation);
    
    // Transform translational velocities to global frame
    // local_velocity(0) = v_sideways (X-direction in robot frame)
    // local_velocity(1) = v_forward (Y-direction in robot frame)
    double vx = local_velocity(0) * cos_theta - local_velocity(1) * sin_theta;
    double vy = local_velocity(0) * sin_theta + local_velocity(1) * cos_theta;
    double vtheta = local_velocity(2);
    
    return Eigen::Vector3d(vx, vy, vtheta);
}

/**
 * @brief Calculate effective velocity limits considering orientation and efficiency
 * @param par Field parameters with humanoid-specific limits
 * @param robot_orientation Current robot orientation [rad]
 * @param desired_global_velocity Desired movement direction in global frame
 * @return Effective velocity limits [vmax_x, vmax_y, vmax_theta]
 * Note: X=sideways limits, Y=forward limits
 */
inline Eigen::Vector3d calculateEffectiveVelocityLimits(const FieldParams& par, 
                                                       double robot_orientation,
                                                       const Eigen::Vector3d& desired_global_velocity) {
    // Transform desired velocity to robot-local frame
    Eigen::Vector3d local_desired = globalToLocalVelocity(desired_global_velocity, robot_orientation);
    
    // Calculate efficiency factors based on movement direction
    // local_desired(0) = sideways component (X-direction)
    // local_desired(1) = forward component (Y-direction)
    double sideways_component = std::abs(local_desired(0));
    double forward_component = std::abs(local_desired(1));
    double total_translational = forward_component + sideways_component;
    
    double effective_vmax_x, effective_vmax_y;
    
    if (total_translational > 1e-6) {
        // Weighted efficiency based on movement direction
        double forward_ratio = forward_component / total_translational;
        double sideways_ratio = sideways_component / total_translational;
        
        // Calculate effective limits considering orientation efficiency
        // X-direction (sideways) uses vmax_move_x
        effective_vmax_x = par.vmax_move_x * (forward_ratio * par.forward_efficiency + 
                                             sideways_ratio * par.sideways_efficiency);
        // Y-direction (forward) uses vmax_move_y  
        effective_vmax_y = par.vmax_move_y * (forward_ratio * par.forward_efficiency + 
                                             sideways_ratio * par.sideways_efficiency);
    } else {
        // No movement preference, use full limits
        effective_vmax_x = par.vmax_move_x * par.sideways_efficiency;  // X = sideways
        effective_vmax_y = par.vmax_move_y * par.forward_efficiency;   // Y = forward
    }
    
    // Apply rotation penalty if moving
    double rotation_penalty = 1.0;
    if (total_translational > 0.1) { // If moving with significant translational velocity
        rotation_penalty = 1.0 - par.rotation_while_moving_penalty;
    }
    
    double effective_vmax_theta = par.vmax_rotate * rotation_penalty;
    
    return Eigen::Vector3d(effective_vmax_x, effective_vmax_y, effective_vmax_theta);
}

/**
 * @brief Calculate effective acceleration limits considering orientation and efficiency
 * @param par Field parameters with humanoid-specific limits
 * @param robot_orientation Current robot orientation [rad]
 * @param desired_global_velocity Desired movement direction in global frame
 * @return Effective acceleration limits [amax_x, amax_y, amax_theta]
 * Note: X=sideways limits, Y=forward limits
 */
inline Eigen::Vector3d calculateEffectiveAccelerationLimits(const FieldParams& par,
                                                           double robot_orientation,
                                                           const Eigen::Vector3d& desired_global_velocity) {
    // Similar calculation as velocity but for acceleration
    Eigen::Vector3d local_desired = globalToLocalVelocity(desired_global_velocity, robot_orientation);
    
    // local_desired(0) = sideways component (X-direction)  
    // local_desired(1) = forward component (Y-direction)
    double sideways_component = std::abs(local_desired(0));
    double forward_component = std::abs(local_desired(1));
    double total_translational = forward_component + sideways_component;
    
    double effective_amax_x, effective_amax_y;
    
    if (total_translational > 1e-6) {
        double forward_ratio = forward_component / total_translational;
        double sideways_ratio = sideways_component / total_translational;
        
        // X-direction (sideways) uses amax_move_x
        effective_amax_x = par.amax_move_x * (forward_ratio * par.forward_efficiency + 
                                             sideways_ratio * par.sideways_efficiency);
        // Y-direction (forward) uses amax_move_y
        effective_amax_y = par.amax_move_y * (forward_ratio * par.forward_efficiency + 
                                             sideways_ratio * par.sideways_efficiency);
    } else {
        effective_amax_x = par.amax_move_x * par.sideways_efficiency;  // X = sideways
        effective_amax_y = par.amax_move_y * par.forward_efficiency;   // Y = forward
    }
    
    // Apply rotation penalty
    double rotation_penalty = 1.0;
    if (total_translational > 0.1) {
        rotation_penalty = 1.0 - par.rotation_while_moving_penalty;
    }
    
    double effective_amax_theta = par.amax_rotate * rotation_penalty;
    
    return Eigen::Vector3d(effective_amax_x, effective_amax_y, effective_amax_theta);
}

/**
 * @brief Calculate optimal intermediate orientation for humanoid movement
 * 
 * For humanoid robots that move faster forward than sideways, it's often beneficial
 * to rotate towards the movement direction even if the final orientation is the same.
 * This function finds the optimal orientation that minimizes travel time considering:
 * - Time to rotate to target direction
 * - Time saved by moving forward (faster) vs sideways (slower)
 * 
 * @param current_pos Current robot position [x, y, theta]
 * @param current_orientation Current robot orientation [rad]
 * @param target_pos Target position [x, y, theta]
 * @param target_orientation Target orientation [rad]
 * @param par Field parameters with humanoid constraints
 * @return Optimal intermediate orientation [rad] to face during translation
 */
inline double calculateOptimalMovementOrientation(const Eigen::Vector3d& current_pos,
                                                  double current_orientation,
                                                  const Eigen::Vector3d& target_pos,
                                                  double target_orientation,
                                                  const FieldParams& par) {
    // Calculate movement direction in global frame
    Eigen::Vector2d movement_vector(target_pos(0) - current_pos(0), 
                                    target_pos(1) - current_pos(1));
    double distance = movement_vector.norm();
    
    // If we're already at the target, just return target orientation
    if (distance < 0.01) {
        return target_orientation;
    }
    
    // Angle towards target in global frame
    double angle_to_target = std::atan2(movement_vector(1), movement_vector(0));
    
    // Calculate time cost for different strategies
    double min_time = std::numeric_limits<double>::max();
    double optimal_orientation = current_orientation;
    
    // Try different intermediate orientations (discretized search)
    // We'll check: current orientation, target orientation, and angles towards target
    std::vector<double> candidate_orientations;
    
    // Add current and target orientations
    candidate_orientations.push_back(current_orientation);
    candidate_orientations.push_back(target_orientation);
    
    // Add angle directly towards target (most efficient for forward movement)
    candidate_orientations.push_back(angle_to_target);
    
    // Add angles ±90° from target direction (pure sideways movement)
    candidate_orientations.push_back(angle_to_target + M_PI/2);
    candidate_orientations.push_back(angle_to_target - M_PI/2);
    
    // Add intermediate angles (for finer optimization)
    for (int i = 1; i < 8; ++i) {
        double angle = angle_to_target + (i - 4) * M_PI / 8.0;
        candidate_orientations.push_back(angle);
    }
    
    // Evaluate each candidate
    for (double candidate_phi : candidate_orientations) {
        // Normalize angle to [-π, π]
        double normalized_phi = std::atan2(std::sin(candidate_phi), std::cos(candidate_phi));
        
        // Time to rotate from current to candidate orientation
        double rotation_angle_1 = std::abs(std::atan2(std::sin(normalized_phi - current_orientation),
                                                       std::cos(normalized_phi - current_orientation)));
        double rotation_time_1 = rotation_angle_1 / par.vmax_rotate;
        
        // Time to rotate from candidate to target orientation
        double rotation_angle_2 = std::abs(std::atan2(std::sin(target_orientation - normalized_phi),
                                                       std::cos(target_orientation - normalized_phi)));
        double rotation_time_2 = rotation_angle_2 / par.vmax_rotate;
        
        // Calculate translation time with this orientation
        // Transform movement to robot-local frame
        double cos_phi = std::cos(normalized_phi);
        double sin_phi = std::sin(normalized_phi);
        double local_x = movement_vector(0) * cos_phi + movement_vector(1) * sin_phi;  // sideways
        double local_y = -movement_vector(0) * sin_phi + movement_vector(1) * cos_phi; // forward
        
        // Time for sideways and forward components
        double time_x = std::abs(local_x) / par.vmax_move_x;
        double time_y = std::abs(local_y) / par.vmax_move_y;
        
        // Total translation time (limited by slower axis)
        double translation_time = std::max(time_x, time_y);
        
        // Apply efficiency factors
        double sideways_fraction = std::abs(local_x) / (std::abs(local_x) + std::abs(local_y) + 1e-9);
        double forward_fraction = std::abs(local_y) / (std::abs(local_x) + std::abs(local_y) + 1e-9);
        double efficiency = sideways_fraction * par.sideways_efficiency + 
                          forward_fraction * par.forward_efficiency;
        translation_time /= std::max(efficiency, 0.1); // Avoid division by zero
        
        // Total time: rotate to candidate + translate + rotate to final
        double total_time = rotation_time_1 + translation_time + rotation_time_2;
        
        // Keep track of best orientation
        if (total_time < min_time) {
            min_time = total_time;
            optimal_orientation = normalized_phi;
        }
    }
    
    return optimal_orientation;
}

} // namespace humanoid
} // namespace spg