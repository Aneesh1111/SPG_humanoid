#include <Eigen/Dense>
#include "spg/setpoint/SetHumanoid.hpp"
#include "spg/setpoint/GetSegments.hpp"
#include "spg/setpoint/Traj1.hpp"
#include "spg/setpoint/TrajPredict.hpp"
#include "spg/setpoint/ConvertSegment.hpp"
#include "spg/setpoint/BalanceXY.hpp"
#include "spg/HumanoidUtils.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace spg {
namespace setpoint {

SPGState SetHumanoid(SPGState& d) {
    std::cout << "🤖 Humanoid Setpoint Update (SetHumanoid)" << std::endl;
    
    // Get current robot orientation from IMU
    double robot_orientation = d.input.robot.IMU_orientation(2); // yaw angle
    
    // Calculate distance to target
    Eigen::Vector3d movement_direction = d.subtarget.p - d.input.robot.p;
    double distance_to_target = std::sqrt(movement_direction(0) * movement_direction(0) + 
                                          movement_direction(1) * movement_direction(1));
    
    // Calculate required orientation to face target (for forward-only movement)
    // NOTE: System uses Y-up as 0°, with positive rotation clockwise
    double angle_to_target = -std::atan2(movement_direction(0), movement_direction(1));
    double angle_error_to_target = angle_to_target - robot_orientation;
    angle_error_to_target = std::atan2(std::sin(angle_error_to_target), std::cos(angle_error_to_target));
    
    // Calculate target orientation (where robot should be facing at the end)
    double target_orientation = d.subtarget.p(2);
    double orientation_error_to_final = target_orientation - robot_orientation;
    orientation_error_to_final = std::atan2(std::sin(orientation_error_to_final), std::cos(orientation_error_to_final));
    
    std::cout << "  Robot pos: [" << d.input.robot.p.transpose() << "]" << std::endl;
    std::cout << "  Subtarget pos: [" << d.subtarget.p.transpose() << "]" << std::endl;
    std::cout << "  Current orientation: " << robot_orientation * 180.0 / M_PI << "°" << std::endl;
    std::cout << "  Distance to target: " << distance_to_target << " m" << std::endl;
    std::cout << "  Angle to target: " << angle_to_target * 180.0 / M_PI << "°" << std::endl;
    std::cout << "  Angle error: " << angle_error_to_target * 180.0 / M_PI << "°" << std::endl;
    
    // HUMANOID CONSTRAINT: Robot cannot walk backwards!
    // Strategy: Always rotate to face the target direction, then move forward only
    
    const double ORIENTATION_ALIGNED_THRESHOLD = 15.0 * M_PI / 180.0; // 15 degrees
    const double CLOSE_DISTANCE_THRESHOLD = 0.5; // meters - use omnidirectional only when very close
    
    bool is_facing_target = std::abs(angle_error_to_target) < ORIENTATION_ALIGNED_THRESHOLD;
    bool is_very_close = distance_to_target < CLOSE_DISTANCE_THRESHOLD;
    
    // Use omnidirectional planning only when very close to target (where small sideways adjustments are acceptable)
    bool use_omnidirectional_planning = is_very_close;
    
    double planning_orientation; // Orientation to use for trajectory planning
    
    if (use_omnidirectional_planning) {
        // Very close to target: Allow small omnidirectional movements for fine positioning
        std::cout << "  Mode: Omnidirectional (very close, final positioning)" << std::endl;
        planning_orientation = robot_orientation; // Use current orientation
    } else if (!is_facing_target) {
        // Not facing target: Prioritize rotation, minimal forward movement
        std::cout << "  Mode: Rotate to face target (angle error: " 
                  << angle_error_to_target * 180.0 / M_PI << "°)" << std::endl;
        planning_orientation = angle_to_target; // Plan to rotate towards target
    } else {
        // Facing target: Move forward towards target
        std::cout << "  Mode: Forward movement (aligned with target)" << std::endl;
        planning_orientation = angle_to_target; // Already facing target, move forward
    }
    
    // Field/margin check (same as original Set function)
    double field_x_half = d.par.field_size[0] * 0.5 + d.par.field_border_margin;
    double field_y_half = d.par.field_size[1] * 0.5 + d.par.field_border_margin;
    bool inside_field =
        (-field_x_half < d.input.robot.p[0] && d.input.robot.p[0] < field_x_half) &&
        (-field_y_half < d.input.robot.p[1] && d.input.robot.p[1] < field_y_half);
        
    if (inside_field || d.subtarget.automatic_substitution_flag == 1) {
        // Get effective velocity and acceleration limits
        Eigen::Vector3d effective_vmax, effective_amax;
        
        if (use_omnidirectional_planning) {
            // Very close: Allow omnidirectional movement for final positioning
            effective_vmax = Eigen::Vector3d(d.par.vmax_move_x, d.par.vmax_move_y, d.par.vmax_rotate);
            effective_amax = Eigen::Vector3d(d.par.amax_move_x, d.par.amax_move_y, d.par.amax_rotate);
            std::cout << "  Using omnidirectional limits (X=" << d.par.vmax_move_x 
                      << " m/s, Y=" << d.par.vmax_move_y << " m/s)" << std::endl;
        } else {
            // Forward-only movement: Calculate limits based on how well-aligned we are with target
            // When not aligned, prioritize rotation with minimal forward movement
            // When aligned, use full forward velocity
            
            double alignment_factor = std::cos(angle_error_to_target);
            if (alignment_factor < 0) alignment_factor = 0; // No backward movement!
            
            // Calculate effective velocity in robot's local frame
            // X = sideways (limited), Y = forward (proportional to alignment)
            effective_vmax = spg::humanoid::calculateEffectiveVelocityLimits(
                d.par, planning_orientation, movement_direction);
            effective_amax = spg::humanoid::calculateEffectiveAccelerationLimits(
                d.par, planning_orientation, movement_direction);
            
            // Reduce forward velocity when not facing target (forces rotation first)
            if (!is_facing_target) {
                double rotation_priority_factor = 0.3; // Allow only 30% forward speed while rotating
                effective_vmax(1) *= rotation_priority_factor * alignment_factor;
                effective_amax(1) *= rotation_priority_factor;
                std::cout << "  Prioritizing rotation (forward velocity reduced to " 
                          << effective_vmax(1) << " m/s)" << std::endl;
            }
            
            // Show movement details
            Eigen::Vector3d local_movement = spg::humanoid::globalToLocalVelocity(movement_direction, planning_orientation);
            double sideways_component = std::abs(local_movement(0));
            double forward_component = std::abs(local_movement(1));
            double total = sideways_component + forward_component;
            if (total > 1e-6) {
                double forward_ratio = forward_component / total;
                double sideways_ratio = sideways_component / total;
                double efficiency = forward_ratio * d.par.forward_efficiency + 
                                  sideways_ratio * d.par.sideways_efficiency;
                std::cout << "  Movement efficiency: " << efficiency * 100 << "% (forward: " 
                          << forward_ratio * 100 << "%, sideways: " << sideways_ratio * 100 << "%)" << std::endl;
            }
        }
        
        std::cout << "  Effective limits: vmax=[" << effective_vmax.transpose() << "], amax=[" 
                  << effective_amax.transpose() << "]" << std::endl;
        
        // Tipping check (same as original, but use humanoid deceleration limits)
        double tipping_degrees = 5.0;
        double robot_tipping_degrees = d.input.robot.IMU_orientation[0] * d.input.robot.IMU_orientation[0] +
                                       d.input.robot.IMU_orientation[1] * d.input.robot.IMU_orientation[1];
        Eigen::Vector3d dmax;
        if (std::sqrt(robot_tipping_degrees) > tipping_degrees) {
            // Reduced deceleration when tipping - use humanoid-aware limits
            dmax = 0.5 * Eigen::Vector3d(d.par.amax_move_x, d.par.amax_move_y, d.par.dmax_rotate);
        } else {
            // Normal deceleration - use humanoid-aware limits
            dmax = Eigen::Vector3d(d.par.amax_move_x, d.par.amax_move_y, d.par.dmax_rotate);
        }
        
        // Initialize trajectory segments for humanoid balancing
        std::vector<spg::setpoint::Segment> segments(4); // acceleration, const vel, deceleration, stop
        
        // Compute target velocity for smooth deceleration
        // Instead of forcing zero velocity at target from far away, use a "flyby" velocity
        // that transitions smoothly to zero only when very close
        Eigen::Vector3d target_velocity = d.subtarget.v; // Default: use subtarget velocity (usually zero)
        
        double decel_threshold = 0.3; // Start smooth deceleration within 30cm
        if (distance_to_target > decel_threshold) {
            // Far from target: plan to "fly by" at reduced speed
            // This prevents premature deceleration and stop-start behavior
            double flyby_speed = std::min(
                effective_vmax.segment<2>(0).norm() * 0.5,  // Max 50% of capable speed
                distance_to_target * 2.0  // Or proportional to distance
            );
            
            // Direction towards target
            Eigen::Vector3d direction = d.subtarget.p - d.input.robot.p;
            double distance_xy = std::sqrt(direction(0)*direction(0) + direction(1)*direction(1));
            if (distance_xy > 1e-6) {
                direction(0) /= distance_xy;
                direction(1) /= distance_xy;
                direction(2) = 0;
                
                target_velocity = direction * flyby_speed;
                target_velocity(2) = d.subtarget.v(2); // Keep original rotation target
            }
        }
        
        // Use humanoid-aware balancing instead of standard getSegments
        Eigen::Vector3d balanced_vmax, balanced_amax, balanced_dmax;
        spg::setpoint::balanceXY_humanoid(
            segments,
            d.input.robot.p,       // current position
            d.input.robot.v,       // current velocity  
            d.subtarget.p,         // target position (use subtarget for intermediate goals)
            target_velocity,       // target velocity (flyby or zero depending on distance)
            d.par,                 // parameters with humanoid constraints
            planning_orientation,  // Use planning orientation (current for XY, towards target for Reeds-Shepp)
            balanced_vmax,         // output: balanced max velocity
            balanced_amax,         // output: balanced max acceleration
            balanced_dmax          // output: balanced max deceleration
        );
        
        std::cout << "  Balanced limits: vmax=[" << balanced_vmax.transpose() << "]" << std::endl;
        
        // Use the generated segments for trajectory generation
        if (!segments.empty() && segments[0].dt.norm() > 1e-6) {
            // Propagate 1 sample using humanoid-generated segments
            Traj1(d.traj, segments, d.par.Ts);
            
            d.setpoint.p = d.traj.p[0];
            d.setpoint.v = d.traj.v[0];
            d.setpoint.a = d.traj.a[0];
            
            // Velocity limiting when very close to target to prevent overshoot
            if (distance_to_target < 0.2) {
                // Very close: limit velocity proportional to distance
                double max_approach_vel = std::max(0.1, distance_to_target * 2.0); // Min 0.1 m/s
                double current_vel = std::sqrt(d.setpoint.v(0)*d.setpoint.v(0) + d.setpoint.v(1)*d.setpoint.v(1));
                if (current_vel > max_approach_vel) {
                    double scale = max_approach_vel / current_vel;
                    d.setpoint.v(0) *= scale;
                    d.setpoint.v(1) *= scale;
                    std::cout << "  🎯 Final approach: limiting velocity to " << max_approach_vel << " m/s" << std::endl;
                }
            }
            
            if (use_omnidirectional_planning) {
                // Very close: Plan phi independently towards final target orientation
                double angle_error = target_orientation - robot_orientation;
                angle_error = std::atan2(std::sin(angle_error), std::cos(angle_error));
                
                // Plan rotation towards target orientation (independent from XY movement)
                double rotation_velocity = std::max(-d.par.vmax_rotate, 
                                                   std::min(d.par.vmax_rotate, angle_error / d.par.Ts));
                d.setpoint.p(2) = robot_orientation + rotation_velocity * d.par.Ts;
                d.setpoint.v(2) = rotation_velocity;
                
                std::cout << "  ↻ Independent phi planning to target orientation (error: " << angle_error * 180.0/M_PI << "°)" << std::endl;
            } else {
                // Forward-only mode: Rotate towards target direction
                // Priority: If not facing target, rotate first with minimal forward movement
                // Once aligned, move forward while maintaining orientation
                
                double rotation_velocity = std::max(-d.par.vmax_rotate, 
                                                   std::min(d.par.vmax_rotate, angle_error_to_target / d.par.Ts));
                d.setpoint.p(2) = robot_orientation + rotation_velocity * d.par.Ts;
                d.setpoint.v(2) = rotation_velocity;
                
                if (!is_facing_target) {
                    std::cout << "  🔄 Rotating to face target: " << rotation_velocity * 180.0/M_PI 
                              << " °/s (error: " << angle_error_to_target * 180.0/M_PI << "°)" << std::endl;
                } else {
                    std::cout << "  ➡️  Moving forward while maintaining orientation (aligned)" << std::endl;
                }
            }

            // Generate full predicted trajectory for visualization (all npredict timesteps)
            TrajPredict(d, segments);
            
            std::cout << "  New setpoint: pos=[" << d.setpoint.p.transpose() 
                      << "], vel=[" << d.setpoint.v.transpose() << "]" << std::endl;
        } else {
            // Fallback: use current position with zero velocity
            std::cout << "  Fallback: segments empty or invalid, holding position" << std::endl;
            d.setpoint.p = d.input.robot.p;
            d.setpoint.v = Eigen::Vector3d::Zero();
            d.setpoint.a = Eigen::Vector3d::Zero();
            
            // Clear trajectory prediction to avoid showing old/invalid trajectories
            for (int i = 0; i < d.par.npredict; ++i) {
                d.traj.p[i] = d.input.robot.p; // Set all trajectory points to current position
                d.traj.v[i] = Eigen::Vector3d::Zero();
                d.traj.a[i] = Eigen::Vector3d::Zero();
            }
        }

        // Field margin logic (adapted for humanoid constraints)
        double field_width_half = d.par.field_size[0] * 0.5 + d.par.field_border_margin + 
                                  (d.subtarget.automatic_substitution_flag == 1 ? d.par.technical_area_width : 0.0);
        double field_length_half = d.par.field_size[1] * 0.5 + d.par.field_border_margin;
        
        // X direction (sideways) - use humanoid X constraints
        if (d.setpoint.p[0] < -field_width_half) d.setpoint.p[0] = -field_width_half;
        if (d.setpoint.p[0] >  field_width_half) d.setpoint.p[0] =  field_width_half;
        double dist2sideline = field_width_half - std::abs(d.traj.p[0][0]);
        double vx_max = 2 * d.par.amax_move_x * dist2sideline; // Use humanoid X deceleration
        if (d.setpoint.v[0] < -vx_max) d.setpoint.v[0] = -vx_max;
        if (d.setpoint.v[0] >  vx_max) d.setpoint.v[0] =  vx_max;
        
        // Y direction (forward) - use humanoid Y constraints
        if (d.setpoint.p[1] < -field_length_half) d.setpoint.p[1] = -field_length_half;
        if (d.setpoint.p[1] >  field_length_half) d.setpoint.p[1] =  field_length_half;
        double dist2goalline = field_length_half - std::abs(d.traj.p[0][1]);
        double vy_max = 2 * d.par.amax_move_y * dist2goalline; // Use humanoid Y deceleration
        if (d.setpoint.v[1] < -vy_max) d.setpoint.v[1] = -vy_max;
        if (d.setpoint.v[1] >  vy_max) d.setpoint.v[1] =  vy_max;
        
        // Decelerate if velocity is greater than max allowable velocity (use humanoid limits)
        if (d.setpoint.v[0] > vx_max && d.setpoint.p[0] > 0) d.setpoint.a[0] = -d.par.amax_move_x;
        if (d.setpoint.v[0] < -vx_max && d.setpoint.p[0] < 0) d.setpoint.a[0] = d.par.amax_move_x;
        if (d.setpoint.v[1] > vy_max && d.setpoint.p[1] > 0) d.setpoint.a[1] = -d.par.amax_move_y;
        if (d.setpoint.v[1] < -vy_max && d.setpoint.p[1] < 0) d.setpoint.a[1] = d.par.amax_move_y;
        
    } else {
        // Outside field - same as original Set function
        d.setpoint.p = d.input.robot.p;
        d.setpoint.v = Eigen::Vector3d::Zero();
        d.setpoint.a = Eigen::Vector3d::Zero();
        std::cout << "SetHumanoid: Robot outside field and no automatic substitution, holding position." << std::endl;
    }
    
    return d;
}

} // namespace setpoint
} // namespace spg