#pragma once

#include <vector>
#include <cmath>

namespace spg {

// ============================================================================
// Core Data Structures
// ============================================================================

/**
 * @brief Robot position/velocity/acceleration state
 */
struct RobotState {
    double p[3];    // Position: [x, y, theta]
    double v[3];    // Velocity: [vx, vy, omega]
    double a[3];    // Acceleration: [ax, ay, alpha]
    
    RobotState() {
        for (int i = 0; i < 3; i++) {
            p[i] = 0.0;
            v[i] = 0.0;
            a[i] = 0.0;
        }
    }
};

/**
 * @brief Parameters controlling SPG behavior
 */
struct SPGPar {
    // Mode selection
    bool use_humanoid_mpc = false;  // true: HumanoidMPC, false: MSL
    
    // Robot physical limits (MSL mode)
    double v_max[3] = {2.0, 1.5, 3.0};      // [vf, vs, omega] max velocities
    double a_max[3] = {4.0, 3.0, 6.0};      // [af, as, alpha] max accelerations
    
    // MPC parameters (HumanoidMPC mode)
    int mpc_horizon = 20;                    // Prediction horizon steps
    double mpc_dt = 0.1;                     // Timestep (s)
    double mpc_weights[10] = {              // Weight parameters for tuning
        1.0, 1.0, 1.0,  // Q: position error (x, y, theta)
        0.1, 0.1, 0.1,  // R: control effort (u_x, u_y, u_theta)
        0.01, 0.01, 0.01, 0.01  // L: control smoothness
    };
    double coupling_factor = 0.5;           // Turn-translation coupling [0, 1]
    
    // Field dimensions
    double field_length = 9.0;              // meters
    double field_width = 6.0;               // meters
    double penalty_area_length = 1.0;       // meters
    double penalty_area_width = 3.0;        // meters
    double goal_area_length = 0.5;          // meters
    double goal_area_width = 1.5;           // meters
    
    // Obstacle avoidance
    double obstacle_margin = 0.3;           // Safety distance (m)
    double three_meter_rule = 3.0;          // Ball proximity rule (m)
    
    // Trajectory parameters
    double balance_xy_threshold = 0.1;      // Balance threshold (m)
    int max_segments = 5;                   // Max trajectory segments
};

/**
 * @brief Complete SPG state - includes input, intermediate, and output
 */
struct SPGState {
    // Parameters
    SPGPar par;
    
    // Current robot state (INPUT)
    RobotState robot;
    
    // Ball state (INPUT)
    double ball[3] = {0.0, 0.0, 0.0};      // [x, y, theta]
    
    // Obstacles (INPUT)
    std::vector<double> obstacles_x;        // Obstacle x positions
    std::vector<double> obstacles_y;        // Obstacle y positions
    std::vector<double> obstacles_vx;       // Obstacle x velocities
    std::vector<double> obstacles_vy;       // Obstacle y velocities
    std::vector<double> obstacles_radius;   // Obstacle radii
    
    // Goal (INPUT)
    RobotState goal;                        // Desired final state
    
    // --- Layer outputs (INTERMEDIATE) ---
    
    // Target layer output
    RobotState target;                      // Adjusted safe target
    
    // Subtarget layer output
    RobotState subtarget;                   // Immediate waypoint
    bool collision_free = true;             // Path validity flag
    
    // Setpoint layer output (FINAL OUTPUT)
    RobotState setpoint;                    // Command to robot
    
    // --- Trajectory data (MPC mode only) ---
    std::vector<double> predicted_x;        // Predicted trajectory x
    std::vector<double> predicted_y;        // Predicted trajectory y
    std::vector<double> predicted_theta;    // Predicted trajectory theta
    
    // --- Timing & debugging ---
    double time_target_ms = 0.0;            // Target layer execution time
    double time_subtarget_ms = 0.0;         // Subtarget layer execution time
    double time_setpoint_ms = 0.0;          // Setpoint layer execution time
};

// ============================================================================
// Function Declarations - Three Layer Architecture
// ============================================================================

namespace target {
    /**
     * @brief Adjust goal to be safe and rule-compliant
     * 
     * Applies adjustments in order:
     * 1. Field boundaries
     * 2. Penalty areas (own/opponent)
     * 3. Goal areas
     * 4. Obstacles
     * 5. 3-meter rule (ball proximity)
     * 
     * @param state SPG state (modifies state.target)
     * 
     * TODO MR2: Implement adjustment algorithms
     * - AdjustToField: Clamp to field boundaries
     * - AdjustToPenaltyArea: Avoid penalty boxes
     * - AdjustToGoalArea: Avoid goal boxes
     * - AdjustToObstacles: Maintain obstacle_margin
     * - AdjustTo3mRule: Respect ball proximity rules
     */
    void Target(SPGState& state);
    
} // namespace target

namespace subtarget {
    /**
     * @brief Generate intermediate waypoint for collision-free navigation
     * 
     * Checks direct path to target, replans if collision detected.
     * Adjusts approach angle for goal alignment.
     * 
     * @param state SPG state (modifies state.subtarget, state.collision_free)
     * 
     * TODO MR3: Implement subtarget logic
     * - CheckCollisionFree: Ray-circle intersection tests
     * - AngleUtils: Heading adjustments
     * - ReplanUtils: Dynamic replanning when blocked
     */
    void Subtarget(SPGState& state);
    
} // namespace subtarget

namespace setpoint {
    /**
     * @brief Generate smooth trajectory and control commands
     * 
     * Two modes:
     * - MSL: Traditional trajectory segmentation
     * - HumanoidMPC: Model Predictive Control
     * 
     * @param state SPG state (modifies state.setpoint, predicted trajectories)
     * 
     * TODO MR4 (MSL): Implement trajectory generation
     * - GetSegments: Break into acceleration/cruise/deceleration
     * - BalanceXY: Coordinate motion timing
     * - StateCorrection: Velocity/acceleration limits
     * - TrajectoryUtils: Traj1, TrajPredict, TrajSegment
     * 
     * TODO MR5 (HumanoidMPC): Implement MPC controller
     * - Matrix precomputation (H, Su, Sx)
     * - QP problem formulation
     * - qpOASES integration
     * - Hotstart management
     * - Turn-translation coupling constraints
     */
    void Setpoint(SPGState& state);
    
} // namespace setpoint

// ============================================================================
// Utility Functions (TODO MR6)
// ============================================================================

/**
 * @brief 2D rotation matrix multiplication
 * TODO MR6: Implement rotation utilities
 */
void rot(double angle, const double* in, double* out);

/**
 * @brief Sign function with zero threshold
 * TODO MR6: Implement sign function
 */
double sign(double x);

/**
 * @brief Wrap angle to [-pi, pi]
 * TODO MR6: Implement angle wrapping
 */
double wrap(double theta);

} // namespace spg
