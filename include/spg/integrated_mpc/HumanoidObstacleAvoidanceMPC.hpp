#pragma once

#include <vector>
#include <array>
#include <Eigen/Dense>

// Forward declaration for acados capsule (opaque pointer) - must be in global namespace
struct robot_obstacle_mpc_solver_capsule;

namespace spg {
namespace integrated_mpc {

/**
 * @brief Configuration parameters for the integrated obstacle avoidance MPC
 */
struct ObstacleAvoidanceMPCParams {
    double dt;                    ///< Time step [s]
    int horizon;                  ///< Prediction horizon (number of steps)
    
    // Physical velocity limits
    double vx_max;                ///< Maximum forward velocity [m/s]
    double vy_max;                ///< Maximum sideways velocity [m/s]
    double omega_max;             ///< Maximum yaw rate [rad/s]
    
    // Acceleration limits
    double ax_max;                ///< Maximum forward acceleration [m/s^2]
    double ay_max;                ///< Maximum sideways acceleration [m/s^2]
    double alpha_max;             ///< Maximum angular acceleration [rad/s^2]
    
    // Obstacle avoidance parameters
    int max_obstacles;            ///< Maximum number of obstacles to consider
    double robot_radius;          ///< Robot safety radius [m]
    double obstacle_margin;       ///< Additional safety margin around obstacles [m]
    double collision_weight;      ///< Weight for collision avoidance cost
    
    // Cost function weights
    double Q_pos;                 ///< Position tracking weight
    double Q_theta;               ///< Heading tracking weight
    double Q_vel;                 ///< Velocity tracking weight
    double R_accel;               ///< Acceleration effort weight
    double Q_term_pos;            ///< Terminal position weight
    double Q_term_theta;          ///< Terminal heading weight
    double Q_term_vel;            ///< Terminal velocity weight
    
    ObstacleAvoidanceMPCParams();
};

/**
 * @brief State vector for the integrated MPC
 */
struct ObstacleAvoidanceState {
    double px;                    ///< Position x [m]
    double py;                    ///< Position y [m]
    double theta;                 ///< Heading angle [rad]
    double vx;                    ///< Body-frame forward velocity [m/s]
    double vy;                    ///< Body-frame sideways velocity [m/s]
    double omega;                 ///< Angular velocity [rad/s]
};

/**
 * @brief Control vector (acceleration commands)
 */
struct ObstacleAvoidanceControl {
    double ax;                    ///< Forward acceleration [m/s^2]
    double ay;                    ///< Sideways acceleration [m/s^2]
    double alpha;                 ///< Angular acceleration [rad/s^2]
};

/**
 * @brief Velocity command (for lower-level controller)
 */
struct ObstacleAvoidanceVelocity {
    double vx;                    ///< Forward velocity [m/s]
    double vy;                    ///< Sideways velocity [m/s]
    double omega;                 ///< Angular velocity [rad/s]
};

/**
 * @brief Obstacle representation
 */
struct Obstacle {
    double px;                    ///< Obstacle position x [m]
    double py;                    ///< Obstacle position y [m]
    double radius;                ///< Obstacle radius [m]
    double vx;                    ///< Obstacle velocity x [m/s] (optional)
    double vy;                    ///< Obstacle velocity y [m/s] (optional)
    bool active;                  ///< Whether this obstacle is active
};

/**
 * @brief Integrated MPC with obstacle avoidance
 * 
 * This controller combines:
 * - Target tracking
 * - Obstacle avoidance (via nonlinear constraints or barrier functions)
 * - Velocity/acceleration limits
 * - Smooth trajectory generation
 * 
 * All in a single optimization problem, eliminating the need for
 * separate target cleaning, subtarget selection, and setpoint generation.
 */
class HumanoidObstacleAvoidanceMPC {
public:
    /**
     * @brief Constructor
     * @param params MPC parameters including weights and limits
     */
    explicit HumanoidObstacleAvoidanceMPC(const ObstacleAvoidanceMPCParams& params);
    
    /**
     * @brief Destructor (cleans up acados solver)
     */
    ~HumanoidObstacleAvoidanceMPC();
    
    /**
     * @brief Check if solver is properly initialized
     */
    bool isInitialized() const { return solver_initialized_; }
    
    /**
     * @brief Compute optimal control with obstacle avoidance
     * 
     * @param x0 Current robot state
     * @param x_goal Target state
     * @param obstacles List of obstacles to avoid
     * @param u_out Output: optimal acceleration command
     * @return true if optimization succeeded
     */
    bool computeControl(
        const ObstacleAvoidanceState& x0,
        const ObstacleAvoidanceState& x_goal,
        const std::vector<Obstacle>& obstacles,
        ObstacleAvoidanceControl& u_out
    );
    
    /**
     * @brief Compute control and return full predicted trajectory
     * 
     * @param x0 Current robot state
     * @param x_goal Target state
     * @param obstacles List of obstacles to avoid
     * @param u_out Output: optimal acceleration command
     * @param predicted_states Output: predicted state trajectory
     * @param predicted_controls Output: predicted control sequence
     * @return true if optimization succeeded
     */
    bool computeControlAndTrajectory(
        const ObstacleAvoidanceState& x0,
        const ObstacleAvoidanceState& x_goal,
        const std::vector<Obstacle>& obstacles,
        ObstacleAvoidanceControl& u_out,
        std::vector<ObstacleAvoidanceState>& predicted_states,
        std::vector<ObstacleAvoidanceControl>& predicted_controls
    );
    
    /**
     * @brief Get velocity command (for downstream controller)
     * @return Velocity command derived from first predicted state
     */
    ObstacleAvoidanceVelocity getVelocityCommand() const;
    
    /**
     * @brief Get predicted trajectory from last solve
     */
    void getPredictedTrajectory(
        std::vector<ObstacleAvoidanceState>& states,
        std::vector<ObstacleAvoidanceControl>& controls
    ) const;
    
private:
    ObstacleAvoidanceMPCParams params_;
    robot_obstacle_mpc_solver_capsule* capsule_;
    bool solver_initialized_;
    bool has_previous_solution_;
    
    // Warm-start storage
    std::vector<std::array<double, 6>> prev_states_;
    std::vector<std::array<double, 3>> prev_controls_;
    
    /**
     * @brief Set initial state constraint
     */
    void setInitialState(const ObstacleAvoidanceState& x0);
    
    /**
     * @brief Set goal reference for all stages
     */
    void setGoalReference(const ObstacleAvoidanceState& x_goal);
    
    /**
     * @brief Update obstacle constraints (nonlinear path constraints)
     */
    void updateObstacleConstraints(const std::vector<Obstacle>& obstacles);
    
    /**
     * @brief Extract first control from solution
     */
    ObstacleAvoidanceControl getFirstControl() const;
    
    /**
     * @brief Apply warm-start from previous solution
     */
    void applyWarmStart();
    
    /**
     * @brief Store solution for next warm-start
     */
    void storeSolution();
    
    /**
     * @brief Wrap angle to [-pi, pi]
     */
    static double wrap_pi(double a);
    
    /**
     * @brief Clamp value to range
     */
    static double clamp(double v, double lo, double hi);
};

} // namespace integrated_mpc
} // namespace spg
