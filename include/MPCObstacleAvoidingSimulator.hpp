#pragma once

#include "spg/Init.hpp"
#include "spg/integrated_mpc/HumanoidObstacleAvoidanceMPC.hpp"
#include "visualization/SimulatorVisualizer.hpp"
#include <chrono>
#include <vector>
#include <memory>
#include <Eigen/Dense>

/**
 * @brief Simulator that uses only the integrated obstacle avoidance MPC
 * 
 * Unlike SPGSimulator which uses the full pipeline (target->subtarget->setpoint),
 * this simulator directly calls HumanoidObstacleAvoidanceMPC for control.
 * 
 * This is useful for:
 * - Testing the integrated MPC in isolation
 * - Comparing performance with the traditional SPG approach
 * - Tuning obstacle avoidance weights independently
 */
class MPCObstacleAvoidingSimulator {
public:
    /**
     * @brief Constructor
     * @param initial_pos Initial robot position (x, y, theta)
     * @param initial_vel Initial robot velocity (vx, vy, omega)
     * @param target_pos Target position (x, y, theta)
     * @param mpc_params MPC configuration parameters
     */
    MPCObstacleAvoidingSimulator(
        const Eigen::Vector3d& initial_pos,
        const Eigen::Vector3d& initial_vel,
        const Eigen::Vector3d& target_pos,
        const spg::integrated_mpc::ObstacleAvoidanceMPCParams& mpc_params
    );
    
    /**
     * @brief Run simulation with visualization
     */
    void run();
    
    /**
     * @brief Run one simulation step
     */
    void step();
    
    /**
     * @brief Run simulation without visualization (for benchmarking)
     */
    void runWithoutVisualization();
    
    /**
     * @brief Run simulation without visualization with fixed duration
     * @param max_steps Maximum number of simulation steps
     * @param dt Time step in seconds
     */
    void runWithoutVisualization(int max_steps, double dt);
    
    /**
     * @brief Set obstacle positions and radii
     * @param positions Vector of (x, y) positions
     * @param radii Vector of obstacle radii
     * @param velocities Optional velocities for moving obstacles
     */
    void setObstacles(
        const std::vector<Eigen::Vector2d>& positions,
        const std::vector<double>& radii,
        const std::vector<Eigen::Vector2d>& velocities = {}
    );
    
    /**
     * @brief Get current robot state
     */
    const spg::integrated_mpc::ObstacleAvoidanceState& getCurrentState() const { return current_state_; }
    
    /**
     * @brief Get target state
     */
    const spg::integrated_mpc::ObstacleAvoidanceState& getTargetState() const { return target_state_; }
    
    /**
     * @brief Get simulation time
     */
    double getSimulationTime() const { return simulation_time_; }
    
    /**
     * @brief Check if simulation is completed
     */
    bool isCompleted() const { return simulation_completed_; }
    
    /**
     * @brief Get MPC statistics
     */
    struct MPCStats {
        int call_count;
        double total_time_ms;
        double avg_time_ms;
        double min_time_ms;
        double max_time_ms;
        std::vector<double> times_ms;
    };
    MPCStats getMPCStats() const;

private:
    void reset();
    void updateObstacles(double dt);
    bool checkGoalReached();
    void printCompletionStats();
    
    // MPC controller
    std::unique_ptr<spg::integrated_mpc::HumanoidObstacleAvoidanceMPC> mpc_;
    spg::integrated_mpc::ObstacleAvoidanceMPCParams mpc_params_;
    
    // State
    spg::integrated_mpc::ObstacleAvoidanceState initial_state_;
    spg::integrated_mpc::ObstacleAvoidanceState current_state_;
    spg::integrated_mpc::ObstacleAvoidanceState target_state_;
    
    // Obstacles
    std::vector<spg::integrated_mpc::Obstacle> obstacles_;
    std::vector<Eigen::Vector2d> obstacle_velocities_;
    
    // Visualization
    SimulatorVisualizer visualizer_;
    
    // Timing and statistics
    double simulation_time_;
    int step_count_;
    bool simulation_completed_;
    double total_mpc_time_ms_;
    int mpc_call_count_;
    std::vector<double> mpc_times_ms_;
    std::chrono::high_resolution_clock::time_point start_time_;
    
    // Simulation parameters
    double dt_;  // Time step
    double position_tolerance_;  // Position error threshold for completion
    double velocity_tolerance_;  // Velocity threshold for completion
};
