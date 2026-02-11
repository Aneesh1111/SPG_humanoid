#include "MPCObstacleAvoidingSimulator.hpp"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <thread>
#include <cmath>

MPCObstacleAvoidingSimulator::MPCObstacleAvoidingSimulator(
    const Eigen::Vector3d& initial_pos,
    const Eigen::Vector3d& initial_vel,
    const Eigen::Vector3d& target_pos,
    const spg::integrated_mpc::ObstacleAvoidanceMPCParams& mpc_params)
    : mpc_params_(mpc_params)
    , visualizer_({12.0, 18.0, 2.0, 1.0, 0.7, 2.0})  // Default field dimensions
    , simulation_time_(0.0)
    , step_count_(0)
    , simulation_completed_(false)
    , total_mpc_time_ms_(0.0)
    , mpc_call_count_(0)
    , mpc_failure_count_(0)
    , dt_(mpc_params.dt)
    , position_tolerance_(0.05)  // 5cm
    , velocity_tolerance_(0.05)  // 5cm/s
{
    // Initialize MPC
    mpc_ = std::make_unique<spg::integrated_mpc::HumanoidObstacleAvoidanceMPC>(mpc_params);
    
    // Set up states
    initial_state_.px = initial_pos.x();
    initial_state_.py = initial_pos.y();
    initial_state_.theta = initial_pos.z();
    initial_state_.vx = initial_vel.x();
    initial_state_.vy = initial_vel.y();
    initial_state_.omega = initial_vel.z();
    
    target_state_.px = target_pos.x();
    target_state_.py = target_pos.y();
    target_state_.theta = target_pos.z();
    target_state_.vx = 0.0;
    target_state_.vy = 0.0;
    target_state_.omega = 0.0;
    
    current_state_ = initial_state_;
    start_time_ = std::chrono::high_resolution_clock::now();
    
    std::cout << "MPCObstacleAvoidingSimulator initialized" << std::endl;
    std::cout << "  dt: " << dt_ << " s, horizon: " << mpc_params_.horizon << std::endl;
    std::cout << "  Start: (" << initial_state_.px << ", " << initial_state_.py 
              << ", " << initial_state_.theta << ")" << std::endl;
    std::cout << "  Target: (" << target_state_.px << ", " << target_state_.py 
              << ", " << target_state_.theta << ")" << std::endl;
}

void MPCObstacleAvoidingSimulator::setObstacles(
    const std::vector<Eigen::Vector2d>& positions,
    const std::vector<double>& radii,
    const std::vector<Eigen::Vector2d>& velocities)
{
    obstacles_.clear();
    obstacle_velocities_.clear();
    
    for (size_t i = 0; i < positions.size() && i < radii.size(); ++i) {
        spg::integrated_mpc::Obstacle obs;
        obs.px = positions[i].x();
        obs.py = positions[i].y();
        obs.radius = radii[i];
        obs.active = true;
        obstacles_.push_back(obs);
        
        if (i < velocities.size()) {
            obstacle_velocities_.push_back(velocities[i]);
        } else {
            obstacle_velocities_.push_back(Eigen::Vector2d::Zero());
        }
    }
    
    std::cout << "Set " << obstacles_.size() << " obstacles" << std::endl;
}

void MPCObstacleAvoidingSimulator::step() {
    if (simulation_completed_) return;
    
    // Compute MPC control
    spg::integrated_mpc::ObstacleAvoidanceControl control;
    auto mpc_start = std::chrono::high_resolution_clock::now();
    
    bool success = mpc_->computeControl(current_state_, target_state_, obstacles_, control);
    
    auto mpc_end = std::chrono::high_resolution_clock::now();
    double mpc_ms = std::chrono::duration<double, std::milli>(mpc_end - mpc_start).count();
    total_mpc_time_ms_ += mpc_ms;
    mpc_times_ms_.push_back(mpc_ms);
    mpc_call_count_++;
    
    if (!success) {
        std::cerr << "MPC failed at step " << step_count_ << std::endl;
        mpc_failure_count_++;
    }
    
    // Get velocity command from MPC
    auto vel_cmd = mpc_->getVelocityCommand();
    
    // Integrate dynamics (simple forward Euler)
    // Using the velocity command directly (acceleration is already integrated by MPC)
    double cos_theta = std::cos(current_state_.theta);
    double sin_theta = std::sin(current_state_.theta);
    
    // Update velocities from MPC command
    current_state_.vx = vel_cmd.vx;
    current_state_.vy = vel_cmd.vy;
    current_state_.omega = vel_cmd.omega;
    
    // Update position (in global frame)
    current_state_.px += dt_ * (cos_theta * current_state_.vx - sin_theta * current_state_.vy);
    current_state_.py += dt_ * (sin_theta * current_state_.vx + cos_theta * current_state_.vy);
    current_state_.theta += dt_ * current_state_.omega;
    
    // Wrap theta to [-pi, pi]
    while (current_state_.theta > M_PI) current_state_.theta -= 2.0 * M_PI;
    while (current_state_.theta < -M_PI) current_state_.theta += 2.0 * M_PI;
    
    // Update obstacles
    updateObstacles(dt_);
    
    // Update counters
    simulation_time_ += dt_;
    step_count_++;
    
    // Check goal
    if (checkGoalReached()) {
        simulation_completed_ = true;
        printCompletionStats();
    }
}

void MPCObstacleAvoidingSimulator::updateObstacles(double dt) {
    // Simple field dimensions
    const double field_x_half = 6.0;
    const double field_y_half = 9.0;
    
    for (size_t i = 0; i < obstacles_.size(); ++i) {
        if (!obstacles_[i].active) continue;
        
        // Update position
        obstacles_[i].px += obstacle_velocities_[i].x() * dt;
        obstacles_[i].py += obstacle_velocities_[i].y() * dt;
        
        // Bounce off boundaries
        const double radius = obstacles_[i].radius;
        if (obstacles_[i].px - radius < -field_x_half || obstacles_[i].px + radius > field_x_half) {
            obstacle_velocities_[i].x() *= -1.0;
            obstacles_[i].px = std::max(-field_x_half + radius, std::min(field_x_half - radius, obstacles_[i].px));
        }
        if (obstacles_[i].py - radius < -field_y_half || obstacles_[i].py + radius > field_y_half) {
            obstacle_velocities_[i].y() *= -1.0;
            obstacles_[i].py = std::max(-field_y_half + radius, std::min(field_y_half - radius, obstacles_[i].py));
        }
    }
}

bool MPCObstacleAvoidingSimulator::checkGoalReached() {
    double dx = target_state_.px - current_state_.px;
    double dy = target_state_.py - current_state_.py;
    double position_error = std::sqrt(dx * dx + dy * dy);
    
    double velocity_norm = std::sqrt(
        current_state_.vx * current_state_.vx + 
        current_state_.vy * current_state_.vy
    );
    
    return (position_error < position_tolerance_ && velocity_norm < velocity_tolerance_);
}

void MPCObstacleAvoidingSimulator::printCompletionStats() {
    auto end_time = std::chrono::high_resolution_clock::now();
    double wall_time = std::chrono::duration<double>(end_time - start_time_).count();
    
    double dx = target_state_.px - current_state_.px;
    double dy = target_state_.py - current_state_.py;
    double position_error = std::sqrt(dx * dx + dy * dy);
    double velocity_norm = std::sqrt(
        current_state_.vx * current_state_.vx + 
        current_state_.vy * current_state_.vy
    );
    
    std::cout << "\n╔══════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║       MPC Obstacle Avoidance Simulation Completed!          ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════════╝" << std::endl;
    std::cout << "\nPerformance Metrics:" << std::endl;
    std::cout << "  Final position error: " << position_error << " m" << std::endl;
    std::cout << "  Final velocity: " << velocity_norm << " m/s" << std::endl;
    std::cout << "  Simulation time: " << simulation_time_ << " s" << std::endl;
    std::cout << "  Wall clock time: " << wall_time << " s" << std::endl;
    std::cout << "  Total steps: " << step_count_ << std::endl;
    
    if (mpc_call_count_ > 0) {
        double avg_mpc_ms = total_mpc_time_ms_ / mpc_call_count_;
        double min_mpc_ms = *std::min_element(mpc_times_ms_.begin(), mpc_times_ms_.end());
        double max_mpc_ms = *std::max_element(mpc_times_ms_.begin(), mpc_times_ms_.end());
        
        std::cout << "\nMPC Performance:" << std::endl;
        std::cout << "  MPC calls: " << mpc_call_count_ << std::endl;
        std::cout << "  Total MPC time: " << total_mpc_time_ms_ << " ms" << std::endl;
        std::cout << "  Average MPC time: " << avg_mpc_ms << " ms/step" << std::endl;
        std::cout << "  Min MPC time: " << min_mpc_ms << " ms" << std::endl;
        std::cout << "  Max MPC time: " << max_mpc_ms << " ms" << std::endl;
        std::cout << "  MPC frequency: " << (1000.0 / avg_mpc_ms) << " Hz" << std::endl;
        
        // Save timing data
        std::ofstream timing_file("mpc_obstacle_timing_results.json");
        if (timing_file.is_open()) {
            timing_file << "{" << std::endl;
            timing_file << "  \"simulation_time_s\": " << simulation_time_ << "," << std::endl;
            timing_file << "  \"wall_time_s\": " << wall_time << "," << std::endl;
            timing_file << "  \"total_steps\": " << step_count_ << "," << std::endl;
            timing_file << "  \"mpc_call_count\": " << mpc_call_count_ << "," << std::endl;
            timing_file << "  \"total_mpc_time_ms\": " << total_mpc_time_ms_ << "," << std::endl;
            timing_file << "  \"avg_mpc_time_ms\": " << avg_mpc_ms << "," << std::endl;
            timing_file << "  \"min_mpc_time_ms\": " << min_mpc_ms << "," << std::endl;
            timing_file << "  \"max_mpc_time_ms\": " << max_mpc_ms << "," << std::endl;
            timing_file << "  \"mpc_frequency_hz\": " << (1000.0 / avg_mpc_ms) << "," << std::endl;
            timing_file << "  \"mpc_failure_count\": " << mpc_failure_count_ << "," << std::endl;
            timing_file << "  \"mpc_success_rate\": " << (mpc_call_count_ > 0 ? (1.0 - (double)mpc_failure_count_ / mpc_call_count_) : 0.0) << "," << std::endl;
            timing_file << "  \"final_position_error_m\": " << position_error << "," << std::endl;
            timing_file << "  \"final_velocity_ms\": " << velocity_norm << "," << std::endl;
            timing_file << "  \"mpc_times_ms\": [";
            for (size_t i = 0; i < mpc_times_ms_.size(); ++i) {
                timing_file << mpc_times_ms_[i];
                if (i < mpc_times_ms_.size() - 1) timing_file << ", ";
            }
            timing_file << "]" << std::endl;
            timing_file << "}" << std::endl;
            timing_file.close();
            std::cout << "  Timing data saved to: mpc_obstacle_timing_results.json" << std::endl;
        }
    }
    std::cout << std::endl;
}

void MPCObstacleAvoidingSimulator::reset() {
    current_state_ = initial_state_;
    simulation_time_ = 0.0;
    step_count_ = 0;
    simulation_completed_ = false;
    total_mpc_time_ms_ = 0.0;
    mpc_call_count_ = 0;
    mpc_times_ms_.clear();
    start_time_ = std::chrono::high_resolution_clock::now();
    
    std::cout << "Simulation reset" << std::endl;
}

MPCObstacleAvoidingSimulator::MPCStats MPCObstacleAvoidingSimulator::getMPCStats() const {
    MPCStats stats;
    stats.call_count = mpc_call_count_;
    stats.total_time_ms = total_mpc_time_ms_;
    stats.avg_time_ms = (mpc_call_count_ > 0) ? (total_mpc_time_ms_ / mpc_call_count_) : 0.0;
    stats.min_time_ms = mpc_times_ms_.empty() ? 0.0 : *std::min_element(mpc_times_ms_.begin(), mpc_times_ms_.end());
    stats.max_time_ms = mpc_times_ms_.empty() ? 0.0 : *std::max_element(mpc_times_ms_.begin(), mpc_times_ms_.end());
    stats.times_ms = mpc_times_ms_;
    return stats;
}

void MPCObstacleAvoidingSimulator::run() {
    // Initialize visualization
    if (!visualizer_.initialize()) {
        std::cerr << "Failed to initialize visualizer, running without visualization" << std::endl;
        runWithoutVisualization();
        return;
    }
    
    auto last_time = std::chrono::high_resolution_clock::now();
    
    while (!visualizer_.shouldClose()) {
        visualizer_.beginFrame();
        
        // Check for reset
        if (visualizer_.shouldReset()) {
            reset();
            visualizer_.clearResetRequest();
        }
        
        // Handle step mode
        bool should_advance = true;
        if (visualizer_.isStepMode()) {
            should_advance = visualizer_.shouldStep();
            if (should_advance) {
                visualizer_.clearStepRequest();
            }
        } else {
            // Normal mode - handle timing
            auto current_time = std::chrono::high_resolution_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(current_time - last_time);
            
            double expected_dt_us = (dt_ * 1000000.0) / visualizer_.getSimulationSpeed();
            
            if (elapsed.count() < expected_dt_us) {
                auto sleep_time = std::chrono::microseconds(static_cast<long>(expected_dt_us - elapsed.count()));
                std::this_thread::sleep_for(sleep_time);
            }
            
            last_time = std::chrono::high_resolution_clock::now();
        }
        
        // Advance simulation
        if (should_advance && !simulation_completed_) {
            step();
        }
        
        // Visualize - convert to SPGState-like format for visualizer
        // We need to create dummy SPG state for visualization
        spg::SPGState dummy_state;
        dummy_state.input.robot.p = Eigen::Vector3d(current_state_.px, current_state_.py, current_state_.theta);
        dummy_state.input.robot.v = Eigen::Vector3d(current_state_.vx, current_state_.vy, current_state_.omega);
        dummy_state.target.p = Eigen::Vector3d(target_state_.px, target_state_.py, target_state_.theta);
        
        // Set obstacles
        dummy_state.input.obstacles.p.clear();
        dummy_state.input.obstacles.r.clear();
        dummy_state.input.obstacles.active.clear();
        for (const auto& obs : obstacles_) {
            if (obs.active) {
                dummy_state.input.obstacles.p.push_back(Eigen::Vector2d(obs.px, obs.py));
                dummy_state.input.obstacles.r.push_back(obs.radius);
                dummy_state.input.obstacles.active.push_back(true);
            }
        }
        
        // Get predicted trajectory
        std::vector<spg::integrated_mpc::ObstacleAvoidanceState> pred_states;
        std::vector<spg::integrated_mpc::ObstacleAvoidanceControl> pred_controls;
        mpc_->getPredictedTrajectory(pred_states, pred_controls);
        
        // Convert predicted trajectory to visualizer format
        std::vector<Eigen::Vector2d> predicted_traj;
        for (const auto& ps : pred_states) {
            predicted_traj.push_back(Eigen::Vector2d(ps.px, ps.py));
        }
        
        // Render using low-level visualizer API
        RobotState robot{dummy_state.input.robot.p, dummy_state.input.robot.v};
        std::vector<ObstacleState> obstacles_vis;
        for (size_t i = 0; i < dummy_state.input.obstacles.p.size(); ++i) {
            if (dummy_state.input.obstacles.active[i]) {
                obstacles_vis.push_back({dummy_state.input.obstacles.p[i], dummy_state.input.obstacles.r[i], true});
            }
        }
        BallState ball{{0, 0}, {0, 0}};
        std::vector<Eigen::Vector2d> empty_intercepts;
        std::vector<std::vector<Eigen::Vector2d>> empty_paths;
        
        // Debug: Print obstacle info every 50 frames
        static int viz_debug_ctr = 0;
        if (viz_debug_ctr++ % 50 == 0) {
            std::cout << "\n=== Visualization Debug (frame " << step_count_ << ") ===" << std::endl;
            std::cout << "Obstacles in simulator: " << obstacles_.size() << std::endl;
            for (size_t i = 0; i < obstacles_.size(); ++i) {
                std::cout << "  Obs " << i << ": pos=(" << obstacles_[i].px << "," << obstacles_[i].py 
                          << ") r=" << obstacles_[i].radius << " active=" << obstacles_[i].active << std::endl;
            }
            std::cout << "Obstacles sent to visualizer: " << obstacles_vis.size() << std::endl;
            for (size_t i = 0; i < obstacles_vis.size(); ++i) {
                std::cout << "  VisObs " << i << ": pos=(" << obstacles_vis[i].pos.x() << "," 
                          << obstacles_vis[i].pos.y() << ") r=" << obstacles_vis[i].radius << std::endl;
            }
        }
        
        visualizer_.render(robot, obstacles_vis, ball, predicted_traj, empty_intercepts, 
                          empty_paths, dummy_state.target.p, dummy_state.input.robot.p, 
                          simulation_time_, step_count_, false);
        visualizer_.endFrame();
        
        if (simulation_completed_) {
            visualizer_.requestClose();
        }
    }
}

void MPCObstacleAvoidingSimulator::runWithoutVisualization() {
    while (!simulation_completed_) {
        step();
        
        // Safety limit
        if (step_count_ > 10000) {
            std::cerr << "Exceeded maximum steps, terminating" << std::endl;
            break;
        }
    }
}

void MPCObstacleAvoidingSimulator::runWithoutVisualization(int max_steps, double dt) {
    dt_ = dt;
    
    for (int i = 0; i < max_steps && !simulation_completed_; ++i) {
        step();
    }
}
