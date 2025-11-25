#include "SPGSimulator.hpp"
#include "spg/setpoint/Setpoint.hpp"
#include "spg/setpoint/StateCorrection.hpp"
#include "spg/subtarget/SubtargetSet.hpp"
#include "spg/target/Target.hpp"
#include "visualization/SimulatorVisualizer.hpp"
#include <iostream>
#include <vector>
#include <chrono>
#include <thread>

SPGSimulator::SPGSimulator(const spg::SPGState& initial_state)
    : state_(initial_state), initial_state_(initial_state),
      visualizer({
          state_.par.field_size[0], state_.par.field_size[1],
          state_.par.field_penalty_area[0], state_.par.field_penalty_area[1],
          state_.par.field_border_margin, state_.par.field_circle_radius
      }), simulation_time_(0.0), step_count_(0), simulation_completed_(false) {}



void SPGSimulator::run() {
    // Initialize visualization
    if (!visualizer.initialize()) {
        std::cerr << "Failed to initialize visualizer, running without visualization" << std::endl;
        // Run without visualization
        runWithoutVisualization();
        return;
    }
    
    // Example simulation loop with visualization
    int step = 0;
    auto last_time = std::chrono::high_resolution_clock::now();
    
    while (!visualizer.shouldClose()) {
        visualizer.beginFrame();
        
        // Check for reset request
        if (visualizer.shouldReset()) {
            state_ = initial_state_; // Reset to initial state
            step = 0;
            simulation_time_ = 0.0; // Reset simulation time
            step_count_ = 0; // Reset step count
            simulation_completed_ = false; // Reset completion status
            last_time = std::chrono::high_resolution_clock::now();
            std::cout << "Simulation reset to initial state" << std::endl;
            visualizer.clearResetRequest();
        }
        
        // Handle step mode
        bool should_advance = true;
        if (visualizer.isStepMode()) {
            // In step mode, only advance if step is requested
            should_advance = visualizer.shouldStep();
            if (should_advance) {
                visualizer.clearStepRequest();
            }
        } else {
            // In normal mode, handle timing
            auto current_time = std::chrono::high_resolution_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(current_time - last_time);
            
            // Expected time step in microseconds based on simulation speed
            double expected_dt_us = (state_.par.Ts * 1000000.0) / visualizer.getSimulationSpeed();
            
            // Sleep if we're running too fast
            if (elapsed.count() < expected_dt_us) {
                auto sleep_time = std::chrono::microseconds(static_cast<long>(expected_dt_us - elapsed.count()));
                std::this_thread::sleep_for(sleep_time);
            }
            
            last_time = std::chrono::high_resolution_clock::now();
        }
        
        // Only advance simulation if allowed and not completed
        if (should_advance && !simulation_completed_) {
            // State correction (e.g., reset triggers)
            spg::setpoint::stateCorrection(state_);
            // Update target
            spg::target::updateTarget(state_);
            // Update subtarget
            spg::subtarget::updateSubtarget(state_);
            // Update setpoint
            spg::setpoint::updateSetpoint(state_);

            // Simulate robot motion (simple propagation)
            for (int i = 0; i < 3; ++i) {
                state_.input.robot.p[i] = state_.setpoint.p[i];
                state_.input.robot.v[i] = state_.setpoint.v[i];
            }

            // Simulate obstacle motion (simple integration)
            double dt = state_.par.Ts; // Use the sampling time from parameters
            simulation_time_ += dt; // Update simulation time
            step_count_++; // Increment step counter
            
            // Check if simulation should be completed (robot reached target with low velocity)
            Eigen::Vector3d position_error = state_.input.robot.p - state_.target.p;
            double position_error_norm = position_error.norm();
            double velocity_norm = state_.input.robot.v.norm();
            
            if (position_error_norm < 1e-3 && velocity_norm < 1e-3) { // Using 1e-3 for better practical threshold
                simulation_completed_ = true;
                std::cout << "Simulation completed! Robot reached target." << std::endl;
                std::cout << "Final position error: " << position_error_norm << std::endl;
                std::cout << "Final velocity: " << velocity_norm << std::endl;
                std::cout << "Total simulation time: " << simulation_time_ << " seconds" << std::endl;
                std::cout << "Total steps: " << step_count_ << std::endl;
            }
            for (size_t i = 0; i < state_.input.obstacles.p.size(); ++i) {
                if (state_.input.obstacles.active[i]) {
                    // Update obstacle position based on velocity
                    state_.input.obstacles.p[i] += state_.input.obstacles.v[i] * dt;
                    
                    // Bounce off field boundaries
                    // Use field dimensions from SPG parameters for proper field size
                    const double field_x_half = state_.par.field_size[0] / 2.0;  // field_width / 2
                    const double field_y_half = state_.par.field_size[1] / 2.0;  // field_height / 2
                    const double field_x_min = -field_x_half, field_x_max = field_x_half;
                    const double field_y_min = -field_y_half, field_y_max = field_y_half;
                    const double radius = state_.input.obstacles.r[i];
                    
                    // Bounce off X boundaries
                    if (state_.input.obstacles.p[i](0) - radius < field_x_min) {
                        state_.input.obstacles.p[i](0) = field_x_min + radius;
                        state_.input.obstacles.v[i](0) = -0.8 * state_.input.obstacles.v[i](0); // Reverse X velocity with damping
                    }
                    if (state_.input.obstacles.p[i](0) + radius > field_x_max) {
                        state_.input.obstacles.p[i](0) = field_x_max - radius;
                        state_.input.obstacles.v[i](0) = -0.8 * state_.input.obstacles.v[i](0); // Reverse X velocity with damping
                    }
                    
                    // Bounce off Y boundaries
                    if (state_.input.obstacles.p[i](1) - radius < field_y_min) {
                        state_.input.obstacles.p[i](1) = field_y_min + radius;
                        state_.input.obstacles.v[i](1) = -0.8 * state_.input.obstacles.v[i](1); // Reverse Y velocity with damping
                    }
                    if (state_.input.obstacles.p[i](1) + radius > field_y_max) {
                        state_.input.obstacles.p[i](1) = field_y_max - radius;
                        state_.input.obstacles.v[i](1) = -0.8 * state_.input.obstacles.v[i](1); // Reverse Y velocity with damping
                    }
                }
            }

            // Log current position
            std::cout << "Step " << step << ": Robot position = ("
                      << state_.input.robot.p[0] << ", "
                      << state_.input.robot.p[1] << ", "
                      << state_.input.robot.p[2] << ")\n";
            
            // Only increment step counter when we actually advance
            step++;
        }
        
        // Always render current state (even when paused in step mode)
        RobotState robot_display{
            state_.input.robot.p,
            state_.input.robot.v
        };
        std::vector<ObstacleState> obstacles_display;
        for (size_t i = 0; i < state_.input.obstacles.p.size(); ++i) {
            obstacles_display.push_back(ObstacleState{
                state_.input.obstacles.p[i],
                state_.input.obstacles.r[i],
                state_.input.obstacles.active[i]
            });
        }
        BallState ball_display{
            state_.input.ball.p,
            state_.input.ball.v
        };
        std::vector<Eigen::Vector2d> robot_traj_display;
        for (const auto& p : state_.traj.p) {
            robot_traj_display.push_back(p.head<2>());
        }
        std::vector<std::vector<Eigen::Vector2d>> obs_traj_display; // Fill if you have obstacle trajectories
        visualizer.render(robot_display, obstacles_display, ball_display, robot_traj_display, obs_traj_display, state_.target.p, state_.subtarget.p, simulation_time_, step_count_, simulation_completed_);
        
        visualizer.endFrame();
    }
    
    visualizer.cleanup();
}

void SPGSimulator::runHumanoid() {
    // Initialize visualization
    if (!visualizer.initialize()) {
        std::cerr << "Failed to initialize visualizer, running without visualization" << std::endl;
        // Run without visualization
        runWithoutVisualization();
        return;
    }
    
    std::cout << "🤖 STARTING HUMANOID SIMULATION" << std::endl;
    std::cout << "===============================" << std::endl;
    std::cout << "Using humanoid-aware trajectory generation with:" << std::endl;
    std::cout << "  vmax_move_x (sideways): " << state_.par.vmax_move_x << " m/s" << std::endl;
    std::cout << "  vmax_move_y (forward):  " << state_.par.vmax_move_y << " m/s" << std::endl;
    std::cout << "  forward_efficiency:     " << state_.par.forward_efficiency * 100 << "%" << std::endl;
    std::cout << "  sideways_efficiency:    " << state_.par.sideways_efficiency * 100 << "%" << std::endl;
    std::cout << "" << std::endl;
    
    // Example simulation loop with visualization
    int step = 0;
    auto last_time = std::chrono::high_resolution_clock::now();
    
    while (!visualizer.shouldClose()) {
        visualizer.beginFrame();
        
        // Check for reset request
        if (visualizer.shouldReset()) {
            state_ = initial_state_; // Reset to initial state
            step = 0;
            simulation_time_ = 0.0; // Reset simulation time
            step_count_ = 0; // Reset step count
            simulation_completed_ = false; // Reset completion status
            last_time = std::chrono::high_resolution_clock::now();
            std::cout << "🔄 Humanoid simulation reset to initial state" << std::endl;
            visualizer.clearResetRequest();
        }
        
        // Handle step mode
        bool should_advance = true;
        if (visualizer.isStepMode()) {
            // In step mode, only advance if step is requested
            should_advance = visualizer.shouldStep();
            if (should_advance) {
                visualizer.clearStepRequest();
            }
        } else {
            // In normal mode, handle timing
            auto current_time = std::chrono::high_resolution_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(current_time - last_time);
            
            // Expected time step in microseconds based on simulation speed
            double expected_dt_us = (state_.par.Ts * 1000000.0) / visualizer.getSimulationSpeed();
            
            // Sleep if we're running too fast
            if (elapsed.count() < expected_dt_us) {
                auto sleep_time = std::chrono::microseconds(static_cast<long>(expected_dt_us - elapsed.count()));
                std::this_thread::sleep_for(sleep_time);
            }
            
            last_time = std::chrono::high_resolution_clock::now();
        }
        
        // Only advance simulation if allowed and not completed
        if (should_advance && !simulation_completed_) {
            std::cout << "\n--- Step " << step << " ---" << std::endl;
            
            // State correction (e.g., reset triggers)
            spg::setpoint::stateCorrection(state_);
            // Update target
            spg::target::updateTarget(state_);
            // Update subtarget
            spg::subtarget::updateSubtarget(state_);
            // 🤖 USE HUMANOID SETPOINT UPDATE instead of standard
            spg::setpoint::updateSetpointHumanoid(state_);

            // Simulate robot motion (simple propagation)
            for (int i = 0; i < 3; ++i) {
                state_.input.robot.p[i] = state_.setpoint.p[i];
                state_.input.robot.v[i] = state_.setpoint.v[i];
            }
            
            // CRITICAL: Update IMU orientation from setpoint (for humanoid rotation)
            state_.input.robot.IMU_orientation(2) = state_.setpoint.p(2); // Update yaw angle

            // Simulate obstacle motion (simple integration)
            double dt = state_.par.Ts; // Use the sampling time from parameters
            simulation_time_ += dt; // Update simulation time
            step_count_++; // Increment step counter
            
            // Check if simulation should be completed (robot reached target with low velocity)
            Eigen::Vector3d position_error = state_.input.robot.p - state_.target.p;
            double position_error_norm = position_error.norm();
            double velocity_norm = state_.input.robot.v.norm();
            
            if (position_error_norm < 0.1 && velocity_norm < 0.1) { // More relaxed threshold for humanoid
                simulation_completed_ = true;
                std::cout << "\n🎉 HUMANOID SIMULATION COMPLETED!" << std::endl;
                std::cout << "Robot successfully reached target using humanoid constraints." << std::endl;
                std::cout << "Final position error: " << position_error_norm << " m" << std::endl;
                std::cout << "Final velocity: " << velocity_norm << " m/s" << std::endl;
                std::cout << "Total simulation time: " << simulation_time_ << " seconds" << std::endl;
                std::cout << "Total steps: " << step_count_ << std::endl;
            }
            
            for (size_t i = 0; i < state_.input.obstacles.p.size(); ++i) {
                if (state_.input.obstacles.active[i]) {
                    // Update obstacle position based on velocity
                    state_.input.obstacles.p[i] += state_.input.obstacles.v[i] * dt;
                    
                    // Bounce off field boundaries
                    // Use field dimensions from SPG parameters for proper field size
                    const double field_x_half = state_.par.field_size[0] / 2.0;  // field_width / 2
                    const double field_y_half = state_.par.field_size[1] / 2.0;  // field_height / 2
                    const double field_x_min = -field_x_half, field_x_max = field_x_half;
                    const double field_y_min = -field_y_half, field_y_max = field_y_half;
                    const double radius = state_.input.obstacles.r[i];
                    
                    // Bounce off X boundaries
                    if (state_.input.obstacles.p[i](0) - radius < field_x_min) {
                        state_.input.obstacles.p[i](0) = field_x_min + radius;
                        state_.input.obstacles.v[i](0) = -0.8 * state_.input.obstacles.v[i](0); // Reverse X velocity with damping
                    }
                    if (state_.input.obstacles.p[i](0) + radius > field_x_max) {
                        state_.input.obstacles.p[i](0) = field_x_max - radius;
                        state_.input.obstacles.v[i](0) = -0.8 * state_.input.obstacles.v[i](0); // Reverse X velocity with damping
                    }
                    
                    // Bounce off Y boundaries
                    if (state_.input.obstacles.p[i](1) - radius < field_y_min) {
                        state_.input.obstacles.p[i](1) = field_y_min + radius;
                        state_.input.obstacles.v[i](1) = -0.8 * state_.input.obstacles.v[i](1); // Reverse Y velocity with damping
                    }
                    if (state_.input.obstacles.p[i](1) + radius > field_y_max) {
                        state_.input.obstacles.p[i](1) = field_y_max - radius;
                        state_.input.obstacles.v[i](1) = -0.8 * state_.input.obstacles.v[i](1); // Reverse Y velocity with damping
                    }
                }
            }
            
            // Only increment step counter when we actually advance
            step++;
        }
        
        // Always render current state (even when paused in step mode)
        RobotState robot_display{
            state_.input.robot.p,
            state_.input.robot.v
        };
        std::vector<ObstacleState> obstacles_display;
        for (size_t i = 0; i < state_.input.obstacles.p.size(); ++i) {
            obstacles_display.push_back(ObstacleState{
                state_.input.obstacles.p[i],
                state_.input.obstacles.r[i],
                state_.input.obstacles.active[i]
            });
        }
        BallState ball_display{
            state_.input.ball.p,
            state_.input.ball.v
        };
        std::vector<Eigen::Vector2d> robot_traj_display;
        for (const auto& p : state_.traj.p) {
            robot_traj_display.push_back(p.head<2>());
        }
        std::vector<std::vector<Eigen::Vector2d>> obs_traj_display; // Fill if you have obstacle trajectories
        visualizer.render(robot_display, obstacles_display, ball_display, robot_traj_display, obs_traj_display, state_.target.p, state_.subtarget.p, simulation_time_, step_count_, simulation_completed_);
        
        visualizer.endFrame();
    }
    
    visualizer.cleanup();
}

void SPGSimulator::runWithoutVisualization() {
    // Fallback simulation loop without visualization
    for (int step = 0; step < 100; ++step) {
        // State correction (e.g., reset triggers)
        spg::setpoint::stateCorrection(state_);
        // Update target
        spg::target::updateTarget(state_);
        // Update subtarget
        spg::subtarget::updateSubtarget(state_);
        // Update setpoint
        spg::setpoint::updateSetpoint(state_);

        // Simulate robot motion (simple propagation)
        for (int i = 0; i < 3; ++i) {
            state_.input.robot.p[i] = state_.setpoint.p[i];
            state_.input.robot.v[i] = state_.setpoint.v[i];
        }

        // Simulate obstacle motion (simple integration)
        double dt = state_.par.Ts; // Use the sampling time from parameters
        for (size_t i = 0; i < state_.input.obstacles.p.size(); ++i) {
            if (state_.input.obstacles.active[i]) {
                // Update obstacle position based on velocity
                state_.input.obstacles.p[i] += state_.input.obstacles.v[i] * dt;
                
                // Bounce off field boundaries
                // Field dimensions: -4.5 to 4.5 in X, -3 to 3 in Y (typical soccer field)
                const double field_x_min = -4.5, field_x_max = 4.5;
                const double field_y_min = -3.0, field_y_max = 3.0;
                const double radius = state_.input.obstacles.r[i];
                
                // Bounce off X boundaries
                if (state_.input.obstacles.p[i](0) - radius < field_x_min) {
                    state_.input.obstacles.p[i](0) = field_x_min + radius;
                    state_.input.obstacles.v[i](0) = -0.8 * state_.input.obstacles.v[i](0); // Reverse X velocity with damping
                }
                if (state_.input.obstacles.p[i](0) + radius > field_x_max) {
                    state_.input.obstacles.p[i](0) = field_x_max - radius;
                    state_.input.obstacles.v[i](0) = -0.8 * state_.input.obstacles.v[i](0); // Reverse X velocity with damping
                }
                
                // Bounce off Y boundaries
                if (state_.input.obstacles.p[i](1) - radius < field_y_min) {
                    state_.input.obstacles.p[i](1) = field_y_min + radius;
                    state_.input.obstacles.v[i](1) = -0.8 * state_.input.obstacles.v[i](1); // Reverse Y velocity with damping
                }
                if (state_.input.obstacles.p[i](1) + radius > field_y_max) {
                    state_.input.obstacles.p[i](1) = field_y_max - radius;
                    state_.input.obstacles.v[i](1) = -0.8 * state_.input.obstacles.v[i](1); // Reverse Y velocity with damping
                }
            }
        }

        // Log current position
        std::cout << "Step " << step << ": Robot position = ("
                  << state_.input.robot.p[0] << ", "
                  << state_.input.robot.p[1] << ", "
                  << state_.input.robot.p[2] << ")\n";
    }
}

void SPGSimulator::runWithoutVisualization(int max_steps, double dt_override) {
    // Parameterized simulation loop without visualization for testing
    for (int step = 0; step < max_steps; ++step) {
        // State correction (e.g., reset triggers)
        spg::setpoint::stateCorrection(state_);
        // Update target
        spg::target::updateTarget(state_);
        // Update subtarget
        spg::subtarget::updateSubtarget(state_);
        // Update setpoint
        spg::setpoint::updateSetpoint(state_);

        // Simulate robot motion (simple propagation)
        for (int i = 0; i < 3; ++i) {
            state_.input.robot.p[i] = state_.setpoint.p[i];
            state_.input.robot.v[i] = state_.setpoint.v[i];
        }

        // Simulate obstacle motion (simple integration)
        double dt = dt_override; // Use provided dt
        for (size_t i = 0; i < state_.input.obstacles.p.size(); ++i) {
            if (state_.input.obstacles.active[i]) {
                // Update obstacle position based on velocity
                state_.input.obstacles.p[i] += state_.input.obstacles.v[i] * dt;
                
                // Bounce off field boundaries
                // Field dimensions: -4.5 to 4.5 in X, -3 to 3 in Y (typical soccer field)
                const double field_x_min = -4.5, field_x_max = 4.5;
                const double field_y_min = -3.0, field_y_max = 3.0;
                const double radius = state_.input.obstacles.r[i];
                
                // Bounce off X boundaries
                if (state_.input.obstacles.p[i](0) - radius < field_x_min) {
                    state_.input.obstacles.p[i](0) = field_x_min + radius;
                    state_.input.obstacles.v[i](0) = -0.8 * state_.input.obstacles.v[i](0); // Reverse X velocity with damping
                }
                if (state_.input.obstacles.p[i](0) + radius > field_x_max) {
                    state_.input.obstacles.p[i](0) = field_x_max - radius;
                    state_.input.obstacles.v[i](0) = -0.8 * state_.input.obstacles.v[i](0); // Reverse X velocity with damping
                }
                
                // Bounce off Y boundaries
                if (state_.input.obstacles.p[i](1) - radius < field_y_min) {
                    state_.input.obstacles.p[i](1) = field_y_min + radius;
                    state_.input.obstacles.v[i](1) = -0.8 * state_.input.obstacles.v[i](1); // Reverse Y velocity with damping
                }
                if (state_.input.obstacles.p[i](1) + radius > field_y_max) {
                    state_.input.obstacles.p[i](1) = field_y_max - radius;
                    state_.input.obstacles.v[i](1) = -0.8 * state_.input.obstacles.v[i](1); // Reverse Y velocity with damping
                }
            }
        }
    }
}

void SPGSimulator::step() {
    spg::setpoint::stateCorrection(state_);
    spg::target::updateTarget(state_);
    spg::subtarget::updateSubtarget(state_);
    spg::setpoint::updateSetpoint(state_);
    for (int i = 0; i < 3; ++i) {
        state_.input.robot.p[i] = state_.setpoint.p[i];
        state_.input.robot.v[i] = state_.setpoint.v[i];
    }
    // Visualization integration
    RobotState robot{
        state_.input.robot.p,
        state_.input.robot.v
    };
    std::vector<ObstacleState> obstacles;
    for (size_t i = 0; i < state_.input.obstacles.p.size(); ++i) {
        obstacles.push_back(ObstacleState{
            state_.input.obstacles.p[i],
            state_.input.obstacles.r[i],
            state_.input.obstacles.active[i]
        });
    }
    BallState ball{
        state_.input.ball.p,
        state_.input.ball.v
    };
    std::vector<Eigen::Vector2d> robot_traj;
    for (const auto& p : state_.traj.p) {
        robot_traj.push_back(p.head<2>());
    }
    std::vector<std::vector<Eigen::Vector2d>> obs_traj; // Fill if you have obstacle trajectories
    visualizer.render(robot, obstacles, ball, robot_traj, obs_traj, state_.target.p, state_.subtarget.p, simulation_time_, step_count_, simulation_completed_);
}