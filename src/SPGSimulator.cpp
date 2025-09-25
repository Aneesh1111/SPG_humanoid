#include "SPGSimulator.hpp"
#include "spg/setpoint/Setpoint.hpp"
#include "spg/setpoint/StateCorrection.hpp"
#include "spg/subtarget/SubtargetSet.hpp"
#include "spg/target/Target.hpp"
#include "visualization/SimulatorVisualizer.hpp"
#include <iostream>
#include <vector>

SPGSimulator::SPGSimulator(const spg::SPGState& initial_state)
    : state_(initial_state),
      visualizer({
          state_.par.field_size[0], state_.par.field_size[1],
          state_.par.field_penalty_area[0], state_.par.field_penalty_area[1],
          state_.par.field_border_margin, state_.par.field_circle_radius
      }) {}

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
    while (step < 1000 && !visualizer.shouldClose()) {
        visualizer.beginFrame();
        
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

        // Log current position
        std::cout << "Step " << step << ": Robot position = ("
                  << state_.input.robot.p[0] << ", "
                  << state_.input.robot.p[1] << ", "
                  << state_.input.robot.p[2] << ")\n";

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
        visualizer.render(robot, obstacles, ball, robot_traj, obs_traj, state_.target.p);
        
        visualizer.endFrame();
        step++;
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

        // Log current position
        std::cout << "Step " << step << ": Robot position = ("
                  << state_.input.robot.p[0] << ", "
                  << state_.input.robot.p[1] << ", "
                  << state_.input.robot.p[2] << ")\n";
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
    visualizer.render(robot, obstacles, ball, robot_traj, obs_traj, state_.target.p);
}