/**
 * @file demo_humanoid_mpc.cpp
 * @brief Visual demonstration of HumanoidMPC integration with SPG
 * 
 * This demo shows how to switch between:
 * 1. Traditional MSL (Middle Size League) setpoint generation
 * 2. HumanoidMPC-based trajectory generation
 * 
 * Interactive controls:
 * - Press 'M' to toggle between MSL and HumanoidMPC modes
 * - Press 'R' to reset simulation
 * - Click to set new target
 */

#include <iostream>
#include <Eigen/Dense>
#include "SPGSimulator.hpp"
#include "spg/Init.hpp"

int main() {
    std::cout << "\n╔══════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║      SPG Controller Mode Demonstration (Visual)             ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════════╝\n" << std::endl;
    
    std::cout << "Controls:" << std::endl;
    std::cout << "  M - Toggle between MSL and HumanoidMPC modes" << std::endl;
    std::cout << "  R - Reset simulation" << std::endl;
    std::cout << "  Click - Set new target position" << std::endl;
    std::cout << "  ESC - Exit\n" << std::endl;
    
    // Initial parameters
    Eigen::Vector3d p_initial(-3, -4, 0);      // Start position
    Eigen::Vector3d v_initial(0, 0, 0);        // Initially at rest
    int npredict = 20;                         // Prediction horizon for visualization
    int nobstacles = 5;                        // Add some obstacles
    int nintercept_positions = 15;
    Eigen::Vector2d p_initial_ball(0, 0);
    Eigen::Vector2d v_initial_ball(0, 0);

    // Initialize state using spg::Init
    auto initial_state = spg::Init(p_initial, v_initial, nobstacles, npredict, 
                                    p_initial_ball, v_initial_ball, nintercept_positions);

    // ========================================
    // Choose initial mode: MSL or HumanoidMPC
    // ========================================
    // Change this to test different modes:
    initial_state.par.use_humanoid_mpc = true;  // Start with HumanoidMPC mode
    
    if (initial_state.par.use_humanoid_mpc) {
        std::cout << "Starting in HUMANOID MPC MODE" << std::endl;
        std::cout << "  • Body-frame control (forward/sideways/yaw)" << std::endl;
        std::cout << "  • MPC optimization with smoothness cost" << std::endl;
        std::cout << "  • Physical limits: vf=1.5 m/s, vs=0.5 m/s, ω=2.0 rad/s" << std::endl;
    } else {
        std::cout << "Starting in MSL MODE (Traditional)" << std::endl;
        std::cout << "  • Omnidirectional control" << std::endl;
        std::cout << "  • Segment-based trajectory planning" << std::endl;
        std::cout << "  • Fast computation, no optimization" << std::endl;
    }
    std::cout << std::endl;

    // Set target position (goal)
    Eigen::Vector3d target(3, 4, 0);  // Move to opposite corner
    initial_state.input.robot.target = target;
    initial_state.input.robot.target_vel = Eigen::Vector3d(0, 0, 0);

    // Initialize obstacles
    std::cout << "Setting up " << nobstacles << " obstacles..." << std::endl;
    for (int i = 0; i < nobstacles; ++i) {
        // Place obstacles in a line between start and goal
        double t = (i + 1.0) / (nobstacles + 1.0);
        Eigen::Vector2d obs_pos = (1.0 - t) * p_initial.head<2>() + t * target.head<2>();
        
        // Add some offset to make path more interesting
        double offset_angle = M_PI / 2.0;
        double offset_mag = 1.5 * (i % 2 == 0 ? 1 : -1);
        obs_pos += offset_mag * Eigen::Vector2d(cos(offset_angle), sin(offset_angle));
        
        initial_state.input.obstacles.p[i] = obs_pos;
        
        // Small random velocities
        initial_state.input.obstacles.v[i] = Eigen::Vector2d(
            0.3 * cos(i * 1.5), 
            0.3 * sin(i * 1.5)
        );
        
        initial_state.input.obstacles.r[i] = 0.35;  // 35cm radius
        initial_state.input.obstacles.active[i] = true;
        
        std::cout << "  Obstacle " << i << ": (" 
                  << initial_state.input.obstacles.p[i].x() << ", " 
                  << initial_state.input.obstacles.p[i].y() << ")" << std::endl;
    }
    std::cout << std::endl;

    // Print mode information
    std::cout << "╔══════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║  Press 'M' in the visualization window to toggle modes      ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════════╝\n" << std::endl;

    // Create and run simulator
    SPGSimulator simulator(initial_state);
    simulator.run();

    std::cout << "\nSimulation complete." << std::endl;
    std::cout << "\n╔══════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║                    Quick Reference                           ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════════╝" << std::endl;
    std::cout << "\nTo switch modes programmatically:" << std::endl;
    std::cout << "  d.par.use_humanoid_mpc = true;   // HumanoidMPC mode" << std::endl;
    std::cout << "  d.par.use_humanoid_mpc = false;  // MSL mode (default)" << std::endl;
    std::cout << "\nSame inputs/outputs for both modes:" << std::endl;
    std::cout << "  Input:  d.subtarget.p (goal position)" << std::endl;
    std::cout << "  Output: d.setpoint.{p,v,a} (commands)" << std::endl;
    std::cout << "\n" << std::endl;
    
    return 0;
}
