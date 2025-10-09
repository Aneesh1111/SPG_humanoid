#include <iostream>
#include <Eigen/Dense>
#include "SPGSimulator.hpp"
#include "spg/Init.hpp"

int main() {
    // Initial parameters (from MATLAB main.m)
    Eigen::Vector3d p_initial(4, -6, 0);
    Eigen::Vector3d v_initial(0, 0, 0);
    int npredict = 20;
    int nobstacles = 0; // min(10, 78)
    int nintercept_positions = 15;
    Eigen::Vector2d p_initial_ball(0, 0);
    Eigen::Vector2d v_initial_ball(0, 0);

    // Initialize state using spg::Init
    auto initial_state = spg::Init(p_initial, v_initial, nobstacles, npredict, p_initial_ball, v_initial_ball, nintercept_positions);

    // Set target position and velocity (CORRECT WAY: set input.robot.target, not target.p directly)
    initial_state.input.robot.target = Eigen::Vector3d(0, 0, 0);
    initial_state.input.robot.target_vel = Eigen::Vector3d(0, 0, 0);

    // Initialize obstacles with positions, velocities, and activate them
    std::cout << "Initializing " << nobstacles << " obstacles..." << std::endl;
    for (int i = 0; i < nobstacles && i < 10; ++i) { // Only activate first 10 obstacles for demo
        // Place obstacles at various positions around the field
        double angle = 2.0 * M_PI * i / 5.0; // Distribute around a circle
        double radius = 2.0 + i * 0.5; // Vary the radius
        
        initial_state.input.obstacles.p[i] = Eigen::Vector2d(
            radius * cos(angle), 
            radius * sin(angle)
        );
        
        // Give obstacles some velocity (moving in tangential direction)
        initial_state.input.obstacles.v[i] = Eigen::Vector2d(
            -2 * sin(angle), 
            2 * cos(angle)
        );
        
        // Set obstacle radius
        initial_state.input.obstacles.r[i] = 0.3; // 30cm radius
        
        // Activate the obstacle
        initial_state.input.obstacles.active[i] = true;
        
        std::cout << "Obstacle " << i << ": pos=(" << initial_state.input.obstacles.p[i].x() 
                  << ", " << initial_state.input.obstacles.p[i].y() << "), vel=(" 
                  << initial_state.input.obstacles.v[i].x() << ", " 
                  << initial_state.input.obstacles.v[i].y() << ")" << std::endl;
    }

    // Optionally set other fields if needed (e.g., initial_state.input.subtarget_avoid_polygon.valid = true; ...)
    // initial_state.input.subtarget_avoid_polygon.valid = true;
    // initial_state.input.subtarget_avoid_polygon.polygon << -2, -0.5, -1.5, 0.5, 2, 1, 2, 0;

    // Create and run simulator
    SPGSimulator simulator(initial_state);
    simulator.run();

    std::cout << "Simulation complete." << std::endl;
    return 0;
}
