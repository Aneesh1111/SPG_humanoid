#include <iostream>
#include <Eigen/Dense>
#include "SPGSimulator.hpp"
#include "spg/Init.hpp"

int main() {
    // Initial parameters (from MATLAB main.m)
    Eigen::Vector3d p_initial(3, 0, 0);
    Eigen::Vector3d v_initial(0, 3, 0);
    int npredict = 20;
    int nobstacles = 10; // min(10, 78)
    int nintercept_positions = 15;
    Eigen::Vector2d p_initial_ball(0, 0);
    Eigen::Vector2d v_initial_ball(0, 0);

    // Initialize state using spg::Init
    auto initial_state = spg::Init(p_initial, v_initial, nobstacles, npredict, p_initial_ball, v_initial_ball, nintercept_positions);

    // Set target position and velocity (CORRECT WAY: set input.robot.target, not target.p directly)
    initial_state.input.robot.target = Eigen::Vector3d(-1, -1, 0);
    initial_state.input.robot.target_vel = Eigen::Vector3d(0, 0, 0);

    // Optionally set other fields if needed (e.g., initial_state.input.subtarget_avoid_polygon.valid = true; ...)
    // initial_state.input.subtarget_avoid_polygon.valid = true;
    // initial_state.input.subtarget_avoid_polygon.polygon << -2, -0.5, -1.5, 0.5, 2, 1, 2, 0;

    // Create and run simulator
    SPGSimulator simulator(initial_state);
    simulator.run();

    std::cout << "Simulation complete." << std::endl;
    return 0;
}
