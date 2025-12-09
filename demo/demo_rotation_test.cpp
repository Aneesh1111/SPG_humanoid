/**
 * @file demo_rotation_test.cpp
 * @brief Test demonstration showing robot rotation visualization
 * 
 * This demo makes the robot rotate to show the enhanced orientation visualization
 */

#include <iostream>
#include <Eigen/Dense>
#include "SPGSimulator.hpp"
#include "spg/Init.hpp"

int main() {
    std::cout << "\n╔══════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║        Rotation Visualization Test                          ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════════╝\n" << std::endl;
    
    std::cout << "This demo tests the rotation visualization enhancements:" << std::endl;
    std::cout << "  ✓ Robot shown as triangle + circle + heading arrow" << std::endl;
    std::cout << "  ✓ Heading arrow extends beyond robot body" << std::endl;
    std::cout << "  ✓ Arrowhead clearly shows direction" << std::endl;
    std::cout << "  ✓ Orientation displayed in radians and degrees" << std::endl;
    std::cout << "  ✓ Target/subtarget orientations also visualized\n" << std::endl;
    
    // Initial parameters - robot at origin
    Eigen::Vector3d p_initial(0, -2, 0);           // Start at bottom center
    Eigen::Vector3d v_initial(0, 0, 0);            // Initially at rest
    int npredict = 20;
    int nobstacles = 0;                            // No obstacles for this test
    int nintercept_positions = 15;
    Eigen::Vector2d p_initial_ball(0, 0);
    Eigen::Vector2d v_initial_ball(0, 0);

    // Initialize state
    auto initial_state = spg::Init(p_initial, v_initial, nobstacles, npredict, 
                                    p_initial_ball, v_initial_ball, nintercept_positions);

    // Set to MSL mode
    initial_state.par.use_humanoid_mpc = false;

    // Set target with specific orientation (90 degrees = π/2 radians)
    // This will make the robot go to (0, 2) and rotate to face 90 degrees
    Eigen::Vector3d target(0, 2, M_PI/2);  // Go up and rotate to face right
    initial_state.input.robot.target = target;
    initial_state.input.robot.target_vel = Eigen::Vector3d(0, 0, 0);

    std::cout << "Test scenario:" << std::endl;
    std::cout << "  Start: Position (0, -2), Orientation: 0° (facing up)" << std::endl;
    std::cout << "  Goal:  Position (0,  2), Orientation: 90° (facing right)" << std::endl;
    std::cout << "\nWatch the robot:" << std::endl;
    std::cout << "  • Blue triangle shows robot body" << std::endl;
    std::cout << "  • Circle shows robot footprint (radius = 0.25m)" << std::endl;
    std::cout << "  • Bold arrow extends from center showing heading" << std::endl;
    std::cout << "  • Red cross + arrow shows target with desired orientation" << std::endl;
    std::cout << "  • Yellow diamond + arrow shows current subtarget\n" << std::endl;

    std::cout << "Press ESC in the window to exit.\n" << std::endl;

    // Create and run simulator
    SPGSimulator simulator(initial_state);
    simulator.run();

    std::cout << "\nTest complete!" << std::endl;
    std::cout << "\n╔══════════════════════════════════════════════════════════════╗" << std::endl;
    std::cout << "║                Rotation Visualization Features              ║" << std::endl;
    std::cout << "╚══════════════════════════════════════════════════════════════╝" << std::endl;
    
    std::cout << "\nWhat you saw:" << std::endl;
    std::cout << "1. Robot triangle - Points in heading direction" << std::endl;
    std::cout << "2. Robot circle - Shows physical footprint" << std::endl;
    std::cout << "3. Heading arrow - Extends 0.4m from center with arrowhead" << std::endl;
    std::cout << "4. Orientation display - Shows θ in radians and degrees" << std::endl;
    std::cout << "5. Target orientation - Red arrow shows desired final heading" << std::endl;
    std::cout << "6. Subtarget orientation - Yellow arrow shows intermediate heading" << std::endl;
    
    std::cout << "\nVisualization makes it easy to see:" << std::endl;
    std::cout << "  • Current robot heading (real-time)" << std::endl;
    std::cout << "  • Desired heading at target" << std::endl;
    std::cout << "  • How robot rotates while moving" << std::endl;
    std::cout << "  • Difference between MSL (omnidirectional) and Humanoid (body-frame)" << std::endl;
    
    std::cout << "\n" << std::endl;
    
    return 0;
}
