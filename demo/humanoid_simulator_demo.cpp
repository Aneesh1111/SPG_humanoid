#include "SPGSimulator.hpp"
#include "spg/Init.hpp"
#include <iostream>

int main() {
    std::cout << "🤖 Starting Humanoid Simulator Demo" << std::endl;
    std::cout << "====================================" << std::endl;
    
    // Initialize SPG state with humanoid parameters
    Eigen::Vector3d p_initial(0.0, 5.0, 0.0);  // Start position
    Eigen::Vector3d v_initial = Eigen::Vector3d::Zero();  // Start velocity
    Eigen::Vector2d p_initial_ball(0.0, 0.0);  // Ball position (2D)
    Eigen::Vector2d v_initial_ball = Eigen::Vector2d::Zero();  // Ball velocity (2D)
    
    spg::SPGState state = spg::Init(
        p_initial, v_initial, 
        0,  // nobstacles
        50, // npredict
        p_initial_ball, v_initial_ball,
        10  // nintercept_positions
    );
    
    // Set humanoid-specific parameters
    // Robot moves faster forward (Y-direction) than sideways (X-direction)
    state.par.vmax_move_x = 1.5;  // Max sideways velocity (X-direction)
    state.par.vmax_move_y = 4.0;  // Max forward velocity (Y-direction) 
    state.par.amax_move_x = 0.5;  // Max sideways acceleration
    state.par.amax_move_y = 2.0;  // Max forward acceleration
    
    // Efficiency factors
    state.par.forward_efficiency = 1.0;   // 100% efficient when moving forward
    state.par.sideways_efficiency = 0.20;  // 20% efficient when moving sideways
    state.par.rotation_while_moving_penalty = 0.0; // 0% rotation penalty when moving
    
    // Set up demo scenario: robot starts at (0, -3) and needs to reach (0, 3)
    // This is primarily forward movement (Y-direction) which should be efficient
    // state.input.robot.p = Eigen::Vector3d(0.0, -3.0, 0.0);
    // state.input.robot.v = Eigen::Vector3d::Zero();
    // state.input.robot.IMU_orientation = Eigen::Vector3d(0.0, 0.0, 0.0); // Facing forward
    
    // Set target - IMPORTANT: Set this AFTER initialization to override any defaults
    // The Init function sets target.p to p_initial, so we must override it here
    state.input.robot.target = Eigen::Vector3d(0.0, 0.0, 0.0);  // Also update robot.target
    
    std::cout << "Humanoid Parameters:" << std::endl;
    std::cout << "  vmax_move_x (sideways): " << state.par.vmax_move_x << " m/s" << std::endl;
    std::cout << "  vmax_move_y (forward):  " << state.par.vmax_move_y << " m/s" << std::endl;
    std::cout << "  amax_move_x (sideways): " << state.par.amax_move_x << " m/s²" << std::endl;
    std::cout << "  amax_move_y (forward):  " << state.par.amax_move_y << " m/s²" << std::endl;
    std::cout << "  forward_efficiency:     " << state.par.forward_efficiency * 100 << "%" << std::endl;
    std::cout << "  sideways_efficiency:    " << state.par.sideways_efficiency * 100 << "%" << std::endl;
    std::cout << "" << std::endl;
    
    std::cout << "Demo Scenario:" << std::endl;
    std::cout << "  Robot starts at: (" << state.input.robot.p.transpose() << ")" << std::endl;
    std::cout << "  Robot faces: " << state.input.robot.IMU_orientation(2) * 180.0/M_PI << "° (forward)" << std::endl;
    std::cout << "  Target at: (" << state.target.p.transpose() << ")" << std::endl;
    std::cout << "  Movement type: Forward (Y-direction) - should be efficient!" << std::endl;
    std::cout << "" << std::endl;
    
    // // Add some obstacles for variety
    // state.input.obstacles.p.resize(3);
    // state.input.obstacles.v.resize(3);
    // state.input.obstacles.r.resize(3);
    // state.input.obstacles.active.resize(3);
    
    // // Obstacle 1: Moving left-right
    // state.input.obstacles.p[0] = Eigen::Vector2d(-1.5, 0.0);
    // state.input.obstacles.v[0] = Eigen::Vector2d(1.0, 0.0);
    // state.input.obstacles.r[0] = 0.3;
    // state.input.obstacles.active[0] = true;
    
    // // Obstacle 2: Moving up-down  
    // state.input.obstacles.p[1] = Eigen::Vector2d(1.5, -1.0);
    // state.input.obstacles.v[1] = Eigen::Vector2d(0.0, 0.8);
    // state.input.obstacles.r[1] = 0.25;
    // state.input.obstacles.active[1] = true;
    
    // // Obstacle 3: Stationary
    // state.input.obstacles.p[2] = Eigen::Vector2d(0.5, 1.5);
    // state.input.obstacles.v[2] = Eigen::Vector2d::Zero();
    // state.input.obstacles.r[2] = 0.4;
    // state.input.obstacles.active[2] = true;
    
    // std::cout << "Added 3 obstacles:" << std::endl;
    // std::cout << "  Obstacle 1: Moving horizontally at (" << state.input.obstacles.p[0].transpose() << ")" << std::endl;
    // std::cout << "  Obstacle 2: Moving vertically at (" << state.input.obstacles.p[1].transpose() << ")" << std::endl;
    // std::cout << "  Obstacle 3: Stationary at (" << state.input.obstacles.p[2].transpose() << ")" << std::endl;
    // std::cout << "" << std::endl;
    
    // Initialize and run the humanoid simulator
    SPGSimulator simulator(state);
    
    std::cout << "🚀 Starting humanoid simulation..." << std::endl;
    std::cout << "Controls:" << std::endl;
    std::cout << "  - Close window to exit" << std::endl;
    std::cout << "  - Press 'R' to reset simulation" << std::endl;
    std::cout << "  - Press 'Space' to toggle step mode" << std::endl;
    std::cout << "  - Press 'S' to step when in step mode" << std::endl;
    std::cout << "  - Use speed slider to control simulation speed" << std::endl;
    std::cout << "" << std::endl;
    std::cout << "Watch how the humanoid robot efficiently moves forward!" << std::endl;
    std::cout << "The robot should move faster and more smoothly in the Y-direction." << std::endl;
    
    // Run the humanoid version of the simulator
    simulator.runHumanoid();
    
    std::cout << "🎉 Humanoid simulation completed!" << std::endl;
    
    return 0;
}