#include <iostream>
#include <Eigen/Dense>
#include "spg/Init.hpp"
#include "spg/NextSample.hpp"

int main() {
    // Test parameters - same as in the test
    Eigen::Vector3d p_initial(-3.0, 5.0, 0.0);
    Eigen::Vector3d v_initial(0.0, 0.0, 0.0);
    Eigen::Vector3d target(0.0, 0.0, 0.0);
    Eigen::Vector3d target_vel(0.0, 0.0, 0.0);
    
    std::cout << "=== Debug Straight Line Movement ===" << std::endl;
    std::cout << "Start: (" << p_initial.transpose() << ")" << std::endl;
    std::cout << "Target: (" << target.transpose() << ")" << std::endl;
    
    // Calculate ideal straight line parameters
    Eigen::Vector3d direction = target - p_initial;
    double distance = direction.norm();
    Eigen::Vector3d unit_direction = direction.normalized();
    double ideal_angle = atan2(direction.y(), direction.x()) * 180.0 / M_PI;
    
    std::cout << "Ideal direction: (" << direction.transpose() << ")" << std::endl;
    std::cout << "Distance: " << distance << " m" << std::endl;
    std::cout << "Unit direction: (" << unit_direction.transpose() << ")" << std::endl;
    std::cout << "Ideal angle: " << ideal_angle << " degrees" << std::endl;
    
    // Initialize SPG state
    int nobstacles = 0;
    int npredict = 20;
    Eigen::Vector2d ball_pos(0.0, 0.0);
    Eigen::Vector2d ball_vel(0.0, 0.0);
    int nintercept = 1;
    
    auto state = spg::Init(p_initial, v_initial, nobstacles, npredict, ball_pos, ball_vel, nintercept);
    
    // Set target
    state.input.robot.target = target;
    state.input.robot.target_vel = target_vel;
    state.target.p = target;
    state.target.v = target_vel;
    
    std::cout << "\n=== SPG Parameters ===" << std::endl;
    std::cout << "vmax_move: " << state.par.vmax_move << " m/s" << std::endl;
    std::cout << "amax_move: " << state.par.amax_move << " m/s²" << std::endl;
    std::cout << "dmax_move: " << state.par.dmax_move << " m/s²" << std::endl;
    
    // Simulate several steps and track the path
    std::cout << "\n=== Trajectory Analysis ===" << std::endl;
    std::cout << "Step | Position | Velocity | Direction Error" << std::endl;
    std::cout << "------+----------+----------+----------------" << std::endl;
    
    for (int step = 0; step < 50; ++step) {
        // Run one step of SPG
        state = spg::NextSample(state);
        
        Eigen::Vector3d current_pos = state.setpoint.p;
        Eigen::Vector3d current_vel = state.setpoint.v;
        
        // Calculate direction error
        Eigen::Vector3d current_to_target = target - current_pos;
        double remaining_distance = current_to_target.norm();
        
        if (remaining_distance < 0.01) {
            std::cout << "Reached target at step " << step << std::endl;
            break;
        }
        
        // Calculate direction error (angle between current velocity and ideal direction)
        double direction_error = 0.0;
        if (current_vel.head<2>().norm() > 0.01) {
            Eigen::Vector2d vel_2d = current_vel.head<2>().normalized();
            Eigen::Vector2d ideal_2d = current_to_target.head<2>().normalized();
            direction_error = acos(vel_2d.dot(ideal_2d)) * 180.0 / M_PI;
        }
        
        if (step % 5 == 0) {  // Print every 5 steps
            std::printf("%4d | (%6.2f,%6.2f) | (%6.2f,%6.2f) | %8.2f°\n", 
                       step, current_pos.x(), current_pos.y(), 
                       current_vel.x(), current_vel.y(), direction_error);
        }
        
        // Check if we've deviated significantly from the straight line
        Eigen::Vector3d displacement_from_start = current_pos - p_initial;
        double progress_along_ideal = displacement_from_start.dot(unit_direction);
        Eigen::Vector3d point_on_ideal_line = p_initial + progress_along_ideal * unit_direction;
        double deviation = (current_pos - point_on_ideal_line).norm();
        
        if (step == 10 || step == 20 || step == 30) {
            std::cout << "Step " << step << " deviation from ideal line: " << deviation << " m" << std::endl;
        }
        
        if (deviation > 0.5) {
            std::cout << "WARNING: Large deviation from straight line at step " << step 
                      << ": " << deviation << " m" << std::endl;
        }
    }
    
    return 0;
}
