#include "SPG.hpp"
#include <iostream>
#include <iomanip>

int main() {
    std::cout << "SPG Framework - Skeleton Demo\n";
    std::cout << "================================\n\n";
    
    // Initialize state
    spg::SPGState state;
    
    // Configure parameters
    state.par.use_humanoid_mpc = false;  // Use MSL mode (not implemented yet)
    state.par.field_length = 9.0;
    state.par.field_width = 6.0;
    
    // Set robot current state
    state.robot.p[0] = 0.0;  // x
    state.robot.p[1] = 0.0;  // y
    state.robot.p[2] = 0.0;  // theta
    
    state.robot.v[0] = 0.0;
    state.robot.v[1] = 0.0;
    state.robot.v[2] = 0.0;
    
    // Set goal
    state.goal.p[0] = 3.0;
    state.goal.p[1] = 2.0;
    state.goal.p[2] = 0.5;
    
    std::cout << "Initial Setup:\n";
    std::cout << "  Robot at: (" << state.robot.p[0] << ", " 
              << state.robot.p[1] << ", " << state.robot.p[2] << ")\n";
    std::cout << "  Goal at:  (" << state.goal.p[0] << ", " 
              << state.goal.p[1] << ", " << state.goal.p[2] << ")\n\n";
    
    // Run SPG pipeline
    std::cout << "Running SPG Pipeline...\n\n";
    
    spg::target::Target(state);
    std::cout << "✓ Target layer complete (" 
              << std::fixed << std::setprecision(3) 
              << state.time_target_ms << " ms)\n";
    std::cout << "  Target: (" << state.target.p[0] << ", " 
              << state.target.p[1] << ", " << state.target.p[2] << ")\n";
    
    spg::subtarget::Subtarget(state);
    std::cout << "✓ Subtarget layer complete (" 
              << state.time_subtarget_ms << " ms)\n";
    std::cout << "  Subtarget: (" << state.subtarget.p[0] << ", " 
              << state.subtarget.p[1] << ", " << state.subtarget.p[2] << ")\n";
    std::cout << "  Collision-free: " << (state.collision_free ? "Yes" : "No") << "\n";
    
    spg::setpoint::Setpoint(state);
    std::cout << "✓ Setpoint layer complete (" 
              << state.time_setpoint_ms << " ms)\n";
    std::cout << "  Setpoint: (" << state.setpoint.p[0] << ", " 
              << state.setpoint.p[1] << ", " << state.setpoint.p[2] << ")\n";
    std::cout << "  Velocity: (" << state.setpoint.v[0] << ", " 
              << state.setpoint.v[1] << ", " << state.setpoint.v[2] << ")\n";
    
    double total_time = state.time_target_ms + state.time_subtarget_ms + state.time_setpoint_ms;
    std::cout << "\nTotal pipeline time: " << total_time << " ms\n";
    
    std::cout << "\n================================\n";
    std::cout << "NOTE: This is skeleton code only.\n";
    std::cout << "Full implementation coming in MR2-MR6.\n";
    std::cout << "See README.md for details.\n";
    
    return 0;
}
