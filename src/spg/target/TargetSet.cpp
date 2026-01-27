#include "spg/target/TargetSet.hpp"
#include <iostream>

namespace spg {
namespace target {

void set(SPGState& state) {
    // Initialize target output
    state.target.p = state.input.robot.target;
    state.target.v = state.input.robot.target_vel;
    state.target.eta = 0;
    
    // Debug: Print target heading once
    static bool printed = false;
    if (!printed) {
        std::cout << "Target::Set - input.robot.target = (" << state.input.robot.target[0] 
                  << ", " << state.input.robot.target[1] << ", " << state.input.robot.target[2] << ")" << std::endl;
        std::cout << "Target::Set - target.p after init = (" << state.target.p[0] 
                  << ", " << state.target.p[1] << ", " << state.target.p[2] << ")" << std::endl;
        printed = true;
    }
    // Adjust target
    adjustToField(state);
    adjustToPenaltyArea(state);
    adjustToGoalArea(state);
    adjustToObstacles(state);
    adjustTo3mRule(state);
}

}}
