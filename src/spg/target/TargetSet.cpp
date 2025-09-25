#include "spg/target/TargetSet.hpp"

namespace spg {
namespace target {

void set(SPGState& state) {
    // Initialize target output
    state.target.p = state.input.robot.target;
    state.target.v = state.input.robot.target_vel;
    state.target.eta = 0;
    // Adjust target
    adjustToField(state);
    adjustToPenaltyArea(state);
    adjustToGoalArea(state);
    adjustToObstacles(state);
    adjustTo3mRule(state);
}

}}
