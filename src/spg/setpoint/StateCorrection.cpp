#include <Eigen/Dense>
#include "spg/setpoint/StateCorrection.hpp"

namespace spg {
namespace setpoint {

void stateCorrection(SPGState& state) {
    if (state.input.robot.reset_trigger > 0.5) {
        state.setpoint.p = state.input.robot.p;
        state.setpoint.v = state.input.robot.v;
    }
}

}}
