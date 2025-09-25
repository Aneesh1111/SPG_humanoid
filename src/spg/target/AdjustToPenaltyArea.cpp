#include "spg/target/AdjustToPenaltyArea.hpp"
#include <algorithm>
#include <cmath>

namespace spg {
namespace target {

inline double sign(double x) { return (x > 0) - (x < 0); }

void adjustToPenaltyArea(SPGState& state) {
    double xpos = 0.5 * state.par.field_penalty_area[0];
    double ypos = state.par.field_size[1] * 0.5 - state.par.field_penalty_area[1];
    bool is_in_penalty_area = std::abs(state.target.p[0]) < xpos && std::abs(state.target.p[1]) > ypos;
    if (is_in_penalty_area && !state.input.robot.CPPA) {
        double dist_x = xpos - std::abs(state.target.p[0]);
        double dist_y = std::abs(state.target.p[1]) - ypos;
        if (dist_x < dist_y) {
            state.target.p[0] = xpos * sign(state.target.p[0]);
            if (std::abs(state.target.p[1]) > state.par.field_size[1] * 0.5)
                state.target.p[1] = state.par.field_size[1] * 0.5 * sign(state.target.p[1]);
        } else {
            state.target.p[1] = ypos * sign(state.target.p[1]);
        }
    }
}

}}
