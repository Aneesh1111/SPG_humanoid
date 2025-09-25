#include "spg/target/AdjustTo3mRule.hpp"
#include <cmath>
#include <algorithm>

namespace spg {
namespace target {

void adjustTo3mRule(SPGState& state) {
    double max_radius = 3.0;
    double dx = state.target.p[0] - state.input.robot.cpb_poi_xy[0];
    double dy = state.target.p[1] - state.input.robot.cpb_poi_xy[1];
    double distance = std::sqrt(dx*dx + dy*dy);
    double angle = std::atan2(dy, dx);
    // SkillID 1-4 and not human dribble
    if ((state.input.robot.skillID >= 1 && state.input.robot.skillID <= 4) && (distance > max_radius) && !state.input.robot.human_dribble_flag) {
        state.target.p[0] = max_radius * std::cos(angle);
        state.target.p[1] = max_radius * std::sin(angle);
    }
}

}}
