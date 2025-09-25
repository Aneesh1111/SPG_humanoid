#include "spg/target/AdjustToField.hpp"
#include <algorithm>
#include <cmath>

namespace spg {
namespace target {

void adjustToField(SPGState& state) {
    if (state.input.robot.skillID >= 1 && state.input.robot.skillID <= 4) {
        double robot_ball_distance = state.par.robot_radius + state.par.ball_radius;
        double dball_x = -robot_ball_distance * std::sin(state.target.p[2]);
        double dball_y = robot_ball_distance * std::cos(state.target.p[2]);
        double ball_pos_x = state.target.p[0] + dball_x;
        double ball_pos_y = state.target.p[1] + dball_y;
        ball_pos_x = std::max(ball_pos_x, -state.par.field_size[0] * 0.5);
        ball_pos_x = std::min(ball_pos_x, state.par.field_size[0] * 0.5);
        ball_pos_y = std::max(ball_pos_y, -state.par.field_size[1] * 0.5);
        ball_pos_y = std::min(ball_pos_y, state.par.field_size[1] * 0.5);
        state.target.p[0] = ball_pos_x - dball_x;
        state.target.p[1] = ball_pos_y - dball_y;
    } else {
        double margin = state.par.field_border_margin;
        if (state.subtarget.automatic_substitution_flag) {
            margin += state.par.technical_area_width;
        }
        state.target.p[0] = std::max(state.target.p[0], -state.par.field_size[0] * 0.5 - margin);
        state.target.p[0] = std::min(state.target.p[0], state.par.field_size[0] * 0.5 + margin);
        state.target.p[1] = std::max(state.target.p[1], -state.par.field_size[1] * 0.5 - margin);
        state.target.p[1] = std::min(state.target.p[1], state.par.field_size[1] * 0.5 + margin);
    }
}

}}
