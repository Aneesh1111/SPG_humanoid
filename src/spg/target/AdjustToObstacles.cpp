#include "spg/target/AdjustToObstacles.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

namespace spg {
namespace target {

void adjustToObstacles(SPGState& state) {
    double extra_margin = 0.05;
    int nAttempts = 10;
    // Assume obstacles_p, r_obstacles, collision_distance are available in state
    // This is a stub, you should fill in obstacle data access as per your State struct
    Eigen::Vector2d adjusted_p = state.target.p.head<2>();
    for (size_t i = 0; i < state.par.nobstacles; ++i) {
        if (!state.input.obstacles.active[i]) continue;
        double min_dist = state.par.robot_radius + state.input.obstacles.r[i] + extra_margin; // margin
        Eigen::Vector2d diff = adjusted_p - state.input.obstacles.p[i];
        double dist = diff.norm();
        if (dist < min_dist && dist > 1e-6) {
            adjusted_p = state.input.obstacles.p[i] + diff.normalized() * min_dist;
        }
    }
    state.target.p.head<2>() = adjusted_p;
}

}}
