#include "spg/subtarget/replan/search/BesideObstacle.hpp"
#include "spg/subtarget/replan/ReplanUtils.hpp"
#include "spg/subtarget/CheckCollisionFree.hpp"
#include <cmath>

namespace spg {
namespace subtarget {
namespace replan {
namespace search {

Subtarget besideObstacle(const SPGState& d, const Subtarget& best) {
    Subtarget subtarget_candidate = best;
    double additional_obstacle_margin = 0.2; // [m]
    for (size_t i = 0; i < d.input.obstacles.active.size(); ++i) {
        if (d.input.obstacles.active[i]) {
            Eigen::Vector2d pos = d.input.obstacles.p[i];
            double obstacle_radius = d.input.obstacles.r[i];
            Eigen::Vector2d v1 = d.target.p.head<2>() - d.setpoint.p.head<2>();
            Eigen::Vector2d v2 = pos - d.setpoint.p.head<2>();
            Eigen::Vector2d v3 = v1 * v2.dot(v1) / v1.squaredNorm();
            Eigen::Vector2d v4 = v2 - v3;
            for (double speed : std::vector<double>{0.0, d.par.vmax_move*0.2, d.par.vmax_move*0.4, d.par.vmax_move*0.6}) {
                Eigen::Vector2d displacement = v4.normalized() * (d.par.robot_radius + obstacle_radius + 0.1 + d.par.margin_replan * std::abs(speed));
                double obstacle_margin = d.par.robot_radius + obstacle_radius + additional_obstacle_margin;
                for (int side : {-1, 1}) {
                    Eigen::Vector2d new_pos = pos + side * displacement;
                    double angle = std::atan2(-v1(0), v1(1));
                    subtarget_candidate.p = Eigen::Vector3d(new_pos(0), new_pos(1), angle);
                    // Convert obstacle positions to Eigen::MatrixXd
                    Eigen::MatrixXd obstacles_mat(d.input.obstacles.p.size(), 2);
                    for (size_t k = 0; k < d.input.obstacles.p.size(); ++k) obstacles_mat.row(k) = d.input.obstacles.p[k];
                    if (spg::subtarget::replan::notTouchingObstacle(subtarget_candidate.p.head<2>(), obstacles_mat, obstacle_margin)) {
                        Eigen::Vector2d v_xy = v1.normalized() * speed;
                        subtarget_candidate.v = Eigen::Vector3d(v_xy(0), v_xy(1), 0);
                        subtarget_candidate = spg::subtarget::replan::determineSetpointLimits(d, subtarget_candidate);
                        subtarget_candidate = spg::subtarget::checkCollisionFree(d, subtarget_candidate, d.par.margin_replan);
                        // Update best
                        subtarget_candidate = spg::subtarget::replan::updateBest(best, subtarget_candidate, d.target.p);
                    }
                }
            }
        }
    }
    return subtarget_candidate;
}

} // namespace search
} // namespace replan
} // namespace subtarget
} // namespace spg
