#include "spg/subtarget/replan/search/Random.hpp"
#include "spg/subtarget/replan/ReplanUtils.hpp"
#include "spg/subtarget/CheckCollisionFree.hpp"
#include <Eigen/Dense>
#include <random>
#include <cmath>

namespace spg {
namespace subtarget {
namespace replan {
namespace search {

Subtarget random(SPGState& d, const Subtarget& best, const Eigen::Vector3d& search_point, double search_distance, int defending_opp_with_or_without_ball) {
    Subtarget best_so_far = best;
    Subtarget subtarget_candidate = best;
    double vmax_random = d.par.vmax_move * 0.5;
    std::mt19937 gen(std::random_device{}());
    for (int i = 0; i < d.par.nattempts_replan; ++i) {
        Eigen::Vector2d lowerbound = (search_point.head<2>() - Eigen::Vector2d::Constant(search_distance)).cwiseMax(-d.par.field_size[0]*0.5);
        Eigen::Vector2d upperbound = (search_point.head<2>() + Eigen::Vector2d::Constant(search_distance)).cwiseMin(d.par.field_size[0]*0.5);
        int nattempts = 10;
        std::vector<double> obstacle_margin;
        for (double r : d.input.obstacles.r) obstacle_margin.push_back(d.par.robot_radius + r + 0.05);
        for (int j = 0; j < nattempts; ++j) {
            Eigen::Vector3d p_candidate;
            if (defending_opp_with_or_without_ball == 1) {
                Eigen::Vector2d ub = upperbound.cwiseMin(search_point.head<2>() + Eigen::Vector2d::Constant(search_distance * 0.5));
                p_candidate.head<2>() = (ub - lowerbound).cwiseProduct(Eigen::Vector2d::Random().cwiseAbs()) + lowerbound;
                p_candidate(2) = search_point(2);
                // TODO: inpolygon logic for region
            } else {
                p_candidate.head<2>() = (upperbound - lowerbound).cwiseProduct(Eigen::Vector2d::Random().cwiseAbs()) + lowerbound;
                p_candidate(2) = search_point(2);
            }
            double subtarget_target_distance = (p_candidate.head<2>() - d.target.p.head<2>()).norm();
            double subtarget_robot_distance = (p_candidate.head<2>() - d.setpoint.p.head<2>()).norm();
            double target_robot_distance = (d.target.p.head<2>() - d.setpoint.p.head<2>()).norm();
            double vnorm = d.setpoint.v.head<2>().norm();
            if (subtarget_target_distance < target_robot_distance + d.par.replan_uphill_distance) {
                Eigen::MatrixXd obstacles_mat(d.input.obstacles.p.size(), 2);
                for (size_t k = 0; k < d.input.obstacles.p.size(); ++k) obstacles_mat.row(k) = d.input.obstacles.p[k];
                if (spg::subtarget::replan::notTouchingObstacle(p_candidate.head<2>(), obstacles_mat, obstacle_margin[j])) {
                    subtarget_candidate.p = p_candidate;
                    // Velocity logic
                    double max_accel_x = std::sqrt(std::abs(d.input.robot.v(0)) + 2 * (d.par.amax_move/std::sqrt(2)) * std::abs(subtarget_candidate.p(0) - d.input.robot.p(0)));
                    double max_accel_y = std::sqrt(std::abs(d.input.robot.v(1)) + 2 * (d.par.amax_move/std::sqrt(2)) * std::abs(subtarget_candidate.p(1) - d.input.robot.p(1)));
                    double max_decel_x = std::sqrt(std::abs(d.target.v(0)) + 2 * (d.par.dmax_move/std::sqrt(2)) * std::abs(d.target.p(0) - subtarget_candidate.p(0)));
                    double max_decel_y = std::sqrt(std::abs(d.target.v(1)) + 2 * (d.par.dmax_move/std::sqrt(2)) * std::abs(d.target.p(1) - subtarget_candidate.p(1)));
                    double vx_max = std::min(d.par.vmax_move*0.707, std::min(max_accel_x, max_decel_x));
                    double vy_max = std::min(d.par.vmax_move*0.707, std::min(max_accel_y, max_decel_y));
                    double v_subtarget_x = std::copysign(std::min(std::abs(d.target.v(0) - d.input.robot.v(0)), vx_max), d.target.v(0) - d.input.robot.v(0));
                    double v_subtarget_y = std::copysign(std::min(std::abs(d.target.v(1) - d.input.robot.v(1)), vy_max), d.target.v(1) - d.input.robot.v(1));
                    subtarget_candidate.v = Eigen::Vector3d(v_subtarget_x, v_subtarget_y, 0);
                    subtarget_candidate = spg::subtarget::replan::determineSetpointLimits(d, subtarget_candidate);
                    subtarget_candidate = spg::subtarget::checkCollisionFree(d, subtarget_candidate, d.par.margin_replan);
                    // Optionally update best
                    best_so_far = spg::subtarget::replan::updateBest(best_so_far, subtarget_candidate, d.target.p);
                }
            }
        }
    }
    return best_so_far;
}

} // namespace search
} // namespace replan
} // namespace subtarget
} // namespace spg
