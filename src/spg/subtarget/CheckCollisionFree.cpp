#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include "spg/subtarget/CheckCollisionFree.hpp"
#include "spg/setpoint/GetSegments.hpp"
#include "spg/setpoint/TrajectoryUtils.hpp"
#include "spg/subtarget/CheckViolation.hpp"
#include "spg/setpoint/ConvertSegment.hpp"
#include "spg/setpoint/TrajPredict.hpp"

namespace spg {
namespace subtarget {

Subtarget checkCollisionFree(const SPGState& d, Subtarget subtarget, double obstacle_margin, std::vector<Eigen::Vector2d>* p_robot_out, std::vector<std::vector<Eigen::Vector2d>>* p_obstacles_traj_out) {
    // Update subtarget.segment, subtarget.collisionfree, subtarget.eta
    // Convert segment type if needed
    // Use conversion functions from spg::setpoint namespace
    auto setpointSegments = spg::setpoint::convertSegmentVector(subtarget.segment);
    setpointSegments = setpoint::getSegments(setpointSegments, d.setpoint.p, d.setpoint.v, subtarget.p, subtarget.v, subtarget.vmax, subtarget.amax, {d.par.dmax_move, d.par.dmax_move, d.par.dmax_rotate});
    subtarget.segment = spg::setpoint::convertBackSegmentVector(setpointSegments);
    subtarget.eta = std::max(subtarget.segment[2].t[0], subtarget.segment[2].t[1]);
    SPGState d2 = d;
    auto setpointSegmentsForTraj = spg::setpoint::convertSegmentVector(subtarget.segment);
    spg::setpoint::TrajPredict(d2, setpointSegmentsForTraj);
    if (d2.traj.segment_id.size() >= 3) {
        subtarget.segment_id = Eigen::Vector3i(
            d2.traj.segment_id[0][0],
            d2.traj.segment_id[1][1],
            d2.traj.segment_id[2][2]
        );
    }
    if (p_robot_out) {
        p_robot_out->clear();
        for (const auto& p : d2.traj.p) p_robot_out->push_back(p.head<2>());
    }
    if (p_obstacles_traj_out) p_obstacles_traj_out->clear();
    // Prepare for trajectory check
    std::vector<Eigen::Vector2d> p_obstacles;
    std::vector<Eigen::Vector2d> v_obstacles;
    std::vector<double> r_obstacles;
    for (size_t i = 0; i < d.input.obstacles.active.size(); ++i) {
        if (d.input.obstacles.active[i]) {
            p_obstacles.push_back(d.input.obstacles.p[i]);
            v_obstacles.push_back(d.input.obstacles.v[i]);
            r_obstacles.push_back(d.input.obstacles.r[i]);
        }
    }
    std::vector<double> collision_distance(r_obstacles.size());
    for (size_t i = 0; i < r_obstacles.size(); ++i) collision_distance[i] = d.par.robot_radius + r_obstacles[i];
    Violation violation;
    violation.collisionfree = true;
    violation.count = 0;
    violation.values["SubtargetAvoidPolygon"] = 0;
    violation.values["obstacle"] = 1e10;
    violation.values["field"] = 1e10;
    size_t npropagate = d2.traj.p.size();
    if (p_obstacles_traj_out) p_obstacles_traj_out->resize(npropagate);
    // Propagate to assess trajectory feasibility
    for (size_t i = 0; i < npropagate; ++i) {
        // Obstacle path (constant/declining velocity model)
        for (auto& v : v_obstacles) v *= d.par.obstacle_vel_gain;
        for (size_t j = 0; j < p_obstacles.size(); ++j) p_obstacles[j] += v_obstacles[j] * d.par.Ts_predict;
        if (p_obstacles_traj_out) (*p_obstacles_traj_out)[i] = p_obstacles;
        // Check collision
        if (p_obstacles.empty()) {
            updateViolation(violation, "obstacle", 0);
        } else {
            std::vector<double> p_diff_norm(p_obstacles.size());
            for (size_t j = 0; j < p_obstacles.size(); ++j) {
                p_diff_norm[j] = (p_obstacles[j] - d2.traj.p[i].head<2>()).norm();
            }
            // Intercept logic
            double p_diff_obst2Target = 1e10;
            for (const auto& obs : p_obstacles) {
                double dist = (obs - d.target.p.head<2>()).norm();
                if (dist < p_diff_obst2Target) p_diff_obst2Target = dist;
            }
            Eigen::Vector2d p_diff_robot2Target = d2.traj.p[i].head<2>() - d.target.p.head<2>();
            double target2ball_distance = (d.target.p.head<2>() - d.input.ball.p.head<2>()).norm();
            double margin = d.input.robot.dist2ball_vs_opp;
            std::vector<Eigen::Vector2d> ball2obstacle, robot2ball;
            for (const auto& obs : p_obstacles) ball2obstacle.push_back(obs - d.input.ball.p.head<2>());
            Eigen::Vector2d robot2ball_v = d2.traj.p[i].head<2>() - d.input.ball.p.head<2>();
            std::vector<double> angle(ball2obstacle.size());
            for (size_t j = 0; j < ball2obstacle.size(); ++j) {
                Eigen::Vector3d a(ball2obstacle[j](0), ball2obstacle[j](1), 0);
                Eigen::Vector3d b(robot2ball_v(0), robot2ball_v(1), 0);
                double norm_arr = a.cross(b).norm();
                double dot_arr = a.head<2>().dot(b.head<2>());
                angle[j] = std::abs(std::atan2(norm_arr, dot_arr)) * 180.0 / M_PI;
            }
            bool no_obstacle_between_me_and_ball = std::all_of(angle.begin(), angle.end(), [](double a) { return a >= 45.0; });
            double violation_value = 0;
            if (((d.input.robot.skillID == 5) || (d.input.robot.skillID == 0)) && (p_diff_obst2Target + margin > p_diff_robot2Target.norm()) && (target2ball_distance < 0.5) && no_obstacle_between_me_and_ball) {
                violation_value = 0;
            } else if ((d.input.robot.skillID == 5) && (p_diff_obst2Target > p_diff_robot2Target.norm())) {
                violation_value = 0;
            } else {
                std::vector<double> obs_violation(p_diff_norm.size());
                for (size_t j = 0; j < p_diff_norm.size(); ++j) {
                    obs_violation[j] = std::max(0.0, (collision_distance[j] + obstacle_margin * d2.traj.v[i].head<2>().norm()) - p_diff_norm[j]);
                }
                violation_value = *std::max_element(obs_violation.begin(), obs_violation.end());
            }
            updateViolation(violation, "obstacle", violation_value);
        }
        // Check illegal driving zone
        if (d.input.subtarget_avoid_polygon.valid) {
            // inpolygon logic needed here (implement as utility)
            // For now, set to 0
            updateViolation(violation, "SubtargetAvoidPolygon", 0);
        } else {
            updateViolation(violation, "SubtargetAvoidPolygon", 0);
        }
        // Check field if possess ball
        if ((subtarget.p.head<2>() - d.target.p.head<2>()).norm() > 1e-3) {
            if (d.input.robot.skillID >= 1 && d.input.robot.skillID <= 4) {
                double robot_ball_distance = d.par.robot_radius + d.par.ball_radius;
                Eigen::Vector2d ball_pos = d2.traj.p[i].head<2>() + robot_ball_distance * Eigen::Vector2d(-std::sin(d2.traj.p[i](2)), std::cos(d2.traj.p[i](2)));
                double violation_value = std::max(0.0, std::abs(ball_pos.norm()) - d.par.field_size[0] * 0.5);
                updateViolation(violation, "field", violation_value);
            } else {
                updateViolation(violation, "field", 0);
            }
            double field_x_half = d.par.field_size[0] * 0.5 + d.par.field_border_margin;
            double field_y_half = d.par.field_size[1] * 0.5 + d.par.field_border_margin;
            if (-field_x_half < subtarget.p(0) && subtarget.p(0) < field_x_half && -field_y_half < subtarget.p(1) && subtarget.p(1) < field_y_half) {
                double max_radius = 3.0;
                double distance_between_cpb_poi_xy_and_subtarget = d.input.robot.human_dribble_flag == 1 ? 0.0 : (subtarget.p.head<2>() - d.input.robot.cpb_poi_xy).norm();
                double xpos = 0.5 * d.par.field_goal_area[0];
                double ypos = d.par.field_size[1] * 0.5 - d.par.field_goal_area[1];
                bool subtarget_is_in_goal_area = std::abs(subtarget.p(0)) < xpos && std::abs(subtarget.p(1)) > ypos;
                double violation_value = 0;
                if ((d.input.robot.skillID >= 1 && d.input.robot.skillID <= 4 && distance_between_cpb_poi_xy_and_subtarget > max_radius) || subtarget_is_in_goal_area) {
                    violation_value = 1e11;
                }
                updateViolation(violation, "field", violation_value);
            } else {
                double field_x_tech = field_x_half + d.par.technical_area_width;
                if (subtarget.automatic_substitution_flag == 1 && -field_x_tech < subtarget.p(0) && subtarget.p(0) < field_x_tech && -field_y_half < subtarget.p(1) && subtarget.p(1) < field_y_half) {
                    updateViolation(violation, "field", 0);
                } else {
                    updateViolation(violation, "field", 1e11);
                }
            }
        }
        // Check if close to arrival
        if ((d2.traj.p[i].head<2>() - subtarget.p.head<2>()).norm() < 1e-2) {
            break;
        }
    }
    // Clip subtarget x-velocity such that robot stays in field (can brake in time)
    double dist2sideline = d.par.field_size[0] * 0.5 + d.par.field_border_margin - std::abs(subtarget.p(0));
    if (std::abs(2 * d.par.dmax_move * dist2sideline) < std::pow(subtarget.v(0), 2)) {
        subtarget.v(0) = 2 * d.par.dmax_move * dist2sideline;
    }
    // Clip subtarget y-velocity such that robot stays in field (can brake in time)
    double dist2goalline = d.par.field_size[1] * 0.5 + d.par.field_border_margin - std::abs(subtarget.p(1));
    if (std::abs(2 * d.par.dmax_move * dist2goalline) < std::pow(subtarget.v(1), 2)) {
        subtarget.v(1) = 2 * d.par.dmax_move * dist2goalline;
    }
    subtarget.collisionfree = violation.collisionfree;
    subtarget.violation_count = violation.count;
    return subtarget;
}

} // namespace subtarget
} // namespace spg
