#include "spg/subtarget/replan/ReplanUtils.hpp"
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include "spg/setpoint/BalanceXY.hpp"
#include "spg/setpoint/Wrap.hpp"
#include "spg/subtarget/CheckCollisionFree.hpp"
#include "spg/setpoint/ConvertSegment.hpp"
#include "spg/subtarget/replan/search/BesideObstacle.hpp"
#include "spg/subtarget/replan/search/Random.hpp"
#include <iostream>

namespace spg {
namespace subtarget {
namespace replan {

Subtarget determineSetpointLimits(const SPGState& d, const Subtarget& subtarget) {
    Subtarget subtarget_copy = subtarget;
    double angle_diff = std::fmod(d.setpoint.p(2) - subtarget_copy.p(2) + M_PI, 2 * M_PI) - M_PI;
    double sc = d.par.scale_rotate;
    bool possess_ball = (d.input.robot.skillID >= 1 && d.input.robot.skillID <= 4);
    bool large_angle = std::abs(angle_diff) > d.par.scale_angle;
    bool low_velocity = d.setpoint.v.head<2>().norm() < 1.0;
    Eigen::Vector3d vmax, amax, dmax;
    if (possess_ball && large_angle && low_velocity) {
        vmax = Eigen::Vector3d(d.par.vmax_move * sc / std::sqrt(2), d.par.vmax_move * sc / std::sqrt(2), d.par.vmax_rotate);
        amax = Eigen::Vector3d(d.par.amax_move * sc / std::sqrt(2), d.par.amax_move * sc / std::sqrt(2), d.par.amax_rotate);
        dmax = Eigen::Vector3d(d.par.dmax_move * sc / std::sqrt(2), d.par.dmax_move * sc / std::sqrt(2), d.par.dmax_rotate);
    } else {
        vmax = Eigen::Vector3d(d.par.vmax_move / std::sqrt(2), d.par.vmax_move / std::sqrt(2), d.par.vmax_rotate);
        amax = Eigen::Vector3d(d.par.amax_move / std::sqrt(2), d.par.amax_move / std::sqrt(2), d.par.amax_rotate);
        dmax = Eigen::Vector3d(d.par.dmax_move / std::sqrt(2), d.par.dmax_move / std::sqrt(2), d.par.dmax_rotate);
    }
    if (subtarget_copy.v.head<2>().norm() > vmax.head<2>().norm() && d.input.robot.skillID != 5) {
        subtarget_copy.v.head<2>() = subtarget_copy.v.head<2>().normalized() * vmax.head<2>().norm();
    }
    // Convert Eigen::Vector3d to std::array<double, 3>
    auto to_array = [](const Eigen::Vector3d& v) {
        return std::array<double, 3>{v[0], v[1], v[2]};
    };
    std::vector<Segment> balanced_segments = subtarget_copy.segment;
    std::array<double, 3> p0 = to_array(d.setpoint.p);
    std::array<double, 3> v0 = to_array(d.setpoint.v);
    std::array<double, 3> pe = to_array(subtarget_copy.p);
    std::array<double, 3> ve = to_array(subtarget_copy.v);
    std::array<double, 3> vm = to_array(vmax);
    std::array<double, 3> am = to_array(amax);
    std::array<double, 3> dm = to_array(dmax);
    std::array<double, 3> vmax_arr, amax_arr, dmax_arr;
    // Convert to setpoint::Segment for balanceXY
    auto balanced_segments_setpoint = spg::setpoint::convertSegmentVector(balanced_segments);
    Eigen::Vector3d vmax_vec, amax_vec, dmax_vec;
    setpoint::balanceXY(balanced_segments_setpoint, d.setpoint.p, d.setpoint.v, subtarget_copy.p, subtarget_copy.v, vmax, amax, dmax, vmax_vec, amax_vec, dmax_vec);
    // Convert back if needed
    balanced_segments = spg::setpoint::convertBackSegmentVector(balanced_segments_setpoint);
    subtarget_copy.segment = balanced_segments;
    subtarget_copy.vmax = vmax_vec;
    subtarget_copy.amax = amax_vec;
    subtarget_copy.dmax = dmax_vec;
    subtarget_copy.eta = std::max(subtarget_copy.segment[2].t[0], subtarget_copy.segment[2].t[1]);
    return subtarget_copy;
}

double getDistanceInsidePenaltyArea(const SPGState& d, const Eigen::Vector3d& pos) {
    double x = -std::abs(pos(0)) + d.par.field_penalty_area[0] * 0.5;
    double y = std::abs(pos(1)) - (d.par.field_size[1] * 0.5 - d.par.field_penalty_area[1]);
    return std::max(0.0, std::min(x, y));
}

CheckResults getChecks(const SPGState& d) {
    CheckResults chk;
    
    double arrival_margin = 1.5; // [m]
    double eta_margin = 0.5; // [s]
    double thr_vrotate = d.par.vmax_rotate * 0.1;
    double thr_vmove = d.par.vmax_move * 0.1;
    double age_threshold = 3.0 / d.par.Ts;
    
    double angle_diff = std::abs(std::fmod(d.setpoint.p(2) - d.subtarget.p(2) + M_PI, 2 * M_PI) - M_PI);
    bool small_angle_diff = angle_diff < d.par.scale_angle && std::abs(d.setpoint.v(2)) < thr_vrotate;
    
    chk.age_threshold_reached = d.subtarget.age >= age_threshold;
    chk.hasball = (d.input.robot.skillID >= 1 && d.input.robot.skillID <= 4);
    chk.is_colliding = !d.subtarget.collisionfree;
    chk.start_braking = (d.subtarget.segment_id(0) >= 2) && (d.subtarget.segment_id(1) >= 2);
    chk.small_angle_diff = small_angle_diff;
    chk.rotate_slow_with_large_angle_diff = !small_angle_diff && d.subtarget.amax(2) < d.par.amax_rotate && d.setpoint.v.head<2>().norm() < thr_vmove;
    chk.rotate_fast_with_small_angle_diff = small_angle_diff && d.subtarget.amax(2) == d.par.amax_rotate && std::abs(d.setpoint.v(2)) < thr_vrotate;
    chk.is_close_to_subtarget = (d.subtarget.p.head<2>() - d.setpoint.p.head<2>()).norm() < arrival_margin;
    chk.is_at_subtarget_soon = d.subtarget.eta < eta_margin;
    chk.subtarget_at_target = (d.subtarget.p.head<2>() - d.target.p.head<2>()).norm() < arrival_margin;
    chk.finished_fast_rotation = small_angle_diff && d.subtarget.amax(2) == d.par.amax_rotate;
    chk.fast_rotation_needed = !small_angle_diff && d.subtarget.amax(2) < d.par.amax_rotate && d.setpoint.v.head<2>().norm() < 0.3;
    chk.move_to_worse_position = (d.setpoint.p.head<2>() - d.target.p.head<2>()).norm() + 1.0 < (d.subtarget.p.head<2>() - d.target.p.head<2>()).norm();
    chk.move_slowly_to_better_position = (d.setpoint.p.head<2>() - d.target.p.head<2>()).norm() - 2.0 > (d.subtarget.p.head<2>() - d.target.p.head<2>()).norm() && 
                                         d.subtarget.vmax.head<2>().norm() < 0.99 * d.par.vmax_move && small_angle_diff;
    chk.large_angle_diff_with_ball = !small_angle_diff && d.input.robot.skillID && d.subtarget.amax(2) < d.par.amax_rotate;
    chk.starts_with_violation = d.subtarget.violation_count > 0;
    chk.target_outside_penalty_area_while_subtarget_inside = getDistanceInsidePenaltyArea(d, d.subtarget.p) > 0 && 
                                                             getDistanceInsidePenaltyArea(d, d.target.p) < std::numeric_limits<double>::epsilon();
    
    return chk;
}

bool newSubtargetDesired(const SPGState& d) {
    CheckResults chk = getChecks(d);
    
    if (chk.hasball) {
        std::vector<bool> checks = {
            chk.is_colliding,
            chk.is_at_subtarget_soon,
            chk.age_threshold_reached,
            chk.starts_with_violation
            // Note: Some checks are commented out in MATLAB:
            // chk.target_outside_penalty_area_while_subtarget_inside
            // chk.start_braking
            // chk.rotate_slow_with_large_angle_diff
            // chk.rotate_fast_with_small_angle_diff
            // chk.move_to_worse_position
            // chk.move_slowly_to_better_position
            // chk.large_angle_diff_with_ball
        };
        
        return std::any_of(checks.begin(), checks.end(), [](bool check) { return check; });
    } else {
        // Moving without ball
        std::vector<bool> checks = {
            chk.is_colliding,
            chk.age_threshold_reached,
            chk.is_at_subtarget_soon
            // Note: Some checks are commented out in MATLAB:
            // chk.start_braking
            // chk.starts_with_violation
            // chk.target_outside_penalty_area_while_subtarget_inside
            // chk.is_close_to_subtarget
        };
        
        return std::any_of(checks.begin(), checks.end(), [](bool check) { return check; });
    }
}

Subtarget newSubtarget(SPGState& d, const Subtarget& subtarget) {
    Subtarget best = d.subtarget;
    double phi = subtarget.p(2);
    // TODO: Implement defending logic if needed
    // Search beside obstacles
    best = subtarget::replan::search::besideObstacle(d, best);
    // Search near target (without ball)
    if (d.input.robot.skillID == 0 || d.input.robot.skillID == 1 || d.input.robot.skillID == 5 || d.input.robot.human_dribble_flag == 1) {
        Eigen::Vector3d search_point = {d.target.p(0), d.target.p(1), phi};
        double search_distance = std::max(d.par.search_distance, (d.setpoint.p.head<2>() - d.target.p.head<2>()).norm());
        best = subtarget::replan::search::random(d, best, search_point, search_distance, false);
    }
    // Search near current position
    Eigen::Vector3d search_point = {d.setpoint.p(0), d.setpoint.p(1), phi};
    best = subtarget::replan::search::random(d, best, search_point, d.par.search_distance, false);
    return best;
}

bool notTouchingObstacle(const Eigen::Vector2d& p_robot, const Eigen::MatrixXd& p_obstacles, double obstacle_margin) {
    for (int i = 0; i < p_obstacles.rows(); ++i) {
        if ((p_obstacles.row(i).transpose() - p_robot).squaredNorm() <= obstacle_margin * obstacle_margin) return false;
    }
    return true;
}

bool quickstopDesired(const SPGState& d) {
    Eigen::Vector2d subtarget_direction = d.subtarget.p.head<2>() - d.setpoint.p.head<2>();
    double L = subtarget_direction.norm();
    if (L > 1e-6) subtarget_direction /= L; else subtarget_direction.setZero();
    Eigen::Vector2d vel = d.setpoint.v.head<2>();
    double speed = vel.norm();
    if (speed > 1e-6) vel /= speed; else vel.setZero();
    bool moving_in_opposite_direction = subtarget_direction.dot(vel) < -0.5;
    double v_threshold = 1.0;
    bool having_high_velocity = speed > v_threshold;
    having_high_velocity = false; // as in MATLAB code
    return (((moving_in_opposite_direction && having_high_velocity) || (d.subtarget.action == 0 && having_high_velocity)) && d.input.robot.human_dribble_flag == 0) || d.input.robot.quickstop_trigger;
}

Subtarget quickstop(const SPGState& d, const Subtarget& subtarget) {
    Subtarget subtarget_copy = subtarget;
    subtarget_copy.p = d.setpoint.p;
    if (d.input.robot.quickstop_trigger == 2) subtarget_copy.p(2) = d.target.p(2);
    subtarget_copy.v = Eigen::Vector3d::Zero();
    double max_downscale = 0.01;
    double sc = std::max(max_downscale, std::abs(d.setpoint.v.head<2>().norm()) / d.setpoint.v.head<2>().norm());
    subtarget_copy.vmax = Eigen::Vector3d(d.par.vmax_move * sc, d.par.vmax_move * sc, d.par.vmax_rotate);
    subtarget_copy.amax = Eigen::Vector3d(d.par.amax_quickstop * sc, d.par.amax_quickstop * sc, d.par.dmax_rotate);
    subtarget_copy.dmax = Eigen::Vector3d(d.par.dmax_move, d.par.dmax_move, d.par.dmax_rotate);
    subtarget_copy.p.head<2>() += (d.setpoint.v.head<2>().cwiseAbs().cwiseProduct(d.setpoint.v.head<2>()).array() / subtarget_copy.dmax.head<2>().array() / 2.0).matrix();
    subtarget_copy.eta = 0;
    subtarget_copy.age = 0;
    return subtarget_copy;
}

Subtarget toTarget(SPGState& d) {
    Subtarget subtarget_target = d.subtarget;
    subtarget_target.p = {d.target.p(0), d.target.p(1), d.subtarget.p(2)};
    subtarget_target.v = d.target.v;
    subtarget_target = determineSetpointLimits(d, subtarget_target);
    subtarget_target.target = d.target.p;
    subtarget_target = subtarget::checkCollisionFree(d, subtarget_target, d.par.margin_replan);
    subtarget_target.age = 0;
    return subtarget_target;
}

Subtarget updateBest(const Subtarget& best, const Subtarget& subtarget_candidate, const Eigen::Vector3d& target) {
    double eta_margin = 0.5;
    double minimal_improvement = 0.5;
    double time_extrapolation = 0.5;
    Subtarget updated = best;
    if (subtarget_candidate.collisionfree) {
        if (best.collisionfree && best.eta > eta_margin) {
            if (subtarget_candidate.violation_count <= best.violation_count &&
                (subtarget_candidate.p.head<2>() + subtarget_candidate.v.head<2>() * time_extrapolation - target.head<2>()).norm() <
                (best.p.head<2>() + best.v.head<2>() * time_extrapolation - target.head<2>()).norm() - minimal_improvement) {
                updated = subtarget_candidate;
                updated.age = 0;
            }
        } else {
            updated = subtarget_candidate;
            updated.age = 0;
        }
    }
    return updated;
}

} // namespace replan
} // namespace subtarget
} // namespace spg