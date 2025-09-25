#include <Eigen/Dense>
#include <vector>
#include <algorithm>
#include <cmath>
#include "spg/setpoint/GetSegments.hpp"
#include "spg/setpoint/Wrap.hpp"
#include "functions/sign.hpp"

namespace spg {
namespace setpoint {

Segment move_to_vel(Segment segment, const Eigen::Vector3d& p0, const Eigen::Vector3d& v0, const Eigen::Vector3d& t0, const Eigen::Vector3d& ve, const Eigen::Vector3d& am, const Eigen::Vector3d& dm) {
    Eigen::Vector3d acc = (ve - v0).norm() > v0.norm() ? am : dm;
    segment.dt = (ve - v0).cwiseAbs().cwiseQuotient(acc);
    segment.a = functions::sign((ve - v0).array()) * acc.array();
    segment.t = t0.array() + segment.dt.array();
    segment.p = p0 + v0.cwiseProduct(segment.dt) + 0.5 * segment.a.cwiseProduct(segment.dt.cwiseProduct(segment.dt));
    segment.v = ve;
    return segment;
}

Segment move_at_constant_vel(Segment segment, const Eigen::Vector3d& p0, const Eigen::Vector3d& v0, const Eigen::Vector3d& t0, const Eigen::Vector3d& dt) {
    segment.dt = dt.cwiseMax(0.0);
    segment.a = Eigen::Vector3d::Zero();
    segment.t = t0.array() + segment.dt.array();
    segment.p = p0 + v0.cwiseProduct(segment.dt) + 0.5 * segment.a.cwiseProduct(segment.dt.cwiseProduct(segment.dt));
    segment.v = v0;
    return segment;
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> get_max_speed(Segment segment, const Eigen::Vector3d& p0, const Eigen::Vector3d& v0, const Eigen::Vector3d& pe, const Eigen::Vector3d& ve, const Eigen::Vector3d& vm, const Eigen::Vector3d& am, const Eigen::Vector3d& dm) {
    Eigen::Array3d t0(0,0,0);
    segment = move_to_vel(segment, p0, v0, t0, ve, am, dm);
    Eigen::Array3d direction = functions::sign((pe - segment.p).array());
    Eigen::Array3d a = direction * am.array();
    Eigen::Array3d d = direction * dm.array();
    Eigen::Array3d acc_ratio = a / d;
    Eigen::Array3d numerator = v0.array().square() + acc_ratio * ve.array().square() + 2.0 * a * (pe - p0).array();
    Eigen::Array3d denom = 1.0 + acc_ratio;
    Eigen::Array3d tmp = (numerator / denom).max(0.0);
    Eigen::Array3d v = tmp.sqrt();
    Eigen::Array3d v1 = direction * v.min(vm.array());
    Eigen::Array3d v_sq = v.square();
    Eigen::Array3d vm_sq = vm.array().square();
    Eigen::Array3d tmax = ((v_sq - vm_sq) / (2.0 * dm.array() * vm.array()) + (v_sq - vm_sq) / (2.0 * am.array() * vm.array())).max(0.0);

    for (int i = 0; i < 3; ++i) {
        if (std::abs(v0[i]) > vm[i]) {
            tmax[i] = (pe[i] - segment.p[i]) / vm[i] * direction[i];
        }
        if (std::abs(pe[i] - segment.p[i]) < 1e-8) {
            tmax[i] = 0;
            v1[i] = v0[i];
        }
    }

    return {v1.matrix(), tmax.matrix()};
}

std::vector<Segment> getSegments(std::vector<Segment> segment, const Eigen::Vector3d& p0, const Eigen::Vector3d& v0, Eigen::Vector3d pe, const Eigen::Vector3d& ve, const Eigen::Vector3d& vm, const Eigen::Vector3d& am, const Eigen::Vector3d& dm) {
    // Correct for rotation
    pe[2] = wrap(pe[2], p0[2]);
    // Determine maximum speed
    auto [v1, tmax] = get_max_speed(segment[0], p0, v0, pe, ve, vm, am, dm);
    // Move towards maximum speed
    Eigen::Array3d t0(0,0,0);
    segment[0] = move_to_vel(segment[0], p0, v0, t0, v1, am, dm);
    // Move at max constant speed
    segment[1] = move_at_constant_vel(segment[1], segment[0].p, segment[0].v, segment[0].t, tmax);
    // Move towards end speed/position
    segment[2] = move_to_vel(segment[2], segment[1].p, segment[1].v, segment[1].t, ve, am, dm);
    // Move at max constant speed (dummy, as in MATLAB)
    segment[3] = move_at_constant_vel(segment[3], segment[2].p, segment[2].v, segment[2].t, Eigen::Vector3d(1e10, 1e10, 1e10));
    return segment;
}

} // namespace setpoint
} // namespace spg
