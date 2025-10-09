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
    Eigen::Vector3d acc = am; // MATLAB: acc = am;
    segment.dt = (ve - v0).cwiseAbs().cwiseQuotient(acc); // MATLAB: segment.dt = abs(ve-v0)./acc;
    segment.a = functions::sign((ve - v0).array()) * acc.array(); // MATLAB: segment.a = functions.sign(ve-v0).*acc;
    segment.t = t0.array() + segment.dt.array(); // MATLAB: segment.t = t0+segment.dt;
    segment.p = p0 + v0.cwiseProduct(segment.dt) + 0.5 * segment.a.cwiseProduct(segment.dt.cwiseProduct(segment.dt)); // MATLAB: segment.p = p0+v0.*segment.dt+.5*segment.a.*segment.dt.^2;
    segment.v = ve; // MATLAB: segment.v = ve;
    return segment;
}

Segment move_at_constant_vel(Segment segment, const Eigen::Vector3d& p0, const Eigen::Vector3d& v0, const Eigen::Vector3d& t0, const Eigen::Vector3d& dt) {
    segment.dt = dt.cwiseMax(0.0); // MATLAB: segment.dt = max(0,dt);
    segment.a = Eigen::Vector3d::Zero(); // MATLAB: segment.a = zeros(size(dt));
    segment.t = t0.array() + segment.dt.array(); // MATLAB: segment.t = t0+segment.dt;
    segment.p = p0 + v0.cwiseProduct(segment.dt); // MATLAB: segment.p = p0+v0.*segment.dt;
    segment.v = v0; // MATLAB: segment.v = v0;
    return segment;
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> get_max_speed(Segment segment, const Eigen::Vector3d& p0, const Eigen::Vector3d& v0, const Eigen::Vector3d& pe, const Eigen::Vector3d& ve, const Eigen::Vector3d& vm, const Eigen::Vector3d& am, const Eigen::Vector3d& dm) {
    // MATLAB: segment = move_to_vel(segment, p0, v0, 0, ve, am, dm);
    segment = move_to_vel(segment, p0, v0, Eigen::Vector3d::Zero(), ve, am, dm);
    
    // MATLAB: direction = functions.sign(pe-segment.p);
    Eigen::Array3d direction = functions::sign((pe - segment.p).array());
    
    // MATLAB: a = direction.*am; d = direction.*dm;
    Eigen::Array3d a = direction * am.array();
    Eigen::Array3d d = direction * dm.array();
    
    // MATLAB: tmp = max(0,.5*v0.^2+.5*ve.^2+a.*(pe-p0));
    Eigen::Array3d tmp = (0.5 * v0.array().square() + 0.5 * ve.array().square() + a * (pe - p0).array()).max(0.0);
    
    // MATLAB: v = sqrt(tmp);
    Eigen::Array3d v = tmp.sqrt();
    
    // MATLAB: v1 = direction.*min(v,vm);
    Eigen::Array3d v1 = direction * v.min(vm.array());
    
    // MATLAB: tmax = max(0,(v.^2-vm.^2)./am./vm);
    Eigen::Array3d v_sq = v.square();
    Eigen::Array3d vm_sq = vm.array().square();
    Eigen::Array3d tmax = ((v_sq - vm_sq) / am.array() / vm.array()).max(0.0);

    // MATLAB: sel = abs(v0)>vm; if nnz(sel) tmax(sel) = (pe(sel)-segment.p(sel))./vm(sel).*direction(sel); end
    for (int i = 0; i < 3; ++i) {
        if (std::abs(v0[i]) > vm[i]) {
            tmax[i] = (pe[i] - segment.p[i]) / vm[i] * direction[i];
        }
    }
    
    // MATLAB: sel = abs(pe-segment.p)<1e-8; tmax(sel) = 0; v1(sel) = v0(sel);
    for (int i = 0; i < 3; ++i) {
        if (std::abs(pe[i] - segment.p[i]) < 1e-8) {
            tmax[i] = 0;
            v1[i] = v0[i];
        }
    }

    return {v1.matrix(), tmax.matrix()};
}

std::vector<Segment> getSegments(std::vector<Segment> segment, const Eigen::Vector3d& p0, const Eigen::Vector3d& v0, Eigen::Vector3d pe, const Eigen::Vector3d& ve, const Eigen::Vector3d& vm, const Eigen::Vector3d& am, const Eigen::Vector3d& dm) {
    // MATLAB: pe(3) = spg.setpoint.wrap(pe(3), p0(3));
    pe[2] = wrap(pe[2], p0[2]);
    
    // MATLAB: [v1, tmax] = get_max_speed(segment(1), p0, v0, pe, ve, vm, am, dm);
    auto [v1, tmax] = get_max_speed(segment[0], p0, v0, pe, ve, vm, am, dm);
    
    // MATLAB: segment(1) = move_to_vel(segment(1), p0, v0, 0, v1, am, dm);
    segment[0] = move_to_vel(segment[0], p0, v0, Eigen::Vector3d::Zero(), v1, am, dm);
    
    // MATLAB: segment(2) = move_at_constant_vel(segment(2), segment(1).p, segment(1).v, segment(1).t, tmax);
    segment[1] = move_at_constant_vel(segment[1], segment[0].p, segment[0].v, segment[0].t, tmax);
    
    // MATLAB: segment(3) = move_to_vel(segment(3), segment(2).p, segment(2).v, segment(2).t, ve, am, dm);
    segment[2] = move_to_vel(segment[2], segment[1].p, segment[1].v, segment[1].t, ve, am, dm);
    
    // MATLAB: segment(4) = move_at_constant_vel(segment(4), segment(3).p, segment(3).v, segment(3).t, [1e10 1e10 1e10]);
    segment[3] = move_at_constant_vel(segment[3], segment[2].p, segment[2].v, segment[2].t, Eigen::Vector3d(1e10, 1e10, 1e10));
    
    return segment;
}

} // namespace setpoint
} // namespace spg
