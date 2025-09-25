#include <vector>
#include <Eigen/Dense>
#include "spg/setpoint/TrajPredict.hpp"
#include "spg/setpoint/TrajSegment.hpp"

namespace spg {
namespace setpoint {

void TrajPredict(SPGState& d, const std::vector<Segment>& segment) {
    size_t n = d.traj.p.size();
    std::vector<int> nvec(n);
    for (size_t i = 0; i < n; ++i) nvec[i] = i + 1;
    double Ts = d.par.Ts_predict;
    std::vector<double> t(n);
    for (size_t i = 0; i < n; ++i) t[i] = nvec[i] * Ts;
    size_t ndim = segment[0].p.size();
    std::vector<double> P, V, A;
    std::vector<bool> tseg;
    TrajSegment(segment, t, P, V, A, tseg);
    // Assign trajectory data
    d.traj.t = t;
    d.traj.p.clear();
    d.traj.v.clear();
    d.traj.a.clear();
    d.traj.segment_id.clear();
    for (size_t i = 0; i < n; ++i) {
        d.traj.p.push_back(Eigen::Vector3d(P[i * ndim], P[i * ndim + 1], P[i * ndim + 2]));
        d.traj.v.push_back(Eigen::Vector3d(V[i * ndim], V[i * ndim + 1], V[i * ndim + 2]));
        d.traj.a.push_back(Eigen::Vector3d(A[i * ndim], A[i * ndim + 1], A[i * ndim + 2]));
        Eigen::Vector3i seg_id;
        for (size_t j = 0; j < ndim; ++j) {
            size_t idx = j + i * ndim;
            seg_id[j] = tseg[idx] ? 1 : 0;
        }
        d.traj.segment_id.push_back(seg_id);
    }
    // Dribble orientation update if needed
    if (d.input.robot.skillID == 1) {
        for (size_t i = 0; i < n; ++i) {
            double vnorm = d.traj.v[i].head(ndim - 1).squaredNorm();
            if (vnorm > 1e-12) {
                d.traj.p[i][2] = std::atan2(-d.traj.v[i][0], d.traj.v[i][1]);
            }
        }
    }
}

} // namespace setpoint
} // namespace spg