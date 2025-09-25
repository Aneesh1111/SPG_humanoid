#include <vector>
#include <Eigen/Dense>
#include "spg/setpoint/Traj1.hpp"
#include "spg/setpoint/TrajSegment.hpp"

namespace spg {
namespace setpoint {

void Traj1(Traject& traject, const std::vector<Segment>& segment, double Ts) {
    size_t n = 1;
    std::vector<double> t = {Ts};
    size_t ndim = segment[0].p.size();
    std::vector<double> P, V, A;
    std::vector<bool> tseg;
    TrajSegment(segment, t, P, V, A, tseg);
    Eigen::Vector3i segment_id;
    for (size_t j = 0; j < ndim; ++j) {
        segment_id[j] = tseg[j] ? 1 : 0;
    }
    traject.t = t;
    traject.p.clear();
    traject.v.clear();
    traject.a.clear();
    traject.segment_id.clear();
    traject.p.push_back(Eigen::Vector3d(P[0], P[1], P[2]));
    traject.v.push_back(Eigen::Vector3d(V[0], V[1], V[2]));
    traject.a.push_back(Eigen::Vector3d(A[0], A[1], A[2]));
    traject.segment_id.push_back(segment_id);
}

} // namespace setpoint
} // namespace spg