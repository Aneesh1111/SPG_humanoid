#include <vector>
#include <Eigen/Dense>
#include "spg/setpoint/TrajSegment.hpp"
#include "spg/setpoint/CombineSegmentData.hpp"

namespace spg {
namespace setpoint {

void TrajSegment(const std::vector<Segment>& segment, const std::vector<double>& time,
                 std::vector<double>& P, std::vector<double>& V, std::vector<double>& A, std::vector<bool>& tseg) {
    std::vector<double> t, p, v, a;
    CombineSegmentData(segment, t, p, v, a);
    size_t ntime = time.size();
    size_t ndim = segment[0].p.size();
    P.resize(ntime * ndim);
    V.resize(ntime * ndim);
    A.resize(ntime * ndim);
    tseg.resize(ntime * ndim);
    for (size_t i = 0; i < ntime; ++i) {
        for (size_t j = 0; j < ndim; ++j) {
            size_t idx = j + i * ndim;
            double t_rel = time[i] - t[idx];
            double vt = v[idx] * t_rel;
            double at = a[idx] * t_rel;
            P[idx] = p[idx] + vt + 0.5 * at * t_rel;
            V[idx] = v[idx] + at;
            A[idx] = a[idx];
            tseg[idx] = (time[i] > t[idx]);
        }
    }
}

} // namespace setpoint
} // namespace spg