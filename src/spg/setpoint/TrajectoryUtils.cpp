#include "spg/setpoint/TrajectoryUtils.hpp"
#include <algorithm>
#include <cmath>

namespace spg {
namespace setpoint {

void combineSegmentData(const std::vector<Segment>& segments,
                        std::vector<double>& t,
                        std::vector<double>& p,
                        std::vector<double>& v,
                        std::vector<double>& a) {
    size_t nseg = segments.size();
    size_t ndim = 3;
    t.resize(nseg * ndim);
    p.resize(nseg * ndim);
    v.resize(nseg * ndim);
    a.resize(nseg * ndim);
    size_t ind = 0;
    for (size_t i = 0; i < nseg; ++i) {
        for (size_t j = 0; j < ndim; ++j) {
            t[ind] = segments[i].t[j];
            p[ind] = segments[i].p[j];
            v[ind] = segments[i].v[j];
            a[ind] = segments[i].a[j];
            ++ind;
        }
    }
}

void trajSegment(const std::vector<Segment>& segments,
                 const std::vector<double>& time,
                 std::vector<double>& P,
                 std::vector<double>& V,
                 std::vector<double>& A,
                 std::vector<bool>& tseg) {
    std::vector<double> t, p, v, a;
    combineSegmentData(segments, t, p, v, a);
    size_t n = time.size();
    size_t ndim = 3;
    P.resize(n * ndim);
    V.resize(n * ndim);
    A.resize(n * ndim);
    tseg.resize(n * ndim);
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = 0; j < ndim; ++j) {
            size_t idx = i * ndim + j;
            double t_rel = time[i] - t[j];
            double vt = v[j] * t_rel;
            double at = a[j] * t_rel;
            P[idx] = p[j] + vt + 0.5 * at * t_rel;
            V[idx] = v[j] + at;
            A[idx] = a[j];
            tseg[idx] = (time[i] > t[j]);
        }
    }
}

}}
