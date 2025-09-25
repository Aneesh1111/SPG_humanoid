#include <vector>
#include <Eigen/Dense>
#include "spg/setpoint/CombineSegmentData.hpp"

namespace spg {
namespace setpoint {

void CombineSegmentData(const std::vector<Segment>& segment, std::vector<double>& t, std::vector<double>& p, std::vector<double>& v, std::vector<double>& a) {
    size_t nseg = segment.size();
    size_t ndim = segment[0].p.size();
    t.resize(nseg * ndim);
    p.resize(nseg * ndim);
    v.resize(nseg * ndim);
    a.resize(nseg * ndim);
    size_t ind = 0;
    for (size_t i = 0; i < nseg; ++i) {
        for (size_t j = 0; j < ndim; ++j) {
            t[ind] = segment[i].t[j];
            p[ind] = segment[i].p[j];
            v[ind] = segment[i].v[j];
            a[ind] = segment[i].a[j];
            ++ind;
        }
    }
}

} // namespace setpoint
} // namespace spg