#include "spg/setpoint/ConvertSegment.hpp"
#include "spg/Init.hpp"

namespace spg {
namespace setpoint {
std::vector<spg::setpoint::Segment> convertSegmentVector(const std::vector<spg::Segment>& segs) {
    std::vector<spg::setpoint::Segment> result;
    result.reserve(segs.size());
    for (const auto& seg : segs) {
        spg::setpoint::Segment s;
        s.t = seg.t;
        s.p = seg.p;
        s.v = seg.v;
        s.a = seg.a;
        s.dt = seg.dt;
        result.push_back(s);
    }
    return result;
}

std::vector<spg::Segment> convertBackSegmentVector(const std::vector<spg::setpoint::Segment>& segs) {
    std::vector<spg::Segment> result;
    result.reserve(segs.size());
    for (const auto& seg : segs) {
        spg::Segment s;
        s.t = seg.t;
        s.p = seg.p;
        s.v = seg.v;
        s.a = seg.a;
        s.dt = seg.dt;
        result.push_back(s);
    }
    return result;
}
} // namespace setpoint
} // namespace spg
