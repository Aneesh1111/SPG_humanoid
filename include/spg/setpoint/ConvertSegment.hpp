#pragma once
#include <vector>
#include "spg/Init.hpp"
#include "spg/setpoint/Segment.hpp"

namespace spg {
namespace setpoint {
std::vector<spg::setpoint::Segment> convertSegmentVector(const std::vector<spg::Segment>& segs);
std::vector<spg::Segment> convertBackSegmentVector(const std::vector<spg::setpoint::Segment>& segs);
} // namespace setpoint
} // namespace spg
