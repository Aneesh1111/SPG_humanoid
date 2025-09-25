#pragma once
#include <vector>
#include "spg/setpoint/Segment.hpp"

namespace spg {
namespace setpoint {

void TrajSegment(const std::vector<Segment>& segment, const std::vector<double>& time,
                 std::vector<double>& P, std::vector<double>& V, std::vector<double>& A, std::vector<bool>& tseg);

} // namespace setpoint
} // namespace spg