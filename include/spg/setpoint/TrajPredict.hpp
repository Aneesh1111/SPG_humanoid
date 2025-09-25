#pragma once
#include <vector>
#include "spg/setpoint/Segment.hpp"
#include "spg/Init.hpp"

namespace spg {
namespace setpoint {

void TrajPredict(SPGState& state, const std::vector<Segment>& segment);

} // namespace setpoint
}