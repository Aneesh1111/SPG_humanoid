#pragma once
#include <vector>
#include "spg/setpoint/Segment.hpp"
#include "spg/Init.hpp"

namespace spg {
namespace setpoint {

void Traj1(Traject& traject, const std::vector<Segment>& segment, double Ts);

} // namespace setpoint
} // namespace spg