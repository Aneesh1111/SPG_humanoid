#pragma once
#include <Eigen/Dense>
#include <vector>
#include "spg/Init.hpp"

namespace spg {
namespace subtarget {
namespace angle {

double aimAtTarget(const SPGState& d);
double dribble(const SPGState& d);
double shield(const SPGState& d);
double set(const SPGState& d);

} // namespace angle
} // namespace subtarget
} // namespace spg
