#pragma once
#include "spg/Init.hpp"

namespace spg {
namespace subtarget {
namespace replan {
namespace search {

Subtarget random(SPGState& d, const Subtarget& best, const Eigen::Vector3d& search_point, double search_distance, int defending_opp_with_or_without_ball);

} // namespace search
} // namespace replan
} // namespace subtarget
} // namespace spg
