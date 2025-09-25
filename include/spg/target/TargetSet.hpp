#pragma once
#include "spg/Init.hpp"
#include "AdjustToField.hpp"
#include "AdjustToPenaltyArea.hpp"
#include "AdjustToGoalArea.hpp"
#include "AdjustToObstacles.hpp"
#include "AdjustTo3mRule.hpp"

namespace spg {
namespace target {

void set(SPGState& state);

}}
