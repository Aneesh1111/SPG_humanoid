#pragma once
#include "spg/Init.hpp"

namespace spg {

SPGState NextSample(SPGState d);
SPGState NextSampleSetpoint(SPGState d);
SPGState NextSampleIntercept(SPGState d);

} // namespace spg
