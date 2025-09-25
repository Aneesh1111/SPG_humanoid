#pragma once
#include <vector>
#include <array>
#include "Segment.hpp"

namespace spg {
namespace setpoint {

void CombineSegmentData(const std::vector<Segment>& segment, 
                        std::vector<double>& t, 
                        std::vector<double>& p, 
                        std::vector<double>& v, 
                        std::vector<double>& a);

}}
