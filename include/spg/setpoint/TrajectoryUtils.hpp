#pragma once
#include <vector>
#include <array>
#include "Segment.hpp"

namespace spg {
namespace setpoint {

void combineSegmentData(const std::vector<Segment>& segments,
                        std::vector<double>& t,
                        std::vector<double>& p,
                        std::vector<double>& v,
                        std::vector<double>& a);

void trajSegment(const std::vector<Segment>& segments,
                 const std::vector<double>& time,
                 std::vector<double>& P,
                 std::vector<double>& V,
                 std::vector<double>& A,
                 std::vector<bool>& tseg);

}}
