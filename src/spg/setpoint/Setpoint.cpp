#include "spg/setpoint/Setpoint.hpp"
#include "spg/setpoint/GetSegments.hpp"
#include "spg/setpoint/Set.hpp"

namespace spg {
namespace setpoint {

void updateSetpoint(SPGState& state) {
    Set(state); // Call the Set function from Set.cpp
}

}}
