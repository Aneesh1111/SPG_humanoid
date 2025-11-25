#include "spg/setpoint/Setpoint.hpp"
#include "spg/setpoint/GetSegments.hpp"
#include "spg/setpoint/Set.hpp"
#include "spg/setpoint/SetHumanoid.hpp"

namespace spg {
namespace setpoint {

void updateSetpoint(SPGState& state) {
    Set(state); // Call the Set function from Set.cpp
}

void updateSetpointHumanoid(SPGState& state) {
    SetHumanoid(state); // Call the SetHumanoid function from SetHumanoid.cpp
}

}}
