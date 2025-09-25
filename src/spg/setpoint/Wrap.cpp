#include <cmath>
#include "spg/setpoint/Wrap.hpp"

namespace spg {
namespace setpoint {

double wrap(double angle, double angle_setpoint) {
    return std::fmod(angle - angle_setpoint + M_PI, 2 * M_PI) + angle_setpoint - M_PI;
}

} // namespace setpoint
} // namespace spg
