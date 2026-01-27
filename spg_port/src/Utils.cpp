#include "SPG.hpp"
#include <cmath>

namespace spg {

// ============================================================================
// Utility Functions - TODO MR6
// ============================================================================

void rot(double angle, const double* in, double* out) {
    // TODO MR6: Implement 2D rotation matrix multiplication
    // out[0] = cos(angle) * in[0] - sin(angle) * in[1]
    // out[1] = sin(angle) * in[0] + cos(angle) * in[1]
    
    // Placeholder: identity transform
    out[0] = in[0];
    out[1] = in[1];
}

double sign(double x) {
    // TODO MR6: Implement sign function with zero threshold
    // if (fabs(x) < 1e-10) return 0.0;
    // return (x > 0.0) ? 1.0 : -1.0;
    
    // Placeholder
    return (x >= 0.0) ? 1.0 : -1.0;
}

double wrap(double theta) {
    // TODO MR6: Implement angle wrapping to [-pi, pi]
    // while (theta > M_PI) theta -= 2 * M_PI;
    // while (theta < -M_PI) theta += 2 * M_PI;
    // return theta;
    
    // Placeholder
    return theta;
}

} // namespace spg
