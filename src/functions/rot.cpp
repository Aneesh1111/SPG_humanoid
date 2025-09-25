#include <Eigen/Dense>
#include <cmath>

namespace functions {

Eigen::Matrix2d rot(double phi) {
    double cosphi = std::cos(phi);
    double sinphi = std::sin(phi);
    Eigen::Matrix2d R;
    R << cosphi, -sinphi,
         sinphi,  cosphi;
    return R;
}

} // namespace functions