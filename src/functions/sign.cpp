#include <Eigen/Dense>
#include "functions/sign.hpp"

namespace functions {

Eigen::ArrayXd sign(const Eigen::ArrayXd& u) {
    Eigen::ArrayXd y = Eigen::ArrayXd::Ones(u.size());
    for (int i = 0; i < u.size(); ++i) {
        if (u(i) < 0) y(i) = -1.0;
    }
    return y;
}

} // namespace functions