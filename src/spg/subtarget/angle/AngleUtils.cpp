#include <Eigen/Dense>
#include <cmath>
#include "spg/subtarget/angle/AngleUtils.hpp"
#include "spg/setpoint/Wrap.hpp"

namespace spg {
namespace subtarget {
namespace angle {

double AimAtTarget(const SPGState& d) {
    Eigen::Vector2d v = d.target.p.head<2>() - d.setpoint.p.head<2>();
    if (v.norm() > 1e-9) {
        return std::atan2(-v(0), v(1));
    } else {
        return d.subtarget.p(2);
    }
}

double Dribble(const SPGState& d) {
    if (d.setpoint.v.head<2>().norm() > 1e-6) {
        Eigen::Vector2d v = d.setpoint.v.head<2>();
        return std::atan2(-v(0), v(1));
    } else {
        return d.subtarget.p(2);
    }
}

double Shield(const SPGState& d) {
    // Get distance to nearest obstacle
    std::vector<Eigen::Vector2d> obstacles;
    for (size_t i = 0; i < d.input.obstacles.active.size(); ++i) {
        if (d.input.obstacles.active[i]) obstacles.push_back(d.input.obstacles.p[i]);
    }
    double mindist2 = 1e10;
    size_t nearest_obstacle = 0;
    for (size_t i = 0; i < obstacles.size(); ++i) {
        double dist2 = (obstacles[i] - d.setpoint.p.head<2>()).squaredNorm();
        if (dist2 < mindist2) {
            mindist2 = dist2;
            nearest_obstacle = i;
        }
    }
    Eigen::Vector2d v = -(obstacles[nearest_obstacle] - d.setpoint.p.head<2>());
    v.normalize();
    double x_end = 2, x_start = 1;
    double frac = std::max(0.0, std::min(1.0, (x_end - std::sqrt(mindist2)) / (x_end - x_start)));
    if (d.setpoint.v.head<2>().norm() > 1e-9) {
        Eigen::Vector2d v_robot = d.setpoint.v.head<2>().normalized();
        v = frac * v + (1 - frac) * v_robot;
    } else {
        Eigen::Vector2d robot_direction(-std::sin(d.subtarget.p(2)), std::cos(d.subtarget.p(2)));
        v = frac * v + (1 - frac) * robot_direction;
    }
    return std::atan2(-v(0), v(1));
}

double set(const SPGState& d) {
    double angle = 0.0;
    switch (d.input.robot.skillID) {
        case 1: // dribble
            angle = Dribble(d);
            break;
        default: // 0 move, 2 aim, 3 kick, 4 shield
            angle = d.target.p(2);
            break;
    }
    // wrap around current setpoint
    angle = setpoint::wrap(angle, d.setpoint.p(2));
    return angle;
}

} // namespace angle
} // namespace subtarget
} // namespace spg
