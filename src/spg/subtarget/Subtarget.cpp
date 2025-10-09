#include <Eigen/Dense>
#include "spg/subtarget/Subtarget.hpp"
#include "spg/subtarget/angle/AngleUtils.hpp"
#include "spg/subtarget/CheckCollisionFree.hpp"
#include "spg/subtarget/replan/ReplanUtils.hpp"
#include <iostream>

namespace spg {
namespace subtarget {

SPGState Set(SPGState& d) {
    double robot2ball_dist = (d.input.ball.p.head<2>() - d.input.robot.p.head<2>()).norm();
    Eigen::Vector2d robot_v = d.input.robot.v.head<2>().normalized();
    Eigen::Vector2d ball_v = d.input.ball.v.head<2>().normalized();
    bool robotVel_same_direction_as_ballVel = robot_v.dot(ball_v) > std::cos(M_PI / 18.0); // 10 degrees
    Eigen::Vector2d robot2ball = d.input.ball.p.head<2>() - d.input.robot.p.head<2>();
    bool behind_ball_flag = robot2ball.normalized().dot(ball_v) > std::cos(M_PI / 4.0); // 45 degrees
    if (d.input.robot.human_dribble_flag == 1 && robot2ball_dist < 3.5 && behind_ball_flag && robotVel_same_direction_as_ballVel) {
        d.subtarget.p = {d.target.p(0), d.target.p(1), d.subtarget.p(2)};
        d.subtarget.v = d.target.v;
        d.subtarget = replan::determineSetpointLimits(d, d.subtarget);
        d.subtarget.target = d.target.p;
        d.subtarget.age = 0;
        d.subtarget.action = 1;
        return d;
    } else {
        d.subtarget.p(2) = angle::set(d);
        d.subtarget = subtarget::checkCollisionFree(d, d.subtarget, 0);
        d.subtarget.age += 1;
        if (replan::quickstopDesired(d)) {
            d.subtarget = replan::quickstop(d, d.subtarget);
            d.subtarget.action = 0;
            return d;
        }
        Subtarget subtarget_target = replan::toTarget(d);
        if (subtarget_target.collisionfree) {
            subtarget_target.action = 1;
            d.subtarget = subtarget_target;
            return d;
        }
        if (replan::newSubtargetDesired(d)) {
            Subtarget subtarget = replan::newSubtarget(d, d.subtarget);
            if (subtarget.collisionfree) {
                subtarget.action = 2;
                d.subtarget = subtarget;
                return d;
            }
        }
        if (d.subtarget.collisionfree) {
            d.subtarget.action = 3;
            return d;
        }
        d.subtarget = replan::quickstop(d, d.subtarget);
        d.subtarget.action = 0;
        return d;
    }
}

} // namespace subtarget
} // namespace spg
