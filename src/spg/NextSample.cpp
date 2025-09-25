#include "spg/NextSample.hpp"
#include "spg/setpoint/StateCorrection.hpp"
#include "spg/setpoint/Set.hpp"
#include "spg/setpoint/GetSegments.hpp"
#include "spg/target/TargetSet.hpp"
#include "spg/subtarget/Subtarget.hpp"
#include <algorithm>

namespace spg {

SPGState NextSample(SPGState d) {
    d = setpoint::StateCorrection(d);
    d = target::Set(d);
    d = subtarget::Set(d);
    d = setpoint::Set(d);
    // Calculate eta to subtarget
    auto subtarget_segments = d.subtarget.segment;
    subtarget_segments = setpoint::getSegments(subtarget_segments, d.setpoint.p, d.setpoint.v, d.subtarget.p, d.subtarget.v, d.subtarget.vmax, d.subtarget.amax, {d.par.dmax_move, d.par.dmax_move, d.par.dmax_rotate});
    double subtarget_eta = std::max(subtarget_segments[2].t[0], subtarget_segments[2].t[1]);
    // Calculate eta from subtarget to target
    double subtarget2target_eta = 0;
    if (std::abs(d.subtarget.p[0] - d.target.p[0]) > 1e-2 || std::abs(d.subtarget.p[1] - d.target.p[1]) > 1e-2) {
        subtarget_segments = setpoint::getSegments(subtarget_segments, d.subtarget.p, d.subtarget.v, d.target.p, d.target.v, d.par.vmax_move, d.par.amax_move, {d.par.dmax_move, d.par.dmax_move, d.par.dmax_rotate});
        subtarget2target_eta = std::max(subtarget_segments[2].t[0], subtarget_segments[2].t[1]);
    }
    d.target.eta = subtarget_eta + subtarget2target_eta;
    return d;
}

SPGState NextSampleSetpoint(SPGState d) {
    d = setpoint::StateCorrection(d);
    d = setpoint::Set(d);
    return d;
}

SPGState NextSampleIntercept(SPGState d) {
    for (int i = 0; i < d.par.nintercept_positions; ++i) {
        d.input.robot.quickstop_trigger = 0.0;
        d.input.robot.target = d.intercept_positions_etas.sample[i].p;
        d.input.robot.target_vel = Eigen::Vector3d::Zero();
        d.input.robot.human_dribble_flag = 0;
        d.input.robot.skillID = 0;
        d.input.robot.reset_trigger = 0;
        d.target.p = d.input.robot.target;
        d.target.v = d.input.robot.target_vel;
        d.target.eta = 0;
        d = target::Set(d);
        d = subtarget::Set(d);
        d = setpoint::Set(d);
        auto subtarget_segments = d.subtarget.segment;
        subtarget_segments = setpoint::getSegments(subtarget_segments, d.setpoint.p, d.setpoint.v, d.subtarget.p, d.subtarget.v, d.subtarget.vmax, d.subtarget.amax, {d.par.dmax_move, d.par.dmax_move, d.par.dmax_rotate});
        double subtarget_eta = std::max(subtarget_segments[2].t[0], subtarget_segments[2].t[1]);
        double subtarget2target_eta = 0;
        if (std::abs(d.subtarget.p[0] - d.target.p[0]) > 1e-2 || std::abs(d.subtarget.p[1] - d.target.p[1]) > 1e-2) {
            subtarget_segments = setpoint::getSegments(subtarget_segments, d.subtarget.p, d.subtarget.v, d.target.p, d.target.v, d.par.vmax_move, d.par.amax_move, {d.par.dmax_move, d.par.dmax_move, d.par.dmax_rotate});
            subtarget2target_eta = std::max(subtarget_segments[2].t[0], subtarget_segments[2].t[1]);
        }
        d.intercept_positions_etas.sample[i].eta = subtarget_eta + subtarget2target_eta;
    }
    return d;
}

} // namespace spg
