#include <Eigen/Dense>
#include "spg/setpoint/Set.hpp"
#include "spg/setpoint/GetSegments.hpp"
#include "spg/setpoint/Traj1.hpp"
#include "spg/setpoint/TrajPredict.hpp"
#include "spg/setpoint/ConvertSegment.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace spg {
namespace setpoint {

SPGState Set(SPGState& d) {
    // Field/margin check
    double field_x_half = d.par.field_size[0] * 0.5 + d.par.field_border_margin;
    double field_y_half = d.par.field_size[1] * 0.5 + d.par.field_border_margin;
    bool inside_field =
        (-field_x_half < d.input.robot.p[0] && d.input.robot.p[0] < field_x_half) &&
        (-field_y_half < d.input.robot.p[1] && d.input.robot.p[1] < field_y_half);
    if (inside_field || d.subtarget.automatic_substitution_flag == 1) {
        Eigen::Vector3d dmax;
        dmax = Eigen::Vector3d(d.par.dmax_move, d.par.dmax_move, d.par.dmax_rotate);
        // Convert segment type before calling getSegments
        auto setpointSegments = spg::setpoint::convertSegmentVector(d.aux.segment);
        setpointSegments = getSegments(setpointSegments, d.setpoint.p, d.setpoint.v, d.subtarget.p, d.subtarget.v, d.subtarget.vmax, d.subtarget.amax, dmax);
        // d.aux.segment = spg::setpoint::convertBackSegmentVector(setpointSegments); // convert back if needed
        // // Propagate 1 sample
        // // Convert segment type before calling Traj1
        // auto trajSegments = spg::setpoint::convertSegmentVector(d.aux.segment);
        // Traj1(d.traj, trajSegments, d.par.Ts);
        Traj1(d.traj, setpointSegments, d.par.Ts);
        
        d.setpoint.p = d.traj.p[0];
        d.setpoint.v = d.traj.v[0];
        d.setpoint.a = d.traj.a[0];

        // Generate full predicted trajectory for visualization (all npredict timesteps)
        TrajPredict(d, setpointSegments);

        // Field margin logic
        double field_width_half = d.par.field_size[0] * 0.5 + d.par.field_border_margin + (d.subtarget.automatic_substitution_flag == 1 ? d.par.technical_area_width : 0.0);
        double field_length_half = d.par.field_size[1] * 0.5 + d.par.field_border_margin;
        // X direction
        if (d.setpoint.p[0] < -field_width_half) d.setpoint.p[0] = -field_width_half;
        if (d.setpoint.p[0] >  field_width_half) d.setpoint.p[0] =  field_width_half;
        double dist2sideline = field_width_half - std::abs(d.traj.p[0][0]);
        double vx_max = 2 * d.par.dmax_move * dist2sideline;
        if (d.setpoint.v[0] < -vx_max) d.setpoint.v[0] = -vx_max;
        if (d.setpoint.v[0] >  vx_max) d.setpoint.v[0] =  vx_max;
        // Y direction
        if (d.setpoint.p[1] < -field_length_half) d.setpoint.p[1] = -field_length_half;
        if (d.setpoint.p[1] >  field_length_half) d.setpoint.p[1] =  field_length_half;
        double dist2goalline = field_length_half - std::abs(d.traj.p[0][1]);
        double vy_max = 2 * d.par.dmax_move * dist2goalline;
        if (d.setpoint.v[1] < -vy_max) d.setpoint.v[1] = -vy_max;
        if (d.setpoint.v[1] >  vy_max) d.setpoint.v[1] =  vy_max;
        // Decelerate if velocity is greater than max allowable velocity
        if (d.setpoint.v[0] > vx_max && d.setpoint.p[0] > 0) d.setpoint.a[0] = -d.par.dmax_move;
        if (d.setpoint.v[0] < -vx_max && d.setpoint.p[0] < 0) d.setpoint.a[0] = d.par.dmax_move;
        if (d.setpoint.v[1] > vy_max && d.setpoint.p[1] > 0) d.setpoint.a[1] = -d.par.dmax_move;
        if (d.setpoint.v[1] < -vy_max && d.setpoint.p[1] < 0) d.setpoint.a[1] = d.par.dmax_move;
    } else {
        d.setpoint.p = d.input.robot.p;
        d.setpoint.v = Eigen::Vector3d::Zero();
        d.setpoint.a = Eigen::Vector3d::Zero();
        std::cout << "Set: Robot outside field and no automatic substitution, holding position." << std::endl;
    }
    return d;
}

} // namespace setpoint
} // namespace spg