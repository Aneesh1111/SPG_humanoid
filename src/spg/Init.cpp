#include "spg/Init.hpp"
#include <algorithm>

namespace spg {

SPGState Init(const Eigen::Vector3d& p_initial,
              const Eigen::Vector3d& v_initial,
              int nobstacles,
              int npredict,
              const Eigen::Vector2d& p_initial_ball,
              const Eigen::Vector2d& v_initial_ball,
              int nintercept_positions) {
    SPGState d;
    d.par = FieldParams{
        nobstacles,
        0.02,
        {8, 12},
        1.5,
        {4, 1.5},
        {2, 0.4},
        0.5,
        2.4,
        0.9,
        0.1,
        npredict,
        0.25,
        0.11,
        0.95,
        10,
        6,
        2,
        0.1,
        4,
        13,
        1.8,
        3.5,
        13,
        0.3,
        40.0/180.0*M_PI,
        1.8,
        13,
        nintercept_positions
    };
    d.input.obstacles.p = std::vector<Eigen::Vector2d>(nobstacles, Eigen::Vector2d::Zero());
    d.input.obstacles.v = std::vector<Eigen::Vector2d>(nobstacles, Eigen::Vector2d::Zero());
    d.input.obstacles.r = std::vector<double>(nobstacles, 0.0);
    d.input.obstacles.active = std::vector<bool>(nobstacles, false);
    d.input.robot = RobotInput{p_initial, v_initial, p_initial, 0, 0, 0, 0, 0, Eigen::Vector2d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0, 1.0};
    d.input.ball = BallInput{p_initial_ball, v_initial_ball};
    d.input.subtarget_avoid_polygon = SubtargetAvoidPolygon{Eigen::Matrix<double,2,4>::Zero(), false};
    int nsegments = 4;
    d.aux.segment = std::vector<Segment>(nsegments, Segment{Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()});
    d.traj.p = std::vector<Eigen::Vector3d>(npredict, Eigen::Vector3d::Zero());
    d.traj.v = std::vector<Eigen::Vector3d>(npredict, Eigen::Vector3d::Zero());
    d.traj.a = std::vector<Eigen::Vector3d>(npredict, Eigen::Vector3d::Zero());
    d.traj.t = std::vector<double>(npredict, 0.0);
    d.traj.segment_id = std::vector<Eigen::Vector3i>(npredict, Eigen::Vector3i::Zero());
    d.setpoint.p = p_initial;
    d.setpoint.v = Eigen::Vector3d::Zero();
    d.setpoint.a = Eigen::Vector3d::Zero();
    d.subtarget = Subtarget{p_initial, Eigen::Vector3d::Zero(), Eigen::Vector3d::Ones(), Eigen::Vector3d::Ones(), Eigen::Vector3d::Ones(), 0, false, Eigen::Vector3d::Zero(), 0, Eigen::Vector3i::Zero(), 1e10, 0, std::vector<Segment>(nsegments, Segment{Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()}), 0};
    d.target = Target{p_initial, Eigen::Vector3d::Zero(), 0};
    Subtarget subtarget_template = Subtarget{p_initial, Eigen::Vector3d::Zero(), Eigen::Vector3d::Ones(), Eigen::Vector3d::Ones(), Eigen::Vector3d::Ones(), 0, false, Eigen::Vector3d::Zero(), 0, Eigen::Vector3i::Zero(), 1e10, 0, std::vector<Segment>(nsegments, Segment{Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()}), 0};
    Traject traj_template;
    traj_template.p = std::vector<Eigen::Vector3d>(npredict, Eigen::Vector3d::Zero());
    traj_template.v = std::vector<Eigen::Vector3d>(npredict, Eigen::Vector3d::Zero());
    traj_template.a = std::vector<Eigen::Vector3d>(npredict, Eigen::Vector3d::Zero());
    traj_template.t = std::vector<double>(npredict, 0.0);
    traj_template.segment_id = std::vector<Eigen::Vector3i>(npredict, Eigen::Vector3i::Zero());
    int num_subtargets = 100;
    d.subtarget_array = std::vector<Subtarget>(num_subtargets, subtarget_template);
    d.traj_array = std::vector<Traject>(num_subtargets, traj_template);
    InterceptSample intercept_sample_template{Eigen::Vector3d::Zero(), -1.0};
    d.intercept_positions_etas.sample = std::vector<InterceptSample>(nintercept_positions, intercept_sample_template);
    return d;
}

} // namespace spg
