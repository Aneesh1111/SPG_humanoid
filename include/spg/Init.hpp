#pragma once
#include <Eigen/Dense>
#include <vector>
#include <array>

namespace spg {

struct FieldParams {
    int nobstacles;
    double Ts;
    std::array<double,2> field_size;
    double field_circle_radius;
    std::array<double,2> field_penalty_area;
    std::array<double,2> field_goal_area;
    double field_border_margin;
    double goalwidth;
    double technical_area_width;
    double Ts_predict;
    int npredict;
    double robot_radius;
    double ball_radius;
    double obstacle_vel_gain;
    int nattempts_replan;
    double search_distance;
    double replan_uphill_distance;
    double margin_replan;
    double vmax_move;
    double vmax_rotate;
    double amax_move;
    double amax_quickstop;
    double amax_rotate;
    double scale_rotate;
    double scale_angle;
    double dmax_move;
    double dmax_rotate;
    int nintercept_positions;
};

struct ObstacleInput {
    std::vector<Eigen::Vector2d> p;
    std::vector<Eigen::Vector2d> v;
    std::vector<double> r;
    std::vector<bool> active;
};

struct RobotInput {
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    Eigen::Vector3d target;
    int skillID;
    double CPPA;
    double CPBteam;
    double reset_trigger;
    double quickstop_trigger;
    Eigen::Vector2d cpb_poi_xy;
    Eigen::Vector3d target_vel;
    Eigen::Vector3d IMU_orientation;
    int human_dribble_flag;
    double dist2ball_vs_opp;
};

struct BallInput {
    Eigen::Vector2d p;
    Eigen::Vector2d v;
};

struct SubtargetAvoidPolygon {
    Eigen::Matrix<double,2,4> polygon;
    bool valid;
};

struct Segment {
    Eigen::Vector3d dt, t, p, v, a;
};

struct Traject {
    std::vector<Eigen::Vector3d> p, v, a;
    std::vector<double> t;
    std::vector<Eigen::Vector3i> segment_id;
};

struct Subtarget {
    Eigen::Vector3d p, v, vmax, amax, dmax;
    int action;
    bool collisionfree;
    Eigen::Vector3d target;
    double eta;
    Eigen::Vector3i segment_id;
    double age;
    int violation_count;
    std::vector<Segment> segment;
    int automatic_substitution_flag;
};

struct Target {
    Eigen::Vector3d p, v;
    double eta;
};

struct InterceptSample {
    Eigen::Vector3d p;
    double eta;
};

struct InterceptPositionsEtas {
    std::vector<InterceptSample> sample;
};

struct SPGState {
    FieldParams par;
    struct {
        ObstacleInput obstacles;
        RobotInput robot;
        BallInput ball;
        SubtargetAvoidPolygon subtarget_avoid_polygon;
    } input;
    struct {
        std::vector<Segment> segment;
    } aux;
    Traject traj;
    struct {
        Eigen::Vector3d p, v, a;
    } setpoint;
    Subtarget subtarget;
    Target target;
    std::vector<Subtarget> subtarget_array;
    std::vector<Traject> traj_array;
    InterceptPositionsEtas intercept_positions_etas;
};

SPGState Init(const Eigen::Vector3d& p_initial,
              const Eigen::Vector3d& v_initial,
              int nobstacles,
              int npredict,
              const Eigen::Vector2d& p_initial_ball,
              const Eigen::Vector2d& v_initial_ball,
              int nintercept_positions);

} // namespace spg
