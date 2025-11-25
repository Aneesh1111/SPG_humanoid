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
    
    // Initialize field parameters
    d.par = FieldParams{
        nobstacles,                    // number of obstacles
        0.02,                         // Ts: sampling time [s]
        {8, 12},                      // field_size: [width, length] [m] 
        1.5,                          // field_circle_radius [m]
        {4, 1.5},                     // field_penalty_area: [width, length] [m]
        {2, 0.4},                     // field_goal_area: [width, length] [m]
        0.5,                          // field_border_margin [m]
        2.4,                          // goalwidth [m]
        0.9,                          // technical_area_width [m]
        0.1,                          // Ts_predict: prediction sampling time [s]
        npredict,                     // npredict: number of prediction samples
        0.25,                         // robot_radius [m]
        0.11,                         // ball_radius [m]
        0.95,                         // obstacle_vel_gain: velocity decay factor
        10,                           // nattempts_replan: max replanning attempts
        6,                            // search_distance [m]
        2,                            // replan_uphill_distance [m]
        0.1,                          // margin_replan [m]
        4,                            // vmax_move: max translational velocity [m/s] (legacy)
        3,                            // vmax_rotate: max rotational velocity [rad/s]
        1.8,                          // amax_move: max translational acceleration [m/s²] (legacy)
        3.5,                          // amax_quickstop: max quickstop acceleration [m/s²]
        2,                            // amax_rotate: max rotational acceleration [rad/s²]
        
        // Humanoid-specific directional parameters
        1.5,                          // vmax_move_x: max sideways velocity [m/s] (left/right)
        4.0,                          // vmax_move_y: max forward/backward velocity [m/s] (front/back)
        1.2,                          // amax_move_x: max sideways acceleration [m/s²] (left/right)
        2.8,                          // amax_move_y: max forward/backward acceleration [m/s²] (front/back)
        
        // Orientation-dependent efficiency factors
        1.0,                          // forward_efficiency: 100% when moving forward/backward
        0.3,                          // sideways_efficiency: 30% when moving sideways
        0.0,                          // rotation_while_moving_penalty: 0% penalty for rotating while moving
        0.3,                          // scale_rotate: rotation scaling factor
        40.0/180.0*M_PI,             // scale_angle: angle scaling factor [rad]
        1.8,                          // dmax_move: max translational deceleration [m/s²]
        2.0,                          // dmax_rotate: max rotational deceleration [rad/s²]
        nintercept_positions          // number of intercept positions
    };
    // Initialize obstacle input arrays
    d.input.obstacles.p = std::vector<Eigen::Vector2d>(nobstacles, Eigen::Vector2d::Zero());      // positions [m]
    d.input.obstacles.v = std::vector<Eigen::Vector2d>(nobstacles, Eigen::Vector2d::Zero());      // velocities [m/s]
    d.input.obstacles.r = std::vector<double>(nobstacles, 0.0);                                   // radii [m]
    d.input.obstacles.active = std::vector<bool>(nobstacles, false);                              // active flags
    
    // Initialize robot input state
    d.input.robot = RobotInput{
        p_initial,                    // p: initial position [m]
        v_initial,                    // v: initial velocity [m/s]
        p_initial,                    // target: target position [m]
        0,                            // skillID: current skill identifier
        0,                            // CPPA: closest point to penalty area
        0,                            // CPBteam: closest point to ball (team)
        0,                            // reset_trigger: reset flag
        0,                            // quickstop_trigger: quickstop flag
        Eigen::Vector2d::Zero(),      // cpb_poi_xy: closest point to ball (point of interest)
        Eigen::Vector3d::Zero(),      // target_vel: target velocity [m/s]
        Eigen::Vector3d::Zero(),      // IMU_orientation: IMU orientation [rad]
        0,                            // human_dribble_flag: human dribble mode
        1.0                           // dist2ball_vs_opp: distance to ball vs opponent
    };
    
    // Initialize ball input state
    d.input.ball = BallInput{p_initial_ball, v_initial_ball};                                     // position and velocity [m, m/s]
    
    // Initialize subtarget avoid polygon (disabled by default)
    d.input.subtarget_avoid_polygon = SubtargetAvoidPolygon{Eigen::Matrix<double,2,4>::Zero(), false};
    // Initialize auxiliary trajectory segments (acceleration, constant velocity, deceleration, stop)
    int nsegments = 4;
    d.aux.segment = std::vector<Segment>(nsegments, Segment{
        Eigen::Vector3d::Zero(),      // dt: time duration [s]
        Eigen::Vector3d::Zero(),      // t: time stamps [s]
        Eigen::Vector3d::Zero(),      // p: position [m]
        Eigen::Vector3d::Zero(),      // v: velocity [m/s]
        Eigen::Vector3d::Zero()       // a: acceleration [m/s²]
    });
    
    // Initialize predicted trajectory arrays
    d.traj.p = std::vector<Eigen::Vector3d>(npredict, Eigen::Vector3d::Zero());                   // positions [m]
    d.traj.v = std::vector<Eigen::Vector3d>(npredict, Eigen::Vector3d::Zero());                   // velocities [m/s]
    d.traj.a = std::vector<Eigen::Vector3d>(npredict, Eigen::Vector3d::Zero());                   // accelerations [m/s²]
    d.traj.t = std::vector<double>(npredict, 0.0);                                                // time stamps [s]
    d.traj.segment_id = std::vector<Eigen::Vector3i>(npredict, Eigen::Vector3i::Zero());          // segment identifiers
    // Initialize current setpoint state
    d.setpoint.p = p_initial;                                                                     // position [m]
    d.setpoint.v = Eigen::Vector3d::Zero();                                                       // velocity [m/s]
    d.setpoint.a = Eigen::Vector3d::Zero();                                                       // acceleration [m/s²]
    
    // Initialize subtarget (intermediate waypoint for obstacle avoidance)
    d.subtarget = Subtarget{
        p_initial,                    // p: position [m]
        Eigen::Vector3d::Zero(),      // v: velocity [m/s]
        Eigen::Vector3d::Ones(),      // vmax: maximum velocity [m/s]
        Eigen::Vector3d::Ones(),      // amax: maximum acceleration [m/s²]
        Eigen::Vector3d::Ones(),      // dmax: maximum deceleration [m/s²]
        0,                            // action: action type (0=stop, 1=target, 2=subtarget)
        false,                        // collisionfree: collision-free flag
        Eigen::Vector3d::Zero(),      // target: target position [m]
        0,                            // eta: estimated time of arrival [s]
        Eigen::Vector3i::Zero(),      // segment_id: current trajectory segment
        1e10,                         // age: age of subtarget [steps]
        0,                            // violation_count: number of violations
        std::vector<Segment>(nsegments, Segment{
            Eigen::Vector3d::Zero(),  // trajectory segments
            Eigen::Vector3d::Zero(),
            Eigen::Vector3d::Zero(),
            Eigen::Vector3d::Zero(),
            Eigen::Vector3d::Zero()
        }),
        0                             // automatic_substitution_flag: substitution flag
    };
    
    // Initialize target (final destination)
    d.target = Target{
        p_initial,                    // p: position [m]
        Eigen::Vector3d::Zero(),      // v: velocity [m/s]
        0                             // eta: estimated time of arrival [s]
    };
    // Create subtarget template for replanning array
    Subtarget subtarget_template = Subtarget{
        p_initial,                    // template based on initial position
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Ones(),
        Eigen::Vector3d::Ones(),
        Eigen::Vector3d::Ones(),
        0, false,
        Eigen::Vector3d::Zero(),
        0,
        Eigen::Vector3i::Zero(),
        1e10, 0,
        std::vector<Segment>(nsegments, Segment{
            Eigen::Vector3d::Zero(),
            Eigen::Vector3d::Zero(),
            Eigen::Vector3d::Zero(),
            Eigen::Vector3d::Zero(),
            Eigen::Vector3d::Zero()
        }), 0
    };
    
    // Create trajectory template for replanning array
    Traject traj_template;
    traj_template.p = std::vector<Eigen::Vector3d>(npredict, Eigen::Vector3d::Zero());            // positions [m]
    traj_template.v = std::vector<Eigen::Vector3d>(npredict, Eigen::Vector3d::Zero());            // velocities [m/s]
    traj_template.a = std::vector<Eigen::Vector3d>(npredict, Eigen::Vector3d::Zero());            // accelerations [m/s²]
    traj_template.t = std::vector<double>(npredict, 0.0);                                         // time stamps [s]
    traj_template.segment_id = std::vector<Eigen::Vector3i>(npredict, Eigen::Vector3i::Zero());   // segment IDs
    
    // Initialize replanning arrays (for multiple path options)
    int num_subtargets = 100;                                                                     // maximum number of subtarget candidates
    d.subtarget_array = std::vector<Subtarget>(num_subtargets, subtarget_template);               // subtarget candidates array
    d.traj_array = std::vector<Traject>(num_subtargets, traj_template);                          // corresponding trajectories array
    
    // Initialize ball intercept positions array
    InterceptSample intercept_sample_template{Eigen::Vector3d::Zero(), -1.0};                     // position and eta template
    d.intercept_positions_etas.sample = std::vector<InterceptSample>(nintercept_positions, intercept_sample_template);
    return d;
}

} // namespace spg
