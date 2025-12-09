#include <Eigen/Dense>
#include "spg/setpoint/Set.hpp"
#include "spg/setpoint/GetSegments.hpp"
#include "spg/setpoint/Traj1.hpp"
#include "spg/setpoint/TrajPredict.hpp"
#include "spg/setpoint/ConvertSegment.hpp"
#ifdef HAVE_QPOASES
#include "spg/setpoint/HumanoidMPC.hpp"
#endif
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
        
        // ============================================================
        // Choose controller: HumanoidMPC or traditional MSL setpoint
        // ============================================================
        if (d.par.use_humanoid_mpc) {
#ifdef HAVE_QPOASES
            // -------------------- HUMANOID MPC MODE --------------------
            // Use MPC-based trajectory generation for humanoid robots
            
            // Static MPC instance (created once)
            static QPOasesSolver qp_solver;
            static MPCParams mpc_params;
            static bool mpc_initialized = false;
            
            if (!mpc_initialized) {
                // Initialize MPC parameters
                mpc_params.dt = d.par.Ts;
                mpc_params.horizon = 10;  // 10 steps @ 0.02s = 0.2s horizon
                
                // Physical velocity limits for humanoid
                mpc_params.vf_max = 1.2;      // forward velocity [m/s]
                mpc_params.vs_max = 0.4;      // sideways velocity [m/s]
                mpc_params.omega_max = 1.0;   // yaw rate [rad/s]
                
                // BO-ready cost weights (tunable via Bayesian optimization)
                mpc_params.weights.q_pos = 1.0;       // stage position cost
                mpc_params.weights.q_phi = 0.1;       // stage heading cost
                mpc_params.weights.qf_pos = 8.0;      // terminal position cost
                mpc_params.weights.qf_phi = 1.0;      // terminal heading cost
                mpc_params.weights.r_vf = 0.1;        // forward velocity effort
                mpc_params.weights.r_vs = 0.5;        // sideways velocity effort
                mpc_params.weights.r_omega = 0.2;     // rotation effort
                mpc_params.weights.s_vf = 0.2;        // forward velocity smoothness
                mpc_params.weights.s_vs = 0.8;        // sideways velocity smoothness
                mpc_params.weights.s_omega = 0.4;     // rotation smoothness
                
                mpc_initialized = true;
            }
            static HumanoidMPC mpc(mpc_params, &qp_solver);
            
            // MPC solves in ROBOT LOCAL FRAME, so we need coordinate transformations:
            // 1. Transform goal from global → local frame
            // 2. Solve MPC (always at origin with phi=0 in local frame)
            // 3. Transform result back from local → global frame
            
            double robot_x_global = d.setpoint.p[0];
            double robot_y_global = d.setpoint.p[1];
            double robot_phi_global = d.setpoint.p[2];
            
            double cos_phi = std::cos(robot_phi_global);
            double sin_phi = std::sin(robot_phi_global);
            
            // Transform goal from global frame to robot local frame
            double dx_global = d.subtarget.p[0] - robot_x_global;
            double dy_global = d.subtarget.p[1] - robot_y_global;
            double dphi_global = d.subtarget.p[2] - robot_phi_global;
            
            // Rotate goal position into robot local frame
            double goal_x_local =  dx_global * cos_phi + dy_global * sin_phi;
            double goal_y_local = -dx_global * sin_phi + dy_global * cos_phi;
            double goal_phi_local = dphi_global;  // relative heading
            
            // Normalize goal heading to [-π, π]
            while (goal_phi_local > M_PI) goal_phi_local -= 2.0 * M_PI;
            while (goal_phi_local < -M_PI) goal_phi_local += 2.0 * M_PI;
            
            // In robot local frame, current state is always at origin
            MPCState current_state;
            current_state.x = 0.0;
            current_state.y = 0.0;
            current_state.phi = 0.0;
            
            // Goal in local frame
            MPCState goal_state;
            goal_state.x = goal_x_local;
            goal_state.y = goal_y_local;
            goal_state.phi = goal_phi_local;
            
            // Convert current global-frame velocity to body-frame control (measured velocity)
            MPCControl u_meas;
            u_meas.vf    = d.setpoint.v[0] * cos_phi + d.setpoint.v[1] * sin_phi;   // global → body
            u_meas.vs    = -d.setpoint.v[0] * sin_phi + d.setpoint.v[1] * cos_phi;  // global → body
            u_meas.omega = d.setpoint.v[2];
            
            // Compute optimal control AND get predicted trajectory
            MPCControl u;
            std::vector<MPCState> predicted_states;
            std::vector<MPCControl> predicted_controls;
            bool mpc_success = mpc.computeControlAndTrajectory(current_state, goal_state, u_meas, u, 
                                                                predicted_states, predicted_controls);
            
            if (mpc_success) {
                // MPC returns control in body frame, which is what we need
                // Now transform back to global frame for setpoint update
                
                // u is in body frame: vf = forward, vs = sideways, omega = yaw rate
                // Convert body frame velocity to global frame
                double vx_global = u.vf * cos_phi - u.vs * sin_phi;
                double vy_global = u.vf * sin_phi + u.vs * cos_phi;
                double omega_global = u.omega;
                
                // Update setpoint position in global frame
                d.setpoint.p[0] = robot_x_global + vx_global * d.par.Ts;
                d.setpoint.p[1] = robot_y_global + vy_global * d.par.Ts;
                d.setpoint.p[2] = robot_phi_global + omega_global * d.par.Ts;
                
                // Update setpoint velocity in global frame
                d.setpoint.v[0] = vx_global;
                d.setpoint.v[1] = vy_global;
                d.setpoint.v[2] = omega_global;
                
                // Compute acceleration from velocity change
                // We can now calculate this from predicted_controls[0] vs predicted_controls[1]
                if (predicted_controls.size() > 1) {
                    double dvf = predicted_controls[1].vf - predicted_controls[0].vf;
                    double dvs = predicted_controls[1].vs - predicted_controls[0].vs;
                    double domega = predicted_controls[1].omega - predicted_controls[0].omega;
                    
                    // Convert body-frame acceleration to world frame
                    double ax_body = dvf / d.par.Ts;
                    double ay_body = dvs / d.par.Ts;
                    d.setpoint.a[0] = ax_body * cos_phi - ay_body * sin_phi;
                    d.setpoint.a[1] = ax_body * sin_phi + ay_body * cos_phi;
                    d.setpoint.a[2] = domega / d.par.Ts;
                } else {
                    d.setpoint.a = Eigen::Vector3d::Zero();
                }
                
                // Use MPC predicted trajectory for visualization!
                // predicted_states are in ROBOT LOCAL FRAME, need to transform to GLOBAL FRAME
                const int N_mpc = static_cast<int>(predicted_states.size());
                const int N_traj = static_cast<int>(d.traj.p.size());
                
                // Fill trajectory with MPC predictions (transform local → global)
                for (int i = 0; i < std::min(N_mpc, N_traj); ++i) {
                    // Transform position from robot local frame to global frame
                    double x_local = predicted_states[i].x;
                    double y_local = predicted_states[i].y;
                    double phi_local = predicted_states[i].phi;
                    
                    // Rotate position back to global frame
                    d.traj.p[i][0] = robot_x_global + (x_local * cos_phi - y_local * sin_phi);
                    d.traj.p[i][1] = robot_y_global + (x_local * sin_phi + y_local * cos_phi);
                    d.traj.p[i][2] = robot_phi_global + phi_local;
                    d.traj.t[i] = i * d.par.Ts;
                    
                    if (i < static_cast<int>(predicted_controls.size())) {
                        // Controls are in body frame, transform to global frame
                        // Use the global heading at this timestep for transformation
                        double phi_global_i = d.traj.p[i][2];
                        double cos_phi_i = std::cos(phi_global_i);
                        double sin_phi_i = std::sin(phi_global_i);
                        
                        d.traj.v[i][0] = predicted_controls[i].vf * cos_phi_i - predicted_controls[i].vs * sin_phi_i;
                        d.traj.v[i][1] = predicted_controls[i].vf * sin_phi_i + predicted_controls[i].vs * cos_phi_i;
                        d.traj.v[i][2] = predicted_controls[i].omega;
                    } else {
                        d.traj.v[i] = Eigen::Vector3d::Zero();
                    }
                    
                    // Acceleration approximation between steps
                    if (i > 0 && i < static_cast<int>(predicted_controls.size())) {
                        d.traj.a[i] = (d.traj.v[i] - d.traj.v[i-1]) / d.par.Ts;
                    } else {
                        d.traj.a[i] = Eigen::Vector3d::Zero();
                    }
                }
                
                // If MPC horizon is shorter than visualization needs, extrapolate with last control
                if (N_mpc < N_traj && N_mpc > 0) {
                    Eigen::Vector3d last_p = d.traj.p[N_mpc - 1];
                    Eigen::Vector3d last_v = d.traj.v[N_mpc - 1];
                    
                    for (int i = N_mpc; i < N_traj; ++i) {
                        double dt = (i - N_mpc + 1) * d.par.Ts_predict;
                        d.traj.p[i][0] = last_p[0] + last_v[0] * dt;
                        d.traj.p[i][1] = last_p[1] + last_v[1] * dt;
                        d.traj.p[i][2] = last_p[2] + last_v[2] * dt;
                        d.traj.v[i] = last_v;
                        d.traj.a[i] = Eigen::Vector3d::Zero();
                        d.traj.t[i] = d.traj.t[N_mpc - 1] + dt;
                    }
                }
            } else {
                std::cout << "Set: HumanoidMPC failed, holding position." << std::endl;
                d.setpoint.v = Eigen::Vector3d::Zero();
                d.setpoint.a = Eigen::Vector3d::Zero();
            }
#else
            std::cerr << "Set: HumanoidMPC requested but qpOASES not available! Falling back to MSL mode." << std::endl;
            d.par.use_humanoid_mpc = false;  // Disable for future calls
#endif
        }
        
        if (!d.par.use_humanoid_mpc) {
            // -------------------- TRADITIONAL MSL MODE --------------------
            // Use traditional segment-based trajectory generation
            
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
        }

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