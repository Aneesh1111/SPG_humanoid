#pragma once

#include <Eigen/Dense>
#include <vector>
#include <memory>

namespace spg {
namespace setpoint {

// ===================== Basic Types =====================

struct MPCState {
    double x;
    double y;
    double phi;  // heading [rad]
};

struct MPCControl {
    double vf;     // forward velocity (body frame) [m/s]
    double vs;     // sideways velocity (body frame) [m/s]
    double omega;  // yaw rate [rad/s]
};

// ===================== "BO-ready" Weight Parameterization =====================
//
// Small set of scalars you can tune (e.g., via Bayesian optimization).
// Internally mapped to diagonal Q, Qf, R, S.

struct MPCWeights {
    // Stage position (x,y)
    double q_pos;     // stage weight for position error
    // Stage heading (phi)
    double q_phi;

    // Terminal position
    double qf_pos;    // terminal weight for position error
    // Terminal heading
    double qf_phi;

    // Control magnitude (on normalized controls)
    double r_vf;
    double r_vs;
    double r_omega;

    // Δu (smoothness) weights (on normalized controls)
    double s_vf;
    double s_vs;
    double s_omega;

    MPCWeights();
};

struct MPCParams {
    double dt;         // time step [s]
    int horizon;       // N steps

    // Robot-specific max physical velocities:
    double vf_max;     // [m/s]
    double vs_max;     // [m/s]
    double omega_max;  // [rad/s]

    // BO-ready weights:
    MPCWeights weights;

    MPCParams();
};

// ===================== QP Solver Interface =====================

class QPSolver {
public:
    virtual ~QPSolver() = default;

    // Solve:
    //   min 0.5 * z^T H z + f^T z
    //   s.t. lb <= z <= ub
    //
    // Returns true on success, and writes solution into z.
    virtual bool solve(const Eigen::MatrixXd& H,
                       const Eigen::VectorXd& f,
                       const Eigen::VectorXd& lb,
                       const Eigen::VectorXd& ub,
                       Eigen::VectorXd& z) = 0;
};

// ===================== qpOASES-based QP Solver =====================

class QPOasesSolver : public QPSolver {
public:
    QPOasesSolver();
    ~QPOasesSolver();

    bool solve(const Eigen::MatrixXd& H,
               const Eigen::VectorXd& f,
               const Eigen::VectorXd& lb,
               const Eigen::VectorXd& ub,
               Eigen::VectorXd& z) override;

private:
    class Impl;
    std::unique_ptr<Impl> pImpl_;
};

// ===================== Humanoid MPC Class =====================

class HumanoidMPC {
public:
    explicit HumanoidMPC(const MPCParams& params, QPSolver* solver);

    // Goal-based MPC (NEW API with measured velocity):
    //  - Input: current state x0, goal state x_goal, measured control u_meas.
    //  - Output: first physical control (vf, vs, omega)
    //  - Returns false if QP solve fails.
    //
    // Uses u_meas for Δu_0 = u_0 - u_meas to ensure smooth transitions.
    bool computeControl(const MPCState& x0,
                        const MPCState& x_goal,
                        const MPCControl& u_meas,
                        MPCControl& u_out);
    
    // Goal-based MPC with full trajectory prediction:
    //   input: current state x0, goal state x_goal, measured control u_meas
    //   output: first control + predicted trajectory (N+1 states, N controls)
    // Returns false if QP solve fails.
    bool computeControlAndTrajectory(const MPCState& x0,
                                      const MPCState& x_goal,
                                      const MPCControl& u_meas,
                                      MPCControl& u_out,
                                      std::vector<MPCState>& predicted_states,
                                      std::vector<MPCControl>& predicted_controls);

private:
    MPCParams params_;
    QPSolver* solver_;
};

} // namespace setpoint
} // namespace spg
