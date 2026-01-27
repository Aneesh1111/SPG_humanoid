// HumanoidMPC.cpp
//
// Goal-based MPC for a point-mass SE(2) robot, solved entirely in a
// robot-centric frame (local frame attached to current robot pose).
//
// Features:
//  - State x = [x, y, phi] in *robot frame*:
//      x: forward displacement
//      y: lateral displacement
//      phi: yaw (heading) relative to current orientation
//  - Goal x_goal also in robot frame.
//  - Normalized body-frame controls u_tilde = [vf_norm, vs_norm, omega_norm] ∈ [-1,1]^3.
//  - Physical limits: vf_max, vs_max, omega_max, via u_phys = D * u_tilde.
//  - Cost on distance-to-goal, control magnitude, and Δu (change in control) for smoothness.
//  - Δu_0 = u_0 - u_meas_norm uses measured body velocity to avoid jumps.
//  - Compact "BO-ready" weight parameterization.
//
// Dependencies:
//   - Eigen (https://eigen.tuxfamily.org/)
//   - qpOASES (https://github.com/coin-or/qpOASES)

#include "spg/setpoint/HumanoidMPC.hpp"
#include <iostream>
#include <cmath>
#include <stdexcept>

// qpOASES
#include <qpOASES.hpp>
USING_NAMESPACE_QPOASES

namespace spg {
namespace setpoint {

// ===================== MPCWeights Implementation =====================

MPCWeights::MPCWeights()
    : q_pos(1.0),
      q_phi(0.1),
      qf_pos(8.0),
      qf_phi(1.0),
      r_vf(0.1),
      r_vs(0.5),
      r_omega(0.2),
      s_vf(0.2),
      s_vs(0.8),
      s_omega(0.4) {}

// ===================== MPCParams Implementation =====================

MPCParams::MPCParams()
    : dt(0.1),
      horizon(10),
      vf_max(1.0),
      vs_max(0.3),
      omega_max(1.0),
      coupling_factor(0.5),  // Moderate coupling by default
      weights() {}

// ===================== qpOASES Solver Implementation =====================

class QPOasesSolver::Impl {
public:
    QProblem qp_;
    bool initialized_;
    int nV_;
    returnValue returnCode_;

    Impl() : qp_(0, 0), initialized_(false), nV_(0), returnCode_(SUCCESSFUL_RETURN) {
        Options options;
        options.setToReliable();
        qp_.setOptions(options);
    }
};

QPOasesSolver::QPOasesSolver() : pImpl_(std::make_unique<Impl>()) {}

QPOasesSolver::~QPOasesSolver() = default;

bool QPOasesSolver::solve(const Eigen::MatrixXd& H,
                           const Eigen::VectorXd& f,
                           const Eigen::VectorXd& lb,
                           const Eigen::VectorXd& ub,
                           Eigen::VectorXd& z)
{
    const int nV = static_cast<int>(H.rows());
    if (H.cols() != nV) {
        std::cerr << "QPOasesSolver: H must be square." << std::endl;
        return false;
    }
    if (f.size() != nV || lb.size() != nV || ub.size() != nV) {
        std::cerr << "QPOasesSolver: dimension mismatch in variables." << std::endl;
        return false;
    }

    const int nC = 0;
    real_t* H_qp   = const_cast<real_t*>(H.data());
    real_t* g_qp   = const_cast<real_t*>(f.data());
    real_t* lb_qp  = const_cast<real_t*>(lb.data());
    real_t* ub_qp  = const_cast<real_t*>(ub.data());
    real_t* A_qp   = nullptr;
    real_t* lbA_qp = nullptr;
    real_t* ubA_qp = nullptr;

    if (!pImpl_->initialized_ || pImpl_->nV_ != nV) {
        pImpl_->nV_ = nV;
        pImpl_->qp_ = QProblem(nV, nC);

        int_t nWSR = 50;
        pImpl_->returnCode_ = pImpl_->qp_.init(H_qp, g_qp,
                               A_qp, lb_qp, ub_qp,
                               lbA_qp, ubA_qp,
                               nWSR);
        if (pImpl_->returnCode_ != SUCCESSFUL_RETURN) {
            std::cerr << "qpOASES init failed with code "
                      << pImpl_->returnCode_ << std::endl;
            pImpl_->initialized_ = false;
            return false;
        }
        pImpl_->initialized_ = true;
    } else {
        int_t nWSR = 20;
        pImpl_->returnCode_ = pImpl_->qp_.hotstart(g_qp, lb_qp, ub_qp,
                                   lbA_qp, ubA_qp,
                                   nWSR);
        if (pImpl_->returnCode_ != SUCCESSFUL_RETURN) {
            std::cerr << "qpOASES hotstart failed with code "
                      << pImpl_->returnCode_ << ", retrying init."
                      << std::endl;

            pImpl_->qp_ = QProblem(nV, nC);
            int_t nWSR2 = 50;
            pImpl_->returnCode_ = pImpl_->qp_.init(H_qp, g_qp,
                                   A_qp, lb_qp, ub_qp,
                                   lbA_qp, ubA_qp,
                                   nWSR2);
            if (pImpl_->returnCode_ != SUCCESSFUL_RETURN) {
                std::cerr << "qpOASES init failed with code "
                          << pImpl_->returnCode_ << std::endl;
                pImpl_->initialized_ = false;
                return false;
            }
        }
    }

    z.resize(nV);
    real_t* z_qp = z.data();
    pImpl_->returnCode_ = pImpl_->qp_.getPrimalSolution(z_qp);
    if (pImpl_->returnCode_ != SUCCESSFUL_RETURN) {
        std::cerr << "qpOASES getPrimalSolution failed with code "
                  << pImpl_->returnCode_ << std::endl;
        return false;
    }

    return true;
}

// ===================== Humanoid MPC Implementation =====================

HumanoidMPC::HumanoidMPC(const MPCParams& params, QPSolver* solver)
    : params_(params), solver_(solver), matrices_precomputed_(false)
{
    if (!solver_) {
        throw std::runtime_error("QPSolver pointer is null.");
    }
    
    // Precompute constant matrices (assuming phi0 = 0 in robot frame)
    precomputeMatrices();
}

// Precompute matrices that don't change (assumes phi0 = 0)
void HumanoidMPC::precomputeMatrices() {
    const int N = params_.horizon;
    constexpr int nx = 3;
    constexpr int nu = 3;
    const int Nu = N * nu;
    
    // Normalization diagonal
    Eigen::Matrix3d D = Eigen::Matrix3d::Zero();
    D(0, 0) = params_.vf_max;
    D(1, 1) = params_.vs_max;
    D(2, 2) = params_.omega_max;
    
    // B matrix at phi0 = 0: cos(0)=1, sin(0)=0
    Eigen::Matrix<double, nx, 3> B;
    B.setZero();
    B(0, 0) = params_.dt;  // vf affects x
    B(0, 1) = 0.0;         // vs doesn't affect x at phi=0
    B(1, 0) = 0.0;         // vf doesn't affect y at phi=0
    B(1, 1) = params_.dt;  // vs affects y
    B(2, 2) = params_.dt;  // omega affects phi
    
    Eigen::Matrix<double, nx, nu> B_norm = B * D;
    
    // Build Su, Sx
    Su_precomp_ = Eigen::MatrixXd::Zero(nx * N, Nu);
    Sx_precomp_ = Eigen::MatrixXd::Zero(nx * N, nx);
    
    for (int k = 0; k < N; ++k) {
        int row_start = k * nx;
        Sx_precomp_.block(row_start, 0, nx, nx) = Eigen::Matrix3d::Identity();
        
        for (int j = 0; j <= k; ++j) {
            int col_start = j * nu;
            Su_precomp_.block(row_start, col_start, nx, nu) += B_norm;
        }
    }
    
    // Build weight matrices
    const MPCWeights& w = params_.weights;
    
    Eigen::Vector3d Q_diag(w.q_pos, w.q_pos, w.q_phi);
    Eigen::Vector3d Qf_diag(w.qf_pos, w.qf_pos, w.qf_phi);
    Eigen::Vector3d R_diag(w.r_vf, w.r_vs, w.r_omega);
    Eigen::Vector3d S_diag(w.s_vf, w.s_vs, w.s_omega);
    
    Eigen::Matrix3d Q  = Q_diag.asDiagonal();
    Eigen::Matrix3d Qf = Qf_diag.asDiagonal();
    Eigen::Matrix3d R  = R_diag.asDiagonal();
    Eigen::Matrix3d S  = S_diag.asDiagonal();
    
    // Qbar
    Eigen::MatrixXd Qbar = Eigen::MatrixXd::Zero(nx * N, nx * N);
    for (int k = 0; k < N; ++k) {
        Eigen::Matrix3d Qk = (k == N - 1) ? Qf : Q;
        Qbar.block<3,3>(k * nx, k * nx) = Qk;
    }
    
    // Rbar
    Eigen::MatrixXd Rbar = Eigen::MatrixXd::Zero(Nu, Nu);
    for (int k = 0; k < N; ++k) {
        Rbar.block<3,3>(k * nu, k * nu) = R;
    }
    
    // L matrix for Δu
    Eigen::MatrixXd L = Eigen::MatrixXd::Zero(Nu, Nu);
    for (int k = 0; k < N; ++k) {
        int row_block = k * nu;
        L.block(row_block, row_block, nu, nu).setIdentity();
        if (k > 0) {
            L.block(row_block, (k-1) * nu, nu, nu) -= Eigen::Matrix3d::Identity();
        }
    }
    
    // Sbar
    Eigen::MatrixXd Sbar = Eigen::MatrixXd::Zero(Nu, Nu);
    for (int k = 0; k < N; ++k) {
        Sbar.block<3,3>(k * nu, k * nu) = S;
    }
    
    // Precompute expensive matrix products
    SuT_Qbar_Su_precomp_ = Su_precomp_.transpose() * Qbar * Su_precomp_;
    LT_Sbar_L_precomp_ = L.transpose() * Sbar * L;
    SuT_Qbar_precomp_ = Su_precomp_.transpose() * Qbar;
    LT_Sbar_precomp_ = L.transpose() * Sbar;
    
    // Final Hessian (constant part)
    H_precomp_ = 2.0 * (SuT_Qbar_Su_precomp_ + Rbar + 2.0 * LT_Sbar_L_precomp_);
    
    matrices_precomputed_ = true;
}

// Goal-based MPC in robot frame:
//  - x0:   current robot-frame state (usually (0,0,0) each cycle).
//  - goal: goal in robot frame (forward, lateral, heading).
//  - u_meas: measured body-frame velocity (vf, vs, omega).
bool HumanoidMPC::computeControl(const MPCState& x0,
                                 const MPCState& goal,
                                 const MPCControl& u_meas,
                                 MPCControl& u_out)
{
    if (!matrices_precomputed_) {
        std::cerr << "HumanoidMPC: matrices not precomputed!\n";
        return false;
    }
    
    const int N = params_.horizon;
    constexpr int nx = 3;
    constexpr int nu = 3;
    const int Nu = N * nu;

    // Warn if phi0 assumption violated
    if (std::abs(x0.phi) > 0.1) {
        std::cerr << "Warning: x0.phi=" << x0.phi 
                  << " rad (expected ~0 in robot frame)\n";
    }

    Eigen::Vector3d x0_vec(x0.x, x0.y, x0.phi);
    Eigen::Vector3d xg_vec(goal.x, goal.y, goal.phi);

    // Goal stack: same robot-frame goal at each stage
    Eigen::VectorXd xg_stack(nx * N);
    for (int k = 0; k < N; ++k) {
        xg_stack.segment<3>(k * nx) = xg_vec;
    }

    // Measured control normalization (robot frame)
    Eigen::Vector3d u_meas_norm(0.0, 0.0, 0.0);
    if (params_.vf_max    > 1e-9) u_meas_norm(0) = u_meas.vf    / params_.vf_max;
    if (params_.vs_max    > 1e-9) u_meas_norm(1) = u_meas.vs    / params_.vs_max;
    if (params_.omega_max > 1e-9) u_meas_norm(2) = u_meas.omega / params_.omega_max;

    for (int i = 0; i < 3; ++i) {
        if (u_meas_norm(i) >  1.0) u_meas_norm(i) =  1.0;
        if (u_meas_norm(i) < -1.0) u_meas_norm(i) = -1.0;
    }

    // l0 encodes -u_meas_norm for k=0, 0 otherwise
    Eigen::VectorXd l0 = Eigen::VectorXd::Zero(Nu);
    l0.segment<3>(0) = -u_meas_norm;

    // State error part (using precomputed matrices)
    Eigen::VectorXd X0_stack = Sx_precomp_ * x0_vec;
    Eigen::VectorXd diff = X0_stack - xg_stack;

    // Compute f using precomputed products
    Eigen::VectorXd f = 2.0 * (SuT_Qbar_precomp_ * diff);
    Eigen::VectorXd f_du = 2.0 * (LT_Sbar_precomp_ * l0);
    f += f_du;

    // Bounds on normalized controls: each ∈ [-1, 1]
    Eigen::VectorXd lb = Eigen::VectorXd::Constant(Nu, -1.0);
    Eigen::VectorXd ub = Eigen::VectorXd::Constant(Nu,  1.0);
    
    // Turn-translation coupling constraint:
    // Reduce translation speed when turning
    // vf_max_effective = vf_max * (1 - coupling_factor * |omega_norm|)
    // vs_max_effective = vs_max * (1 - coupling_factor * |omega_norm|)
    //
    // For each timestep k, we modify bounds based on coupling
    if (params_.coupling_factor > 1e-6) {
        // Note: We can't apply perfect coupling without general constraints,
        // but we can approximate by reducing velocity bounds based on expected turn rate
        // This is a conservative approximation
        
        // Estimate average turn rate from goal heading
        double goal_heading_error = std::abs(goal.phi);
        double estimated_omega_norm = std::min(1.0, goal_heading_error / (params_.dt * N));
        double speed_reduction = 1.0 - params_.coupling_factor * estimated_omega_norm;
        speed_reduction = std::max(0.1, speed_reduction);  // Keep at least 10% speed
        
        // Apply to vf and vs bounds for all timesteps
        for (int k = 0; k < N; ++k) {
            int vf_idx = k * nu + 0;
            int vs_idx = k * nu + 1;
            lb(vf_idx) = -speed_reduction;
            ub(vf_idx) = speed_reduction;
            lb(vs_idx) = -speed_reduction;
            ub(vs_idx) = speed_reduction;
        }
    }

    // Solve using precomputed H
    Eigen::VectorXd U_tilde(Nu);
    bool ok = solver_->solve(H_precomp_, f, lb, ub, U_tilde);
    if (!ok) {
        std::cerr << "HumanoidMPC (robot frame): QP solve failed.\n";
        return false;
    }

    // First normalized control -> physical (robot-frame command)
    double vf_norm    = U_tilde(0);
    double vs_norm    = U_tilde(1);
    double omega_norm = U_tilde(2);

    u_out.vf    = params_.vf_max    * vf_norm;
    u_out.vs    = params_.vs_max    * vs_norm;
    u_out.omega = params_.omega_max * omega_norm;

    return true;
}

// Goal-based MPC with full trajectory prediction
bool HumanoidMPC::computeControlAndTrajectory(const MPCState& x0,
                                               const MPCState& goal,
                                               const MPCControl& u_meas,
                                               MPCControl& u_out,
                                               std::vector<MPCState>& predicted_states,
                                               std::vector<MPCControl>& predicted_controls)
{
    if (!matrices_precomputed_) {
        std::cerr << "HumanoidMPC: matrices not precomputed!\n";
        return false;
    }
    
    const int N = params_.horizon;
    constexpr int nx = 3;
    constexpr int nu = 3;
    const int Nu = N * nu;

    Eigen::Vector3d x0_vec(x0.x, x0.y, x0.phi);
    Eigen::Vector3d xg_vec(goal.x, goal.y, goal.phi);

    // Goal stack: same robot-frame goal at each stage
    Eigen::VectorXd xg_stack(nx * N);
    for (int k = 0; k < N; ++k) {
        xg_stack.segment<3>(k * nx) = xg_vec;
    }

    // Measured control normalization (robot frame)
    Eigen::Vector3d u_meas_norm(0.0, 0.0, 0.0);
    if (params_.vf_max    > 1e-9) u_meas_norm(0) = u_meas.vf    / params_.vf_max;
    if (params_.vs_max    > 1e-9) u_meas_norm(1) = u_meas.vs    / params_.vs_max;
    if (params_.omega_max > 1e-9) u_meas_norm(2) = u_meas.omega / params_.omega_max;

    for (int i = 0; i < 3; ++i) {
        if (u_meas_norm(i) >  1.0) u_meas_norm(i) =  1.0;
        if (u_meas_norm(i) < -1.0) u_meas_norm(i) = -1.0;
    }

    // l0 encodes -u_meas_norm for k=0, 0 otherwise
    Eigen::VectorXd l0 = Eigen::VectorXd::Zero(Nu);
    l0.segment<3>(0) = -u_meas_norm;

    // State error part (using precomputed matrices)
    Eigen::VectorXd X0_stack = Sx_precomp_ * x0_vec;
    Eigen::VectorXd diff = X0_stack - xg_stack;

    // Compute f using precomputed products
    Eigen::VectorXd f = 2.0 * (SuT_Qbar_precomp_ * diff);
    Eigen::VectorXd f_du = 2.0 * (LT_Sbar_precomp_ * l0);
    f += f_du;

    // Bounds on normalized controls: each ∈ [-1, 1]
    Eigen::VectorXd lb = Eigen::VectorXd::Constant(Nu, -1.0);
    Eigen::VectorXd ub = Eigen::VectorXd::Constant(Nu,  1.0);
    
    // Turn-translation coupling constraint (same as computeControl)
    if (params_.coupling_factor > 1e-6) {
        double goal_heading_error = std::abs(goal.phi);
        double estimated_omega_norm = std::min(1.0, goal_heading_error / (params_.dt * N));
        double speed_reduction = 1.0 - params_.coupling_factor * estimated_omega_norm;
        speed_reduction = std::max(0.1, speed_reduction);
        
        for (int k = 0; k < N; ++k) {
            int vf_idx = k * nu + 0;
            int vs_idx = k * nu + 1;
            lb(vf_idx) = -speed_reduction;
            ub(vf_idx) = speed_reduction;
            lb(vs_idx) = -speed_reduction;
            ub(vs_idx) = speed_reduction;
        }
    }

    // Solve using precomputed H
    Eigen::VectorXd U_tilde(Nu);
    bool ok = solver_->solve(H_precomp_, f, lb, ub, U_tilde);
    if (!ok) {
        std::cerr << "HumanoidMPC (robot frame): QP solve failed.\n";
        return false;
    }

    // First normalized control -> physical (robot-frame command)
    double vf_norm    = U_tilde(0);
    double vs_norm    = U_tilde(1);
    double omega_norm = U_tilde(2);

    u_out.vf    = params_.vf_max    * vf_norm;
    u_out.vs    = params_.vs_max    * vs_norm;
    u_out.omega = params_.omega_max * omega_norm;

    // Forward simulate trajectory using the full control sequence
    predicted_states.clear();
    predicted_controls.clear();
    
    predicted_states.reserve(N + 1);
    predicted_controls.reserve(N);
    
    // Initial state
    MPCState current;
    current.x = x0.x;
    current.y = x0.y;
    current.phi = x0.phi;
    predicted_states.push_back(current);
    
    // Forward simulation
    for (int k = 0; k < N; ++k) {
        // Extract control for this timestep
        double vf_k_norm = U_tilde(k * nu + 0);
        double vs_k_norm = U_tilde(k * nu + 1);
        double omega_k_norm = U_tilde(k * nu + 2);
        
        MPCControl u_k;
        u_k.vf = params_.vf_max * vf_k_norm;
        u_k.vs = params_.vs_max * vs_k_norm;
        u_k.omega = params_.omega_max * omega_k_norm;
        predicted_controls.push_back(u_k);
        
        // Propagate dynamics (robot frame kinematics)
        double c_k = std::cos(current.phi);
        double s_k = std::sin(current.phi);
        
        MPCState next;
        next.x = current.x + params_.dt * (u_k.vf * c_k - u_k.vs * s_k);
        next.y = current.y + params_.dt * (u_k.vf * s_k + u_k.vs * c_k);
        next.phi = current.phi + params_.dt * u_k.omega;
        
        predicted_states.push_back(next);
        current = next;
    }

    return true;
}

} // namespace setpoint
} // namespace spg
