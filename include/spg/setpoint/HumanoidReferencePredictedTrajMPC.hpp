#pragma once

#include <array>
#include <vector>
#include <memory>

namespace spg {
namespace setpoint {

// Forward declaration for acados solver capsule (from global namespace)
} // namespace setpoint
} // namespace spg

struct robot_mpc_predicted_traj_solver_capsule;

namespace spg {
namespace setpoint {

// State representation (position + velocity)
struct AcadosMPCStatePredictedTraj {
    double px;      // x position [m]
    double py;      // y position [m]
    double theta;   // heading [rad]
    double vx;      // forward velocity (body frame) [m/s]
    double vy;      // sideways velocity (body frame) [m/s]
    double omega;   // yaw rate [rad/s]
};

// Control representation (body-frame accelerations)
struct AcadosMPCControlPredictedTraj {
    double ax;      // forward acceleration (body frame) [m/s^2]
    double ay;      // sideways acceleration (body frame) [m/s^2]
    double alpha;   // angular acceleration [rad/s^2]
};

// Velocity output (what gets sent to the robot)
struct AcadosMPCVelocityPredictedTraj {
    double vx;      // forward velocity (body frame) [m/s]
    double vy;      // sideways velocity (body frame) [m/s]
    double omega;   // yaw rate [rad/s]
};

// Configuration parameters
struct AcadosMPCParamsPredictedTraj {
    double dt;              // time step [s]
    int horizon;            // N steps
    
    // Physical velocity limits (constraints on state)
    double vx_max;          // [m/s]
    double vy_max;          // [m/s]
    double omega_max;       // [rad/s]
    
    // Acceleration limits (constraints on control)
    double ax_max;          // [m/s^2]
    double ay_max;          // [m/s^2]
    double alpha_max;       // [rad/s^2]
    
    // Polygon sides for ellipse approximation
    int polygon_sides;      // typically 8-16
    
    AcadosMPCParamsPredictedTraj();
};

// Reference trajectory builder
struct AcadosMPCReferencePredictedTraj {
    std::vector<std::array<double, 6>> x_ref;  // size N+1: [px, py, theta, vx, vy, omega]
    std::vector<std::array<double, 3>> u_ref;  // size N: [ax, ay, alpha]
};

// Acados-based MPC solver for humanoid robot
class HumanoidReferencePredictedTrajMPC {
public:
    explicit HumanoidReferencePredictedTrajMPC(const AcadosMPCParamsPredictedTraj& params);
    ~HumanoidReferencePredictedTrajMPC();
    
    // Main solve function
    // Returns false if solver fails
    // Note: u_out contains ACCELERATIONS, velocity output is in x0 state
    bool computeControl(
        const AcadosMPCStatePredictedTraj& x0,
        const AcadosMPCStatePredictedTraj& x_goal,
        AcadosMPCControlPredictedTraj& u_out);
    
    // Solve and get full trajectory prediction
    bool computeControlAndTrajectory(
        const AcadosMPCStatePredictedTraj& x0,
        const AcadosMPCStatePredictedTraj& x_goal,
        AcadosMPCControlPredictedTraj& u_out,
        std::vector<AcadosMPCStatePredictedTraj>& predicted_states,
        std::vector<AcadosMPCControlPredictedTraj>& predicted_controls);
    
    // Extract velocity from first predicted state (for robot command)
    AcadosMPCVelocityPredictedTraj getVelocityCommand() const;
    
    // Check if solver is ready
    bool isInitialized() const { return solver_initialized_; }
    
private:
    // Build reference trajectory using heuristic
    AcadosMPCReferencePredictedTraj buildReference(
        const AcadosMPCStatePredictedTraj& x0,
        const AcadosMPCStatePredictedTraj& x_goal) const;
    
    // Helper functions for acados interface
    void setInitialState(const AcadosMPCStatePredictedTraj& x0);
    void setReferences(const AcadosMPCReferencePredictedTraj& ref);
    AcadosMPCControlPredictedTraj getFirstControl() const;
    void getPredictedTrajectory(
        std::vector<AcadosMPCStatePredictedTraj>& states,
        std::vector<AcadosMPCControlPredictedTraj>& controls) const;
    
    // Utility functions
    static double wrap_pi(double a);
    static double clamp(double v, double lo, double hi);
    
    // Warm-start management
    void applyWarmStart();
    void storeSolution();
    
    AcadosMPCParamsPredictedTraj params_;
    robot_mpc_predicted_traj_solver_capsule* capsule_;
    bool solver_initialized_;
    
    // Warm-start storage
    std::vector<std::array<double, 6>> prev_states_;    // Size N+1
    std::vector<std::array<double, 3>> prev_controls_;  // Size N
    bool has_previous_solution_;
};

} // namespace setpoint
} // namespace spg
