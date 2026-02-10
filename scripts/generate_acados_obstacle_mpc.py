#!/usr/bin/env python3
"""
Generate acados solver for integrated obstacle avoidance MPC.

This solver combines:
- Target tracking
- Obstacle avoidance (via nonlinear constraints)
- Velocity/acceleration limits
- Smooth trajectory generation

Model: Unicycle with body-frame velocities
States: [px, py, theta, vx, vy, omega]
Controls: [ax, ay, alpha]  (accelerations)

Obstacle avoidance implemented as nonlinear path constraints:
  (px - obs_px)^2 + (py - obs_py)^2 >= (r_robot + r_obs + margin)^2
"""

from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
import casadi as ca
import numpy as np
import os
import sys
import shutil

def create_robot_obstacle_model():
    """Create CasADi model for robot with obstacle avoidance"""
    model_name = 'robot_obstacle_mpc'
    
    # States
    px = ca.SX.sym('px')        # Position x [m]
    py = ca.SX.sym('py')        # Position y [m]
    theta = ca.SX.sym('theta')  # Heading [rad]
    vx = ca.SX.sym('vx')        # Body-frame forward velocity [m/s]
    vy = ca.SX.sym('vy')        # Body-frame sideways velocity [m/s]
    omega = ca.SX.sym('omega')  # Angular velocity [rad/s]
    
    x = ca.vertcat(px, py, theta, vx, vy, omega)
    nx = 6
    
    # Controls (accelerations)
    ax = ca.SX.sym('ax')        # Forward acceleration [m/s^2]
    ay = ca.SX.sym('ay')        # Sideways acceleration [m/s^2]
    alpha = ca.SX.sym('alpha')  # Angular acceleration [rad/s^2]
    
    u = ca.vertcat(ax, ay, alpha)
    nu = 3
    
    # Parameters: obstacle positions and radii
    # Format: [obs1_px, obs1_py, obs1_r, obs2_px, obs2_py, obs2_r, ...]
    max_obstacles = 10
    n_params = max_obstacles * 3
    p = ca.SX.sym('p', n_params)
    
    # Dynamics (unicycle with body-frame velocities)
    # World-frame velocity from body-frame
    vx_world = ca.cos(theta) * vx - ca.sin(theta) * vy
    vy_world = ca.sin(theta) * vx + ca.cos(theta) * vy
    
    f_expl = ca.vertcat(
        vx_world,   # dx/dt
        vy_world,   # dy/dt
        omega,      # dtheta/dt
        ax,         # dvx/dt
        ay,         # dvy/dt
        alpha       # domega/dt
    )
    
    # Implicit dynamics: x_dot = f(x, u)
    x_dot = ca.SX.sym('x_dot', nx)
    f_impl = x_dot - f_expl
    
    # Obstacle avoidance constraints using octagonal approximation
    # For each obstacle, use 8 linear half-plane constraints instead of circle
    # This makes the problem CONVEX! Much easier to solve.
    h_expr = []
    n_sides = 8  # Octagon (8 sides)
    
    for i in range(max_obstacles):
        obs_px = p[i * 3 + 0]
        obs_py = p[i * 3 + 1]
        obs_r = p[i * 3 + 2]  # This is the inscribed circle radius
        
        # For each side of the octagon, create a linear constraint
        # Each side is defined by: normal · (pos - obs_pos) >= radius
        for side in range(n_sides):
            angle = side * 2.0 * np.pi / n_sides
            nx = np.cos(angle)  # Normal vector x component
            ny = np.sin(angle)  # Normal vector y component
            
            # Linear constraint: nx·(px - obs_px) + ny·(py - obs_py) >= obs_r
            # Reformulated as: nx·px + ny·py - nx·obs_px - ny·obs_py - obs_r >= 0
            # Simplified: nx·(px - obs_px) + ny·(py - obs_py) - obs_r >= 0
            h_expr.append(nx * (px - obs_px) + ny * (py - obs_py) - obs_r)
    
    h = ca.vertcat(*h_expr)
    
    # Create acados model
    model = AcadosModel()
    model.name = model_name
    model.x = x
    model.xdot = x_dot
    model.u = u
    model.p = p
    model.f_expl_expr = f_expl
    model.f_impl_expr = f_impl
    
    # Add obstacle constraints as nonlinear path constraints
    model.con_h_expr = h
    
    return model

def generate_solver():
    """Generate acados OCP solver with obstacle avoidance"""
    
    # Create model
    model = create_robot_obstacle_model()
    nx = 6
    nu = 3
    n_obs = 10
    n_sides = 8  # Octagonal approximation
    
    # Create OCP
    ocp = AcadosOcp()
    ocp.model = model
    
    # Horizon
    N = 20
    dt = 0.15
    
    ocp.dims.N = N
    ocp.solver_options.tf = N * dt
    
    # Initialize parameter values (obstacles far away by default)
    n_params = n_obs * 3
    ocp.parameter_values = np.ones(n_params) * 1e6  # Initialize far away (inactive)
    
    # Cost function (LINEAR_LS)
    Q_pos = 15.0
    Q_theta = 2.0
    Q_vel = 0.5
    R_accel = 0.1
    
    Q = np.diag([Q_pos, Q_pos, Q_theta, Q_vel, Q_vel, Q_vel])
    R = np.diag([R_accel, R_accel, R_accel])
    
    ocp.cost.cost_type = 'LINEAR_LS'
    ny = nx + nu
    ocp.cost.W = np.block([[Q, np.zeros((nx, nu))], [np.zeros((nu, nx)), R]])
    ocp.cost.Vx = np.vstack([np.eye(nx), np.zeros((nu, nx))])
    ocp.cost.Vu = np.vstack([np.zeros((nx, nu)), np.eye(nu)])
    ocp.cost.yref = np.zeros(ny)
    
    ocp.cost.cost_type_e = 'LINEAR_LS'
    Q_term = np.diag([100.0, 100.0, 10.0, 5.0, 5.0, 2.0])
    ocp.cost.W_e = Q_term
    ocp.cost.Vx_e = np.eye(nx)
    ocp.cost.yref_e = np.zeros(nx)
    
    # Constraints
    ocp.constraints.x0 = np.zeros(nx)
    
    # Acceleration limits
    ax_max = 4.0
    ay_max = 3.0
    alpha_max = 2.0
    
    ocp.constraints.lbu = np.array([-ax_max, -ay_max, -alpha_max])
    ocp.constraints.ubu = np.array([ax_max, ay_max, alpha_max])
    ocp.constraints.idxbu = np.array([0, 1, 2])
    
    # Velocity limits (only constrain velocity states: vx, vy, omega)
    vx_max = 0.8
    vy_max = 0.3
    omega_max = 1.2
    
    ocp.constraints.lbx = np.array([-vx_max, -vy_max, -omega_max])
    ocp.constraints.ubx = np.array([vx_max, vy_max, omega_max])
    ocp.constraints.idxbx = np.array([3, 4, 5])
    
    # Obstacle avoidance constraints (linear half-planes forming octagons)
    # Each obstacle has n_sides linear constraints
    # h(x,u,p) >= 0
    n_constraints = n_obs * n_sides  # 10 obstacles × 8 sides = 80 linear constraints
    ocp.constraints.lh = np.zeros(n_constraints)  # >= 0
    ocp.constraints.uh = np.ones(n_constraints) * 1e6  # no upper limit
    
    # Optional: Add slack variables for robustness (not strictly needed with linear constraints)
    ocp.constraints.lsh = np.zeros(n_constraints)
    ocp.constraints.ush = np.ones(n_constraints) * 1e6
    ocp.constraints.idxsh = np.arange(n_constraints)
    
    # Smaller penalties since constraints are now convex (easier to satisfy)
    ocp.cost.zl = 10.0 * np.ones(n_constraints)
    ocp.cost.zu = 10.0 * np.ones(n_constraints)
    ocp.cost.Zl = 10.0 * np.ones(n_constraints)
    ocp.cost.Zu = 10.0 * np.ones(n_constraints)
    
    # Same constraints at terminal stage
    ocp.constraints.lbx_e = np.array([-vx_max, -vy_max, -omega_max])
    ocp.constraints.ubx_e = np.array([vx_max, vy_max, omega_max])
    ocp.constraints.idxbx_e = np.array([3, 4, 5])
    
    # Solver options
    # Try FULL_CONDENSING_QPOASES - different QP solver that might handle this better
    ocp.solver_options.qp_solver = 'FULL_CONDENSING_QPOASES'  # Try qpOASES instead of HPIPM
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.integrator_type = 'ERK'
    ocp.solver_options.nlp_solver_type = 'SQP'  # Nonlinear solver (handles nonlinear constraints)
    ocp.solver_options.nlp_solver_max_iter = 10  # Reduce to 10 for real-time
    ocp.solver_options.qp_solver_iter_max = 50  # 50 QP iterations max
    ocp.solver_options.tol = 1e-2  # Relaxed but reasonable tolerance
    ocp.solver_options.qp_tol = 1e-3  # Reasonable QP tolerance
    ocp.solver_options.print_level = 1  # Print solver progress
    
    # Regularization to improve numerical conditioning
    ocp.solver_options.regularize_method = 'PROJECT'
    ocp.solver_options.levenberg_marquardt = 1e-4
    
    # Use new API
    ocp.solver_options.N_horizon = N
    
    # Build solver
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(script_dir)
    generated_dir = os.path.join(project_root, 'generated')
    os.makedirs(generated_dir, exist_ok=True)
    
    # Change to generated directory before creating solver
    original_dir = os.getcwd()
    os.chdir(generated_dir)
    
    try:
        print("\n" + "="*70)
        print("Acados Obstacle Avoidance MPC Solver Generator")
        print("="*70 + "\n")
        print(f"Model: {model.name}")
        print(f"Horizon: N={N}, dt={dt}s")
        print(f"Max obstacles: {n_obs}")
        print(f"Output: {os.getcwd()}/c_generated_code/")
        
        solver = AcadosOcpSolver(ocp, json_file='acados_ocp_obstacle.json')
        
        print("\n✅ Solver generated successfully!")
        print(f"  Files in: {os.getcwd()}/c_generated_code/")
        print("\nNext steps:")
        print("  1. cd build && cmake .. && make -j4")
        print("  2. Use HumanoidObstacleAvoidanceMPC class in your code")
        
        return solver
    finally:
        os.chdir(original_dir)

if __name__ == '__main__':
    generate_solver()
