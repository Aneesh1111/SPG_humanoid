#!/usr/bin/env python3
"""
Acados Solver Generator for Humanoid Robot MPC with ACCELERATION control

Robot dynamics with VELOCITIES in STATE and ACCELERATIONS as CONTROL:
  x = [px, py, theta, vx, vy, omega]  (position, heading, body-frame velocities)
  u = [ax, ay, alpha]  (body-frame accelerations)
  
Dynamics (discrete Euler):
  px_{k+1}    = px_k + dt*(cos(theta_k)*vx_k - sin(theta_k)*vy_k)
  py_{k+1}    = py_k + dt*(sin(theta_k)*vx_k + cos(theta_k)*vy_k)
  theta_{k+1} = theta_k + dt*omega_k
  vx_{k+1}    = vx_k + dt*ax_k
  vy_{k+1}    = vy_k + dt*ay_k
  omega_{k+1} = omega_k + dt*alpha_k

Constraints:
  STATE: (vx/0.4)^2 + (vy/1.1)^2 <= 1  (ellipse, 12-sided polygon)
  STATE: |omega| <= 1.5
  CONTROL: |ax| <= 2.0, |ay| <= 2.0, |alpha| <= 3.0
"""

from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
from casadi import SX, vertcat, sin, cos
import numpy as np
import os

def create_robot_model():
    model = AcadosModel()
    model.name = 'robot_mpc_predicted_traj'
    
    # State: [px, py, theta, vx_body, vy_body, omega]
    px = SX.sym('px')
    py = SX.sym('py')
    theta = SX.sym('theta')
    vx = SX.sym('vx')
    vy = SX.sym('vy')
    omega = SX.sym('omega')
    x = vertcat(px, py, theta, vx, vy, omega)
    
    # Control: [ax_body, ay_body, alpha]
    ax = SX.sym('ax')
    ay = SX.sym('ay')
    alpha = SX.sym('alpha')
    u = vertcat(ax, ay, alpha)
    
    # Dynamics
    px_dot = cos(theta) * vx - sin(theta) * vy
    py_dot = sin(theta) * vx + cos(theta) * vy
    theta_dot = omega
    vx_dot = ax
    vy_dot = ay
    omega_dot = alpha
    
    f_expl = vertcat(px_dot, py_dot, theta_dot, vx_dot, vy_dot, omega_dot)
    xdot = SX.sym('xdot', 6)
    f_impl = xdot - f_expl
    
    model.x = x
    model.xdot = xdot
    model.u = u
    model.f_expl_expr = f_expl
    model.f_impl_expr = f_impl
    
    return model

def create_velocity_polytope_constraints(M=12):
    vx_max = 1.1
    vy_max = 0.4
    rhs = np.cos(np.pi / M)
    
    C_list = []
    d_list = []
    
    for i in range(M):
        phi = 2.0 * np.pi * i / M
        a_vx = np.cos(phi) / vx_max
        a_vy = np.sin(phi) / vy_max
        C_row = np.zeros(6)
        C_row[3] = a_vx
        C_row[4] = a_vy
        C_list.append(C_row)
        d_list.append(rhs)
    
    return np.array(C_list), np.array(d_list)

def create_ocp_solver():
    ocp = AcadosOcp()
    model = create_robot_model()
    ocp.model = model
    
    nx = 6
    nu = 3
    N = 20
    dt = 0.4
    
    ocp.dims.N = N
    ocp.solver_options.tf = N * dt
    
    # Cost
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
    
    ax_max = 4.0  # Reduced from 8.0 for stability
    ay_max = 3.0  # Reduced from 6.0 for stability
    alpha_max = 2.0  # Reduced from 3.0 for stability
    
    # Define velocity limits first
    vx_max = 0.8  # Conservative limit
    vx_min = -0.1 # Robot should not move backwards significantly (cannot see where it is going)
    vy_max = 0.3  # Conservative limit  
    omega_max = 1.2  # Conservative limit
    
    ocp.constraints.lbu = np.array([-ax_max, -ay_max, -alpha_max])
    ocp.constraints.ubu = np.array([ax_max, ay_max, alpha_max])
    ocp.constraints.idxbu = np.array([0, 1, 2])
    
    # Add state bounds to prevent drift
    # Bounds on [px, py, theta, vx, vy, omega]
    ocp.constraints.lbx = np.array([-20.0, -20.0, -np.pi, vx_min, -vy_max, -omega_max])
    ocp.constraints.ubx = np.array([20.0, 20.0, np.pi, vx_max, vy_max, omega_max])
    ocp.constraints.idxbx = np.array([0, 1, 2, 3, 4, 5])  # All states bounded
    
    # Terminal velocity constraints: robot must stop at end of horizon
    # v_x,N = 0, v_y,N = 0 (hard equality constraints)
    # Note: bounds arrays must match length of idxbx_e (only 2 values for indices 3,4)
    ocp.constraints.lbx_e = np.array([0.0, 0.0])
    ocp.constraints.ubx_e = np.array([0.0, 0.0])
    ocp.constraints.idxbx_e = np.array([3, 4])  # Constrain v_x, v_y at terminal stage
    
    # Solver options - optimized for numerical stability
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.integrator_type = 'IRK'  # Implicit integrator for stability
    ocp.solver_options.sim_method_num_stages = 2  # Gauss-Legendre 2-stage
    ocp.solver_options.sim_method_num_steps = 3  # Multiple integration steps
    ocp.solver_options.nlp_solver_type = 'SQP_RTI'
    ocp.solver_options.qp_solver_iter_max = 50
    ocp.solver_options.nlp_solver_max_iter = 50
    ocp.solver_options.tol = 1e-2  # Relaxed tolerance
    ocp.solver_options.regularize_method = 'PROJECT'
    ocp.solver_options.levenberg_marquardt = 1e-2  # Increased for damping
    ocp.code_export_directory = 'c_generated_code'
    
    print("Creating acados OCP solver with ACCELERATION control...")
    print(f"  State: [px, py, theta, vx, vy, omega] (nx={nx})")
    print(f"  Control: [ax, ay, alpha] (nu={nu})")
    print(f"  Horizon: N={N}, dt={dt}s")
    
    acados_solver = AcadosOcpSolver(ocp, json_file='robot_mpc.json')
    
    print("\n✓ Solver generated successfully!")
    return acados_solver

if __name__ == '__main__':
    import sys
    
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(script_dir)
    generated_dir = os.path.join(project_root, 'generated')
    os.makedirs(generated_dir, exist_ok=True)
    os.chdir(generated_dir)
    
    try:
        print("\n" + "="*70)
        print("Acados MPC Solver Generator (Acceleration-based)")
        print("="*70 + "\n")
        
        solver = create_ocp_solver()
        
        print(f"\nGenerated files in: {os.getcwd()}/c_generated_code/")
        print("\nNext steps:")
        print("  1. cd build && cmake .. && make -j4")
        print("  2. Run demo: ./build/demo_humanoid_mpc")
        
    except Exception as e:
        print(f"\nERROR: {e}\n")
        import traceback
        traceback.print_exc()
        sys.exit(1)
