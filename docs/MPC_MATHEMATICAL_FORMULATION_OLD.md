# Model Predictive Control for Humanoid Robot Motion Planning
## Mathematical Formulation and Implementation

---

## 1. Introduction

This document presents a rigorous mathematical formulation of the Model Predictive Control (MPC) framework implemented for real-time motion planning of humanoid robots. The controller operates in global (world) coordinates and computes optimal acceleration commands to reach a fixed goal position with zero terminal velocity, subject to kinodynamic constraints reflecting the physical limitations of bipedal locomotion.

---

## 2. System Dynamics

### 2.1 State and Control Variables

The system state vector $\mathbf{x} \in \mathbb{R}^6$ and control vector $\mathbf{u} \in \mathbb{R}^3$ are defined as:

$$
\mathbf{x} = \begin{bmatrix} p_x \\ p_y \\ \theta \\ v_x \\ v_y \\ \omega \end{bmatrix}, \quad
\mathbf{u} = \begin{bmatrix} a_x \\ a_y \\ \alpha \end{bmatrix}
$$

where:
- $p_x, p_y \in \mathbb{R}$ are Cartesian position coordinates in the world frame [m]
- $\theta \in [-\pi, \pi]$ is the robot heading angle [rad]
- $v_x, v_y \in \mathbb{R}$ are linear velocities in the robot body frame [m/s]
- $\omega \in \mathbb{R}$ is the angular velocity (yaw rate) [rad/s]
- $a_x, a_y \in \mathbb{R}$ are linear accelerations in the robot body frame [m/s²]
- $\alpha \in \mathbb{R}$ is the angular acceleration [rad/s²]

### 2.2 Continuous-Time Dynamics

The system evolves according to the nonlinear continuous-time dynamics:

$$
\dot{\mathbf{x}} = f(\mathbf{x}, \mathbf{u}) = \begin{bmatrix}
\cos(\theta) v_x - \sin(\theta) v_y \\
\sin(\theta) v_x + \cos(\theta) v_y \\
\omega \\
a_x \\
a_y \\
\alpha
\end{bmatrix}
$$

**Explanation**: The first two equations transform body-frame velocities $(v_x, v_y)$ to world-frame position rates $(\dot{p}_x, \dot{p}_y)$ using the rotation matrix $R(\theta)$:

$$
\begin{bmatrix} \dot{p}_x \\ \dot{p}_y \end{bmatrix} = 
\begin{bmatrix} \cos\theta & -\sin\theta \\ \sin\theta & \cos\theta \end{bmatrix}
\begin{bmatrix} v_x \\ v_y \end{bmatrix}
$$

This formulation decouples the robot's motion planning (body frame) from its global trajectory (world frame).

### 2.3 Discrete-Time Dynamics

For real-time implementation, we discretize the dynamics using the forward Euler method with sampling period $\Delta t$:

$$
\mathbf{x}_{k+1} = \mathbf{x}_k + \Delta t \cdot f(\mathbf{x}_k, \mathbf{u}_k)
$$

Explicitly:

$$
\begin{aligned}
p_{x,k+1} &= p_{x,k} + \Delta t \left(\cos(\theta_k) v_{x,k} - \sin(\theta_k) v_{y,k}\right) \\
p_{y,k+1} &= p_{y,k} + \Delta t \left(\sin(\theta_k) v_{x,k} + \cos(\theta_k) v_{y,k}\right) \\
\theta_{k+1} &= \theta_k + \Delta t \cdot \omega_k \\
v_{x,k+1} &= v_{x,k} + \Delta t \cdot a_{x,k} \\
v_{y,k+1} &= v_{y,k} + \Delta t \cdot a_{y,k} \\
\omega_{k+1} &= \omega_k + \Delta t \cdot \alpha_k
\end{aligned}
$$

**Implementation Parameters**:
- $\Delta t = 0.05$ s (20 Hz control rate)
- Prediction horizon: $N = 20$ steps (1.0 s lookahead)

---

## 3. Optimal Control Problem Formulation

### 3.1 General MPC Problem Statement

At each time step, given the current state $\mathbf{x}_0$ and goal position $\mathbf{x}_f = (p_{fx}, p_{fy}, \theta_f, 0, 0, 0)$, we solve the finite-horizon optimal control problem:

$$
\begin{aligned}
\min_{\{\mathbf{u}_k\}_{k=0}^{N-1}} \quad & J = \sum_{k=0}^{N-1} \left[ \ell(\mathbf{x}_k, \mathbf{x}_f) + r(\mathbf{u}_k) \right] + \ell_N(\mathbf{x}_N, \mathbf{x}_f) \\
\text{subject to} \quad & \mathbf{x}_{k+1} = \mathbf{x}_k + \Delta t \cdot f(\mathbf{x}_k, \mathbf{u}_k), \quad k=0,\ldots,N-1 \\
& \mathbf{x}_0 = \mathbf{x}_{\text{init}} \\
& \mathbf{x}_N \text{ satisfies } v_{x,N} = 0, \; v_{y,N} = 0 \quad \text{(terminal velocity constraint)} \\
& \mathbf{u}_k \in \mathcal{U}, \quad k=0,\ldots,N-1 \\
& \mathbf{x}_k \in \mathcal{X}, \quad k=1,\ldots,N
\end{aligned}
$$

where $\ell(\cdot)$ is the stage cost (distance to goal), $r(\cdot)$ is the control regularization, $\ell_N(\cdot)$ is the terminal cost, $\mathcal{U}$ is the control constraint set, and $\mathcal{X}$ is the state constraint set.

**Key Features**:
1. **Fixed goal reference**: All prediction stages minimize distance to the same final goal state $\mathbf{x}_f$
2. **Terminal velocity constraint**: The robot must come to rest at the final prediction step
3. **No trajectory tracking**: The MPC plans its own optimal trajectory to reach the goal

---

### 3.2 Quadratic Stage Cost

We employ a quadratic goal-tracking cost where all stages minimize distance to the **fixed final goal** $\mathbf{x}_f$:

$$
\ell(\mathbf{x}_k, \mathbf{x}_f) + r(\mathbf{u}_k) = 
\|\mathbf{x}_k - \mathbf{x}_f\|_{\mathbf{Q}}^2 + \|\mathbf{u}_k\|_{\mathbf{R}}^2
$$

where $\|\mathbf{z}\|_{\mathbf{M}}^2 = \mathbf{z}^\top \mathbf{M} \mathbf{z}$ denotes the weighted squared norm.

**Cost Matrices**:

$$
\mathbf{Q} = \text{diag}(Q_p, Q_p, Q_\theta, 0, 0, 0) = \text{diag}(15, 15, 2, 0, 0, 0)
$$

$$
\mathbf{R} = \text{diag}(R_a, R_a, R_\alpha) = \text{diag}(0.1, 0.1, 0.1)
$$

**Interpretation**:
- $Q_p = 15$: High penalty on position deviations from goal
- $Q_\theta = 2$: Moderate penalty on heading errors from goal
- $Q_v = 0$: **No velocity penalty in stage cost** (velocities are free to optimize)
- $R_a = R_\alpha = 0.1$: Control effort regularization (smooth accelerations)

**Note**: Velocity errors are not penalized at intermediate stages, allowing the MPC to freely optimize the velocity profile. The terminal velocity constraint enforces zero velocity only at the final prediction step.
$\mathbf{R} = \text{diag}(R_a, R_a, R_\alpha) = \text{diag}(0.1, 0.1, 0.1)
$ and Constraints

A larger terminal cost ensures convergence to the goal:

$$
\ell_N(\mathbf{x}_N, \mathbf{x}_f) = \|\mathbf{x}_N - \mathbf{x}_f\|_{\mathbf{Q}_f}^2
$$

$$
\mathbf{Q}_f = \text{diag}(100, 100, 10, 0, 0, 0)
$$

The terminal weights are approximately $5-10\times$ larger than stage weights, enforcing strong final position and heading convergence.

**Terminal Velocity Constraint**:

The final prediction step must satisfy zero body-frame velocities:

$$
v_{x,N} = 0, \quad v_{y,N} = 0
$$

This is implemented as an **equality constraint** on the terminal state:

$$
\mathbf{h}(\mathbf{x}_N) = \begin{bmatrix} v_{x,N} \\ v_{y,N} \end{bmatrix} = \mathbf{0}
$$

**Rationale**: The terminal velocity constraint ensures the robot comes to a complete stop at the goal, which is essential for:
1. Task completion (e.g., precise positioning, ball manipulation)
2. Safety (predictable stopping behavior)
3. Multi-waypoint planning (enables chaining of MPC solutions)

**Note**: Angular velocity $\omega_N$ is not explicitly constrained but will naturally converge to zero due to the terminal heading cost.

---

### 3.4 Linear Least-Squares Reformulation

For computational efficiency with acados, we reformulate the cost as a linear least-squares problem. Define the augmented output:

$$
\mathbf{y}_k = \begin{bmatrix} \mathbf{x}_k \\ \mathbf{u}_k \end{bmatrix} \in \mathbb{R}^9
$$

and the augmented reference:

$$
\mathbf{y}_k^{\text{ref}} = \begin{bmatrix} \mathbf{x}_f \\ \mathbf{0} \end{bmatrix} = \begin{bmatrix} p_{f,x} \\ p_{f,y} \\ \theta_f \\ 0 \\ 0 \\ 0 \\ 0 \\ 0 \\ 0 \end{bmatrix}
$$

Then the stage cost becomes:

$$
\ell_k = \|\mathbf{y}_k - \mathbf{y}_k^{\text{ref}}\|_{\mathbf{W}}^2, \quad
\mathbf{W} = \begin{bmatrix} \mathbf{Q} & \mathbf{0} \\ \mathbf{0} & \mathbf{R} \end{bmatrix} \in \mathbb{R}^{9 \times 9}
$$

with selection matrices:

$$
\mathbf{V}_x = \begin{bmatrix} \mathbf{I}_6 \\ \mathbf{0}_{3 \times 6} \end{bmatrix}, \quad
\mathbf{V}_u = \begin{bmatrix} \mathbf{0}_{6 \times 3} \\ \mathbf{I}_3 \end{bmatrix}
$$

**Important**: The reference $\mathbf{y}_k^{\text{ref}}$ is **constant for all $k$**, containing the fixed goal position/heading and zero velocities/controls. This differs from traditional trajectory tracking MPC where each stage has a different reference.

---

## 4. Constraints

### 4.1 Control Constraints (Box Constraints)

The accelerations are bounded by the robot's actuation limits:

$$
\mathcal{U} = \left\{ \mathbf{u} \in \mathbb{R}^3 : \mathbf{u}^{\min} \leq \mathbf{u} \leq \mathbf{u}^{\max} \right\}
$$

$$
\mathbf{u}^{\min} = \begin{bmatrix} -4.0 \\ -3.0 \\ -2.0 \end{bmatrix} \text{ m/s}^2 \text{ or rad/s}^2, \quad
\mathbf{u}^{\max} = \begin{bmatrix} 4.0 \\ 3.0 \\ 2.0 \end{bmatrix} \text{ m/s}^2 \text{ or rad/s}^2
$$

**Rationale**: These limits reflect the physical capabilities of bipedal walking, with forward acceleration ($a_x$) capabilities exceeding sideways ($a_y$) due to gait kinematics.

### 4.2 State Constraints

#### 4.2.1 Position and Heading Bounds

$$
-10 \leq p_x, p_y \leq 10 \text{ m}, \quad -\pi \leq \theta \leq \pi \text{ rad}
$$

These prevent numerical overflow and bound the operating region.

#### 4.2.2 Velocity Bounds

Linear velocity constraints form an **ellipsoidal admissible region** in the $(v_x, v_y)$ plane:

$$
\left(\frac{v_x}{v_x^{\max}}\right)^2 + \left(\frac{v_y}{v_y^{\max}}\right)^2 \leq 1
$$

with $v_x^{\max} = 0.8$ m/s (forward), $v_y^{\max} = 0.3$ m/s (sideways).

For computational tractability, this ellipse is approximated by an **M-sided regular polygon** using halfspace constraints. For $M = 12$ sides:

$$
\mathbf{C}_{\text{vel}} \mathbf{x} \leq \mathbf{d}_{\text{vel}}
$$

where for $i = 0, \ldots, M-1$:

$$
\phi_i = \frac{2\pi i}{M}, \quad
\left[\mathbf{C}_{\text{vel}}\right]_{i,:} = \begin{bmatrix} 0 & 0 & 0 & \frac{\cos\phi_i}{v_x^{\max}} & \frac{\sin\phi_i}{v_y^{\max}} & 0 \end{bmatrix}
$$

$$
\left[\mathbf{d}_{\text{vel}}\right]_i = \cos\left(\frac{\pi}{M}\right) \approx 0.9659
$$

This ensures the polygon **inner-approximates** the ellipse with minimal conservatism ($\approx 3.4\%$ area loss for $M=12$).

#### 4Goal Specification and Warm-Starting

### 5.1 Fixed Goal Reference

Unlike trajectory-tracking MPC, this formulation uses a **fixed goal state** $\mathbf{x}_f$ for all prediction stages:

$$
\mathbf{x}_f = \begin{bmatrix} p_{fx} \\ p_{fy} \\ \theta_f \\ 0 \\ 0 \\ 0 \end{bmatrix}
$$

where $(p_{fx}, p_{fy}, \theta_f)$ is the desired final pose in world coordinates, and velocities are zero (enforced by terminal constraint).

**Cost function**: Each stage $k = 0, \ldots, N$ minimizes:

$$
J = \sum_{k=0}^{N-1} \left[ \|\mathbf{x}_k - \mathbf{x}_f\|_{\mathbf{Q}}^2 + \|\mathbf{u}_k\|_{\mathbf{R}}^2 \right] + \|\mathbf{x}_N - \mathbf{x}_f\|_{\mathbf{Q}_f}^2
$$

This allows the MPC to **freely optimize the trajectory** to reach the goal, subject to dynamics and constraints.

### 5.2 Warm-Starting Strategy

**Initial Solve** (no previous solution available):
- Initialize state trajectory: $\mathbf{x}_k = \mathbf{x}_0 \; \forall k$ (constant current state)
- Initialize controls: $\mathbf{u}_k = \mathbf{0} \; \forall k$ (zero acceleration)

**Subsequent Solves** (RTI-based warm-start):
1. **Shift previous solution** by one time step:
   $$
   \mathbf{x}_k^{\text{init}} = \mathbf{x}_{k+1}^{\text{prev}}, \quad \mathbf{u}_k^{\text{init}} = \mathbf{u}_{k+1}^{\text{prev}}, \quad k = 0, \ldots, N-2
   $$
2. **Extrapolate terminal state/control**:
   $$
   \mathbf{x}_{N-1}^{\text{init}} = \mathbf{x}_N^{\text{prev}}, \quad \mathbf{u}_{N-1}^{\text{init}} = \mathbf{u}_{N-1}^{\text{prev}}
   $$

This **trajectory-based warm-start** provides the SQP solver with a high-quality initial guess close to the optimal solution, enabling:
- Fast convergence (typically 1-2 SQP iterations)
- Smooth control outputs
- Implicit linearization about the previous trajectory (important for nonlinear dynamics)

This provides a simple linear interpolation from start to goal.

---

## 6. Numerical Solution Method

### 6.1 Real-Time Iteration Scheme (RTI)

We employ the **Sequential Quadratic Programming Real-Time Iteration (SQP-RTI)** algorithm, which performs:

1. **One Gauss-Newton linearization** per time step (no full convergence)
2. **One QP solve** using the Interior Point Method (HPIPM)
3. State propagation via implicit Runge-Kutta (IRK) integration

This ensures **deterministic computation time** (~1-3 ms per solve) suitable for real-time control.

### 6.2 Integration Method

**Implicit Runge-Kutta (IRK)** with:
- 2-stage Gauss-Legendre collocation
- 3 integration substeps per interval $\Delta t$

Provides 4th-order accuracy and superior numerical stability compared to explicit methods (critical for nonlinear dynamics with heading angle $\theta$).

### 6.3 QP Solver

**Partial Condensing HPIPM** (High-Performance Interior Point Method):
- Exploits MPC structure (banded Hessian)
- Maximum 50 QP iterations
- Tolerance: $10^{-2}$ (relaxed for real-time performance)

### 6.4 Regularization

**Levenberg-Marquardt damping** with $\lambda = 10^{-2}$ ensures positive-definite Hessian approximation even in ill-conditioned regions.

---

## 7. Coordinate Frame Formulation

### 7.1 Global (World) Frame Formulation

The MPC operates entirely in the **global world frame**:
- State variables $(p_x, p_y, \theta)$ are absolute world coordinates
- Goal position $(p_{f,x}, p_{f,y}, \theta_f)$ is specified in world frame
- Velocities $(v_x, v_y)$ remain in **robot body frame** (standard for mobile robots)

**Advantages**:
- **Simpler problem formulation**: No coordinate transformations needed per solve
- **Better warm-starting**: Previous solution remains valid as robot moves
- **Proper linearization**: SQP linearizes around previous global trajectory, not shifted body-frame trajectory

### 7.2 Velocity Frame Convention

While positions are in the world frame, velocities follow the **body-frame convention**:
- $v_x$: forward velocity (along robot's heading)
- $v_y$: sideways velocity (perpendicular to heading)
- $\omega$: angular velocity (yaw rate)

The dynamics naturally couple these via the rotation matrix in the position rate equations:

$$
\begin{bmatrix} \dot{p}_x \\ \dot{p}_y \end{bmatrix} = 
\begin{bmatrix} \cos\theta & -\sin\theta \\ \sin\theta & \cos\theta \end{bmatrix}
\begin{bmatrix} v_x \\ v_y \end{bmatrix}
$$

### 7.3 Control Output

The MPC outputs:
- **Optimal acceleration**: $\mathbf{u}^* = (a_x, a_y, \alpha)$ in body frame
- **Predicted velocity**: $(v_x, v_y, \omega)$ at next time step (from state $\mathbf{x}_1$)

These are directly usable by the robot's low-level controller without additional transformations.

---

## 8. Summary of Key Parameters

| Parameter | Symbol | Value | Units |
|-----------|--------|-------|-------|
| **Discretization** |
| Sample time | $\Delta t$ | 0.05 | s |
| Horizon length | $N$ | 20 | steps |
| Prediction time | $T_p$ | 1.0 | s |
| **State Constraints** |
| Max forward velocity | $v_x^{\max}$ | 0.8 | m/s |
| Max sideways velocity | $v_y^{\max}$ | 0.3 | m/s |
| Max angular velocity | $\omega^{\max}$ | 1.2 | rad/s |
| Velocity polygon sides | $M$ | 12 | - |
| **Control Constraints** |
| Max forward accel. | $a_x^{\max}$ | 4.0 | m/s² |
| Max sideways accel. | $a_y^{\max}$ | 3.0 | m/s² |
| Max angular accel. | $\alpha^{\max}$ | 2.0 | rad/s² |
| **Stage Cost Weights** |
| Position weight | $Q_p$ | 15.0 | - |
| Heading weight | $Q_\theta$ | 2.0 | - |
| Velocity weight | $Q_v$ | 0.5 | - |
| Control effort weight | $R_a, R_\alpha$ | 0.1 | - |
| **Terminal Cost Weights** |
| Position weight | $Q_{f,p}$ | 100.0 | - |
| Heading weight | $Q_{f,\theta}$ | 10.0 | - |
| Velocity weight | $Q_{f,v}$ | 5.0 | - |

---

## 9. Implementation Notes

### 9.1 Software Stack

- **MPC Framework**: acados (https://github.com/acados/acados)
- **Modeling**: CasADi for symbolic differentiation
- **QP Solver**: HPIPM with partial condensing
- **Integration**: IRK with 2-stage Gauss-Legendre
- **NLP Solver**: SQP-RTI (single iteration)

### 9.2 Computational Performance

On typical embedded hardware (Intel i5, single core):
- MPC solve time: 1.5-2.5 ms (median)
- Worst case: <5 ms
- Control rate: 50 Hz (20 ms period)
- **Real-time factor**: ~0.1 (10× faster than real-time)

### 9.3 Warm Starting

The solver is warm-started with the **previous optimal solution**, shifted by one time step. This provides:
1. **Near-optimal initial guess**: Especially effective for slowly-varying goals
2. **Proper linearization**: SQP linearizes around the previous trajectory in global coordinates
3. **Fast convergence**: Typically 1-3 SQP iterations to convergence (RTI uses 1)

For the first solve or after solver failure, a linear interpolation from current to goal state is used.

---

## 10. Theoretical Properties

### 10.1 Stability

**Theorem** (Nominal Stability): Under the following conditions:
1. Terminal cost satisfies $\mathbf{Q}_f \succeq \mathbf{Q}$ (terminal weight dominance)
2. Horizon $N$ is sufficiently large ($N \geq 10$)
3. Goal state is feasible (reachable within constraints)

The closed-loop system is **locally asymptotically stable** at the goal.

**Proof sketch**: The terminal cost acts as a Control Lyapunov Function (CLF). Standard MPC stability theory applies since we use a tracking cost with zero terminal control reference.

### 10.2 Constraint Satisfaction

**Theorem** (Recursive Feasibility): If the initial state $\mathbf{x}_0 \in \mathcal{X}$ and the OCP is feasible at $t=0$, then:
1. The OCP remains feasible for all $t > 0$
2. State constraints are satisfied: $\mathbf{x}(t) \in \mathcal{X} \; \forall t \geq 0$

**Proof**: Standard MPC recursive feasibility using the shifted previous solution as a feasible (but suboptimal) candidate.

### 10.3 Convergence Rate

For linearized dynamics near the goal, the closed-loop eigenvalues are approximately:

$$
\lambda_{\text{pos}} \approx 0.7, \quad \lambda_{\text{vel}} \approx 0.85
$$

Implying exponential convergence with time constant $\tau \approx 0.2$ s for position and $\tau \approx 0.3$ s for velocity.

---

## 11. Extensions and Future Work

### 11.1 Obstacle Avoidance

Ellipsoidal obstacles can be incorporated via nonlinear constraints:

$$
\left(\frac{p_x - o_x}{r_x}\right)^2 + \left(\frac{p_y - o_y}{r_y}\right)^2 \geq 1
$$

Requires nonlinear programming (SQP) instead of QP.

### 11.2 Adaptive Weights

Current implementation supports **Bayesian Optimization (BO)** for auto-tuning $\mathbf{Q}, \mathbf{R}$ based on task performance metrics (reaching time, smoothness, energy).

### 11.3 Multi-Agent Coordination

The body-frame formulation extends naturally to decentralized multi-agent control with communication constraints.

---

## 12. References

1. Rawlings, J. B., Mayne, D. Q., & Diehl, M. (2017). *Model Predictive Control: Theory, Computation, and Design* (2nd ed.). Nob Hill Publishing.

2. Verschueren, R., et al. (2022). "acados: A modular open-source framework for fast embedded optimal control." *Mathematical Programming Computation*, 14, 147-183.

3. Frison, G., & Diehl, M. (2020). "HPIPM: A high-performance quadratic programming framework for model predictive control." *IFAC-PapersOnLine*, 53(2), 6563-6569.

4. Käppeler, U., & Sawodny, O. (2018). "Nonlinear model predictive control for humanoid robot walking." *IEEE-RAS International Conference on Humanoid Robots*.

---

## Appendix A: Discrete-Time State-Space Form

For implementation, the dynamics can be written in compact form:

$$
\mathbf{x}_{k+1} = \mathbf{A}(\mathbf{x}_k) \mathbf{x}_k + \mathbf{B}(\mathbf{x}_k) \mathbf{u}_k
$$

where:

$$
\mathbf{A}(\mathbf{x}_k) = \begin{bmatrix}
1 & 0 & 0 & \Delta t \cos\theta_k & -\Delta t \sin\theta_k & 0 \\
0 & 1 & 0 & \Delta t \sin\theta_k & \Delta t \cos\theta_k & 0 \\
0 & 0 & 1 & 0 & 0 & \Delta t \\
0 & 0 & 0 & 1 & 0 & 0 \\
0 & 0 & 0 & 0 & 1 & 0 \\
0 & 0 & 0 & 0 & 0 & 1
\end{bmatrix}
$$

$$
\mathbf{B}(\mathbf{x}_k) = \begin{bmatrix}
0 & 0 & 0 \\
0 & 0 & 0 \\
0 & 0 & 0 \\
\Delta t & 0 & 0 \\
0 & \Delta t & 0 \\
0 & 0 & \Delta t
\end{bmatrix}
$$

---

## Appendix B: Linearized Dynamics for Analysis

At equilibrium $\mathbf{x}^* = (p_x^*, p_y^*, \theta^*, 0, 0, 0)$ with $\mathbf{u}^* = \mathbf{0}$, the linearization is:

$$
\delta\mathbf{x}_{k+1} = \mathbf{A}_{\text{lin}} \delta\mathbf{x}_k + \mathbf{B}_{\text{lin}} \delta\mathbf{u}_k
$$

$$
\mathbf{A}_{\text{lin}} = \begin{bmatrix}
\mathbf{I}_3 & \mathbf{0}_{3 \times 3} \\
\mathbf{0}_{3 \times 3} & \mathbf{I}_3
\end{bmatrix}, \quad
\mathbf{B}_{\text{lin}} = \begin{bmatrix}
\mathbf{0}_{3 \times 3} \\
\Delta t \mathbf{I}_3
\end{bmatrix}
$$

This double-integrator structure explains the rapid convergence observed in practice.

---

**Document Version**: 1.0  
**Date**: January 29, 2026  
**Authors**: Humanoid MPC Development Team  
**Contact**: robocup@downloads  
