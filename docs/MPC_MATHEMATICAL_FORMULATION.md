# Model Predictive Control for Humanoid Robot Motion Planning
## Mathematical Formulation and Implementation

---

## 1. Introduction

This document presents the mathematical formulation of a **nonlinear Model Predictive Control (MPC)** framework for real-time motion planning of humanoid robots. The controller operates in **global (world) coordinates** and computes optimal body-frame acceleration commands to drive the robot from its current state to a fixed goal position with zero terminal velocity.

**Key Features**:
- **Fixed goal reference**: All prediction stages minimize distance to the same goal state (not a time-varying trajectory)
- **Terminal velocity constraints**: Robot comes to rest at the goal ($v_x = v_y = 0$ at final prediction)
- **Warm-start linearization**: SQP solver linearizes around the previous optimal trajectory (shifted by 1 timestep)
- **Global coordinate formulation**: Positions in world frame, velocities in body frame
- **Real-time capable**: SQP-RTI with 1 iteration per control cycle (~2-3 ms solve time)

**Target Application**: Autonomous navigation for bipedal humanoid robots in the RoboCup Standard Platform League, where precise positioning, smooth motion, and real-time performance are critical.

---

## 2. System Dynamics

### 2.1 State and Control Variables

The system state $\mathbf{x} \in \mathbb{R}^6$ and control $\mathbf{u} \in \mathbb{R}^3$ are:

$$
\mathbf{x} = \begin{bmatrix} p_x \\ p_y \\ \theta \\ v_x \\ v_y \\ \omega \end{bmatrix}, \quad
\mathbf{u} = \begin{bmatrix} a_x \\ a_y \\ \alpha \end{bmatrix}
$$

**State Components**:
- $p_x, p_y \in \mathbb{R}$: Position in **world frame** [m]
- $\theta \in [-\pi, \pi]$: Heading angle in world frame [rad]
- $v_x, v_y \in \mathbb{R}$: Linear velocities in **robot body frame** [m/s]
  - $v_x$: forward velocity (along robot heading)
  - $v_y$: sideways velocity (perpendicular to heading)
- $\omega \in \mathbb{R}$: Angular velocity (yaw rate) [rad/s]

**Control Components**:
- $a_x, a_y \in \mathbb{R}$: Linear accelerations in robot body frame [m/s²]
- $\alpha \in \mathbb{R}$: Angular acceleration [rad/s²]

### 2.2 Continuous-Time Dynamics

The nonlinear continuous-time dynamics are:

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

**Position Dynamics**: The world-frame position rates $(\dot{p}_x, \dot{p}_y)$ are obtained by rotating body-frame velocities $(v_x, v_y)$ through the heading angle $\theta$:

$$
\begin{bmatrix} \dot{p}_x \\ \dot{p}_y \end{bmatrix} = 
R(\theta) \begin{bmatrix} v_x \\ v_y \end{bmatrix}, \quad
R(\theta) = \begin{bmatrix} \cos\theta & -\sin\theta \\ \sin\theta & \cos\theta \end{bmatrix}
$$

**Velocity Dynamics**: Body-frame velocities are directly controlled by accelerations (double integrator):

$$
\dot{v}_x = a_x, \quad \dot{v}_y = a_y, \quad \dot{\omega} = \alpha
$$

### 2.3 Discrete-Time Dynamics

For real-time implementation, we discretize using forward Euler with timestep $\Delta t = 0.15$ s:

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

**Note**: The actual solver uses **Implicit Runge-Kutta (IRK)** integration for higher accuracy, but the above Euler form illustrates the structure.

---

## 3. MPC Formulation

### 3.1 General Setup

At each control cycle, the MPC solves the following **finite-horizon Optimal Control Problem (OCP)**:

$$
\begin{aligned}
\min_{\mathbf{u}_0, \ldots, \mathbf{u}_{N-1}} \quad & \sum_{k=0}^{N-1} \left( \|\mathbf{x}_k - \mathbf{x}_f\|^2_{\mathbf{Q}} + \|\mathbf{u}_k\|^2_{\mathbf{R}} \right) + \|\mathbf{x}_N - \mathbf{x}_f\|^2_{\mathbf{Q}_f} \\
\text{subject to} \quad & \mathbf{x}_{k+1} = \mathbf{x}_k + \Delta t \cdot f(\mathbf{x}_k, \mathbf{u}_k), \quad k = 0, \ldots, N-1 \\
& \mathbf{x}_0 = \mathbf{x}_{\text{current}} \quad \text{(initial condition)} \\
& \mathbf{u}_{\min} \leq \mathbf{u}_k \leq \mathbf{u}_{\max}, \quad k = 0, \ldots, N-1 \\
& v_{x,N} = 0, \quad v_{y,N} = 0 \quad \text{(terminal velocity constraints)}
\end{aligned}
$$

**Notation**:
- $N = 20$: Prediction horizon (number of stages)
- $\Delta t = 0.15$ s: Discretization timestep
- $T = N \cdot \Delta t = 3.0$ s: Total prediction time
- $\mathbf{x}_f$: Goal state (fixed for all stages)
- $\mathbf{Q}, \mathbf{Q}_f$: Stage and terminal state cost weights
- $\mathbf{R}$: Control effort weights

The solver returns the optimal control sequence $\{\mathbf{u}_0^*, \ldots, \mathbf{u}_{N-1}^*\}$ and state trajectory $\{\mathbf{x}_0^*, \ldots, \mathbf{x}_N^*\}$. In receding-horizon MPC, only the first control $\mathbf{u}_0^*$ is executed, and the problem is resolved at the next timestep with updated initial conditions.

### 3.2 Reference Definition

The controller uses a **fixed goal reference** for all prediction stages:

$$
\mathbf{x}_{\text{ref},k} = \mathbf{x}_f, \quad \forall k \in \{0, \ldots, N\}
$$

where the goal state is:

$$
\mathbf{x}_f = \begin{bmatrix} p_{x,f} \\ p_{y,f} \\ \theta_f \\ 0 \\ 0 \\ 0 \end{bmatrix}
$$

**Key Properties**:
- Goal position $(p_{x,f}, p_{y,f})$ is specified in **world frame**
- Goal heading $\theta_f$ specifies the desired final orientation
- Goal velocities are zero: $(v_x, v_y, \omega) = (0, 0, 0)$
- The same goal is used at all stages $k$ (not a time-varying trajectory)

**Rationale**: Using a fixed goal (rather than a pre-computed reference trajectory) allows the optimizer maximum freedom to find smooth, dynamically feasible paths. The MPC generates the trajectory implicitly through the dynamics constraints and cost function.

### 3.3 Warm-Start Strategy

The SQP solver requires an initial guess for the state-control trajectory. The warm-start strategy is:

#### **First Solve** (no prior solution available):
$$
\begin{aligned}
\mathbf{x}_k^{(0)} &= \mathbf{x}_0 \quad \forall k \in \{0, \ldots, N\} \quad \text{(static trajectory)} \\
\mathbf{u}_k^{(0)} &= \mathbf{0} \quad \forall k \in \{0, \ldots, N-1\}
\end{aligned}
$$

Initialize with the current state held constant and zero controls.

#### **Subsequent Solves** (prior solution exists):
$$
\begin{aligned}
\mathbf{x}_k^{(0)} &= \mathbf{x}_{k+1}^{*,\text{prev}}, \quad k = 0, \ldots, N-1 \\
\mathbf{x}_N^{(0)} &= \mathbf{x}_N^{*,\text{prev}} + \Delta t \cdot f(\mathbf{x}_N^{*,\text{prev}}, \mathbf{0}) \\
\mathbf{u}_k^{(0)} &= \mathbf{u}_{k+1}^{*,\text{prev}}, \quad k = 0, \ldots, N-2 \\
\mathbf{u}_{N-1}^{(0)} &= \mathbf{0}
\end{aligned}
$$

**Shift the previous optimal trajectory by 1 timestep**:
- States: $\mathbf{x}_k^{(0)} \leftarrow \mathbf{x}_{k+1}^{*,\text{prev}}$ (shift forward)
- Controls: $\mathbf{u}_k^{(0)} \leftarrow \mathbf{u}_{k+1}^{*,\text{prev}}$ (shift forward)
- Extrapolate final state: $\mathbf{x}_N^{(0)} = \mathbf{x}_N^{*,\text{prev}} + \Delta t \cdot f(\mathbf{x}_N^{*,\text{prev}}, \mathbf{0})$
- Append zero control: $\mathbf{u}_{N-1}^{(0)} = \mathbf{0}$

**Rationale**: Since the robot moves approximately one timestep forward between control cycles, the previous optimal trajectory (shifted) provides an excellent initial guess near the true optimum.

### 3.4 Cost Function

#### **Stage Cost** (applied at $k = 0, \ldots, N-1$):
$$
\ell(\mathbf{x}_k, \mathbf{u}_k) = \|\mathbf{x}_k - \mathbf{x}_f\|^2_{\mathbf{Q}} + \|\mathbf{u}_k\|^2_{\mathbf{R}}
$$

Expanded form:
$$
\begin{aligned}
\ell(\mathbf{x}_k, \mathbf{u}_k) = 
& Q_{p_x} (p_{x,k} - p_{x,f})^2 + Q_{p_y} (p_{y,k} - p_{y,f})^2 + Q_\theta (\theta_k - \theta_f)^2 \\
+ & Q_{v_x} v_{x,k}^2 + Q_{v_y} v_{y,k}^2 + Q_\omega \omega_k^2 \\
+ & R_{a_x} a_{x,k}^2 + R_{a_y} a_{y,k}^2 + R_\alpha \alpha_k^2
\end{aligned}
$$

#### **Terminal Cost** (applied at $k = N$):
$$
\ell_N(\mathbf{x}_N) = \|\mathbf{x}_N - \mathbf{x}_f\|^2_{\mathbf{Q}_f}
$$

Expanded:
$$
\begin{aligned}
\ell_N(\mathbf{x}_N) = 
& Q_{f,p_x} (p_{x,N} - p_{x,f})^2 + Q_{f,p_y} (p_{y,N} - p_{y,f})^2 + Q_{f,\theta} (\theta_N - \theta_f)^2 \\
+ & Q_{f,v_x} v_{x,N}^2 + Q_{f,v_y} v_{y,N}^2 + Q_{f,\omega} \omega_N^2
\end{aligned}
$$

**Weight Matrices**:
- $\mathbf{Q} = \text{diag}(Q_{p_x}, Q_{p_y}, Q_\theta, Q_{v_x}, Q_{v_y}, Q_\omega)$
- $\mathbf{Q}_f = \text{diag}(Q_{f,p_x}, Q_{f,p_y}, Q_{f,\theta}, Q_{f,v_x}, Q_{f,v_y}, Q_{f,\omega})$
- $\mathbf{R} = \text{diag}(R_{a_x}, R_{a_y}, R_\alpha)$

Typical values:
- Position: $Q_{p_x} = Q_{p_y} = 50$ (moderate penalty)
- Heading: $Q_\theta = 10$ (less critical than position)
- Velocities: $Q_{v_x} = Q_{v_y} = 2, Q_\omega = 1$ (encourage smooth deceleration)
- Terminal: $\mathbf{Q}_f = 5 \times \mathbf{Q}$ (5x stage weights)
- Controls: $R_{a_x} = R_{a_y} = 0.1, R_\alpha = 0.05$ (penalize large accelerations)

### 3.5 Sequential Quadratic Programming (SQP) and Linearization

The nonlinear OCP is solved using **Sequential Quadratic Programming (SQP)**, which iteratively:
1. **Linearizes** the dynamics around the current trajectory guess
2. **Solves a Quadratic Program (QP)** for the optimal perturbation
3. **Updates** the trajectory guess

For **SQP-RTI (Real-Time Iteration)**, only **1 SQP iteration** is performed per control cycle, making the quality of the warm-start critical.

#### **SQP Iteration**

At iteration $j$, given a current trajectory guess $\{\mathbf{x}_k^{(j)}, \mathbf{u}_k^{(j)}\}$:

**Step 1: Linearize Dynamics**

Compute the Jacobians of the dynamics $f(\mathbf{x}, \mathbf{u})$ at each stage:

$$
\mathbf{A}_k^{(j)} = \frac{\partial f}{\partial \mathbf{x}}\bigg|_{\mathbf{x}_k^{(j)}, \mathbf{u}_k^{(j)}}, \quad
\mathbf{B}_k^{(j)} = \frac{\partial f}{\partial \mathbf{u}}\bigg|_{\mathbf{x}_k^{(j)}, \mathbf{u}_k^{(j)}}
$$

These are $6 \times 6$ and $6 \times 3$ matrices, respectively.

**Discrete-time linearization** (forward Euler):

$$
\begin{aligned}
\mathbf{A}_k^{(j)} &= \mathbf{I} + \Delta t \cdot \frac{\partial f}{\partial \mathbf{x}}\bigg|_{\mathbf{x}_k^{(j)}, \mathbf{u}_k^{(j)}} \\
\mathbf{B}_k^{(j)} &= \Delta t \cdot \frac{\partial f}{\partial \mathbf{u}}\bigg|_{\mathbf{x}_k^{(j)}, \mathbf{u}_k^{(j)}}
\end{aligned}
$$

**Jacobian structure** (for forward Euler discretization):

$$
\mathbf{A}_k = \begin{bmatrix}
1 & 0 & \Delta t \frac{\partial \dot{p}_x}{\partial \theta} & \Delta t \cos\theta & -\Delta t \sin\theta & 0 \\
0 & 1 & \Delta t \frac{\partial \dot{p}_y}{\partial \theta} & \Delta t \sin\theta & \Delta t \cos\theta & 0 \\
0 & 0 & 1 & 0 & 0 & \Delta t \\
0 & 0 & 0 & 1 & 0 & 0 \\
0 & 0 & 0 & 0 & 1 & 0 \\
0 & 0 & 0 & 0 & 0 & 1
\end{bmatrix}, \quad
\mathbf{B}_k = \begin{bmatrix}
0 & 0 & 0 \\
0 & 0 & 0 \\
0 & 0 & 0 \\
\Delta t & 0 & 0 \\
0 & \Delta t & 0 \\
0 & 0 & \Delta t
\end{bmatrix}
$$

where:
$$
\frac{\partial \dot{p}_x}{\partial \theta} = -\sin\theta \cdot v_x - \cos\theta \cdot v_y, \quad
\frac{\partial \dot{p}_y}{\partial \theta} = \cos\theta \cdot v_x - \sin\theta \cdot v_y
$$

**Step 2: Formulate QP**

The nonlinear dynamics constraint:
$$
\mathbf{x}_{k+1} = \mathbf{x}_k + \Delta t \cdot f(\mathbf{x}_k, \mathbf{u}_k)
$$

is approximated by a **linear constraint** around the current guess:

$$
\mathbf{x}_{k+1} \approx \mathbf{x}_k^{(j)} + \Delta t \cdot f(\mathbf{x}_k^{(j)}, \mathbf{u}_k^{(j)}) + \mathbf{A}_k^{(j)} (\mathbf{x}_k - \mathbf{x}_k^{(j)}) + \mathbf{B}_k^{(j)} (\mathbf{u}_k - \mathbf{u}_k^{(j)})
$$

Defining perturbations:
$$
\Delta \mathbf{x}_k = \mathbf{x}_k - \mathbf{x}_k^{(j)}, \quad \Delta \mathbf{u}_k = \mathbf{u}_k - \mathbf{u}_k^{(j)}
$$

The linearized dynamics become:
$$
\Delta \mathbf{x}_{k+1} = \mathbf{A}_k^{(j)} \Delta \mathbf{x}_k + \mathbf{B}_k^{(j)} \Delta \mathbf{u}_k
$$

The QP solves for optimal perturbations $\{\Delta \mathbf{x}_k, \Delta \mathbf{u}_k\}$ subject to the linearized dynamics.

**Step 3: Update Trajectory**

$$
\mathbf{x}_k^{(j+1)} = \mathbf{x}_k^{(j)} + \Delta \mathbf{x}_k^*, \quad \mathbf{u}_k^{(j+1)} = \mathbf{u}_k^{(j)} + \Delta \mathbf{u}_k^*
$$

#### **Linearization Point = Warm-Start Trajectory**

In SQP-RTI with 1 iteration:
- The **warm-start trajectory** $\{\mathbf{x}_k^{(0)}, \mathbf{u}_k^{(0)}\}$ **IS the linearization point**
- The Jacobians $\mathbf{A}_k, \mathbf{B}_k$ are evaluated at the warm-start
- The QP is solved once, yielding the final solution

This is why warm-start quality is critical: the linearization is only valid near the warm-start, so a poor initial guess degrades solution quality.

#### **Example: Position Dynamics Linearization**

Consider the $p_x$ dynamics:
$$
\dot{p}_x = \cos(\theta) v_x - \sin(\theta) v_y
$$

**Nonlinear propagation** over $\Delta t = 0.15$ s:
$$
p_{x,k+1} = p_{x,k} + 0.15 \cdot \left(\cos(\theta_k) v_{x,k} - \sin(\theta_k) v_{y,k}\right)
$$

**Linearized propagation** around ($\theta^{(j)}_k, v^{(j)}_{x,k}, v^{(j)}_{y,k}$)

$$
\begin{aligned}
p_{x,k+1} \approx p_{x,k} &+ 0.15 \cdot \left(\cos(\theta_k^{(j)}) v_{x,k}^{(j)} - \sin(\theta_k^{(j)}) v_{y,k}^{(j)}\right) \\
&+ 0.15 \cdot \left(-\sin(\theta_k^{(j)}) v_{x,k}^{(j)} - \cos(\theta_k^{(j)}) v_{y,k}^{(j)}\right) \Delta \theta_k \\
&+ 0.15 \cdot \cos(\theta_k^{(j)}) \Delta v_{x,k} \\
&- 0.15 \cdot \sin(\theta_k^{(j)}) \Delta v_{y,k}
\end{aligned}
$$

where $\Delta \theta_k = \theta_k - \theta_k^{(j)}$, $\Delta v_{x,k} = v_{x,k} - v_{x,k}^{(j)}$, etc.

**Linearization error**: If the actual state deviates significantly from the linearization point, the linear approximation becomes inaccurate. Good warm-starting (via shifted previous solution) keeps the deviation small.

### 3.6 Constraints

#### **Box Constraints on Controls** (enforced at $k = 0, \ldots, N-1$):

$$
\begin{aligned}
-0.5 \leq a_x &\leq 0.5 \quad \text{[m/s²]} \\
-0.5 \leq a_y &\leq 0.5 \quad \text{[m/s²]} \\
-5.0 \leq \alpha &\leq 5.0 \quad \text{[rad/s²]}
\end{aligned}
$$

#### **Velocity Ellipse Constraint** (enforced at $k = 0, \ldots, N$):

$$
\left(\frac{v_x}{v_{\max,x}}\right)^2 + \left(\frac{v_y}{v_{\max,y}}\right)^2 \leq 1
$$

where $v_{\max,x} = 0.3$ m/s (forward), $v_{\max,y} = 0.2$ m/s (sideways).

Prevents commanding velocities the robot cannot execute.

#### **Terminal Velocity Constraints** (enforced at $k = N$):

$$
v_{x,N} = 0, \quad v_{y,N} = 0
$$

**Hard equality constraints** ensuring the robot comes to rest at the goal. These are implemented as bounds:

$$
\mathbf{x}_N \in \{\mathbf{x} : v_x = 0, v_y = 0\}
$$

In acados, set terminal bounds:
```python
ocp.constraints.lbx_e = np.array([..., 0, 0, ...])  # v_x,N = 0, v_y,N = 0
ocp.constraints.ubx_e = np.array([..., 0, 0, ...])
ocp.constraints.idxbx_e = [3, 4]  # Indices for v_x, v_y
```

---

## 4. Solver Configuration

### 4.1 acados Framework

The MPC is implemented using **acados** (https://docs.acados.org/), an open-source optimal control framework providing:
- Automatic differentiation for computing Jacobians
- High-performance QP solvers (HPIPM, OSQP)
- SQP and SQP-RTI algorithms
- C code generation for embedded deployment

**Key acados settings**:
- `nlp_solver_type`: `SQP_RTI` (1 iteration per cycle)
- `qp_solver`: `PARTIAL_CONDENSING_HPIPM` (structure-exploiting QP solver)
- `integrator_type`: `IRK` (Implicit Runge-Kutta, 4th order)
- `sim_method_num_stages`: 4 (RK4)
- `sim_method_num_steps`: 1 (1 integration step per stage)

### 4.2 QP Solver: HPIPM

**HPIPM** (High-Performance Interior-Point Method) exploits the block-tridiagonal structure of the MPC problem:

$$
\begin{bmatrix}
\mathbf{Q}_0 & \mathbf{S}_0^\top & & & \\
\mathbf{S}_0 & \mathbf{R}_0 & & & \\
& & \ddots & & \\
& & & \mathbf{Q}_N &
\end{bmatrix}
$$

Achieves $\mathcal{O}(N)$ complexity (linear in horizon length).

**Performance**: Typical solve time 1-3 ms for $N=20$ on modern CPUs.

### 4.3 Integration Method

**Implicit Runge-Kutta (IRK)** of order 4 is used for discrete-time integration:

$$
\mathbf{x}_{k+1} = \mathbf{x}_k + \Delta t \sum_{i=1}^{s} b_i f(\mathbf{x}_k + \Delta t \sum_{j=1}^{s} a_{ij} \mathbf{k}_j, \mathbf{u}_k)
$$

where $\mathbf{k}_i$ are stage derivatives satisfying implicit equations (Butcher tableau).

**Advantages**:
- Higher accuracy than forward Euler
- Stable for stiff systems
- Suitable for real-time control

---

## 5. Key Parameters

| **Parameter** | **Symbol** | **Value** | **Units** | **Description** |
|---------------|------------|-----------|-----------|-----------------|
| **Prediction Horizon** | $N$ | 20 | stages | Number of prediction steps |
| **Timestep** | $\Delta t$ | 0.15 | s | Discretization timestep |
| **Total Horizon Time** | $T$ | 3.0 | s | $T = N \cdot \Delta t$ |
| **Max Forward Velocity** | $v_{\max,x}$ | 0.3 | m/s | Forward speed limit |
| **Max Sideways Velocity** | $v_{\max,y}$ | 0.2 | m/s | Sideways speed limit |
| **Max Angular Velocity** | $\omega_{\max}$ | 2.0 | rad/s | Yaw rate limit (not explicitly constrained) |
| **Max Linear Acceleration** | $a_{\max}$ | 0.5 | m/s² | Acceleration bounds |
| **Max Angular Acceleration** | $\alpha_{\max}$ | 5.0 | rad/s² | Angular acceleration bounds |
| **Position Cost Weight** | $Q_{p_x}, Q_{p_y}$ | 50 | - | Stage cost for position error |
| **Heading Cost Weight** | $Q_\theta$ | 10 | - | Stage cost for heading error |
| **Velocity Cost Weight** | $Q_{v_x}, Q_{v_y}$ | 2 | - | Stage cost for velocities |
| **Angular Velocity Cost Weight** | $Q_\omega$ | 1 | - | Stage cost for angular velocity |
| **Terminal Position Cost** | $Q_{f,p_x}, Q_{f,p_y}$ | 250 | - | Terminal cost for position (5× stage) |
| **Terminal Heading Cost** | $Q_{f,\theta}$ | 50 | - | Terminal cost for heading (5× stage) |
| **Terminal Velocity Cost** | $Q_{f,v_x}, Q_{f,v_y}$ | 10 | - | Terminal cost for velocities (5× stage) |
| **Terminal Angular Vel. Cost** | $Q_{f,\omega}$ | 5 | - | Terminal cost for angular velocity (5× stage) |
| **Linear Accel. Cost Weight** | $R_{a_x}, R_{a_y}$ | 0.1 | - | Control effort penalty |
| **Angular Accel. Cost Weight** | $R_\alpha$ | 0.05 | - | Angular control effort penalty |
| **SQP Iterations** | - | 1 | - | Real-time iteration (RTI) |
| **QP Solver** | - | HPIPM | - | High-performance interior-point method |
| **Integrator** | - | IRK4 | - | 4th-order implicit Runge-Kutta |

---

## 6. Implementation Workflow

### 6.1 Setup Phase

1. **Generate acados solver** (Python script `generate_acados_solver.py`):
   - Define system dynamics $f(\mathbf{x}, \mathbf{u})$
   - Set cost function (LINEAR_LS type with fixed reference)
   - Configure constraints (box bounds, velocity ellipse, terminal velocity)
   - Specify solver options (SQP-RTI, HPIPM, IRK)
   - Generate C code in `generated/` folder

2. **Compile C++ interface** (`HumanoidReferenceMPC.cpp`):
   - Links to generated acados solver
   - Provides `computeControl()` method
   - Manages warm-starting logic

### 6.2 Real-Time Control Loop

At each control cycle (e.g., 50 Hz):

1. **Measure current state** $\mathbf{x}_0$ from odometry/localization
2. **Set initial condition**: `ocp_nlp_constraints_model_set()` with $\mathbf{x}_0$
3. **Set reference**: `ocp_nlp_cost_model_set()` with goal $\mathbf{x}_f$ for all stages
4. **Warm-start** (if prior solution exists):
   - Shift previous states: `ocp_nlp_out_set()` with $\mathbf{x}_{k+1}^{*,\text{prev}}$
   - Shift previous controls: `ocp_nlp_out_set()` with $\mathbf{u}_{k+1}^{*,\text{prev}}$
5. **Solve OCP**: `robot_mpc_acados_solve()` (1 SQP-RTI iteration)
6. **Extract solution**: `ocp_nlp_out_get()` to retrieve $\mathbf{u}_0^*$
7. **Execute control**: Send $\mathbf{u}_0^*$ to low-level controller
8. **Store solution**: Save $\{\mathbf{x}_k^*, \mathbf{u}_k^*\}$ for next warm-start

### 6.3 C++ API Example

```cpp
// Set current state
mpc.setInitialState(current_state);

// Set goal (world frame)
Eigen::Vector3d goal_position(5.0, 3.0, M_PI/4);  // (x, y, theta)
mpc.setGoal(goal_position);

// Compute optimal control (with warm-starting)
Eigen::Vector3d control = mpc.computeControl();

// Extract: control = (a_x, a_y, alpha)
double a_x = control(0);
double a_y = control(1);
double alpha = control(2);

// Send to robot
robot.setAccelerationCommand(a_x, a_y, alpha);
```

---

## 7. Extensions and Future Work

### 7.1 Obstacle Avoidance

Add nonlinear inequality constraints to avoid circular obstacles:

$$
\|[\mathbf{p}_k - \mathbf{p}_{\text{obs},i}]\|^2 \geq r_{\text{safe}}^2, \quad \forall i \in \{1, \ldots, N_{\text{obs}}\}
$$

where $\mathbf{p}_k = (p_{x,k}, p_{y,k})$ and $r_{\text{safe}}$ is the safety radius.

**Implementation**: Use acados' nonlinear constraint feature (`ocp.constraints.expr_h`).

### 7.2 Multi-Waypoint Navigation

Instead of a single goal, track a sequence of waypoints $\{\mathbf{w}_1, \ldots, \mathbf{w}_M\}$:
- Set $\mathbf{x}_f = \mathbf{w}_1$ initially
- When $\|\mathbf{p}_{\text{current}} - \mathbf{w}_1\| < \epsilon$, switch to $\mathbf{x}_f = \mathbf{w}_2$
- Repeat until all waypoints reached

### 7.3 Adaptive Cost Weight Tuning

Use **Bayesian Optimization** to tune cost weights $\{\mathbf{Q}, \mathbf{R}, \mathbf{Q}_f\}$ for:
- Minimize settling time
- Minimize trajectory smoothness metrics (jerk, curvature)
- Maximize solver reliability (success rate, solve time)

**Objective function**:
$$
J(\mathbf{Q}, \mathbf{R}, \mathbf{Q}_f) = w_1 \cdot t_{\text{settle}} + w_2 \cdot \int \|\mathbf{a}(t)\|^2 dt + w_3 \cdot \bar{t}_{\text{solve}}
$$

### 7.4 Time-Optimal Formulation

Introduce variable timestep $\Delta t$ as an optimization variable (minimum-time problem):

$$
\min_{\Delta t, \mathbf{u}_0, \ldots, \mathbf{u}_{N-1}} \quad N \cdot \Delta t
$$

subject to the same dynamics and constraints. Requires nonlinear reformulation (moving horizon).

### 7.5 Robustness to Model Uncertainty

- **Tube MPC**: Tighten constraints to account for bounded disturbances
- **Stochastic MPC**: Incorporate Gaussian noise in dynamics, solve chance-constrained OCP
- **Adaptive MPC**: Online learning of model parameters (friction, inertia)

---

## 8. References and Resources

### 8.1 Key Papers

1. **acados Framework**:
   - Verschueren et al., "acados—a modular open-source framework for fast embedded optimal control," *Mathematical Programming Computation*, 2022.
   
2. **SQP-RTI**:
   - Diehl et al., "Real-time optimization for large scale nonlinear processes," PhD Thesis, 2001.
   
3. **HPIPM**:
   - Frison et al., "HPIPM: a high-performance quadratic programming framework for model predictive control," *IFAC-PapersOnLine*, 2020.

### 8.2 Documentation

- acados: https://docs.acados.org/
- HPIPM: https://github.com/giaf/hpipm
- RoboCup SPL: https://spl.robocup.org/

### 8.3 Related Work

- **Nonlinear MPC for Mobile Robots**: Similar 3-DOF unicycle models with global positioning
- **Formation Control**: Multi-robot MPC with communication constraints
- **Learning-Based MPC**: Combine RL policies with MPC for improved warm-starting

---

## Appendix A: Implementation Checklist

### A.1 Python Solver Generation (`generate_acados_solver.py`)

- [ ] Define dynamics model with correct coordinate frames (position in world, velocity in body)
- [ ] Set cost to LINEAR_LS with fixed goal reference for all stages
- [ ] Add terminal velocity constraints: `lbx_e[3:5] = [0, 0]`, `ubx_e[3:5] = [0, 0]`
- [ ] Configure SQP-RTI: `nlp_solver_type = 'SQP_RTI'`, `nlp_solver_max_iter = 1`
- [ ] Set integrator: `integrator_type = 'IRK'`, `sim_method_num_stages = 4`
- [ ] Generate code: `ocp_solver = AcadosOcpSolver(ocp)`

### A.2 C++ Interface (`HumanoidReferenceMPC.cpp`)

- [ ] Remove coordinate frame transformations (work in world frame directly)
- [ ] Implement warm-start shifting:
  ```cpp
  for (int i = 0; i < N; ++i) {
      ocp_nlp_out_set(config, dims, out, i, "x", prev_states[i+1].data());
      if (i < N-1) {
          ocp_nlp_out_set(config, dims, out, i, "u", prev_controls[i+1].data());
      }
  }
  ```
- [ ] Set fixed goal reference (same for all stages $k=0,\ldots,N$):
  ```cpp
  for (int i = 0; i <= N; ++i) {
      robot_mpc_acados_update_params(acados_ocp_capsule, i, goal_state.data(), NX);
  }
  ```
- [ ] Store solution after solve for next warm-start

### A.3 Testing

- [ ] Verify zero control input when starting at goal
- [ ] Test straight-line motion (goal ahead at $\theta = 0$)
- [ ] Test turning motion (goal at different heading)
- [ ] Validate terminal constraints: $v_x = v_y = 0$ at horizon end
- [ ] Benchmark solve time (should be $< 5$ ms for $N=20$)
- [ ] Test warm-start quality: compare solve time and cost with/without warm-start

---

**Document Version**: 1.0  
**Last Updated**: 2024  
**Author**: RoboCup MPC Development Team

