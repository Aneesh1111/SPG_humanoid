### HumanoidMPC Mode 

**How it works**:
1. **Frame Transformation**: Convert goal from global → robot local frame
2. **MPC Optimization**: Solve QP problem to find optimal control sequence
3. **Control Extraction**: Take first control from sequence
4. **Frame Transformation**: Convert control from body → global frame
5. **Trajectory Forward Simulation**: Use full control sequence for visualization


**Parameters**:
```cpp

// Physical limits
mpc_params.vf_max = 1.2;      // [m/s] forward velocity
mpc_params.vs_max = 0.4;      // [m/s] sideways velocity
mpc_params.omega_max = 1.0;   // [rad/s] rotation rate

// BO-ready weights (10 tunable parameters)
mpc_params.weights.q_pos = 1.0;       // stage position cost
mpc_params.weights.q_phi = 0.1;       // stage heading cost
mpc_params.weights.qf_pos = 8.0;      // terminal position cost
mpc_params.weights.qf_phi = 1.0;      // terminal heading cost
mpc_params.weights.r_vf = 0.1;        // forward effort cost
mpc_params.weights.r_vs = 0.5;        // sideways effort cost
mpc_params.weights.r_omega = 0.2;     // rotation effort cost
mpc_params.weights.s_vf = 0.2;        // forward smoothness cost
mpc_params.weights.s_vs = 0.8;        // sideways smoothness cost
mpc_params.weights.s_omega = 0.4;     // rotation smoothness cost
```

---

## HumanoidMPC Algorithm Details

### Mathematical Formulation

**File**: `src/spg/setpoint/HumanoidMPC.cpp`

This section presents the complete MPC formulation in standard control-theoretic notation.

---

#### 1. State and Control Spaces

**State vector** (robot-local frame):
$$\mathbf{x}_k = \begin{bmatrix} x_k \\ y_k \\ \phi_k \end{bmatrix} \in \mathbb{R}^3$$

where:
- $x_k$ : forward displacement from robot origin [m]
- $y_k$ : lateral displacement [m]  
- $\phi_k$ : heading relative to robot's initial orientation [rad]

**Control vector** (body frame):
$$\mathbf{u}_k = \begin{bmatrix} v_{f,k} \\ v_{s,k} \\ \omega_k \end{bmatrix} \in \mathbb{R}^3$$

where:
- $v_{f,k}$ : forward velocity [m/s]
- $v_{s,k}$ : sideways velocity [m/s]
- $\omega_k$ : yaw rate [rad/s]

**Control normalization**:
$$\tilde{\mathbf{u}}_k = \begin{bmatrix} \tilde{u}_{f,k} \\ \tilde{u}_{s,k} \\ \tilde{u}_{\omega,k} \end{bmatrix} = \begin{bmatrix} v_{f,k} / v_{f}^{\max} \\ v_{s,k} / v_{s}^{\max} \\ \omega_k / \omega^{\max} \end{bmatrix} \in [-1, 1]^3$$

---

#### 2. System Dynamics

**Nonlinear discrete-time kinematics** (sampling time $\Delta t = 0.02$ s):
$$\mathbf{x}_{k+1} = \mathbf{f}(\mathbf{x}_k, \mathbf{u}_k) = \mathbf{x}_k + \Delta t \begin{bmatrix} v_{f,k} \cos(\phi_k) - v_{s,k} \sin(\phi_k) \\ v_{f,k} \sin(\phi_k) + v_{s,k} \cos(\phi_k) \\ \omega_k \end{bmatrix}$$

**Linearization** about nominal heading $\phi_0$ (typically 0 in local frame):
$$\mathbf{x}_{k+1} \approx \mathbf{x}_k + \mathbf{B} \mathbf{u}_k$$

where the input matrix is:
$$\mathbf{B} = \Delta t \begin{bmatrix} \cos(\phi_0) & -\sin(\phi_0) & 0 \\ \sin(\phi_0) & \cos(\phi_0) & 0 \\ 0 & 0 & 1 \end{bmatrix}$$

---

#### 3. Cost Function

The **goal-tracking MPC** minimizes the following cost over horizon $N$:

$$J(\mathbf{x}_0, \tilde{\mathbf{U}}) = \sum_{k=0}^{N-1} \left[ \ell_k(\mathbf{x}_k, \tilde{\mathbf{u}}_k) + \ell_{\Delta}(\Delta\tilde{\mathbf{u}}_k) \right] + \ell_N(\mathbf{x}_N)$$

**Stage cost** (tracking + control effort):
$$\ell_k(\mathbf{x}_k, \tilde{\mathbf{u}}_k) = \|\mathbf{x}_k - \mathbf{x}_{\text{goal}}\|_{\mathbf{Q}}^2 + \|\tilde{\mathbf{u}}_k\|_{\mathbf{R}}^2$$

**Smoothness cost** (penalize control rate changes):
$$\ell_{\Delta}(\Delta\tilde{\mathbf{u}}_k) = \|\Delta\tilde{\mathbf{u}}_k\|_{\mathbf{S}}^2$$

where:
$$\Delta\tilde{\mathbf{u}}_k = \begin{cases} \tilde{\mathbf{u}}_0 - \tilde{\mathbf{u}}_{\text{meas}} & k = 0 \\ \tilde{\mathbf{u}}_k - \tilde{\mathbf{u}}_{k-1} & k \geq 1 \end{cases}$$

and $\tilde{\mathbf{u}}_{\text{meas}}$ is the **measured current velocity** (normalized), ensuring smooth transitions.

**Terminal cost** (strong end-state tracking):
$$\ell_N(\mathbf{x}_N) = \|\mathbf{x}_N - \mathbf{x}_{\text{goal}}\|_{\mathbf{Q}_f}^2$$

**Cost weight matrices**:
$$\mathbf{Q} = \text{diag}(q_{\text{pos}}, q_{\text{pos}}, q_{\phi}) \in \mathbb{R}^{3 \times 3}$$
$$\mathbf{Q}_f = \text{diag}(q_{f,\text{pos}}, q_{f,\text{pos}}, q_{f,\phi}) \in \mathbb{R}^{3 \times 3}$$
$$\mathbf{R} = \text{diag}(r_{v_f}, r_{v_s}, r_{\omega}) \in \mathbb{R}^{3 \times 3}$$
$$\mathbf{S} = \text{diag}(s_{v_f}, s_{v_s}, s_{\omega}) \in \mathbb{R}^{3 \times 3}$$

**Note**: $\mathbf{Q}_f \succ \mathbf{Q}$ (terminal cost stronger than stage cost) to ensure goal convergence.

---

#### 4. Optimization Problem

The finite-horizon optimal control problem is:

$$\begin{aligned}
\min_{\tilde{\mathbf{U}}} \quad & J(\mathbf{x}_0, \tilde{\mathbf{U}}) \\
\text{s.t.} \quad & \mathbf{x}_{k+1} = \mathbf{f}(\mathbf{x}_k, \mathbf{u}_k), \quad k = 0, \ldots, N-1 \\
& \mathbf{x}_0 = \mathbf{x}_{\text{current}} \quad \text{(initial condition)} \\
& -1 \leq \tilde{u}_{i,k} \leq 1, \quad i \in \{f, s, \omega\}, \; k = 0, \ldots, N-1
\end{aligned}$$

where the decision variable is the stacked control sequence:
$$\tilde{\mathbf{U}} = \begin{bmatrix} \tilde{\mathbf{u}}_0 \\ \tilde{\mathbf{u}}_1 \\ \vdots \\ \tilde{\mathbf{u}}_{N-1} \end{bmatrix} \in \mathbb{R}^{3N}$$

---

#### 5. Quadratic Program (QP) Reformulation

By substituting the linear dynamics recursively, we eliminate state variables and obtain a dense QP in **control space only**:

$$\begin{aligned}
\min_{\tilde{\mathbf{U}}} \quad & \frac{1}{2} \tilde{\mathbf{U}}^T \mathbf{H} \tilde{\mathbf{U}} + \mathbf{f}^T \tilde{\mathbf{U}} \\
\text{s.t.} \quad & \mathbf{lb} \leq \tilde{\mathbf{U}} \leq \mathbf{ub}
\end{aligned}$$

**Box constraints**:
$$\mathbf{lb} = -\mathbf{1}_{3N}, \quad \mathbf{ub} = +\mathbf{1}_{3N}$$

**Hessian matrix** (positive definite, $3N \times 3N$):
$$\mathbf{H} = 2\left( \mathbf{S}_u^T \bar{\mathbf{Q}} \mathbf{S}_u + \bar{\mathbf{R}} + \mathbf{L}^T \bar{\mathbf{S}} \mathbf{L} \right)$$

**Gradient vector** ($3N \times 1$):
$$\mathbf{f} = 2\left( \mathbf{S}_u^T \bar{\mathbf{Q}} (\mathbf{S}_x \mathbf{x}_0 - \bar{\mathbf{x}}_{\text{goal}}) + \mathbf{L}^T \bar{\mathbf{S}} \mathbf{l}_0 \right)$$

where:

**State propagation matrix** $\mathbf{S}_u \in \mathbb{R}^{3(N+1) \times 3N}$:
$$\mathbf{X} = \mathbf{S}_x \mathbf{x}_0 + \mathbf{S}_u \tilde{\mathbf{U}}$$

with $\mathbf{X} = [\mathbf{x}_1^T, \ldots, \mathbf{x}_N^T]^T$ (excludes $\mathbf{x}_0$).

**Control difference matrix** $\mathbf{L} \in \mathbb{R}^{3N \times 3N}$:
$$\Delta\tilde{\mathbf{U}} = \mathbf{L} \tilde{\mathbf{U}} + \mathbf{l}_0$$

where $\mathbf{l}_0 = [-\tilde{\mathbf{u}}_{\text{meas}}^T, \mathbf{0}_{3(N-1)}^T]^T$ encodes the measured initial velocity.

**Block-diagonal weight matrices**:
$$\bar{\mathbf{Q}} = \text{diag}(\mathbf{Q}, \ldots, \mathbf{Q}, \mathbf{Q}_f) \in \mathbb{R}^{3(N+1) \times 3(N+1)}$$
$$\bar{\mathbf{R}} = \text{diag}(\mathbf{R}, \ldots, \mathbf{R}) \in \mathbb{R}^{3N \times 3N}$$
$$\bar{\mathbf{S}} = \text{diag}(\mathbf{S}, \ldots, \mathbf{S}) \in \mathbb{R}^{3N \times 3N}$$

**Goal reference vector**:
$$\bar{\mathbf{x}}_{\text{goal}} = \begin{bmatrix} \mathbf{x}_{\text{goal}} \\ \vdots \\ \mathbf{x}_{\text{goal}} \end{bmatrix} \in \mathbb{R}^{3(N+1)}$$

---

#### 6. QP Solver and Receding Horizon

**Solver**: [qpOASES](https://projects.coin-or.org/qpOASES) (online active-set method)
- **Warm-starting**: Solution from previous timestep used as initial guess
- **Max iterations**: 50 (typically converges in 20-40)
- **Computation time**: 1-3 ms on modern CPU (Intel i7, 3.5 GHz)

**Receding horizon policy**:
1. Solve QP at time $t$ to obtain $\tilde{\mathbf{U}}^* = [\tilde{\mathbf{u}}_0^*, \ldots, \tilde{\mathbf{u}}_{N-1}^*]$
2. Apply **only first control**: $\mathbf{u}(t) = \mathbf{u}_0^*$
3. At time $t + \Delta t$, measure new state $\mathbf{x}_{\text{new}}$ and repeat

---

#### 7. Trajectory Prediction (Forward Simulation)

For visualization and planning, we propagate the **full control sequence** using the **nonlinear dynamics**:

$$\mathbf{x}_{k+1} = \mathbf{x}_k + \Delta t \begin{bmatrix} v_{f,k}^* \cos(\phi_k) - v_{s,k}^* \sin(\phi_k) \\ v_{f,k}^* \sin(\phi_k) + v_{s,k}^* \cos(\phi_k) \\ \omega_k^* \end{bmatrix}, \quad k = 0, \ldots, N-1$$

This produces:
- **State trajectory**: $\{\mathbf{x}_0, \mathbf{x}_1, \ldots, \mathbf{x}_N\}$ (in robot-local frame)
- **Control trajectory**: $\{\mathbf{u}_0^*, \mathbf{u}_1^*, \ldots, \mathbf{u}_{N-1}^*\}$

These are transformed back to the **global frame** for visualization and integration with the SPG system.

---

#### 8. Key Design Choices

**Why separate stage and terminal costs?**
- **Stage cost** $\mathbf{Q}$: Encourages progress toward goal during motion
- **Terminal cost** $\mathbf{Q}_f \succ \mathbf{Q}$: Ensures convergence to goal at end of horizon
- Without strong terminal cost, robot may "orbit" near goal without reaching it
- Typical ratio: $q_{f,\text{pos}} / q_{\text{pos}} \approx 8$

**Why measure velocity for smoothness?**
- Using $\Delta\tilde{\mathbf{u}}_0 = \tilde{\mathbf{u}}_0 - \tilde{\mathbf{u}}_{\text{meas}}$ ensures **continuity** across control cycles
- Prevents jerky transitions when re-planning at 50 Hz
- Critical for physical robots with momentum

**Why normalize controls?**
- Asymmetric velocity limits ($v_f^{\max} \neq v_s^{\max}$) would distort QP geometry
- Normalization to $[-1, 1]^3$ makes box constraints uniform
- Weight matrices $\mathbf{R}$, $\mathbf{S}$ can then balance control priorities directly

---

#### 9. Computational Complexity

- **QP size**: $3N$ decision variables, $6N$ box constraints (with $N = 10$: 30 vars, 60 constraints)
- **Dense formulation**: $\mathbf{H}$ is $3N \times 3N$ dense (no sparsity exploited)
- **Factorization**: One-time Cholesky of $\mathbf{H}$ per QP solve
- **Active-set updates**: $O(n^2)$ per iteration, warm-starting reduces iterations
- **Real-time feasibility**: Consistently meets 20 ms deadline (50 Hz control loop)
