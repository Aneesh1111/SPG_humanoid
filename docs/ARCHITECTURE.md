# SPG System Architecture Documentation

## Table of Contents
1. [System Overview](#system-overview)
2. [Control Flow](#control-flow)
3. [Coordinate Frames](#coordinate-frames)
4. [MSL Mode vs HumanoidMPC Mode](#msl-mode-vs-humanoidmpc-mode)
5. [HumanoidMPC Algorithm Details](#humanoidmpc-algorithm-details)
6. [Code Structure and Integration](#code-structure-and-integration)
7. [Visualization System](#visualization-system)

---

## System Overview

The **SPG (Setpoint Generator)** framework is a real-time path planning and control system for soccer robots. It provides two control modes:

1. **MSL Mode**: Traditional Middle Size League controller using trajectory segments
2. **HumanoidMPC Mode**: Model Predictive Control for humanoid robots

### Key Features
- **50 Hz control loop** (20ms timestep)
- **Real-time trajectory planning**
- **Obstacle avoidance** (static and dynamic)
- **Field boundary awareness**
- **ImGui/ImPlot visualization**
- **Mode switching** between MSL and HumanoidMPC

---

## Control Flow

```
┌─────────────────────────────────────────────────────────────┐
│                     SPG Control Loop                         │
│                     (50 Hz / 20ms)                          │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
         ┌──────────────────────────────────┐
         │   1. Initialize SPG State (d)    │
         │   - Robot position (global)       │
         │   - Current velocity (global)     │
         │   - Target/Subtarget              │
         └──────────────────────────────────┘
                            │
                            ▼
         ┌──────────────────────────────────┐
         │   2. Target Calculation          │
         │   Target::Set(d)                 │
         │   - Adjust for field boundaries   │
         │   - Adjust for obstacles          │
         │   - Apply 3m rule                 │
         └──────────────────────────────────┘
                            │
                            ▼
         ┌──────────────────────────────────┐
         │   3. Subtarget Calculation       │
         │   Subtarget::Set(d)              │
         │   - Intermediate waypoint         │
         │   - Collision checking            │
         │   - Replanning if needed          │
         └──────────────────────────────────┘
                            │
                            ▼
         ┌──────────────────────────────────┐
         │   4. Setpoint Generation         │
         │   setpoint::Set(d)               │
         │   ┌───────────────────────────┐  │
         │   │ Mode Selection:            │  │
         │   │                            │  │
         │   │ if use_humanoid_mpc:      │  │
         │   │   ├─> HumanoidMPC Mode    │  │
         │   │   └─> (MPC optimization)  │  │
         │   │ else:                      │  │
         │   │   ├─> MSL Mode             │  │
         │   │   └─> (Segment-based)      │  │
         │   └───────────────────────────┘  │
         └──────────────────────────────────┘
                            │
                            ▼
         ┌──────────────────────────────────┐
         │   5. Output                      │
         │   - d.setpoint.p (position)      │
         │   - d.setpoint.v (velocity)      │
         │   - d.setpoint.a (acceleration)  │
         │   - d.traj (predicted trajectory)│
         └──────────────────────────────────┘
```

---

## Coordinate Frames

The system uses **two coordinate frames** that must be carefully managed:

### Global (World) Frame
- **Origin**: Fixed point on field (typically field center)
- **X-axis**: Along field length
- **Y-axis**: Along field width
- **Used by**:
  - SPG state variables (`d.setpoint.p`, `d.setpoint.v`)
  - Field boundaries
  - Target and subtarget positions
  - Visualization

### Robot Local (Body) Frame
- **Origin**: Robot's current position
- **X-axis**: Robot's forward direction (aligned with heading φ)
- **Y-axis**: Robot's left direction (90° from heading)
- **Used by**:
  - HumanoidMPC solver (solves in local frame)
  - Control commands (vf = forward, vs = sideways)

### Frame Transformation Math

**Global → Local:**
```math
[x_local]   [  cos(φ)  sin(φ) ] [x_global - robot_x]
[y_local] = [ -sin(φ)  cos(φ) ] [y_global - robot_y]
```

**Local → Global:**
```math
[x_global]   [cos(φ) -sin(φ)] [x_local]   [robot_x]
[y_global] = [sin(φ)  cos(φ)] [y_local] + [robot_y]
```

**Velocity Transformation:**
```cpp
// Global → Body frame
vf = vx_global * cos(φ) + vy_global * sin(φ);
vs = -vx_global * sin(φ) + vy_global * cos(φ);

// Body → Global frame
vx_global = vf * cos(φ) - vs * sin(φ);
vy_global = vf * sin(φ) + vs * cos(φ);
```

---

## MSL Mode vs HumanoidMPC Mode

### MSL Mode (Traditional)

**File**: `src/spg/setpoint/Set.cpp` (lines 206-218)

**How it works**:
1. **Segment Generation**: Compute trajectory segments from current state to subtarget
   - Uses `getSegments()` to plan acceleration/deceleration phases
   - Respects velocity and acceleration limits
2. **Trajectory Propagation**: Generate full trajectory using `Traj1()` and `TrajPredict()`
3. **Update Setpoint**: Take first timestep from trajectory

**Characteristics**:
- ✅ Fast computation (~microseconds)
- ✅ Proven in MSL competitions
- ✅ Good for wheeled robots
- ❌ Open-loop (no optimization)
- ❌ Fixed segment structure

**Code Flow**:
```cpp
// Convert segment type
auto setpointSegments = convertSegmentVector(d.aux.segment);

// Generate segments (acceleration profile)
setpointSegments = getSegments(setpointSegments, 
                                d.setpoint.p, d.setpoint.v,
                                d.subtarget.p, d.subtarget.v,
                                d.subtarget.vmax, d.subtarget.amax, dmax);

// Propagate 1 timestep
Traj1(d.traj, setpointSegments, d.par.Ts);

// Update setpoint
d.setpoint.p = d.traj.p[0];
d.setpoint.v = d.traj.v[0];
d.setpoint.a = d.traj.a[0];

// Generate full predicted trajectory
TrajPredict(d, setpointSegments);
```

---

### MSL Mode: Mathematical Formulation

**Files**: `src/spg/setpoint/GetSegments.cpp`, `src/spg/setpoint/BalanceXY.cpp`, `src/spg/setpoint/Traj1.cpp`

This section presents the MSL trajectory generation as a structured control algorithm (not true MPC, but a time-optimal open-loop planner).

---

#### 1. Problem Statement

Given:
- **Initial state**: $\mathbf{p}_0 = [x_0, y_0, \phi_0]^T$, $\mathbf{v}_0 = [v_{x,0}, v_{y,0}, \omega_0]^T$
- **Goal state**: $\mathbf{p}_e = [x_e, y_e, \phi_e]^T$, $\mathbf{v}_e = [v_{x,e}, v_{y,e}, \omega_e]^T$
- **Physical limits**: $\mathbf{v}_{\max}$, $\mathbf{a}_{\max}$, $\mathbf{d}_{\max}$ (velocity, acceleration, deceleration)

**Find**: A feasible trajectory $\mathbf{p}(t)$, $\mathbf{v}(t)$ that reaches the goal while respecting kinematic constraints.

---

#### 2. Four-Segment Trajectory Structure

The MSL planner generates a **fixed structure** with 4 segments per degree of freedom (x, y, φ):

$$\text{Trajectory} = \{\text{Seg}_1, \text{Seg}_2, \text{Seg}_3, \text{Seg}_4\}$$

**Segment types**:
1. **Acceleration** ($\text{Seg}_1$): Accelerate from $\mathbf{v}_0$ to intermediate velocity $\mathbf{v}_1$
2. **Cruise** ($\text{Seg}_2$): Maintain constant velocity $\mathbf{v}_1$ for time $t_{\max}$
3. **Deceleration** ($\text{Seg}_3$): Decelerate from $\mathbf{v}_1$ to final velocity $\mathbf{v}_e$
4. **Final** ($\text{Seg}_4$): Hold at final velocity $\mathbf{v}_e$ indefinitely

---

#### 3. Segment Dynamics

Each segment $i$ is characterized by:
$$\text{Segment}_i = \{t_i, \mathbf{p}_i, \mathbf{v}_i, \mathbf{a}_i, \Delta t_i\}$$

where:
- $t_i$ : segment start time [s]
- $\mathbf{p}_i$ : position at segment end [m, m, rad]
- $\mathbf{v}_i$ : velocity at segment end [m/s, m/s, rad/s]
- $\mathbf{a}_i$ : constant acceleration during segment [m/s², m/s², rad/s²]
- $\Delta t_i$ : segment duration [s]

**Constant-acceleration kinematics** (per DOF $j \in \{x, y, \phi\}$):
$$p_{i,j} = p_{i-1,j} + v_{i-1,j} \cdot \Delta t_{i,j} + \frac{1}{2} a_{i,j} \cdot \Delta t_{i,j}^2$$
$$v_{i,j} = v_{i-1,j} + a_{i,j} \cdot \Delta t_{i,j}$$

---

#### 4. Segment 1: Acceleration to Intermediate Velocity

**Goal**: Reach velocity $\mathbf{v}_1$ that maximizes progress while allowing deceleration to $\mathbf{v}_e$.

**Maximum achievable speed** (per DOF):
$$v_{1,j} = \min\left(v_{\max,j}, \sqrt{\frac{1}{2}v_{0,j}^2 + \frac{1}{2}v_{e,j}^2 + a_{j} \cdot (p_{e,j} - p_{0,j})}\right)$$

where:
$$a_j = \text{sign}(p_{e,j} - p_{0,j}) \cdot a_{\max,j}$$

**Duration and end state**:
$$\Delta t_{1,j} = \frac{|v_{1,j} - v_{0,j}|}{a_{\max,j}}$$
$$a_{1,j} = \text{sign}(v_{1,j} - v_{0,j}) \cdot a_{\max,j}$$
$$p_{1,j} = p_{0,j} + v_{0,j} \cdot \Delta t_{1,j} + \frac{1}{2} a_{1,j} \cdot \Delta t_{1,j}^2$$

---

#### 5. Segment 2: Constant Velocity Cruise

**Duration** (per DOF):
$$t_{\max,j} = \max\left(0, \frac{v_{1,j}^2 - v_{\max,j}^2}{a_{\max,j} \cdot v_{\max,j}}\right)$$

**Special case** if $|v_{0,j}| > v_{\max,j}$ (already over-speed):
$$t_{\max,j} = \frac{p_{e,j} - p_{1,j}}{v_{\max,j}} \cdot \text{sign}(p_{e,j} - p_{1,j})$$

**End state**:
$$\Delta t_{2,j} = \max(0, t_{\max,j})$$
$$a_{2,j} = 0$$
$$p_{2,j} = p_{1,j} + v_{1,j} \cdot \Delta t_{2,j}$$
$$v_{2,j} = v_{1,j}$$

---

#### 6. Segment 3: Deceleration to Final Velocity

**Duration and end state**:
$$\Delta t_{3,j} = \frac{|v_{e,j} - v_{2,j}|}{a_{\max,j}}$$
$$a_{3,j} = \text{sign}(v_{e,j} - v_{2,j}) \cdot a_{\max,j}$$
$$p_{3,j} = p_{2,j} + v_{2,j} \cdot \Delta t_{3,j} + \frac{1}{2} a_{3,j} \cdot \Delta t_{3,j}^2$$
$$v_{3,j} = v_{e,j}$$

---

#### 7. Segment 4: Final Hold

**Purpose**: Extend trajectory indefinitely (for prediction beyond goal).

$$\Delta t_{4,j} = 10^{10} \quad \text{(effectively infinite)}$$
$$a_{4,j} = 0$$
$$v_{4,j} = v_{e,j}$$
$$p_{4,j}(t) = p_{3,j} + v_{e,j} \cdot (t - t_3)$$

---

#### 8. XY-Balancing Algorithm

**Problem**: For 2D motion, x and y segments may complete at different times, causing inefficient paths (e.g., robot moves in x, then separately in y).

**Solution**: Iteratively adjust acceleration/velocity limits to synchronize x and y motion.

**Algorithm** (binary search over direction angle $\alpha$):

$$\text{Initialize: } \alpha = 45°, \; \text{step} = 22.5°$$

**For** $i = 1, \ldots, 12$ **iterations**:
1. Compute scaling factors:
   $$A_x = \max(0.01, \cos(\alpha)), \quad A_y = \max(0.01, \sin(\alpha))$$

2. Scale physical limits:
   $$v_{\max,x} = v_{\text{move}} \cdot A_x, \quad v_{\max,y} = v_{\text{move}} \cdot A_y$$
   $$a_{\max,x} = a_{\text{move}} \cdot A_x, \quad a_{\max,y} = a_{\text{move}} \cdot A_y$$
   $$d_{\max,x} = d_{\text{move}} \cdot A_x, \quad d_{\max,y} = d_{\text{move}} \cdot A_y$$

3. Generate segments with scaled limits: $\{\text{Seg}_1, \text{Seg}_2, \text{Seg}_3, \text{Seg}_4\}$

4. **Check balance**: If $\Delta t_{2,y} > 0$ (y-axis has cruise phase), x is too fast:
   $$\alpha \leftarrow \alpha + \text{step}$$
   else (y finishes first), x is too slow:
   $$\alpha \leftarrow \alpha - \text{step}$$

5. Update: $\text{step} \leftarrow \text{step} / 2$

**Output**: Balanced limits $\mathbf{v}_{\max}$, $\mathbf{a}_{\max}$, $\mathbf{d}_{\max}$ such that x and y motions complete simultaneously (within tolerance).

---

#### 9. Trajectory Evaluation at Arbitrary Time

Given segments and query time $t_q$, find state $\mathbf{p}(t_q)$, $\mathbf{v}(t_q)$, $\mathbf{a}(t_q)$:

**For each DOF** $j$:
1. **Determine active segment**: Find $k$ such that $t_k \leq t_q < t_{k+1}$
2. **Relative time**: $\tau = t_q - t_k$
3. **Evaluate kinematics**:
   $$p_j(t_q) = p_{k,j} + v_{k,j} \cdot \tau + \frac{1}{2} a_{k,j} \cdot \tau^2$$
   $$v_j(t_q) = v_{k,j} + a_{k,j} \cdot \tau$$
   $$a_j(t_q) = a_{k,j}$$

**Multi-step prediction**: For visualization, evaluate at $t_1 = T_s$, $t_2 = 2T_s$, ..., $t_N = N \cdot T_s$.

---

#### 10. Receding Horizon Execution

**Discretization**: Sample trajectory at control period $T_s = 0.02$ s (50 Hz).

**Algorithm** at each control cycle:
1. **Measure** current state $\mathbf{p}_0$, $\mathbf{v}_0$
2. **Get goal** from subtarget: $\mathbf{p}_e$, $\mathbf{v}_e$
3. **Generate segments**: 
   - Call `getSegments()` → produces 4-segment trajectory
   - Optionally call `balanceXY()` for synchronized 2D motion
4. **Extract setpoint**: Evaluate trajectory at $t = T_s$ (one step ahead):
   $$\mathbf{p}_{\text{setpoint}} = \mathbf{p}(T_s), \quad \mathbf{v}_{\text{setpoint}} = \mathbf{v}(T_s), \quad \mathbf{a}_{\text{setpoint}} = \mathbf{a}(T_s)$$
5. **Predict trajectory**: Evaluate at $t_i = i \cdot T_s$ for $i = 1, \ldots, N_{\text{pred}}$ (e.g., $N_{\text{pred}} = 50$)
6. **Apply** $\mathbf{v}_{\text{setpoint}}$ to robot actuators
7. **Repeat** at next cycle with updated state

---

#### 11. Field Boundary Safety Constraints

**Soft constraints** (velocity clamping near boundaries):

$$v_{\max,x}^{\text{safe}} = 2 \cdot d_{\max} \cdot \text{dist}_{\text{sideline}}$$
$$v_{\max,y}^{\text{safe}} = 2 \cdot d_{\max} \cdot \text{dist}_{\text{goalline}}$$

where:
$$\text{dist}_{\text{sideline}} = \frac{W_{\text{field}}}{2} - |x|$$
$$\text{dist}_{\text{goalline}} = \frac{L_{\text{field}}}{2} - |y|$$

**Clamping rule**:
$$v_x \leftarrow \text{clamp}(v_x, -v_{\max,x}^{\text{safe}}, +v_{\max,x}^{\text{safe}})$$
$$v_y \leftarrow \text{clamp}(v_y, -v_{\max,y}^{\text{safe}}, +v_{\max,y}^{\text{safe}})$$

**Emergency deceleration**: If approaching boundary too fast:
$$\text{if } v_x > v_{\max,x}^{\text{safe}} \text{ and } x > 0: \quad a_x \leftarrow -d_{\max}$$

---

#### 12. Key Properties and Limitations

**Advantages**:
- ✅ **Time-optimal** for single-DOF motion (bang-bang control)
- ✅ **Exact** trajectory (no linearization errors)
- ✅ **Fast computation** (~50 μs per cycle)
- ✅ **Predictable** behavior (fixed segment structure)
- ✅ **Decoupled** DOFs (x, y, φ independent, except via balanceXY)

**Limitations**:
- ❌ **Open-loop**: No re-optimization after disturbances within segment
- ❌ **No coupled costs**: Cannot trade off position error vs. control effort
- ❌ **Heuristic balancing**: XY-balancing uses iterative search, not optimization
- ❌ **No obstacle avoidance**: Relies on higher-level subtarget planner
- ❌ **Assumes holonomic motion**: Ignores nonholonomic constraints (orientation-dependent velocity limits)

---

#### 13. Comparison: MSL vs. HumanoidMPC

| Property | MSL Mode | HumanoidMPC Mode |
|----------|----------|------------------|
| **Formulation** | Analytical (closed-form segments) | Optimization (QP) |
| **Horizon** | Entire trajectory to goal | Fixed $N$ steps (e.g., 10) |
| **Computation** | ~50 μs | ~1-3 ms |
| **Replanning** | Every cycle (fast re-compute) | Every cycle (warm-start QP) |
| **Smoothness** | Bang-bang (max accel/decel) | Tunable via $\mathbf{S}$ matrix |
| **Optimality** | Time-optimal per DOF | Cost-optimal over horizon |
| **Dynamics** | Linear per DOF | Nonlinear (coupled x-y-φ) |
| **Use case** | Wheeled MSL robots | Humanoid robots with drift |

---

#### 14. Hypothetical MPC Reformulation of MSL Mode

**Question**: What if we formulated MSL's analytical planner as an optimization problem?

This section shows how to express the MSL approach as a proper MPC problem that would recover similar behavior.

---

##### A. Time-Optimal MPC Formulation

**Decision variables**: Acceleration sequence over horizon $N$:
$$\mathbf{A} = \begin{bmatrix} \mathbf{a}_0 \\ \mathbf{a}_1 \\ \vdots \\ \mathbf{a}_{N-1} \end{bmatrix} \in \mathbb{R}^{3N}$$

where $\mathbf{a}_k = [a_{x,k}, a_{y,k}, a_{\phi,k}]^T$ at timestep $k$.

**Dynamics** (decoupled per DOF, e.g., for x-axis):
$$x_{k+1} = x_k + v_{x,k} \cdot \Delta t$$
$$v_{x,k+1} = v_{x,k} + a_{x,k} \cdot \Delta t$$

**Cost function** (time-to-goal minimization):
$$J = \underbrace{w_T \cdot \|\mathbf{x}_N - \mathbf{x}_{\text{goal}}\|^2}_{\text{terminal error}} + \underbrace{w_t \sum_{k=0}^{N-1} \|\mathbf{x}_k - \mathbf{x}_{\text{goal}}\|^2}_{\text{stage error (small weight)}}$$

**Constraints**:
1. **Acceleration limits** (box constraints):
   $$-a_{\max,j} \leq a_{j,k} \leq a_{\max,j}, \quad j \in \{x, y, \phi\}, \; k = 0, \ldots, N-1$$

2. **Velocity limits**:
   $$-v_{\max,j} \leq v_{j,k} \leq v_{\max,j}, \quad j \in \{x, y, \phi\}, \; k = 1, \ldots, N$$

3. **Terminal constraint** (must reach goal):
   $$\|\mathbf{x}_N - \mathbf{x}_{\text{goal}}\| \leq \epsilon_{\text{tol}}$$

4. **Terminal velocity constraint**:
   $$\mathbf{v}_N = \mathbf{v}_{\text{goal}}$$

**Key insight**: With large $w_T$ and tight terminal constraints, the optimal solution exhibits **bang-bang control**:
$$a_{j,k}^* \in \{-a_{\max,j}, 0, +a_{\max,j}\}$$

This recovers the MSL segment structure: max acceleration → cruise → max deceleration.

---

##### B. Minimum-Time MPC (Explicit Time Variable)

**Extended state**: Include time-to-goal as a state variable:
$$\mathbf{z}_k = [\mathbf{x}_k^T, \mathbf{v}_k^T, \tau_k]^T$$

where $\tau_k$ is remaining time to reach goal.

**Cost function**:
$$J = \tau_N + \lambda \sum_{k=0}^{N-1} \|\mathbf{a}_k\|^2$$

where $\lambda \ll 1$ is a small regularization term to break ties.

**Dynamics**:
$$\tau_{k+1} = \tau_k - \Delta t$$
$$(x, v \text{ as before})$$

**Terminal constraint**:
$$\mathbf{x}_N = \mathbf{x}_{\text{goal}}, \quad \mathbf{v}_N = \mathbf{v}_{\text{goal}}, \quad \tau_N \geq 0$$

**Result**: Minimizing $\tau_N$ directly optimizes for minimum time, which produces the 4-segment bang-bang structure analytically derived in MSL mode.

---

##### C. Mixed-Integer MPC (Explicit Segment Structure)

**Decision variables**: 
- Segment switching times $\{t_1, t_2, t_3\}$
- Binary variables $\delta_k^{(i)} \in \{0, 1\}$ indicating active segment at timestep $k$

**Cost function**:
$$J = t_3 + \sum_{k=0}^{N-1} \sum_{i=1}^{4} \delta_k^{(i)} \cdot c_i$$

where $c_i$ is cost of being in segment $i$ (e.g., $c_2 = 0$ for cruise, $c_1 = c_3 = \epsilon > 0$ for accel/decel).

**Constraints**:
1. **Segment logic**:
   $$\sum_{i=1}^{4} \delta_k^{(i)} = 1, \quad \forall k \quad \text{(exactly one segment active)}$$

2. **Segment order**:
   $$\delta_k^{(1)} \implies \delta_{k+1}^{(1)} \vee \delta_{k+1}^{(2)}$$
   $$\delta_k^{(2)} \implies \delta_{k+1}^{(2)} \vee \delta_{k+1}^{(3)}$$
   (logical implications enforced via big-M constraints)

3. **Acceleration by segment**:
   $$a_{j,k} = \sum_{i=1}^{4} \delta_k^{(i)} \cdot a_{j}^{(i)}$$
   where $a_j^{(1)} = +a_{\max,j}$, $a_j^{(2)} = 0$, $a_j^{(3)} = -a_{\max,j}$, $a_j^{(4)} = 0$

**Result**: This explicitly encodes the 4-segment structure and finds optimal switching times $\{t_1, t_2, t_3\}$. Equivalent to MSL's analytical solution but much slower (MILP vs. closed-form).

---

##### D. XY-Coupling via Multi-Objective Cost

To replicate `balanceXY()`, introduce a coupling cost:

**Cost function**:
$$J = \|\mathbf{x}_N - \mathbf{x}_{\text{goal}}\|^2 + w_{\text{balance}} \cdot (t_{x,\text{done}} - t_{y,\text{done}})^2$$

where $t_{x,\text{done}}$, $t_{y,\text{done}}$ are times when x and y DOFs reach their goals.

**Penalty formulation**:
$$w_{\text{balance}} \cdot \sum_{k=0}^{N-1} \left(\|\mathbf{x}_{k,x} - x_{\text{goal}}\|^2 - \|\mathbf{x}_{k,y} - y_{\text{goal}}\|^2\right)^2$$

This encourages x and y to converge simultaneously, similar to MSL's iterative angle search.

---

##### E. Acceleration-Level MPC (Matching Current Implementation)

**Decision variables**: Acceleration inputs $\mathbf{a}_k$ (not velocities).

**Dynamics**:
$$\mathbf{x}_{k+1} = \mathbf{x}_k + \mathbf{v}_k \cdot \Delta t + \frac{1}{2} \mathbf{a}_k \cdot \Delta t^2$$
$$\mathbf{v}_{k+1} = \mathbf{v}_k + \mathbf{a}_k \cdot \Delta t$$

**Cost function**:
$$J = \underbrace{\|\mathbf{x}_N - \mathbf{x}_{\text{goal}}\|_{\mathbf{Q}_f}^2}_{\text{terminal position}} + \underbrace{\|\mathbf{v}_N - \mathbf{v}_{\text{goal}}\|_{\mathbf{Q}_v}^2}_{\text{terminal velocity}} + \underbrace{\sum_{k=0}^{N-1} \|\mathbf{a}_k\|_{\mathbf{R}}^2}_{\text{control effort}}$$

**Smoothness** (jerk minimization):
$$J_{\text{smooth}} = \sum_{k=1}^{N-1} \|\mathbf{a}_k - \mathbf{a}_{k-1}\|_{\mathbf{S}}^2$$

**Complete formulation**:
$$\begin{aligned}
\min_{\mathbf{A}} \quad & J = \|\mathbf{x}_N - \mathbf{x}_{\text{goal}}\|_{\mathbf{Q}_f}^2 + \|\mathbf{v}_N - \mathbf{v}_{\text{goal}}\|_{\mathbf{Q}_v}^2 \\
& \quad + \sum_{k=0}^{N-1} \left( \|\mathbf{x}_k - \mathbf{x}_{\text{goal}}\|_{\mathbf{Q}}^2 + \|\mathbf{a}_k\|_{\mathbf{R}}^2 + \|\Delta\mathbf{a}_k\|_{\mathbf{S}}^2 \right) \\
\text{s.t.} \quad & \mathbf{x}_{k+1} = \mathbf{x}_k + \mathbf{v}_k \Delta t + \frac{1}{2}\mathbf{a}_k \Delta t^2 \\
& \mathbf{v}_{k+1} = \mathbf{v}_k + \mathbf{a}_k \Delta t \\
& -\mathbf{a}_{\max} \leq \mathbf{a}_k \leq \mathbf{a}_{\max} \\
& -\mathbf{v}_{\max} \leq \mathbf{v}_k \leq \mathbf{v}_{\max} \\
& \mathbf{x}_0 = \mathbf{x}_{\text{current}}, \quad \mathbf{v}_0 = \mathbf{v}_{\text{current}}
\end{aligned}$$

**Weight tuning to recover MSL behavior**:
- **Large** $\mathbf{Q}_f$: Forces terminal accuracy (like MSL's exact goal reaching)
- **Small** $\mathbf{R}$: Allows aggressive control (like MSL's bang-bang)
- **Zero** $\mathbf{S}$: No smoothness penalty (MSL uses max accel/decel)
- **Long horizon** $N \gg 1$: Allows full trajectory to goal (MSL plans entire path)

**Limit case**: As $\mathbf{Q}_f \to \infty$, $\mathbf{R} \to 0$, $\mathbf{S} \to 0$, $N \to \infty$, solution converges to MSL's time-optimal bang-bang control.

---

##### F. Computational Comparison

| Approach | Solver | Variables | Complexity | Time |
|----------|--------|-----------|------------|------|
| **MSL Analytical** | Closed-form | 0 (direct computation) | $O(1)$ | ~50 μs |
| **QP (formulation E)** | qpOASES | $3N$ (e.g., 30) | $O(n^3)$ factorization | ~1-3 ms |
| **MILP (formulation C)** | Gurobi/CPLEX | $3N + 4N$ binary | Exponential (branch-bound) | ~10-100 ms |
| **Explicit time (B)** | Nonlinear solver | $3N + N$ | $O(n^3)$ per iteration | ~5-20 ms |

**Conclusion**: MSL's analytical approach is computationally optimal for its assumptions (decoupled DOFs, bang-bang optimal). Converting to MPC only makes sense if:
1. Coupling between DOFs is critical (like humanoid kinematics)
2. Smoothness matters more than time-optimality
3. Online re-optimization under disturbances provides value

---

### HumanoidMPC Mode (Advanced)

**File**: `src/spg/setpoint/Set.cpp` (lines 35-204)

**How it works**:
1. **Frame Transformation**: Convert goal from global → robot local frame
2. **MPC Optimization**: Solve QP problem to find optimal control sequence
3. **Control Extraction**: Take first control from sequence
4. **Frame Transformation**: Convert control from body → global frame
5. **Trajectory Forward Simulation**: Use full control sequence for visualization

**Characteristics**:
- ✅ Optimal control (minimizes cost function)
- ✅ Smooth trajectories
- ✅ Velocity continuity (uses measured velocity)
- ✅ Tunable via Bayesian Optimization
- ❌ Slower computation (~1-3ms per cycle)
- ❌ Requires qpOASES library

**Parameters**:
```cpp
// Enabled via flag
d.par.use_humanoid_mpc = true;  // Set in SPGParams

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

---

## Code Structure and Integration

### Directory Structure

```
spg_cmake/
├── include/spg/setpoint/
│   ├── HumanoidMPC.hpp          # MPC interface
│   ├── Setpoint.hpp             # Setpoint structures
│   ├── Set.hpp                  # Main setpoint generation
│   └── ...                      # Other setpoint utilities
├── src/spg/setpoint/
│   ├── HumanoidMPC.cpp          # MPC implementation
│   ├── Set.cpp                  # Mode selection & integration
│   └── ...
├── demo/
│   └── main.cpp                 # Demo with visualization
└── test/
    └── test_humanoid_mpc.cpp    # Unit tests
```

### Key Classes and Structures

#### SPGState (`include/spg/Setpoint.hpp`)
```cpp
struct SPGState {
    SPGParams par;           // Parameters (use_humanoid_mpc flag)
    Input input;             // Robot state, ball, obstacles
    Target target;           // Final goal
    Subtarget subtarget;     // Intermediate waypoint
    Setpoint setpoint;       // Output: position, velocity, acceleration
    Trajectory traj;         // Predicted trajectory for visualization
    AuxData aux;             // Internal computation data
};
```

#### MPCParams (`include/spg/setpoint/HumanoidMPC.hpp`)
```cpp
struct MPCParams {
    double dt;               // Timestep [s]
    int horizon;             // N steps
    
    // Physical limits
    double vf_max;           // [m/s]
    double vs_max;           // [m/s]
    double omega_max;        // [rad/s]
    
    // BO-ready weights
    MPCWeights weights;      // 10 tunable parameters
};
```

#### HumanoidMPC (`include/spg/setpoint/HumanoidMPC.hpp`)
```cpp
class HumanoidMPC {
public:
    HumanoidMPC(const MPCParams& params, QPSolver* solver);
    
    // Compute optimal control (first step only)
    bool computeControl(const MPCState& x0,
                        const MPCState& goal,
                        const MPCControl& u_meas,
                        MPCControl& u_out);
    
    // Compute control + full trajectory prediction
    bool computeControlAndTrajectory(const MPCState& x0,
                                      const MPCState& goal,
                                      const MPCControl& u_meas,
                                      MPCControl& u_out,
                                      std::vector<MPCState>& predicted_states,
                                      std::vector<MPCControl>& predicted_controls);
};
```

### Integration in Set.cpp

**File**: `src/spg/setpoint/Set.cpp`

The main integration point where mode selection happens:

```cpp
SPGState Set(SPGState& d) {
    // Field boundary checking
    bool inside_field = ...;
    
    if (inside_field || d.subtarget.automatic_substitution_flag == 1) {
        
        // ============ MODE SELECTION ============
        if (d.par.use_humanoid_mpc) {
            #ifdef HAVE_QPOASES
            
            // --- HumanoidMPC Mode ---
            
            // 1. Transform goal: global → local
            double dx_global = d.subtarget.p[0] - d.setpoint.p[0];
            double dy_global = d.subtarget.p[1] - d.setpoint.p[1];
            double cos_phi = cos(d.setpoint.p[2]);
            double sin_phi = sin(d.setpoint.p[2]);
            
            goal_x_local = dx_global * cos_phi + dy_global * sin_phi;
            goal_y_local = -dx_global * sin_phi + dy_global * cos_phi;
            
            // 2. Current state in local frame (always at origin)
            current_state = {0, 0, 0};
            goal_state = {goal_x_local, goal_y_local, goal_phi_local};
            
            // 3. Transform current velocity: global → body
            u_meas.vf = d.setpoint.v[0] * cos_phi + d.setpoint.v[1] * sin_phi;
            u_meas.vs = -d.setpoint.v[0] * sin_phi + d.setpoint.v[1] * cos_phi;
            u_meas.omega = d.setpoint.v[2];
            
            // 4. Solve MPC
            bool success = mpc.computeControlAndTrajectory(
                current_state, goal_state, u_meas,
                u, predicted_states, predicted_controls);
            
            // 5. Transform control: body → global
            vx_global = u.vf * cos_phi - u.vs * sin_phi;
            vy_global = u.vf * sin_phi + u.vs * cos_phi;
            
            // 6. Update setpoint
            d.setpoint.p[0] += vx_global * dt;
            d.setpoint.p[1] += vy_global * dt;
            d.setpoint.p[2] += u.omega * dt;
            d.setpoint.v = {vx_global, vy_global, u.omega};
            
            // 7. Transform trajectory: local → global for visualization
            for (int i = 0; i < N; ++i) {
                d.traj.p[i][0] = robot_x + (x_local * cos_phi - y_local * sin_phi);
                d.traj.p[i][1] = robot_y + (x_local * sin_phi + y_local * cos_phi);
                d.traj.p[i][2] = robot_phi + phi_local;
            }
            
            #endif
        }
        
        if (!d.par.use_humanoid_mpc) {
            // --- MSL Mode ---
            auto setpointSegments = getSegments(...);
            Traj1(d.traj, setpointSegments, d.par.Ts);
            d.setpoint.p = d.traj.p[0];
            d.setpoint.v = d.traj.v[0];
            d.setpoint.a = d.traj.a[0];
            TrajPredict(d, setpointSegments);
        }
        
        // Field margin enforcement (both modes)
        // ... clamp positions and velocities ...
    }
    
    return d;
}
```

---

## Visualization System

**File**: `src/visualization/SimulatorVisualizer.cpp`

### ImGui/ImPlot Integration

The system provides real-time visualization of:

1. **Field**: Green soccer field with boundaries, goals, penalty areas
2. **Robot**: Circle with heading arrow showing orientation
3. **Target**: Red circle (final goal)
4. **Subtarget**: Orange circle (intermediate waypoint)
5. **Trajectory**: Cyan line showing predicted path (10 steps @ 0.2s)
6. **Obstacles**: Static (red circles) and dynamic (moving circles)

### Key Visualization Functions

```cpp
// Draw robot with heading arrow
void drawRobot(const Eigen::Vector3d& pose) {
    // Circle (0.25m radius)
    ImPlot::PlotCircle(...);
    
    // Heading arrow (0.4m length)
    double arrow_x = pose[0] + 0.4 * cos(pose[2]);
    double arrow_y = pose[1] + 0.4 * sin(pose[2]);
    ImPlot::PlotArrow(...);
    
    // Orientation display
    ImGui::Text("θ = %.2f rad (%.1f°)", pose[2], pose[2] * 180/M_PI);
}

// Draw predicted trajectory
void drawTrajectory(const Trajectory& traj) {
    std::vector<double> xs, ys;
    for (int i = 0; i < N; ++i) {
        xs.push_back(traj.p[i][0]);
        ys.push_back(traj.p[i][1]);
        
        // Optional: draw orientation at each point
        if (traj.p[i][2] != 0) {
            double arrow_len = 0.3;
            ImPlot::PlotArrow(traj.p[i][0], traj.p[i][1],
                             traj.p[i][0] + arrow_len * cos(traj.p[i][2]),
                             traj.p[i][1] + arrow_len * sin(traj.p[i][2]));
        }
    }
    ImPlot::PlotLine("Trajectory", xs.data(), ys.data(), xs.size());
}
```

### Running the Visualization

```bash
cd build
./demo_humanoid_mpc

# Controls:
# - Target: Click on field to set goal
# - Mode: Toggle MSL/HumanoidMPC in code
# - Obstacles: Add/move in real-time
```

---

## Summary of Key Design Decisions

### 1. Why Robot Local Frame for MPC?
- ✅ **Intuitive**: "Move forward 2m" is relative to robot
- ✅ **Numerical stability**: No large coordinates in optimization
- ✅ **Standard practice**: Matches mobile robot control literature
- ✅ **Simplifies dynamics**: Robot always at origin in its own frame

### 2. Why Measured Velocity Integration?
- ✅ **Smooth transitions**: No sudden velocity jumps
- ✅ **Correct Δu penalty**: Penalizes actual control changes, not fictitious ones
- ✅ **Better tracking**: MPC knows robot's current motion state

### 3. Why BO-Ready Parameters?
- ✅ **Easy tuning**: 10 independent scalars
- ✅ **Bayesian optimization**: Can auto-tune for specific robots
- ✅ **Clear semantics**: Each parameter has specific meaning

### 4. Why Full Trajectory Prediction?
- ✅ **Better visualization**: Shows what MPC actually plans
- ✅ **No waste**: MPC already computes N-step trajectory
- ✅ **Accurate**: Uses actual dynamics, not extrapolation

---

## Further Reading

- **MPC Explanation**: [`docs/MPC_EXPLANATION.md`](MPC_EXPLANATION.md)
- **Frame Transformations**: [`docs/FRAME_TRANSFORMATION_FIX.md`](FRAME_TRANSFORMATION_FIX.md)
- **Velocity Bug Fix**: [`docs/MPC_VELOCITY_BUG.md`](MPC_VELOCITY_BUG.md)
- **Integration Summary**: [`docs/INTEGRATION_SUMMARY.md`](INTEGRATION_SUMMARY.md)

---

**Last Updated**: December 9, 2025
**SPG Version**: HumanoidMPC Local Frame Implementation
