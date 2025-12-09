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

#### 1. State Space (Robot Local Frame)
```
x = [x, y, φ]ᵀ
  x: forward displacement from current robot position [m]
  y: lateral displacement [m]
  φ: heading relative to current robot orientation [rad]
```

#### 2. Control Space (Body Frame)
```
u = [vf, vs, ω]ᵀ
  vf: forward velocity [m/s]
  vs: sideways velocity [m/s]
  ω: yaw rate [rad/s]
```

Internally normalized to `ũ = [ũf, ũs, ũω]ᵀ ∈ [-1, 1]³`:
```
ũf = vf / vf_max
ũs = vs / vs_max
ũω = ω / ω_max
```

#### 3. Dynamics (Linearized Kinematics)

Discrete-time model with timestep `dt = 0.02s`:
```
x_{k+1} = x_k + dt * (vf * cos(φ_k) - vs * sin(φ_k))
y_{k+1} = y_k + dt * (vf * sin(φ_k) + vs * cos(φ_k))
φ_{k+1} = φ_k + dt * ω
```

Linearized about current heading `φ₀` (typically 0 in local frame):
```
x_{k+1} ≈ x_k + B * u_k

where B = [dt*cos(φ₀)  -dt*sin(φ₀)   0    ]
          [dt*sin(φ₀)   dt*cos(φ₀)   0    ]
          [    0            0       dt    ]
```

#### 4. Cost Function

**Goal-based MPC**: Minimize distance to goal over horizon N = 10 steps:

```
J = Σ(k=0 to N-1) [(x_k - x_goal)ᵀ Q (x_k - x_goal)      # state error
                   + ũ_kᵀ R ũ_k                            # control effort
                   + Δũ_kᵀ S Δũ_k]                        # control smoothness
    + (x_N - x_goal)ᵀ Qf (x_N - x_goal)                   # terminal cost
```

**Key Feature**: Smooth transitions via measured velocity:
```
Δũ₀ = ũ₀ - ũ_meas    (uses actual current velocity)
Δũₖ = ũₖ - ũ_{k-1}   (change between timesteps)
```

**Cost Matrices**:
```
Q = diag(q_pos, q_pos, q_phi)        # stage state cost
Qf = diag(qf_pos, qf_pos, qf_phi)    # terminal state cost
R = diag(r_vf, r_vs, r_omega)         # control effort cost
S = diag(s_vf, s_vs, s_omega)         # smoothness cost
```

#### 5. QP Formulation

The MPC problem is reformulated as a Quadratic Program:

```
minimize    ½ Ũᵀ H Ũ + fᵀ Ũ
subject to  -1 ≤ Ũ ≤ 1

where:
  Ũ = [ũ₀ᵀ, ũ₁ᵀ, ..., ũ_{N-1}ᵀ]ᵀ  (stacked controls, 30×1)
  
  H = 2(SuᵀQbarSu + Rbar + LᵀSbarL)
  f = 2(SuᵀQbar(Sx*x₀ - xgoal_stack) + LᵀSbarl₀)
  
  Su: control-to-state mapping (9N × 3N)
  L: control-to-Δcontrol mapping (3N × 3N)
  l₀: measured velocity offset vector
```

**Solver**: Uses **qpOASES** (fast online active-set QP solver)
- Typical convergence: 20-40 iterations
- Computation time: 1-3ms on modern CPU

#### 6. Forward Simulation

After solving QP, forward simulate trajectory using full control sequence:

```cpp
for (k = 0; k < N; ++k) {
    u_k = extract control k from solution
    
    // Propagate dynamics
    x_{k+1} = x_k + dt * (u_k.vf * cos(φ_k) - u_k.vs * sin(φ_k))
    y_{k+1} = y_k + dt * (u_k.vf * sin(φ_k) + u_k.vs * cos(φ_k))
    φ_{k+1} = φ_k + dt * u_k.ω
    
    store state and control for visualization
}
```

Returns: `N+1` states and `N` controls in **robot local frame**.

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
