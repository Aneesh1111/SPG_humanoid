# SPG Quick Reference Guide

## Switching Between MSL and HumanoidMPC Modes

### In Code

```cpp
// Enable HumanoidMPC mode
d.par.use_humanoid_mpc = true;

// Disable HumanoidMPC (use MSL mode)
d.par.use_humanoid_mpc = false;
```

### In Demo (`demo/main.cpp`)

```cpp
// Around line 40-50:
params.use_humanoid_mpc = true;  // Change this flag
```

---

## Tuning HumanoidMPC Parameters

### Physical Limits (Robot Capabilities)

```cpp
mpc_params.vf_max = 1.2;      // Forward velocity limit [m/s]
mpc_params.vs_max = 0.4;      // Sideways velocity limit [m/s]
mpc_params.omega_max = 1.0;   // Rotation rate limit [rad/s]
```

### Cost Weights (Behavior Tuning)

**Higher values = stronger penalty**

#### Position Tracking
```cpp
mpc_params.weights.q_pos = 1.0;    // Stage position: how much to care about being near goal at each step
mpc_params.weights.qf_pos = 8.0;   // Terminal position: strongly penalize ending far from goal
```

#### Heading Tracking
```cpp
mpc_params.weights.q_phi = 0.1;    // Stage heading: small weight, heading matters less during motion
mpc_params.weights.qf_phi = 1.0;   // Terminal heading: moderate weight, should face correct direction at goal
```

#### Control Effort (Energy/Aggressiveness)
```cpp
mpc_params.weights.r_vf = 0.1;     // Forward velocity: small penalty, encourage forward motion
mpc_params.weights.r_vs = 0.5;     // Sideways velocity: larger penalty, sideways motion is less efficient
mpc_params.weights.r_omega = 0.2;  // Rotation: small-medium penalty
```

#### Smoothness (Comfort/Stability)
```cpp
mpc_params.weights.s_vf = 0.2;     // Forward smoothness: moderate, some acceleration changes OK
mpc_params.weights.s_vs = 0.8;     // Sideways smoothness: high, avoid jerky lateral motion
mpc_params.weights.s_omega = 0.4;  // Rotation smoothness: medium, avoid spinning
```

### Tuning Guidelines

**Too slow to reach goal?**
- ↑ Increase `qf_pos` (terminal position cost)
- ↓ Decrease `r_vf`, `r_vs` (control effort costs)

**Too aggressive/jerky motion?**
- ↑ Increase `s_vf`, `s_vs`, `s_omega` (smoothness costs)
- ↓ Decrease `q_pos` (stage position cost)

**Not facing correct direction?**
- ↑ Increase `qf_phi` (terminal heading cost)
- ↑ Increase `q_phi` (stage heading cost)

**Prefers sideways motion too much?**
- ↑ Increase `r_vs` (sideways effort cost)
- Ensure `r_vs > r_vf`

---

## Coordinate Frame Conversions

### When do I need to convert frames?

**Use GLOBAL frame for**:
- Field coordinates (targets, obstacles, boundaries)
- SPG state variables (`d.setpoint.p`, `d.setpoint.v`)
- Visualization

**Use LOCAL (robot body) frame for**:
- MPC solver inputs/outputs
- Control commands to robot actuators

### Quick Conversion Functions

```cpp
// Global velocity → Body velocity
void globalToBody(double vx_global, double vy_global, double robot_phi,
                  double& vf, double& vs) {
    double c = cos(robot_phi);
    double s = sin(robot_phi);
    vf =  vx_global * c + vy_global * s;   // forward
    vs = -vx_global * s + vy_global * c;   // sideways
}

// Body velocity → Global velocity
void bodyToGlobal(double vf, double vs, double robot_phi,
                  double& vx_global, double& vy_global) {
    double c = cos(robot_phi);
    double s = sin(robot_phi);
    vx_global = vf * c - vs * s;
    vy_global = vf * s + vs * c;
}

// Global position → Local position (relative to robot)
void globalToLocal(double x_global, double y_global,
                   double robot_x, double robot_y, double robot_phi,
                   double& x_local, double& y_local) {
    double dx = x_global - robot_x;
    double dy = y_global - robot_y;
    double c = cos(robot_phi);
    double s = sin(robot_phi);
    x_local =  dx * c + dy * s;
    y_local = -dx * s + dy * c;
}
```

---

## Common Tasks

### Adding a New Target

```cpp
// Set target position (global frame)
d.target.p[0] = 3.0;  // x [m]
d.target.p[1] = 2.0;  // y [m]
d.target.p[2] = M_PI / 4;  // heading [rad]

// SPG will automatically:
// 1. Adjust for field boundaries
// 2. Adjust for obstacles
// 3. Calculate subtarget
// 4. Generate setpoint
```

### Adding Static Obstacles

```cpp
// In demo/main.cpp or wherever you initialize
Obstacle obs;
obs.p[0] = 1.0;  // x position [m]
obs.p[1] = 0.5;  // y position [m]
obs.r = 0.3;     // radius [m]
obs.type = ObstacleType::STATIC;

d.input.obstacles.push_back(obs);
```

### Changing MPC Horizon

```cpp
// In Set.cpp initialization section
mpc_params.horizon = 15;  // Default is 10
// Longer horizon = smoother but slower computation
// Shorter horizon = faster but more myopic
```

### Changing Control Frequency

```cpp
// In SPGParams
params.Ts = 0.02;  // Timestep [s], default 50 Hz

// MPC dt should match:
mpc_params.dt = params.Ts;
```

---

## Debugging Tips

### MPC Not Working?

**Check**:
1. Is `HAVE_QPOASES` defined? (CMake should set this automatically)
2. Does QP solve succeed? Look for "QP solve failed" in console
3. Are physical limits reasonable? (`vf_max`, `vs_max`, `omega_max`)
4. Is goal within reachable region? (too far = QP infeasible)

**Enable Debug Output**:
```cpp
// In HumanoidMPC.cpp, uncomment:
std::cout << "MPC: Current state = " << x0.x << ", " << x0.y << ", " << x0.phi << std::endl;
std::cout << "MPC: Goal state = " << goal.x << ", " << goal.y << ", " << goal.phi << std::endl;
std::cout << "MPC: Control output = " << u.vf << ", " << u.vs << ", " << u.omega << std::endl;
```

### Robot Not Moving?

**Check**:
1. Is setpoint being updated? Print `d.setpoint.v`
2. Is target different from current position?
3. Are velocities clamped by field boundaries? (too close to edge)
4. Is `inside_field` flag true?

### Trajectory Looks Wrong in Visualization?

**Check**:
1. Frame transformations correct? (local → global)
2. Is `predicted_states` populated? (should have N+1 states)
3. Are trajectory points outside field? (indicates frame mismatch)

### MSL Mode vs HumanoidMPC Different Behavior?

**This is expected**:
- MSL: Fixed segment structure, no optimization
- HumanoidMPC: Optimizes over entire horizon, considers all costs

---

## Performance Benchmarks

### Typical Computation Times (Intel i7, 3.5 GHz)

| Component | MSL Mode | HumanoidMPC Mode |
|-----------|----------|------------------|
| Target::Set() | ~50 μs | ~50 μs |
| Subtarget::Set() | ~100 μs | ~100 μs |
| setpoint::Set() | ~200 μs | ~1-3 ms |
| **Total Loop** | **~350 μs** | **~1.2-3.2 ms** |

HumanoidMPC is well within 20ms deadline (50 Hz control).

### QP Solver Statistics

- **Typical iterations**: 20-40
- **Max iterations**: 50 (set in QPOasesSolver)
- **Convergence rate**: >99% for reachable goals

---

## File Map

### Core Files (Must Understand)

| File | Purpose | Key Functions |
|------|---------|---------------|
| `src/spg/setpoint/Set.cpp` | Mode selection & integration | `Set()` - main entry point |
| `src/spg/setpoint/HumanoidMPC.cpp` | MPC algorithm | `computeControlAndTrajectory()` |
| `include/spg/setpoint/HumanoidMPC.hpp` | MPC interface | Class/struct definitions |
| `demo/main.cpp` | Demo with visualization | Setup & control loop |

### Supporting Files

| File | Purpose |
|------|---------|
| `src/spg/target/*.cpp` | Target adjustments (field, obstacles, rules) |
| `src/spg/subtarget/*.cpp` | Intermediate waypoint calculation |
| `src/spg/setpoint/GetSegments.cpp` | MSL mode trajectory segments |
| `src/visualization/SimulatorVisualizer.cpp` | ImGui/ImPlot rendering |

---

## Build & Test Commands

```bash
# Build everything
cd build
cmake ..
make -j4

# Run demo
./demo_humanoid_mpc

# Run tests
./test/test_humanoid_mpc

# Run all tests
ctest

# Clean build
rm -rf build/*
cd build && cmake .. && make -j4
```

---

## Common Errors & Solutions

### Error: "HAVE_QPOASES not defined"

**Solution**: Ensure qpOASES is installed and CMake found it
```bash
# Check CMakeLists.txt has:
find_package(qpOASES REQUIRED)
```

### Error: "QP solve failed"

**Possible causes**:
1. Goal too far away (increase horizon or reduce goal distance)
2. Conflicting cost weights (increase terminal costs)
3. Initial guess infeasible (shouldn't happen with box constraints)

**Solution**: Check console for qpOASES return code, adjust weights

### Error: Robot oscillates near goal

**Cause**: Terminal costs too weak, overshoots and corrects repeatedly

**Solution**: Increase `qf_pos` and `qf_phi` by 2-5x

### Error: Trajectory in visualization doesn't match motion

**Cause**: Frame transformation mismatch (local vs global)

**Solution**: Verify transformations in Set.cpp lines 140-180

---

## Quick Start: Your First Modification

### Goal: Make robot more aggressive

```cpp
// In Set.cpp, around line 55-62:
mpc_params.weights.q_pos = 2.0;    // Was 1.0, now stronger tracking
mpc_params.weights.qf_pos = 15.0;  // Was 8.0, now must reach goal
mpc_params.weights.r_vf = 0.05;    // Was 0.1, now cheaper to move fast
mpc_params.weights.s_vf = 0.1;     // Was 0.2, now allows more acceleration
```

### Goal: Make robot smoother

```cpp
// In Set.cpp:
mpc_params.weights.s_vf = 0.5;     // Was 0.2, now penalize jerks more
mpc_params.weights.s_vs = 1.5;     // Was 0.8, now extra smooth sideways
mpc_params.weights.s_omega = 0.8;  // Was 0.4, now smooth rotation
```

---

**Need more help?** Check [`docs/ARCHITECTURE.md`](ARCHITECTURE.md) for detailed explanations.
