# SPG Framework - Port Skeleton

## Overview

This is a **skeleton structure** for porting the SPG (Soccer Path Generator) framework to a new repository. The port will be done incrementally through multiple merge requests to maintain stability.

## Architecture

The SPG framework follows a **three-layer hierarchical planning** approach:

```
Input (Goal) → Target → Subtarget → Setpoint → Output (Commands)
```

### Layer Flow

1. **Target Layer**: High-level path planning
   - Adjusts goal to field boundaries, obstacles, penalty areas
   - Ensures strategic validity (e.g., 3m rule)
   - Output: Safe target position

2. **Subtarget Layer**: Mid-level trajectory planning
   - Collision-free path generation
   - Angle adjustments for approach
   - Dynamic replanning if needed
   - Output: Immediate subtarget waypoint

3. **Setpoint Layer**: Low-level motion control
   - Trajectory generation (position, velocity, acceleration)
   - Smooth motion profiles
   - **HumanoidMPC**: Model Predictive Control for humanoid robots
   - **MSL**: Traditional setpoint generation
   - Output: Robot commands (position, velocity, acceleration)

## Current Status: **SKELETON** (Step 1)

### ✅ What's Included
- Basic data structures (SPGState, SPGPar)
- Function stubs for three layers
- README documentation
- Build system skeleton (CMake)

### 🚧 What's Missing (Future Steps)
- Target adjustment algorithms
- Subtarget collision checking
- Setpoint trajectory generation
- HumanoidMPC implementation
- Utility functions (rotation, wrapping, etc.)
- Visualization tools
- Test suite

## Incremental Port Plan

### Step 1: Skeleton (Current)
- [ ] Basic data structures
- [ ] Function signatures
- [ ] Build system
- [ ] Documentation

### Step 2: Target Layer
- [ ] AdjustToField
- [ ] AdjustToObstacles
- [ ] AdjustToPenaltyArea
- [ ] AdjustToGoalArea
- [ ] AdjustTo3mRule

### Step 3: Subtarget Layer
- [ ] CheckCollisionFree
- [ ] AngleUtils
- [ ] ReplanUtils

### Step 4: HumanoidMPC
- [ ] MPC core algorithm
- [ ] qpOASES integration
- [ ] Weight tuning system
- [ ] Matrix precomputation

### Step 5: Utilities & Polish
- [ ] Math functions (rot, sign, wrap)
- [ ] Visualization tools
- [ ] Unit tests
- [ ] Performance benchmarks

## Usage (After Full Port)

```cpp
#include "SPG.hpp"

// Initialize
SPGState state;
state.par.use_humanoid_mpc = true;  // or false for MSL mode

// Set goal
state.target.p[0] = 5.0;  // x
state.target.p[1] = 3.0;  // y
state.target.p[2] = 0.0;  // theta

// Run SPG pipeline
spg::target::Target(state);
spg::subtarget::Subtarget(state);
spg::setpoint::Setpoint(state);

// Get output
double cmd_x = state.setpoint.p[0];
double cmd_y = state.setpoint.p[1];
double cmd_theta = state.setpoint.p[2];
```

## Key Features

- **Dual Mode**: MSL (traditional) and HumanoidMPC (advanced)
- **Real-time**: 50Hz control loop capable
- **Optimized**: Precomputed matrices, hotstart QP solving
- **Tunable**: 10 MPC weight parameters for Bayesian optimization
- **Robust**: Collision avoidance, field boundaries, rule compliance

## Dependencies

- C++17 or later
- Eigen3 (linear algebra)
- qpOASES (MPC only - quadratic programming)
- CMake 3.10+

## Performance Targets

- Target layer: < 0.1 ms
- Subtarget layer: < 0.2 ms
- Setpoint layer (MSL): < 0.5 ms
- Setpoint layer (MPC): < 1.0 ms
- **Total pipeline: < 2 ms @ 50Hz**

## Contact & Support

For questions about this port, see the original SPG repository documentation.

---

**Next Step**: Implement Target layer (MR2)
