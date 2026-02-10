# Integrated Obstacle Avoidance MPC

This module provides an **integrated Model Predictive Control** approach that combines:

- **Target tracking**
- **Obstacle avoidance** (via nonlinear constraints)
- **Velocity/acceleration limits**
- **Smooth trajectory generation**

All in a **single optimization problem**, eliminating the need for separate:
- Target cleaning
- Subtarget selection
- Setpoint generation

## Architecture Comparison

### Traditional SPG Pipeline
```
Obstacles + Target → Target Cleaning → Subtarget Selection → Setpoint MPC → Control
```

### Integrated MPC (This Module)
```
Obstacles + Target → Integrated MPC → Control
```

## Files

- `HumanoidObstacleAvoidanceMPC.hpp/cpp` - Main controller implementation
- `../../scripts/generate_acados_obstacle_mpc.py` - Acados solver generation script

## Usage

### 1. Generate Acados Solver

```bash
cd scripts
python3 generate_acados_obstacle_mpc.py
```

This creates the solver in `build/c_generated_code/`.

### 2. Recompile Project

```bash
cd build
make -j4
```

### 3. Use in Code

```cpp
#include "spg/integrated_mpc/HumanoidObstacleAvoidanceMPC.hpp"

using namespace spg::integrated_mpc;

// Create MPC controller
ObstacleAvoidanceMPCParams params;
params.dt = 0.15;
params.horizon = 20;
params.max_obstacles = 10;
params.robot_radius = 0.3;
params.obstacle_margin = 0.2;

HumanoidObstacleAvoidanceMPC mpc(params);

// Set up state and goal
ObstacleAvoidanceState current_state;
current_state.px = 0.0;
current_state.py = 0.0;
current_state.theta = 0.0;
current_state.vx = 0.0;
current_state.vy = 0.0;
current_state.omega = 0.0;

ObstacleAvoidanceState goal;
goal.px = 5.0;
goal.py = 5.0;
goal.theta = 0.0;

// Add obstacles
std::vector<Obstacle> obstacles;
Obstacle obs1;
obs1.px = 2.5;
obs1.py = 2.5;
obs1.radius = 0.5;
obs1.active = true;
obstacles.push_back(obs1);

// Compute control
ObstacleAvoidanceControl u;
std::vector<ObstacleAvoidanceState> predicted_states;
std::vector<ObstacleAvoidanceControl> predicted_controls;

bool success = mpc.computeControlAndTrajectory(
    current_state, goal, obstacles, u,
    predicted_states, predicted_controls
);

if (success) {
    // Use acceleration command: u.ax, u.ay, u.alpha
    // Or get velocity command:
    auto vel = mpc.getVelocityCommand();
    // Apply vel.vx, vel.vy, vel.omega
}
```

## Obstacle Avoidance Implementation

The solver uses **nonlinear path constraints** for obstacle avoidance:

$$
\text{dist}(robot, obstacle)^2 \geq (r_{robot} + r_{obstacle} + margin)^2
$$

For multiple obstacles (up to `max_obstacles`), constraints are:

$$
(p_x - obs_x)^2 + (p_y - obs_y)^2 \geq r_{safe}^2
$$

These are enforced at every stage of the MPC horizon.

## Tuning Parameters

### Cost Weights
- `Q_pos`: Position tracking weight (default: 15.0)
- `Q_theta`: Heading tracking weight (default: 2.0)
- `Q_vel`: Velocity tracking weight (default: 0.5)
- `R_accel`: Acceleration effort weight (default: 0.1)
- `Q_term_pos`: Terminal position weight (default: 100.0)

### Physical Limits
- `vx_max`: Max forward velocity (default: 0.8 m/s)
- `vy_max`: Max sideways velocity (default: 0.3 m/s)
- `omega_max`: Max yaw rate (default: 1.2 rad/s)
- `ax_max`: Max forward acceleration (default: 4.0 m/s²)
- `ay_max`: Max sideways acceleration (default: 3.0 m/s²)
- `alpha_max`: Max angular acceleration (default: 2.0 rad/s²)

### Obstacle Parameters
- `robot_radius`: Robot safety radius (default: 0.3 m)
- `obstacle_margin`: Additional safety margin (default: 0.2 m)
- `max_obstacles`: Maximum obstacles to consider (default: 10)

## Advantages

1. **Unified optimization** - No need for separate planning layers
2. **Collision-free trajectories** - Obstacles directly in constraints
3. **Real-time capable** - Uses SQP-RTI solver (< 10 ms per solve)
4. **Predictive avoidance** - Looks ahead over full horizon
5. **Smooth motions** - Acceleration costs ensure jerk-free paths

## Limitations

1. **Static obstacles** - Current implementation assumes obstacles don't move
2. **Convex collision model** - Uses circular robot/obstacle shapes
3. **Local optimality** - May get stuck in local minima
4. **Computational cost** - More expensive than reference tracking MPC

## Future Enhancements

- [ ] Dynamic obstacle prediction (moving obstacles)
- [ ] Non-circular obstacle shapes (ellipses, polygons)
- [ ] Multi-agent coordination
- [ ] Soft constraints with slack variables
- [ ] Adaptive horizon based on obstacle density
