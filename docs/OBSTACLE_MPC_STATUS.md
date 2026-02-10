# Integrated Obstacle Avoidance MPC

## Summary

Created a complete standalone obstacle avoidance MPC simulator that bypasses the SPG pipeline:

### Files Created

1. **MPCObstacleAvoidingSimulator** ([include/MPCObstacleAvoidingSimulator.hpp](include/MPCObstacleAvoidingSimulator.hpp), [src/MPCObstacleAvoidingSimulator.cpp](src/MPCObstacleAvoidingSimulator.cpp))
   - Simulator that uses HumanoidObstacleAvoidanceMPC directly
   - No SPG pipeline (no target→subtarget→setpoint layers)
   - Direct MPC control with obstacle avoidance

2. **demo_obstacle_mpc** ([demo/demo_obstacle_mpc.cpp](demo/demo_obstacle_mpc.cpp))
   - Standalone demo for testing obstacle avoidance MPC
   - Supports JSON config files
   - Headless mode for benchmarking

3. **run_obstacle_bo.py** ([scripts/run_obstacle_bo.py](scripts/run_obstacle_bo.py))
   - Bayesian Optimization script for obstacle MPC weights
   - Tunes 8 weights: Q_pos, Q_theta, Q_vel, R_accel, Q_term_pos, Q_term_theta, Q_term_vel, collision_weight
   - Regenerates acados solver and recompiles for each iteration

### Current Status

⚠️ **Solver is currently failing with ACADOS_MINSTEP error**

The QP solver cannot find a feasible solution. This is likely due to:
- Nonlinear obstacle constraints being too strict
- SQP_RTI (real-time iteration) having difficulty with the constraint formulation
- Need to tune solver parameters or constraint formulation

### Next Steps to Fix

1. **Switch from SQP_RTI to full SQP** in [generate_acados_obstacle_mpc.py](scripts/generate_acados_obstacle_mpc.py)
2. **Relax QP solver tolerances** and increase iterations
3. **Consider softening obstacle constraints** with slack variables
4. **Test without obstacles first** to verify basic tracking works
5. **Adjust constraint formulation** - current distance^2 >= r_safe^2 might need tuning

### How to Use (once fixed)

```bash
# Build
cd build
cmake .. && make demo_obstacle_mpc

# Run interactive demo
./demo_obstacle_mpc

# Run with config file
./demo_obstacle_mpc --config ../demo/configs/obstacle_test.json

# Run headless for benchmarking
./demo_obstacle_mpc --headless --max-steps 1000

# Run Bayesian Optimization
cd ..
python3 scripts/run_obstacle_bo.py --init 5 --iter 20
```

### Comparison with SPG Pipeline

| Feature | SPG Pipeline | Obstacle MPC |
|---------|-------------|--------------|
| Target Selection | ✓ (cleans field) | ✗ (direct goal) |
| Subtarget Planning | ✓ (A* with obstacles) | ✗ (MPC handles) |
| Setpoint Generation | ✓ (HumanoidReferenceMPC) | ✗ (integrated) |
| Obstacle Avoidance | Separate planning | Integrated in MPC |
| Control Output | Velocity | Velocity |
| Complexity | High (3 layers) | Low (1 layer) |
