# Acados MPC Setup Guide

This document explains how to integrate the acados-based MPC (HumanoidReferenceMPC) into the SPG project.

## Architecture Overview

The system now supports **three** motion planning modes:

1. **Acados MPC** (HumanoidReferenceMPC) - Real-time nonlinear MPC using acados (preferred for humanoids)
2. **qpOASES MPC** (HumanoidMPC) - QP-based MPC using qpOASES (fallback option)
3. **Traditional MSL** - Segment-based trajectory generation (original method)

The priority order is: Acados → qpOASES → MSL

## File Structure

```
include/spg/setpoint/HumanoidReferenceMPC.hpp  - Acados MPC interface
src/spg/setpoint/HumanoidReferenceMPC.cpp      - Acados MPC implementation
src/spg/setpoint/Set.cpp                       - Controller selection logic
```

## Installation Steps

### 1. Install Acados

```bash
# Clone acados repository
git clone https://github.com/acados/acados.git
cd acados
git submodule update --recursive --init

# Build and install
mkdir -p build
cd build
cmake -DACADOS_WITH_QPOASES=ON ..
make install -j4

# Install Python interface
pip install -e ../interfaces/acados_template

# Set environment variable (add to ~/.bashrc)
export ACADOS_SOURCE_DIR="/path/to/acados"
```

### 2. Generate Solver Code

Create a Python script to generate the acados solver for your robot:

```python
# generate_robot_mpc.py
from acados_template import AcadosOcp, AcadosOcpSolver
from casadi import SX, vertcat, sin, cos
import numpy as np

def export_robot_mpc_solver():
    ocp = AcadosOcp()
    
    # Model name (IMPORTANT: must match includes in HumanoidReferenceMPC.cpp)
    ocp.model.name = 'robot_mpc'
    
    # Problem dimensions
    nx = 3  # [px, py, theta]
    nu = 3  # [vx, vy, omega]
    
    # States
    px = SX.sym('px')
    py = SX.sym('py')
    theta = SX.sym('theta')
    x = vertcat(px, py, theta)
    
    # Controls (body-frame velocities)
    vx = SX.sym('vx')
    vy = SX.sym('vy')
    omega = SX.sym('omega')
    u = vertcat(vx, vy, omega)
    
    # Dynamics (nonlinear heading-dependent translation)
    dt = 0.1  # time step
    px_dot = cos(theta)*vx - sin(theta)*vy
    py_dot = sin(theta)*vx + cos(theta)*vy
    theta_dot = omega
    
    f_expl = vertcat(px_dot, py_dot, theta_dot)
    f_impl = vertcat(px_dot, py_dot, theta_dot) - vertcat(px_dot, py_dot, theta_dot)
    
    # Model
    ocp.model.x = x
    ocp.model.u = u
    ocp.model.f_expl_expr = f_expl
    ocp.model.f_impl_expr = f_impl
    
    # Discretization
    ocp.dims.N = 20
    ocp.solver_options.tf = 20 * dt
    
    # Cost (LINEAR_LS)
    Q = np.diag([10.0, 10.0, 1.0])  # state weights
    R = np.diag([0.1, 0.1, 0.1])    # control weights
    
    ocp.cost.cost_type = 'LINEAR_LS'
    ocp.cost.cost_type_e = 'LINEAR_LS'
    
    ocp.cost.W = np.block([[Q, np.zeros((3,3))],
                           [np.zeros((3,3)), R]])
    ocp.cost.W_e = Q
    
    ocp.cost.Vx = np.vstack([np.eye(3), np.zeros((3,3))])
    ocp.cost.Vu = np.vstack([np.zeros((3,3)), np.eye(3)])
    ocp.cost.Vx_e = np.eye(3)
    
    ocp.cost.yref = np.zeros(6)
    ocp.cost.yref_e = np.zeros(3)
    
    # Constraints
    # Input ellipse: (vx/0.4)^2 + (vy/1.1)^2 <= 1 (polygon approximation)
    # omega: |omega| <= 1.5
    
    # Simple box constraints (refine with polytope later)
    ocp.constraints.lbu = np.array([-0.4, -1.1, -1.5])
    ocp.constraints.ubu = np.array([0.4, 1.1, 1.5])
    ocp.constraints.idxbu = np.array([0, 1, 2])
    
    # Initial state constraint (will be set at runtime)
    ocp.constraints.x0 = np.zeros(3)
    
    # Solver options
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.integrator_type = 'ERK'
    ocp.solver_options.nlp_solver_type = 'SQP_RTI'
    ocp.solver_options.qp_solver_iter_max = 50
    
    # Generate C code
    solver = AcadosOcpSolver(ocp, json_file='robot_mpc.json')
    
    print("Solver generated successfully!")
    print("Generated files in: c_generated_code/")
    
    return solver

if __name__ == '__main__':
    export_robot_mpc_solver()
```

Run the generator:

```bash
cd /path/to/spg_cmake
mkdir -p generated
cd generated
python ../generate_robot_mpc.py
```

This will create:
- `c_generated_code/` - C source/header files
- `libacados_ocp_solver_robot_mpc.so` - Shared library

### 3. Build the Project

```bash
cd /path/to/spg_cmake/build
cmake ..
make
```

## Code Structure

### HumanoidReferenceMPC.cpp

The implementation includes:

1. **Constructor**: Initializes acados solver capsule
2. **buildReference()**: Creates straight-line reference with heading blending
3. **computeControlAndTrajectory()**: Main solve function
4. **Frame transformations**: Handles global ↔ local coordinate transforms

### Set.cpp Integration

The `Set()` function now:

1. Checks for `d.par.use_humanoid_mpc` flag
2. Tries Acados MPC first (if `HAVE_ACADOS` defined)
3. Falls back to qpOASES MPC (if `HAVE_QPOASES` defined)
4. Falls back to traditional MSL if both unavailable

### Key Differences from Example Code

| Feature | Example (standalone) | Integrated (your system) |
|---------|---------------------|-------------------------|
| Entry point | `main()` loop | Called from `Set.cpp` |
| Solver lifecycle | Create/destroy each run | Static instance (persistent) |
| Coordinate frame | Global frame | Local robot frame |
| State source | Simulated dynamics | Real robot estimator |
| Control output | Print to console | Updates `d.setpoint` |

## Configuration

### Runtime Parameters

Edit your SPG configuration to enable acados MPC:

```cpp
d.par.use_humanoid_mpc = true;  // Enable MPC mode
d.par.Ts = 0.02;                // Control timestep (must match acados dt)
```

### MPC Tuning

Modify `HumanoidReferenceMPC.cpp` constructor:

```cpp
acados_params.dt = 0.1;           // Time step [s]
acados_params.horizon = 20;       // Prediction horizon
acados_params.vx_max = 0.4;       // Max forward velocity [m/s]
acados_params.vy_max = 1.1;       // Max sideways velocity [m/s]
acados_params.omega_max = 1.5;    // Max yaw rate [rad/s]
acados_params.polygon_sides = 12; // Ellipse approximation resolution
```

Cost weights are set in the Python generator script (`Q`, `R` matrices).

## Troubleshooting

### Solver not found

If you see "acados generated solver code not yet generated":

1. Run `generate_robot_mpc.py` to create solver
2. Check that `c_generated_code/` exists in `generated/`
3. Verify `ACADOS_SOURCE_DIR` environment variable is set

### Compilation errors

- Missing headers: Check `ACADOS_SOURCE_DIR` in CMake
- Undefined references: Link the generated `.so` library
- Type mismatches: Ensure generated model name matches includes

### Runtime failures

- "Solver not initialized": acados capsule creation failed
- "Solve failed": QP infeasible or iteration limit exceeded
- Check console for detailed error messages from acados

## Next Steps

1. **Tune cost weights**: Adjust Q, R in generator script for desired behavior
2. **Add polytope constraints**: Replace box constraints with ellipse polygon
3. **Implement warm-starting**: Set initial guess from previous solution
4. **Add online reference adaptation**: Dynamic reference generation based on obstacles

## References

- [acados documentation](https://docs.acados.org/)
- [acados_template Python API](https://github.com/acados/acados/tree/master/interfaces/acados_template)
- Original example code provided by user
