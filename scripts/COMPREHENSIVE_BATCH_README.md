# Comprehensive Batch Simulation Runner

## Overview

`run_comprehensive_batch.py` is a powerful batch simulation tool that runs the HumanoidMPC simulator across multiple scenarios and optionally sweeps MPC parameters to analyze performance.

## Features

- **30+ Simulation Scenarios**: Automatically tests combinations of:
  - 5 initial velocity conditions (stationary, forward motion, rotation)
  - 8 target positions (various distances and angles)
  - Initial robot position always at (0, 0, 0)

- **MPC Parameter Sweeps**: Sweep any MPC weight parameter over a range
  - Support for all 10 MPC weight parameters
  - Configurable range and step size
  - One parameter at a time to isolate effects

- **Comprehensive Results**: Single JSON output with all simulation data
  - Individual run timing data
  - MPC performance metrics
  - Position/velocity errors
  - Configuration details for each run

## Initial Conditions

### Velocities (5 configurations)
1. Stationary: (vf=0, vs=0, omega=0)
2. Forward 1 m/s: (vf=1, vs=0, omega=0)
3. Forward 1 m/s: (vf=1, vs=0, omega=0) [duplicate]
4. Rotating 1 rad/s: (vf=0, vs=0, omega=1)
5. Rotating 1 rad/s: (vf=0, vs=0, omega=1) [duplicate]

### Target Positions (8 configurations)
1. (1, 0, 0)
2. (4, 4, 0)
3. (-4, 0, 0)
4. (1, 0, 0) [duplicate]
5. (0, -5, -π)
6. (0, 5, π/2)
7. (-4, 6, 0)
8. (4, -6, -π/2)

**Total baseline scenarios**: 5 velocities × 8 targets = 40 simulations

## Usage Examples

### 1. Run Baseline Scenarios (No Parameter Sweep)

Run all 40 baseline scenarios with default MPC weights:

```bash
./scripts/run_comprehensive_batch.py --output baseline_results.json
```

### 2. Sweep Terminal Position Weight (qf_pos)

Test different values of the terminal position weight:

```bash
./scripts/run_comprehensive_batch.py \
    --sweep-param qf_pos \
    --sweep-range 4 16 \
    --sweep-steps 5 \
    --output sweep_qf_pos.json
```

This runs: 5 sweep values × 40 scenarios = **200 simulations**

### 3. Sweep Control Input Weight (r_omega)

Test the effect of angular velocity control weight:

```bash
./scripts/run_comprehensive_batch.py \
    --sweep-param r_omega \
    --sweep-range 0.1 0.5 \
    --sweep-steps 4 \
    --output sweep_r_omega.json
```

This runs: 4 sweep values × 40 scenarios = **160 simulations**

### 4. Sweep Position Tracking Weight (q_pos)

```bash
./scripts/run_comprehensive_batch.py \
    --sweep-param q_pos \
    --sweep-range 0.5 2.0 \
    --sweep-steps 6 \
    --output sweep_q_pos.json
```

### 5. Custom MPC Horizon

Change the prediction horizon:

```bash
./scripts/run_comprehensive_batch.py \
    --horizon 15 \
    --output baseline_horizon15.json
```

### 6. Sweep with Custom Base Weights

Set different base weights while sweeping:

```bash
./scripts/run_comprehensive_batch.py \
    --sweep-param qf_pos \
    --sweep-range 6 12 \
    --sweep-steps 4 \
    --qf-phi 2.0 \
    --r-omega 0.3 \
    --output custom_sweep.json
```

### 7. Verbose Mode

See detailed output for each simulation:

```bash
./scripts/run_comprehensive_batch.py \
    --sweep-param r_vf \
    --sweep-range 0.05 0.2 \
    --sweep-steps 4 \
    --verbose \
    --output sweep_r_vf.json
```

## MPC Weight Parameters

All available parameters for sweeping:

| Parameter | Description | Default |
|-----------|-------------|---------|
| `q_pos` | Position tracking weight | 1.0 |
| `q_phi` | Orientation tracking weight | 0.1 |
| `qf_pos` | Terminal position weight | 8.0 |
| `qf_phi` | Terminal orientation weight | 1.0 |
| `r_vf` | Forward velocity control weight | 0.1 |
| `r_vs` | Sideways velocity control weight | 0.5 |
| `r_omega` | Angular velocity control weight | 0.2 |
| `s_vf` | Forward velocity smoothness | 0.2 |
| `s_vs` | Sideways velocity smoothness | 0.8 |
| `s_omega` | Angular velocity smoothness | 0.4 |

## Output Format

The output JSON file contains a dictionary with one entry per simulation:

```json
{
  "baseline_vel0_target0": {
    "simulation_time_s": 4.34,
    "wall_time_s": 4.44,
    "total_steps": 217,
    "mpc_call_count": 217,
    "total_mpc_time_ms": 63.69,
    "avg_mpc_time_ms": 0.294,
    "min_mpc_time_ms": 0.235,
    "max_mpc_time_ms": 2.493,
    "mpc_frequency_hz": 3406.94,
    "final_position_error_m": 0.000541,
    "final_velocity_ms": 0.000971,
    "mpc_times_ms": [...],
    "run_id": "baseline_vel0_target0",
    "config": {
      "start_pos": {"x": 0.0, "y": 0.0, "theta": 0.0},
      "target_pos": {"x": 1.0, "y": 0.0, "theta": 0.0},
      "initial_velocity": {"vf": 0.0, "vs": 0.0, "omega": 0.0},
      "prediction_horizon": 10,
      "mpc_weights": {...}
    }
  },
  "baseline_vel0_target1": {...},
  ...
}
```

## Analysis Tips

### 1. Find Best Performing Configuration

```python
import json

with open('sweep_qf_pos.json', 'r') as f:
    results = json.load(f)

# Sort by position error
best = min(results.items(), key=lambda x: x[1].get('final_position_error_m', float('inf')))
print(f"Best config: {best[0]}")
print(f"Position error: {best[1]['final_position_error_m']:.6f} m")
print(f"Config: {best[1]['config']}")
```

### 2. Compare Sweep Values

```python
import json
import numpy as np

with open('sweep_qf_pos.json', 'r') as f:
    results = json.load(f)

# Group by sweep value
sweep_data = {}
for run_id, data in results.items():
    if 'qf_pos' in run_id and 'avg_mpc_time_ms' in data:
        param_val = float(run_id.split('_')[1])
        if param_val not in sweep_data:
            sweep_data[param_val] = {'mpc_times': [], 'pos_errors': []}
        sweep_data[param_val]['mpc_times'].append(data['avg_mpc_time_ms'])
        sweep_data[param_val]['pos_errors'].append(data['final_position_error_m'])

# Print averages
for param_val in sorted(sweep_data.keys()):
    avg_mpc = np.mean(sweep_data[param_val]['mpc_times'])
    avg_err = np.mean(sweep_data[param_val]['pos_errors'])
    print(f"qf_pos={param_val:.3f}: MPC={avg_mpc:.3f}ms, Error={avg_err:.6f}m")
```

### 3. Visualize Results

```python
import json
import matplotlib.pyplot as plt
import numpy as np

with open('sweep_qf_pos.json', 'r') as f:
    results = json.load(f)

# Extract sweep data
sweep_vals = []
avg_errors = []

sweep_data = {}
for run_id, data in results.items():
    if 'qf_pos' in run_id and 'final_position_error_m' in data:
        param_val = float(run_id.split('_')[1])
        if param_val not in sweep_data:
            sweep_data[param_val] = []
        sweep_data[param_val].append(data['final_position_error_m'])

for param_val in sorted(sweep_data.keys()):
    sweep_vals.append(param_val)
    avg_errors.append(np.mean(sweep_data[param_val]))

plt.figure(figsize=(10, 6))
plt.plot(sweep_vals, avg_errors, 'o-', linewidth=2, markersize=8)
plt.xlabel('qf_pos (Terminal Position Weight)', fontsize=12)
plt.ylabel('Average Position Error (m)', fontsize=12)
plt.title('Effect of Terminal Position Weight on Tracking Accuracy', fontsize=14)
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig('qf_pos_sweep_results.png', dpi=150)
plt.show()
```

## Performance Considerations

- Each simulation takes ~5-10 seconds depending on scenario complexity
- A full parameter sweep (5 steps) takes ~10-20 minutes for all 200 simulations
- Results are saved after all simulations complete
- Use Ctrl+C to cancel the batch before starting (after seeing the plan)

## Troubleshooting

### Timeout Issues

If simulations timeout (>120s), the scenario might be too challenging. Check:
- Target is reachable from initial state
- MPC weights are reasonable (not too extreme)

### No Timing Data

If runs complete but produce no data:
- Check that `demo_humanoid_mpc` saves `mpc_timing_results.json`
- Verify the executable is built correctly

### Memory Issues

For very large sweeps:
- Run smaller batches separately
- Merge results manually using Python

## Examples of Research Questions

1. **What's the optimal terminal weight?**
   ```bash
   ./scripts/run_comprehensive_batch.py --sweep-param qf_pos --sweep-range 2 20 --sweep-steps 10
   ```

2. **How does control smoothness affect performance?**
   ```bash
   ./scripts/run_comprehensive_batch.py --sweep-param s_omega --sweep-range 0.1 1.0 --sweep-steps 6
   ```

3. **Does prediction horizon matter?**
   ```bash
   # Run multiple times with different horizons
   for h in 5 10 15 20; do
     ./scripts/run_comprehensive_batch.py --horizon $h --output baseline_h${h}.json
   done
   ```

4. **Compare aggressive vs smooth control:**
   ```bash
   # Low smoothness (aggressive)
   ./scripts/run_comprehensive_batch.py --s-vf 0.05 --s-vs 0.1 --s-omega 0.1 --output aggressive.json
   
   # High smoothness (gentle)
   ./scripts/run_comprehensive_batch.py --s-vf 0.5 --s-vs 1.5 --s-omega 1.0 --output smooth.json
   ```

## Integration with Other Tools

This script can be used with:
- `analyze_batch_results.py` - For statistical analysis
- `bayesian_optimization.py` - For smart parameter search
- Custom plotting scripts - For visualization

## Notes

- All simulations run in headless mode (no GUI)
- Individual config files are saved in `build/` for reference
- Script assumes demo is built and located at `build/demo_humanoid_mpc`
- Initial robot position is always (0, 0, 0) as specified
