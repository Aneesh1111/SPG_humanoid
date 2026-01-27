# Summary: Comprehensive Batch Simulation System

## Files Created

### 1. Main Batch Runner
**File**: [scripts/run_comprehensive_batch.py](scripts/run_comprehensive_batch.py)

A comprehensive batch simulation runner that:
- ✅ Tests **40 base scenarios** (5 velocities × 8 targets)
- ✅ Initial robot position always at (0, 0, 0)
- ✅ Supports **parameter sweeps** for any of 10 MPC weights
- ✅ Outputs **single JSON dictionary** with all results
- ✅ Includes all timing metrics from `mpc_timing_results.json`
- ✅ Configurable ranges and steps for sweeps
- ✅ Progress tracking and summary statistics

### 2. Analysis Tool
**File**: [scripts/analyze_comprehensive_batch.py](scripts/analyze_comprehensive_batch.py)

Comprehensive analysis script that provides:
- Summary statistics across all runs
- Best/worst performers by any metric
- Parameter sweep analysis with optimal values
- Scenario comparison (by velocity and target)
- Statistical breakdowns (mean, median, std, etc.)

### 3. Documentation
- **[scripts/COMPREHENSIVE_BATCH_README.md](scripts/COMPREHENSIVE_BATCH_README.md)** - Full detailed documentation
- **[scripts/QUICK_START_BATCH.md](scripts/QUICK_START_BATCH.md)** - Quick reference guide

## Test Scenarios

### Initial Velocities (5 configurations)
```
1. (vf=0.0, vs=0.0, omega=0.0)    # Stationary
2. (vf=1.0, vs=0.0, omega=0.0)    # Forward 1 m/s
3. (vf=1.0, vs=0.0, omega=0.0)    # Forward 1 m/s (duplicate)
4. (vf=0.0, vs=0.0, omega=1.0)    # Rotating 1 rad/s
5. (vf=0.0, vs=0.0, omega=1.0)    # Rotating 1 rad/s (duplicate)
```

### Target Positions (8 configurations)
```
1. (1, 0, 0)
2. (4, 4, 0)
3. (-4, 0, 0)
4. (1, 0, 0)           # Duplicate as requested
5. (0, -5, -π)
6. (0, 5, π/2)
7. (-4, 6, 0)
8. (4, -6, -π/2)
```

**Total**: 5 × 8 = 40 scenarios per parameter configuration

## Usage Examples

### 1. Run Baseline (No Sweep)
```bash
./scripts/run_comprehensive_batch.py --output baseline.json
```
**Result**: 40 simulations with default MPC weights

### 2. Sweep Terminal Position Weight
```bash
./scripts/run_comprehensive_batch.py \
    --sweep-param qf_pos \
    --sweep-range 4 16 \
    --sweep-steps 5 \
    --output sweep_qf_pos.json
```
**Result**: 5 weight values × 40 scenarios = 200 simulations

### 3. Sweep with Custom Base Weights
```bash
./scripts/run_comprehensive_batch.py \
    --sweep-param r_omega \
    --sweep-range 0.1 0.5 \
    --sweep-steps 4 \
    --qf-pos 12.0 \
    --qf-phi 2.0 \
    --output custom_sweep.json
```

### 4. Analyze Results
```bash
# Quick summary
./scripts/analyze_comprehensive_batch.py baseline.json --summary-only

# Full analysis with sweep detection
./scripts/analyze_comprehensive_batch.py sweep_qf_pos.json --sweep-param qf_pos

# Find top performers by MPC speed
./scripts/analyze_comprehensive_batch.py baseline.json --metric avg_mpc_time_ms --top-n 10
```

## Output Format

The output JSON contains a dictionary where each key is a run ID and each value contains:

```json
{
  "qf_pos_8.000_vel0_target0": {
    "simulation_time_s": 4.34,
    "wall_time_s": 4.44255,
    "total_steps": 217,
    "mpc_call_count": 217,
    "total_mpc_time_ms": 63.6935,
    "avg_mpc_time_ms": 0.293518,
    "min_mpc_time_ms": 0.235499,
    "max_mpc_time_ms": 2.49262,
    "mpc_frequency_hz": 3406.94,
    "final_position_error_m": 0.000541266,
    "final_velocity_ms": 0.00097128,
    "mpc_times_ms": [...],
    "run_id": "qf_pos_8.000_vel0_target0",
    "config": {
      "start_pos": {"x": 0.0, "y": 0.0, "theta": 0.0},
      "target_pos": {"x": 1.0, "y": 0.0, "theta": 0.0},
      "initial_velocity": {"vf": 0.0, "vs": 0.0, "omega": 0.0},
      "prediction_horizon": 10,
      "mpc_weights": {
        "q_pos": 1.0,
        "q_phi": 0.1,
        "qf_pos": 8.0,
        ...
      }
    }
  },
  ...
}
```

## All Sweepable Parameters

| Parameter | Description | Default | Typical Range |
|-----------|-------------|---------|---------------|
| `q_pos` | Position tracking weight | 1.0 | 0.5 - 2.0 |
| `q_phi` | Orientation tracking weight | 0.1 | 0.05 - 0.5 |
| `qf_pos` | Terminal position weight | 8.0 | 4.0 - 20.0 |
| `qf_phi` | Terminal orientation weight | 1.0 | 0.5 - 5.0 |
| `r_vf` | Forward velocity control | 0.1 | 0.05 - 0.3 |
| `r_vs` | Sideways velocity control | 0.5 | 0.2 - 1.0 |
| `r_omega` | Angular velocity control | 0.2 | 0.1 - 0.5 |
| `s_vf` | Forward velocity smoothness | 0.2 | 0.1 - 0.5 |
| `s_vs` | Sideways velocity smoothness | 0.8 | 0.3 - 1.5 |
| `s_omega` | Angular velocity smoothness | 0.4 | 0.1 - 1.0 |

## Performance Expectations

- **Per simulation**: ~5-10 seconds
- **40 baseline scenarios**: ~5-10 minutes
- **200 simulation sweep** (5 steps): ~20-30 minutes
- **400 simulation sweep** (10 steps): ~40-60 minutes

## Python Analysis Example

```python
import json
import numpy as np

# Load results
with open('sweep_qf_pos.json', 'r') as f:
    results = json.load(f)

# Group by parameter value
sweep_data = {}
for run_id, data in results.items():
    if 'qf_pos' in run_id and 'final_position_error_m' in data:
        param_val = float(run_id.split('_')[1])
        if param_val not in sweep_data:
            sweep_data[param_val] = []
        sweep_data[param_val].append(data['final_position_error_m'])

# Find optimal
for val in sorted(sweep_data.keys()):
    avg_error = np.mean(sweep_data[val])
    print(f"qf_pos={val:.2f}: error={avg_error:.6f}m")

# Find best
best_val = min(sweep_data.items(), key=lambda x: np.mean(x[1]))
print(f"\nOptimal qf_pos: {best_val[0]:.2f}")
```

## Integration

These scripts are designed to work with:
- Existing `demo_humanoid_mpc` executable
- Standard `mpc_timing_results.json` output format
- Existing config file structure
- Other analysis scripts in the project

## Next Steps

1. **Run baseline**: `./scripts/run_comprehensive_batch.py --output baseline.json`
2. **Analyze**: `./scripts/analyze_comprehensive_batch.py baseline.json`
3. **Sweep important parameters**: Start with `qf_pos`, `r_omega`, `q_pos`
4. **Compare results**: Use analysis script or custom Python code
5. **Optimize**: Find best performing parameter combinations

## Files Summary

```
scripts/
├── run_comprehensive_batch.py          # Main batch runner (executable)
├── analyze_comprehensive_batch.py      # Analysis tool (executable)
├── COMPREHENSIVE_BATCH_README.md       # Full documentation
├── QUICK_START_BATCH.md               # Quick reference
└── BATCH_SUMMARY.md                   # This file
```

All scripts are executable and ready to use!
