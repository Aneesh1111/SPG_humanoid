# Quick Start: Comprehensive Batch Simulations

## 🚀 Quick Examples

### Run baseline (40 simulations, ~5-10 min)
```bash
cd /home/robocup/Downloads/spg_cmake
./scripts/run_comprehensive_batch.py --output baseline.json
```

### Sweep a parameter (200 simulations, ~20-30 min)
```bash
# Sweep terminal position weight
./scripts/run_comprehensive_batch.py \
    --sweep-param qf_pos \
    --sweep-range 4 16 \
    --sweep-steps 5 \
    --output sweep_qf_pos.json

# Sweep angular velocity control
./scripts/run_comprehensive_batch.py \
    --sweep-param r_omega \
    --sweep-range 0.1 0.5 \
    --sweep-steps 4 \
    --output sweep_r_omega.json
```

### Analyze results
```bash
# Quick summary
./scripts/analyze_comprehensive_batch.py baseline.json --summary-only

# Full analysis
./scripts/analyze_comprehensive_batch.py baseline.json

# Sweep analysis
./scripts/analyze_comprehensive_batch.py sweep_qf_pos.json --sweep-param qf_pos
```

## 📊 What Gets Tested

**40 base scenarios** = 5 velocities × 8 targets

### Initial Velocities
- Stationary (0 m/s)
- Forward 1 m/s (×2)
- Rotating 1 rad/s (×2)

### Target Positions
- (1, 0, 0), (4, 4, 0), (-4, 0, 0)
- (1, 0, 0), (0, -5, -π), (0, 5, π/2)
- (-4, 6, 0), (4, -6, -π/2)

## ⚙️ All Sweepable Parameters

```bash
--sweep-param q_pos      # Position tracking weight
--sweep-param q_phi      # Orientation tracking weight
--sweep-param qf_pos     # Terminal position weight
--sweep-param qf_phi     # Terminal orientation weight
--sweep-param r_vf       # Forward velocity control
--sweep-param r_vs       # Sideways velocity control
--sweep-param r_omega    # Angular velocity control
--sweep-param s_vf       # Forward velocity smoothness
--sweep-param s_vs       # Sideways velocity smoothness
--sweep-param s_omega    # Angular velocity smoothness
```

## 📁 Output Structure

Single JSON file with all results:
```json
{
  "baseline_vel0_target0": {
    "simulation_time_s": 4.34,
    "avg_mpc_time_ms": 0.294,
    "final_position_error_m": 0.000541,
    "config": {...},
    "mpc_times_ms": [...]
  },
  ...
}
```

## 🔍 Python Analysis Example

```python
import json
import numpy as np

# Load results
with open('baseline.json', 'r') as f:
    results = json.load(f)

# Find best run
best = min(results.items(), 
           key=lambda x: x[1].get('final_position_error_m', float('inf')))
print(f"Best: {best[0]} with error {best[1]['final_position_error_m']:.6f}m")

# Average statistics
errors = [r['final_position_error_m'] for r in results.values() 
          if 'final_position_error_m' in r]
print(f"Mean error: {np.mean(errors):.6f}m")
print(f"Std error: {np.std(errors):.6f}m")
```

## 💡 Tips

1. **Start small**: Run baseline first to verify everything works
2. **Save results**: Use descriptive output filenames
3. **Analyze incrementally**: Check results after each sweep
4. **Use verbose mode** for debugging: `--verbose`
5. **Customize base weights**: Use `--qf-pos 10.0` etc. to set different defaults

## 📖 Full Documentation

See [COMPREHENSIVE_BATCH_README.md](COMPREHENSIVE_BATCH_README.md) for complete details.
