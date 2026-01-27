# HumanoidMPC Batch Runner

Run multiple simulations with different MPC tuning parameters to find optimal configurations.

## Quick Start

### 1. Single Run (Interactive with GUI)
```bash
python3 scripts/run_humanoid_mpc.py
```

With custom parameters:
```bash
python3 scripts/run_humanoid_mpc.py \
  --start-x 2.0 --start-y 4.0 \
  --target-x -1.0 --target-y -2.0 \
  --horizon 15 \
  --qf-pos 12.0
```

### 2. Batch Runs (Automated, Headless)

#### Sweep Terminal Position Weight
Test different values of `qf_pos` (4, 8, 12, 16):
```bash
python3 scripts/run_humanoid_batch.py --sweep-qf-pos
```

#### Sweep Prediction Horizon
Test different horizons (5, 10, 15, 20 steps):
```bash
python3 scripts/run_humanoid_batch.py --sweep-horizon
```

#### Sweep Smoothness Parameters
Test low/medium/high smoothness:
```bash
python3 scripts/run_humanoid_batch.py --sweep-smoothness
```

#### Custom Configuration File
Create a JSON file with multiple weight configurations:
```bash
python3 scripts/run_humanoid_batch.py --config-file scripts/example_mpc_configs.json
```

Example config file format:
```json
[
  {
    "q_pos": 1.0,
    "qf_pos": 8.0,
    "r_vf": 0.1,
    "s_vf": 0.2
  },
  {
    "q_pos": 2.0,
    "qf_pos": 12.0,
    "r_vf": 0.05,
    "s_vf": 0.1
  }
]
```

### 3. Customize Start/Target Positions
```bash
python3 scripts/run_humanoid_batch.py \
  --sweep-qf-pos \
  --start-x 1.0 --start-y 2.0 \
  --target-x -2.0 --target-y -3.0 \
  --output my_results.json
```

## Output

Each batch run generates:
1. **Console output** - Real-time progress and summary statistics
2. **JSON results file** - Complete data including:
   - MPC computation times (all samples)
   - Simulation completion time
   - Configuration used
   - Performance metrics

Example output:
```
╔============================================================╗
║                    BATCH SUMMARY                           ║
╚============================================================╝

Total runs: 4
Successful: 4
Failed: 0
Total time: 45.2 s

📊 Performance Statistics:
   Avg MPC time: 2.234 ms (range: 2.180 - 2.312)
   Sim completion: 8.45 s (range: 7.82 - 9.21)

🏆 Fastest completion: Run 2 (7.82 s)
   Weights: {'q_pos': 2.0, 'qf_pos': 12.0, 'r_vf': 0.05}

💾 Full results saved to: batch_results.json
```

## MPC Tuning Parameters

### Position/Orientation Tracking
- `q_pos` (default: 1.0) - Stage position tracking weight
- `q_phi` (default: 0.1) - Stage orientation tracking weight
- `qf_pos` (default: 8.0) - **Terminal position weight** (higher = must reach goal)
- `qf_phi` (default: 1.0) - Terminal orientation weight

### Control Effort
- `r_vf` (default: 0.1) - Forward velocity effort (lower = faster forward motion)
- `r_vs` (default: 0.5) - Sideways velocity effort (higher = avoid sideways)
- `r_omega` (default: 0.2) - Angular velocity effort

### Smoothness
- `s_vf` (default: 0.2) - Forward acceleration smoothness
- `s_vs` (default: 0.8) - Sideways acceleration smoothness
- `s_omega` (default: 0.4) - Angular acceleration smoothness

### System Parameters
- `horizon` (default: 10) - MPC prediction horizon steps (10 steps = 0.2s @ 50Hz)

## Tips for Tuning

1. **Faster completion**: Increase `qf_pos`, decrease `r_vf`, decrease `s_vf`
2. **Smoother motion**: Increase `s_vf`, `s_vs`, `s_omega`
3. **Better tracking**: Increase `q_pos`, `qf_pos`
4. **More agile**: Decrease `r_omega`, `s_omega`

## Analyzing Results

Results are saved in JSON format. Each run contains:
```json
{
  "run_id": 0,
  "simulation_time_s": 8.24,
  "wall_time_s": 10.5,
  "total_steps": 412,
  "mpc_call_count": 412,
  "avg_mpc_time_ms": 2.234,
  "min_mpc_time_ms": 2.101,
  "max_mpc_time_ms": 3.456,
  "mpc_times_ms": [2.23, 2.18, ...],
  "config": {
    "mpc_weights": { "qf_pos": 12.0, ... }
  }
}
```

You can load and analyze in Python:
```python
import json
import numpy as np

with open('batch_results.json', 'r') as f:
    results = json.load(f)

# Find fastest run
fastest = min(results, key=lambda r: r['simulation_time_s'])
print(f"Best config: {fastest['config']['mpc_weights']}")

# Analyze MPC times
for r in results:
    times = np.array(r['mpc_times_ms'])
    print(f"Run {r['run_id']}: mean={times.mean():.3f}, std={times.std():.3f}")
```
