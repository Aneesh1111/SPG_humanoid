# MPC Weights Configuration

This JSON file contains the default MPC weight parameters for the comprehensive batch simulations.

## Weight Parameters

### Position/Orientation Tracking (Stage Cost)
- **q_pos**: Position tracking weight during trajectory (default: 1.0)
  - Higher values → more aggressive position tracking at each step
- **q_phi**: Orientation tracking weight during trajectory (default: 0.1)
  - Higher values → tighter orientation following

### Terminal Cost (Final State)
- **qf_pos**: Terminal position weight (default: 8.0)
  - Higher values → more precise final position reaching
- **qf_phi**: Terminal orientation weight (default: 1.0)
  - Higher values → more precise final orientation

### Control Input Penalty
- **r_vf**: Forward velocity control weight (default: 0.1)
  - Higher values → less aggressive forward acceleration
- **r_vs**: Sideways velocity control weight (default: 0.5)
  - Higher values → smoother sideways motion
- **r_omega**: Angular velocity control weight (default: 0.2)
  - Higher values → gentler rotation

### Control Smoothness (Rate of Change)
- **s_vf**: Forward velocity smoothness weight (default: 0.2)
  - Higher values → smoother forward acceleration changes
- **s_vs**: Sideways velocity smoothness weight (default: 0.8)
  - Higher values → smoother lateral motion transitions
- **s_omega**: Angular velocity smoothness weight (default: 0.4)
  - Higher values → smoother rotation transitions

## Usage

### Default behavior:
```bash
# Reads from mpc_weights.json by default
./scripts/run_comprehensive_batch.py --output baseline.json
```

### Custom weights file:
```bash
./scripts/run_comprehensive_batch.py \
    --weights-file my_custom_weights.json \
    --output results.json
```

### Override specific weights from file:
```bash
# Loads from mpc_weights.json but overrides qf_pos
./scripts/run_comprehensive_batch.py \
    --qf-pos 12.0 \
    --output results.json
```

### Multiple custom files for comparison:
```bash
# Aggressive settings
./scripts/run_comprehensive_batch.py \
    --weights-file aggressive_weights.json \
    --output aggressive_results.json

# Smooth settings
./scripts/run_comprehensive_batch.py \
    --weights-file smooth_weights.json \
    --output smooth_results.json
```

## Example Custom Weight Files

### aggressive_weights.json (Fast, less smooth)
```json
{
  "q_pos": 1.5,
  "q_phi": 0.2,
  "qf_pos": 12.0,
  "qf_phi": 2.0,
  "r_vf": 0.05,
  "r_vs": 0.3,
  "r_omega": 0.1,
  "s_vf": 0.1,
  "s_vs": 0.4,
  "s_omega": 0.2
}
```

### smooth_weights.json (Gentle, very smooth)
```json
{
  "q_pos": 0.8,
  "q_phi": 0.05,
  "qf_pos": 6.0,
  "qf_phi": 0.5,
  "r_vf": 0.2,
  "r_vs": 0.8,
  "r_omega": 0.4,
  "s_vf": 0.5,
  "s_vs": 1.2,
  "s_omega": 0.8
}
```

### precision_weights.json (Accurate positioning)
```json
{
  "q_pos": 2.0,
  "q_phi": 0.3,
  "qf_pos": 20.0,
  "qf_phi": 5.0,
  "r_vf": 0.1,
  "r_vs": 0.5,
  "r_omega": 0.2,
  "s_vf": 0.2,
  "s_vs": 0.8,
  "s_omega": 0.4
}
```

## Notes

- The script will use default hardcoded values if the weights file is not found
- Command-line arguments always override values from the file
- When sweeping parameters, the base values come from the file (or overrides)
- Only weights present in the JSON file will override defaults (partial files are OK)
