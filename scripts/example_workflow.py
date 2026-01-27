#!/usr/bin/env python3
"""
Example workflow showing how to use the comprehensive batch system
This script demonstrates the complete workflow from running to analyzing
"""

import subprocess
import json
import numpy as np
from pathlib import Path

def example_workflow():
    """Complete example workflow"""
    
    print("="*70)
    print("COMPREHENSIVE BATCH SIMULATION - EXAMPLE WORKFLOW")
    print("="*70)
    print()
    
    # Check if we're in the right directory
    if not Path("scripts/run_comprehensive_batch.py").exists():
        print("⚠️  Please run this from the project root directory")
        print("   cd /home/robocup/Downloads/spg_cmake")
        return
    
    print("This example demonstrates the complete workflow:")
    print()
    print("1. Run baseline simulations (40 scenarios)")
    print("2. Run a parameter sweep (e.g., qf_pos)")
    print("3. Analyze and compare results")
    print("4. Extract insights")
    print()
    
    # Example 1: Baseline
    print("\n" + "="*70)
    print("EXAMPLE 1: Baseline Simulations")
    print("="*70)
    print("\nCommand:")
    print("  ./scripts/run_comprehensive_batch.py --output baseline.json")
    print("\nThis will run:")
    print("  - 40 simulations (5 velocities × 8 targets)")
    print("  - All with default MPC weights")
    print("  - Takes ~5-10 minutes")
    print("\nOutput: baseline.json")
    
    # Example 2: Parameter Sweep
    print("\n" + "="*70)
    print("EXAMPLE 2: Parameter Sweep (qf_pos)")
    print("="*70)
    print("\nCommand:")
    print("  ./scripts/run_comprehensive_batch.py \\")
    print("      --sweep-param qf_pos \\")
    print("      --sweep-range 4 16 \\")
    print("      --sweep-steps 5 \\")
    print("      --output sweep_qf_pos.json")
    print("\nThis will run:")
    print("  - 200 simulations (5 qf_pos values × 40 scenarios)")
    print("  - qf_pos values: [4.0, 7.0, 10.0, 13.0, 16.0]")
    print("  - Takes ~20-30 minutes")
    print("\nOutput: sweep_qf_pos.json")
    
    # Example 3: Analysis
    print("\n" + "="*70)
    print("EXAMPLE 3: Analyze Results")
    print("="*70)
    print("\nCommand for baseline:")
    print("  ./scripts/analyze_comprehensive_batch.py baseline.json")
    print("\nCommand for sweep:")
    print("  ./scripts/analyze_comprehensive_batch.py sweep_qf_pos.json \\")
    print("      --sweep-param qf_pos")
    print("\nThis shows:")
    print("  - Summary statistics")
    print("  - Best/worst performers")
    print("  - Parameter sweep analysis (for sweeps)")
    print("  - Scenario breakdowns")
    
    # Example 4: Custom Analysis
    print("\n" + "="*70)
    print("EXAMPLE 4: Custom Python Analysis")
    print("="*70)
    print("\nPython code:")
    print('''
import json
import numpy as np

# Load results
with open('sweep_qf_pos.json', 'r') as f:
    results = json.load(f)

# Group by qf_pos value
sweep_data = {}
for run_id, data in results.items():
    if 'qf_pos' in run_id and 'final_position_error_m' in data:
        param_val = float(run_id.split('_')[1])
        if param_val not in sweep_data:
            sweep_data[param_val] = {'errors': [], 'mpc_times': []}
        sweep_data[param_val]['errors'].append(data['final_position_error_m'])
        sweep_data[param_val]['mpc_times'].append(data['avg_mpc_time_ms'])

# Print summary
print("qf_pos | Avg Error (m) | Avg MPC Time (ms)")
print("-------|---------------|------------------")
for val in sorted(sweep_data.keys()):
    avg_err = np.mean(sweep_data[val]['errors'])
    avg_mpc = np.mean(sweep_data[val]['mpc_times'])
    print(f"{val:6.1f} | {avg_err:13.6f} | {avg_mpc:17.3f}")

# Find optimal
best = min(sweep_data.items(), key=lambda x: np.mean(x[1]['errors']))
print(f"\\nOptimal qf_pos: {best[0]:.1f}")
print(f"  Average error: {np.mean(best[1]['errors']):.6f} m")
print(f"  Average MPC time: {np.mean(best[1]['mpc_times']):.3f} ms")
    ''')
    
    # Example 5: Visualization
    print("\n" + "="*70)
    print("EXAMPLE 5: Visualization")
    print("="*70)
    print("\nPython code with matplotlib:")
    print('''
import json
import numpy as np
import matplotlib.pyplot as plt

# Load and process data
with open('sweep_qf_pos.json', 'r') as f:
    results = json.load(f)

sweep_data = {}
for run_id, data in results.items():
    if 'qf_pos' in run_id and 'final_position_error_m' in data:
        param_val = float(run_id.split('_')[1])
        if param_val not in sweep_data:
            sweep_data[param_val] = []
        sweep_data[param_val].append(data['final_position_error_m'])

# Plot
vals = sorted(sweep_data.keys())
means = [np.mean(sweep_data[v]) for v in vals]
stds = [np.std(sweep_data[v]) for v in vals]

plt.figure(figsize=(10, 6))
plt.errorbar(vals, means, yerr=stds, marker='o', capsize=5, linewidth=2)
plt.xlabel('qf_pos (Terminal Position Weight)', fontsize=12)
plt.ylabel('Average Position Error (m)', fontsize=12)
plt.title('Effect of Terminal Weight on Tracking Accuracy', fontsize=14)
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig('qf_pos_analysis.png', dpi=150)
plt.show()
    ''')
    
    # Example 6: Multi-Parameter Study
    print("\n" + "="*70)
    print("EXAMPLE 6: Multi-Parameter Study")
    print("="*70)
    print("\nRun multiple sweeps in sequence:")
    print('''
# Sweep 1: Terminal position weight
./scripts/run_comprehensive_batch.py \\
    --sweep-param qf_pos --sweep-range 4 16 --sweep-steps 5 \\
    --output sweep_qf_pos.json

# Sweep 2: Angular velocity control
./scripts/run_comprehensive_batch.py \\
    --sweep-param r_omega --sweep-range 0.1 0.5 --sweep-steps 5 \\
    --output sweep_r_omega.json

# Sweep 3: Position tracking weight
./scripts/run_comprehensive_batch.py \\
    --sweep-param q_pos --sweep-range 0.5 2.0 --sweep-steps 5 \\
    --output sweep_q_pos.json

# Compare all three
./scripts/analyze_comprehensive_batch.py sweep_qf_pos.json --sweep-param qf_pos
./scripts/analyze_comprehensive_batch.py sweep_r_omega.json --sweep-param r_omega
./scripts/analyze_comprehensive_batch.py sweep_q_pos.json --sweep-param q_pos
    ''')
    
    # Summary
    print("\n" + "="*70)
    print("SUMMARY")
    print("="*70)
    print("\n✅ The comprehensive batch system provides:")
    print("   1. Automated testing of 40+ diverse scenarios")
    print("   2. Systematic parameter sweeping")
    print("   3. Complete performance metrics")
    print("   4. Easy analysis and visualization")
    print("\n📚 Documentation:")
    print("   - QUICK_START_BATCH.md - Quick reference")
    print("   - COMPREHENSIVE_BATCH_README.md - Full details")
    print("   - BATCH_SUMMARY.md - Overview")
    print("\n🚀 Ready to start? Run:")
    print("   ./scripts/run_comprehensive_batch.py --output my_first_batch.json")
    print()

if __name__ == "__main__":
    example_workflow()
