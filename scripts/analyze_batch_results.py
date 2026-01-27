#!/usr/bin/env python3
"""
Analyze batch results from HumanoidMPC simulations
"""

import json
import sys
from pathlib import Path

def analyze_batch_results(results_file):
    """Analyze and display batch results"""
    
    with open(results_file, 'r') as f:
        results = json.load(f)
    
    print("=" * 80)
    print("BATCH RESULTS ANALYSIS")
    print("=" * 80)
    
    print(f"\nTotal runs: {len(results)}")
    
    # For each run, show summary
    for run in results:
        run_id = run['run_id']
        print(f"\n{'─' * 80}")
        print(f"RUN {run_id + 1}")
        print(f"{'─' * 80}")
        
        # Configuration
        print("\n📋 Configuration:")
        if 'mpc_weights' in run['config']:
            for key, val in run['config']['mpc_weights'].items():
                print(f"   {key}: {val}")
        print(f"   horizon: {run['config']['prediction_horizon']}")
        
        # Results
        print(f"\n⏱️  Timing:")
        print(f"   Completion time: {run['simulation_time_s']:.2f} s")
        print(f"   Wall clock time: {run['wall_time_s']:.2f} s")
        print(f"   Total steps: {run['total_steps']}")
        
        print(f"\n🚀 MPC Performance:")
        print(f"   Total MPC calls: {run['mpc_call_count']}")
        print(f"   Average MPC time: {run['avg_mpc_time_ms']:.3f} ms")
        print(f"   Min MPC time: {run['min_mpc_time_ms']:.3f} ms")
        print(f"   Max MPC time: {run['max_mpc_time_ms']:.3f} ms")
        print(f"   MPC frequency: {run['mpc_frequency_hz']:.1f} Hz")
        
        print(f"\n📊 MPC Times Array:")
        mpc_times = run['mpc_times_ms']
        print(f"   Total samples: {len(mpc_times)}")
        print(f"   First 5: {[f'{t:.3f}' for t in mpc_times[:5]]}")
        if len(mpc_times) > 5:
            print(f"   Last 5:  {[f'{t:.3f}' for t in mpc_times[-5:]]}")
        
        print(f"\n✅ Final State:")
        print(f"   Position error: {run['final_position_error_m']:.6f} m")
        print(f"   Final velocity: {run['final_velocity_ms']:.6f} m/s")
    
    # Comparison
    print(f"\n\n{'=' * 80}")
    print("COMPARISON")
    print("=" * 80)
    
    # Sort by completion time
    sorted_by_time = sorted(results, key=lambda r: r['simulation_time_s'])
    
    print("\n🏆 Fastest to Slowest Completion:")
    for i, run in enumerate(sorted_by_time):
        config_str = ""
        if 'mpc_weights' in run['config']:
            config_str = ", ".join([f"{k}={v}" for k, v in run['config']['mpc_weights'].items()])
        print(f"   {i+1}. Run {run['run_id']+1}: {run['simulation_time_s']:.2f}s  [{config_str}]")
    
    # Sort by MPC time
    sorted_by_mpc = sorted(results, key=lambda r: r['avg_mpc_time_ms'])
    
    print("\n⚡ Fastest to Slowest MPC Computation:")
    for i, run in enumerate(sorted_by_mpc):
        config_str = ""
        if 'mpc_weights' in run['config']:
            config_str = ", ".join([f"{k}={v}" for k, v in run['config']['mpc_weights'].items()])
        print(f"   {i+1}. Run {run['run_id']+1}: {run['avg_mpc_time_ms']:.3f}ms  [{config_str}]")
    
    print("\n" + "=" * 80)
    
    # Export MPC times to CSV for each run
    print("\n💾 Exporting MPC times to CSV files...")
    for run in results:
        csv_file = f"run_{run['run_id']}_mpc_times.csv"
        with open(csv_file, 'w') as f:
            f.write("step,mpc_time_ms\n")
            for i, time in enumerate(run['mpc_times_ms']):
                f.write(f"{i},{time}\n")
        print(f"   ✓ {csv_file}")
    
    print("\n" + "=" * 80 + "\n")

if __name__ == "__main__":
    if len(sys.argv) > 1:
        results_file = sys.argv[1]
    else:
        results_file = "batch_results.json"
    
    if not Path(results_file).exists():
        print(f"❌ Results file not found: {results_file}")
        sys.exit(1)
    
    analyze_batch_results(results_file)
