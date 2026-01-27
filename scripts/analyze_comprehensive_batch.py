#!/usr/bin/env python3
"""
Analysis script for comprehensive batch simulation results
Provides quick statistics and visualizations
"""

import json
import sys
import argparse
from pathlib import Path
import numpy as np
from typing import Dict, List, Tuple

def load_results(filename: str) -> Dict:
    """Load results from JSON file"""
    with open(filename, 'r') as f:
        return json.load(f)

def print_summary(results: Dict):
    """Print summary statistics"""
    total = len(results)
    successful = sum(1 for r in results.values() if 'simulation_time_s' in r)
    failed = total - successful
    
    print("\n" + "="*70)
    print("BATCH RESULTS SUMMARY")
    print("="*70)
    print(f"\nTotal simulations: {total}")
    print(f"  Successful: {successful}")
    print(f"  Failed/Timeout: {failed}")
    
    if successful == 0:
        print("\n⚠️  No successful simulations to analyze")
        return
    
    # Extract metrics from successful runs
    sim_times = []
    avg_mpc_times = []
    max_mpc_times = []
    position_errors = []
    velocity_errors = []
    mpc_call_counts = []
    
    for run_id, data in results.items():
        if 'simulation_time_s' in data:
            sim_times.append(data['simulation_time_s'])
            avg_mpc_times.append(data['avg_mpc_time_ms'])
            max_mpc_times.append(data['max_mpc_time_ms'])
            position_errors.append(data['final_position_error_m'])
            velocity_errors.append(data.get('final_velocity_ms', 0))
            mpc_call_counts.append(data['mpc_call_count'])
    
    # Print statistics
    print("\n📊 Simulation Time:")
    print(f"   Min: {min(sim_times):.2f} s")
    print(f"   Max: {max(sim_times):.2f} s")
    print(f"   Mean: {np.mean(sim_times):.2f} s")
    print(f"   Median: {np.median(sim_times):.2f} s")
    print(f"   Std: {np.std(sim_times):.2f} s")
    
    print("\n🚀 MPC Performance (Average):")
    print(f"   Min: {min(avg_mpc_times):.3f} ms")
    print(f"   Max: {max(avg_mpc_times):.3f} ms")
    print(f"   Mean: {np.mean(avg_mpc_times):.3f} ms")
    print(f"   Median: {np.median(avg_mpc_times):.3f} ms")
    
    print("\n🚀 MPC Performance (Max):")
    print(f"   Min: {min(max_mpc_times):.3f} ms")
    print(f"   Max: {max(max_mpc_times):.3f} ms")
    print(f"   Mean: {np.mean(max_mpc_times):.3f} ms")
    
    print("\n✅ Position Accuracy:")
    print(f"   Best: {min(position_errors):.6f} m")
    print(f"   Worst: {max(position_errors):.6f} m")
    print(f"   Mean: {np.mean(position_errors):.6f} m")
    print(f"   Median: {np.median(position_errors):.6f} m")
    
    print("\n🎯 MPC Calls:")
    print(f"   Min: {min(mpc_call_counts)}")
    print(f"   Max: {max(mpc_call_counts)}")
    print(f"   Mean: {np.mean(mpc_call_counts):.1f}")

def find_best_worst(results: Dict, metric: str = 'final_position_error_m', n: int = 5):
    """Find best and worst performing runs by a metric"""
    
    # Filter successful runs
    successful = {k: v for k, v in results.items() if metric in v}
    
    if not successful:
        print(f"\n⚠️  No successful runs with metric '{metric}'")
        return
    
    # Sort by metric
    sorted_runs = sorted(successful.items(), key=lambda x: x[1][metric])
    
    print("\n" + "="*70)
    print(f"TOP {n} BEST PERFORMERS ({metric})")
    print("="*70)
    
    for i, (run_id, data) in enumerate(sorted_runs[:n], 1):
        print(f"\n{i}. {run_id}")
        print(f"   {metric}: {data[metric]:.6f}")
        print(f"   Sim time: {data.get('simulation_time_s', 'N/A'):.2f} s")
        print(f"   Avg MPC: {data.get('avg_mpc_time_ms', 'N/A'):.3f} ms")
        if 'config' in data:
            cfg = data['config']
            print(f"   Target: ({cfg['target_pos']['x']:.1f}, {cfg['target_pos']['y']:.1f}, {cfg['target_pos']['theta']:.2f})")
            if 'initial_velocity' in cfg:
                vel = cfg['initial_velocity']
                print(f"   Init vel: (vf={vel['vf']:.2f}, vs={vel['vs']:.2f}, omega={vel['omega']:.2f})")
    
    print("\n" + "="*70)
    print(f"TOP {n} WORST PERFORMERS ({metric})")
    print("="*70)
    
    for i, (run_id, data) in enumerate(sorted_runs[-n:][::-1], 1):
        print(f"\n{i}. {run_id}")
        print(f"   {metric}: {data[metric]:.6f}")
        print(f"   Sim time: {data.get('simulation_time_s', 'N/A'):.2f} s")
        print(f"   Avg MPC: {data.get('avg_mpc_time_ms', 'N/A'):.3f} ms")
        if 'config' in data:
            cfg = data['config']
            print(f"   Target: ({cfg['target_pos']['x']:.1f}, {cfg['target_pos']['y']:.1f}, {cfg['target_pos']['theta']:.2f})")
            if 'initial_velocity' in cfg:
                vel = cfg['initial_velocity']
                print(f"   Init vel: (vf={vel['vf']:.2f}, vs={vel['vs']:.2f}, omega={vel['omega']:.2f})")

def analyze_sweep(results: Dict, param_name: str = None):
    """Analyze parameter sweep results"""
    
    # Try to detect sweep parameter if not provided
    if param_name is None:
        for run_id in results.keys():
            parts = run_id.split('_')
            if len(parts) >= 2 and parts[0] not in ['baseline', 'run']:
                param_name = parts[0]
                break
    
    if param_name is None:
        print("\n⚠️  No parameter sweep detected. Use --sweep-param to specify.")
        return
    
    print("\n" + "="*70)
    print(f"PARAMETER SWEEP ANALYSIS: {param_name}")
    print("="*70)
    
    # Group results by parameter value
    sweep_data = {}
    for run_id, data in results.items():
        if param_name in run_id and 'avg_mpc_time_ms' in data:
            try:
                param_val = float(run_id.split('_')[1])
            except (IndexError, ValueError):
                continue
            
            if param_val not in sweep_data:
                sweep_data[param_val] = {
                    'mpc_times': [],
                    'pos_errors': [],
                    'sim_times': [],
                    'max_mpc_times': []
                }
            
            sweep_data[param_val]['mpc_times'].append(data['avg_mpc_time_ms'])
            sweep_data[param_val]['pos_errors'].append(data['final_position_error_m'])
            sweep_data[param_val]['sim_times'].append(data['simulation_time_s'])
            sweep_data[param_val]['max_mpc_times'].append(data['max_mpc_time_ms'])
    
    if not sweep_data:
        print(f"\n⚠️  No sweep data found for parameter '{param_name}'")
        return
    
    # Print table
    print(f"\n{'Param Value':<12} {'Scenarios':<10} {'Avg MPC (ms)':<15} {'Pos Error (m)':<15} {'Sim Time (s)':<15}")
    print("-" * 70)
    
    for param_val in sorted(sweep_data.keys()):
        data = sweep_data[param_val]
        n_scenarios = len(data['mpc_times'])
        avg_mpc = np.mean(data['mpc_times'])
        avg_err = np.mean(data['pos_errors'])
        avg_sim = np.mean(data['sim_times'])
        
        print(f"{param_val:<12.3f} {n_scenarios:<10} {avg_mpc:<15.3f} {avg_err:<15.6f} {avg_sim:<15.2f}")
    
    # Find optimal value
    print("\n🏆 Optimal Values:")
    
    best_by_error = min(sweep_data.items(), key=lambda x: np.mean(x[1]['pos_errors']))
    best_by_speed = min(sweep_data.items(), key=lambda x: np.mean(x[1]['mpc_times']))
    
    print(f"   Best position error: {param_name}={best_by_error[0]:.3f} (error={np.mean(best_by_error[1]['pos_errors']):.6f}m)")
    print(f"   Fastest MPC: {param_name}={best_by_speed[0]:.3f} (time={np.mean(best_by_speed[1]['mpc_times']):.3f}ms)")

def compare_scenarios(results: Dict):
    """Analyze performance by scenario (velocity and target)"""
    
    print("\n" + "="*70)
    print("SCENARIO ANALYSIS")
    print("="*70)
    
    # Group by velocity
    vel_groups = {}
    for run_id, data in results.items():
        if 'config' not in data or 'avg_mpc_time_ms' not in data:
            continue
        
        vel = data['config'].get('initial_velocity', {})
        vel_key = f"vf={vel.get('vf', 0):.1f}, vs={vel.get('vs', 0):.1f}, omega={vel.get('omega', 0):.1f}"
        
        if vel_key not in vel_groups:
            vel_groups[vel_key] = []
        
        vel_groups[vel_key].append(data)
    
    if vel_groups:
        print("\n📈 Performance by Initial Velocity:")
        print(f"{'Velocity':<30} {'Count':<8} {'Avg Error (m)':<15} {'Avg MPC (ms)':<15}")
        print("-" * 70)
        
        for vel_key in sorted(vel_groups.keys()):
            runs = vel_groups[vel_key]
            avg_err = np.mean([r['final_position_error_m'] for r in runs])
            avg_mpc = np.mean([r['avg_mpc_time_ms'] for r in runs])
            print(f"{vel_key:<30} {len(runs):<8} {avg_err:<15.6f} {avg_mpc:<15.3f}")
    
    # Group by target
    target_groups = {}
    for run_id, data in results.items():
        if 'config' not in data or 'avg_mpc_time_ms' not in data:
            continue
        
        target = data['config'].get('target_pos', {})
        target_key = f"({target.get('x', 0):.1f}, {target.get('y', 0):.1f}, {target.get('theta', 0):.2f})"
        
        if target_key not in target_groups:
            target_groups[target_key] = []
        
        target_groups[target_key].append(data)
    
    if target_groups:
        print("\n🎯 Performance by Target Position:")
        print(f"{'Target (x, y, theta)':<30} {'Count':<8} {'Avg Error (m)':<15} {'Avg MPC (ms)':<15}")
        print("-" * 70)
        
        for target_key in sorted(target_groups.keys()):
            runs = target_groups[target_key]
            avg_err = np.mean([r['final_position_error_m'] for r in runs])
            avg_mpc = np.mean([r['avg_mpc_time_ms'] for r in runs])
            print(f"{target_key:<30} {len(runs):<8} {avg_err:<15.6f} {avg_mpc:<15.3f}")

def main():
    parser = argparse.ArgumentParser(
        description='Analyze comprehensive batch simulation results',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    
    parser.add_argument('results_file', type=str,
                        help='JSON file with batch results')
    parser.add_argument('--sweep-param', type=str,
                        help='Name of parameter being swept (for sweep analysis)')
    parser.add_argument('--metric', type=str, default='final_position_error_m',
                        help='Metric for best/worst analysis (default: final_position_error_m)')
    parser.add_argument('--top-n', type=int, default=5,
                        help='Number of best/worst to show (default: 5)')
    parser.add_argument('--summary-only', action='store_true',
                        help='Show only summary statistics')
    
    args = parser.parse_args()
    
    # Check file exists
    if not Path(args.results_file).exists():
        print(f"❌ Results file not found: {args.results_file}")
        return 1
    
    # Load results
    print(f"Loading results from: {args.results_file}")
    results = load_results(args.results_file)
    
    # Print summary
    print_summary(results)
    
    if not args.summary_only:
        # Analyze best/worst
        find_best_worst(results, args.metric, args.top_n)
        
        # Analyze sweep if applicable
        analyze_sweep(results, args.sweep_param)
        
        # Compare scenarios
        compare_scenarios(results)
    
    print("\n" + "="*70 + "\n")
    
    return 0

if __name__ == "__main__":
    sys.exit(main())
