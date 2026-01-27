#!/usr/bin/env python3
"""
Batch runner for HumanoidMPC simulator
Runs multiple simulations with different MPC tuning parameters
"""

import subprocess
import sys
import json
import argparse
from pathlib import Path
from typing import List, Dict
import time

def run_single_simulation(build_dir: Path, demo_executable: Path, config: Dict, run_id: int) -> Dict:
    """Run a single simulation with given configuration"""
    
    # Write config to file
    config_file = build_dir / f"mpc_config_run_{run_id}.json"
    with open(config_file, 'w') as f:
        json.dump(config, f, indent=2)
    
    print(f"\n{'='*62}")
    print(f"RUN {run_id + 1}")
    print(f"{'='*62}")
    
    if 'mpc_weights' in config:
        print("MPC Weights:")
        for key, val in config['mpc_weights'].items():
            print(f"  {key}: {val}")
    
    # Run the demo
    try:
        result = subprocess.run(
            [str(demo_executable), "--config", str(config_file)],
            cwd=build_dir,
            capture_output=True,
            text=True,
            timeout=120  # 2 minute timeout per run
        )
        
        # Check if timing data was saved
        timing_file = build_dir / "mpc_timing_results.json"
        if timing_file.exists():
            with open(timing_file, 'r') as f:
                data = json.load(f)
            
            print(f"\n✅ Completed:")
            print(f"   Simulation time: {data['simulation_time_s']:.2f} s")
            print(f"   Avg MPC time: {data['avg_mpc_time_ms']:.3f} ms")
            print(f"   MPC calls: {data['mpc_call_count']}")
            
            # Add config info to results
            data['run_id'] = run_id
            data['config'] = config
            
            return data
        else:
            print(f"⚠️  No timing data generated")
            return {
                'run_id': run_id,
                'status': 'no_timing_data',
                'config': config
            }
            
    except subprocess.TimeoutExpired:
        print(f"⚠️  Timeout (>120s)")
        return {
            'run_id': run_id,
            'status': 'timeout',
            'config': config
        }
    except Exception as e:
        print(f"❌ Error: {e}")
        return {
            'run_id': run_id,
            'status': 'error',
            'error': str(e),
            'config': config
        }

def main():
    parser = argparse.ArgumentParser(description='Run HumanoidMPC Simulator in Batch Mode')
    
    # Position arguments
    parser.add_argument('--start-x', type=float, default=0.0, help='Start position X')
    parser.add_argument('--start-y', type=float, default=3.0, help='Start position Y')
    parser.add_argument('--start-theta', type=float, default=0.0, help='Start orientation')
    parser.add_argument('--target-x', type=float, default=0.0, help='Target position X')
    parser.add_argument('--target-y', type=float, default=-3.0, help='Target position Y')
    parser.add_argument('--target-theta', type=float, default=0.0, help='Target orientation')
    parser.add_argument('--horizon', type=int, default=10, help='MPC prediction horizon')
    
    # Batch mode options
    parser.add_argument('--config-file', type=str, help='JSON file with list of MPC weight configurations')
    parser.add_argument('--output', type=str, default='batch_results.json', help='Output file for results')
    
    # Quick sweep options
    parser.add_argument('--sweep-qf-pos', action='store_true', help='Sweep terminal position weight (4, 8, 12, 16)')
    parser.add_argument('--sweep-horizon', action='store_true', help='Sweep prediction horizon (5, 10, 15, 20)')
    parser.add_argument('--sweep-smoothness', action='store_true', help='Sweep smoothness weights (low, med, high)')
    
    args = parser.parse_args()
    
    # Get project paths
    script_dir = Path(__file__).parent.absolute()
    project_root = script_dir.parent
    build_dir = project_root / "build"
    demo_executable = build_dir / "demo_humanoid_mpc"
    
    if not demo_executable.exists():
        print("❌ Demo executable not found. Please build first:")
        print("   cd build && make demo_humanoid_mpc -j4")
        return 1
    
    # Base configuration
    base_config = {
        "start_pos": {
            "x": args.start_x,
            "y": args.start_y,
            "theta": args.start_theta
        },
        "target_pos": {
            "x": args.target_x,
            "y": args.target_y,
            "theta": args.target_theta
        },
        "prediction_horizon": args.horizon
    }
    
    # Generate configurations
    configs = []
    
    if args.config_file:
        # Load from file
        with open(args.config_file, 'r') as f:
            weight_configs = json.load(f)
        for weights in weight_configs:
            config = base_config.copy()
            config['mpc_weights'] = weights
            configs.append(config)
    
    elif args.sweep_qf_pos:
        # Sweep terminal position weight
        for qf_pos in [4.0, 8.0, 12.0, 16.0]:
            config = base_config.copy()
            config['mpc_weights'] = {'qf_pos': qf_pos}
            configs.append(config)
    
    elif args.sweep_horizon:
        # Sweep prediction horizon
        for horizon in [5, 10, 15, 20]:
            config = base_config.copy()
            config['prediction_horizon'] = horizon
            configs.append(config)
    
    elif args.sweep_smoothness:
        # Sweep smoothness parameters
        smoothness_configs = [
            {'s_vf': 0.1, 's_vs': 0.4, 's_omega': 0.2, 'name': 'low_smoothness'},
            {'s_vf': 0.2, 's_vs': 0.8, 's_omega': 0.4, 'name': 'default_smoothness'},
            {'s_vf': 0.5, 's_vs': 1.5, 's_omega': 0.8, 'name': 'high_smoothness'}
        ]
        for smooth_config in smoothness_configs:
            config = base_config.copy()
            name = smooth_config.pop('name')
            config['mpc_weights'] = smooth_config
            config['name'] = name
            configs.append(config)
    
    else:
        print("❌ No batch configuration specified!")
        print("   Use --config-file, --sweep-qf-pos, --sweep-horizon, or --sweep-smoothness")
        return 1
    
    # Run batch
    print("\n" + "╔" + "="*60 + "╗")
    print("║" + " "*15 + "BATCH SIMULATION RUN" + " "*25 + "║")
    print("╚" + "="*60 + "╝")
    print(f"\nTotal runs: {len(configs)}")
    print(f"Start: ({args.start_x}, {args.start_y}, {args.start_theta})")
    print(f"Target: ({args.target_x}, {args.target_y}, {args.target_theta})")
    
    results = []
    start_time = time.time()
    
    for i, config in enumerate(configs):
        result = run_single_simulation(build_dir, demo_executable, config, i)
        results.append(result)
    
    total_time = time.time() - start_time
    
    # Save results
    output_file = Path(args.output)
    with open(output_file, 'w') as f:
        json.dump(results, f, indent=2)
    
    # Print summary
    print("\n\n" + "╔" + "="*60 + "╗")
    print("║" + " "*20 + "BATCH SUMMARY" + " "*27 + "║")
    print("╚" + "="*60 + "╝\n")
    
    successful_runs = [r for r in results if 'simulation_time_s' in r]
    
    print(f"Total runs: {len(results)}")
    print(f"Successful: {len(successful_runs)}")
    print(f"Failed: {len(results) - len(successful_runs)}")
    print(f"Total time: {total_time:.1f} s")
    
    if successful_runs:
        print(f"\n📊 Performance Statistics:")
        avg_times = [r['avg_mpc_time_ms'] for r in successful_runs]
        sim_times = [r['simulation_time_s'] for r in successful_runs]
        
        print(f"   Avg MPC time: {sum(avg_times)/len(avg_times):.3f} ms (range: {min(avg_times):.3f} - {max(avg_times):.3f})")
        print(f"   Sim completion: {sum(sim_times)/len(sim_times):.2f} s (range: {min(sim_times):.2f} - {max(sim_times):.2f})")
        
        # Find best configuration
        best_run = min(successful_runs, key=lambda r: r['simulation_time_s'])
        print(f"\n🏆 Fastest completion: Run {best_run['run_id'] + 1} ({best_run['simulation_time_s']:.2f} s)")
        if 'mpc_weights' in best_run['config']:
            print(f"   Weights: {best_run['config']['mpc_weights']}")
    
    print(f"\n💾 Full results saved to: {output_file.absolute()}")
    print()
    
    return 0

if __name__ == "__main__":
    sys.exit(main())
