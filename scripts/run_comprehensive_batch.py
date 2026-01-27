#!/usr/bin/env python3
"""
Comprehensive Batch Runner for HumanoidMPC simulator
Runs simulations across different initial conditions, targets, and MPC parameter sweeps
"""

import subprocess
import sys
import json
import argparse
from pathlib import Path
from typing import List, Dict, Tuple
import time
import math
from itertools import product

def run_single_simulation(
    build_dir: Path, 
    demo_executable: Path, 
    config: Dict, 
    run_id: str,
    verbose: bool = False
) -> Dict:
    """Run a single simulation with given configuration"""
    
    # Write config to file
    config_file = build_dir / f"mpc_config_{run_id}.json"
    with open(config_file, 'w') as f:
        json.dump(config, f, indent=2)
    
    if verbose:
        print(f"\n{'='*70}")
        print(f"RUN: {run_id}")
        print(f"{'='*70}")
        print(f"Start: ({config['start_pos']['x']:.1f}, {config['start_pos']['y']:.1f}, {config['start_pos']['theta']:.2f})")
        print(f"Target: ({config['target_pos']['x']:.1f}, {config['target_pos']['y']:.1f}, {config['target_pos']['theta']:.2f})")
        if 'initial_velocity' in config:
            vel = config['initial_velocity']
            print(f"Init vel: (vf={vel['vf']:.2f}, vs={vel['vs']:.2f}, omega={vel['omega']:.2f})")
        if 'mpc_weights' in config:
            print(f"MPC weights: {config['mpc_weights']}")
    else:
        print(f"Running {run_id}...", end=' ')
    
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
            
            if verbose:
                print(f"\n✅ Completed:")
                print(f"   Simulation time: {data['simulation_time_s']:.2f} s")
                print(f"   Avg MPC time: {data['avg_mpc_time_ms']:.3f} ms")
                print(f"   MPC calls: {data['mpc_call_count']}")
                print(f"   Position error: {data['final_position_error_m']:.6f} m")
            else:
                print(f"✅ ({data['simulation_time_s']:.2f}s, {data['avg_mpc_time_ms']:.3f}ms avg)")
            
            # Add config info to results
            data['run_id'] = run_id
            data['config'] = config
            
            return data
        else:
            if verbose:
                print(f"⚠️  No timing data generated")
            else:
                print("⚠️  No data")
            return {
                'run_id': run_id,
                'status': 'no_timing_data',
                'config': config
            }
            
    except subprocess.TimeoutExpired:
        print(f"⚠️  Timeout" if not verbose else f"⚠️  Timeout (>120s)")
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

def generate_simulation_configs(
    initial_velocities: List[Tuple[float, float, float]],
    target_positions: List[Tuple[float, float, float]],
    mpc_weight_sweeps: Dict[str, List[float]],
    base_mpc_weights: Dict[str, float],
    horizon: int = 10
) -> List[Tuple[str, Dict]]:
    """
    Generate all simulation configurations
    Returns list of (run_id, config) tuples
    """
    configs = []
    run_counter = 0
    
    # If no weight sweeps, just run with base weights across all scenarios
    if not mpc_weight_sweeps:
        mpc_weight_sweeps = {'none': [0]}  # Dummy sweep
    
    # Iterate over each MPC weight parameter being swept
    for weight_param, weight_values in mpc_weight_sweeps.items():
        for weight_value in weight_values:
            # For each weight value, run all combinations of velocities and targets
            for vel_idx, (vf, vs, omega) in enumerate(initial_velocities):
                for target_idx, (tx, ty, ttheta) in enumerate(target_positions):
                    
                    # Create config
                    config = {
                        "start_pos": {
                            "x": 0.0,
                            "y": 0.0,
                            "theta": 0.0
                        },
                        "target_pos": {
                            "x": tx,
                            "y": ty,
                            "theta": ttheta
                        },
                        "initial_velocity": {
                            "vf": vf,
                            "vs": vs,
                            "omega": omega
                        },
                        "prediction_horizon": horizon
                    }
                    
                    # Add MPC weights
                    mpc_weights = base_mpc_weights.copy()
                    if weight_param != 'none':
                        mpc_weights[weight_param] = weight_value
                    
                    if mpc_weights:
                        config["mpc_weights"] = mpc_weights
                    
                    # Generate run ID
                    if weight_param != 'none':
                        run_id = f"{weight_param}_{weight_value:.3f}_vel{vel_idx}_target{target_idx}"
                    else:
                        run_id = f"baseline_vel{vel_idx}_target{target_idx}"
                    
                    configs.append((run_id, config))
                    run_counter += 1
    
    return configs

def main():
    parser = argparse.ArgumentParser(
        description='Comprehensive Batch Runner for HumanoidMPC Simulator',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Run all baseline scenarios (no weight sweeps)
  ./run_comprehensive_batch.py --output results_baseline.json
  
  # Sweep qf_pos weight from 4 to 16 in 4 steps
  ./run_comprehensive_batch.py --sweep-param qf_pos --sweep-range 4 16 --sweep-steps 4
  
  # Sweep multiple parameters
  ./run_comprehensive_batch.py --sweep-param q_pos --sweep-range 0.5 2.0 --sweep-steps 4
  ./run_comprehensive_batch.py --sweep-param r_omega --sweep-range 0.1 0.5 --sweep-steps 5
        """
    )
    
    # Simulation parameters
    parser.add_argument('--horizon', type=int, default=10, 
                        help='MPC prediction horizon (default: 10)')
    
    # MPC weights configuration
    parser.add_argument('--weights-file', type=str, default='mpc_weights.json',
                        help='JSON file with MPC weights (default: mpc_weights.json)')
    
    # Parameter sweep options
    parser.add_argument('--sweep-param', type=str, 
                        choices=['q_pos', 'q_phi', 'qf_pos', 'qf_phi', 
                                'r_vf', 'r_vs', 'r_omega',
                                's_vf', 's_vs', 's_omega'],
                        help='MPC weight parameter to sweep')
    parser.add_argument('--sweep-range', type=float, nargs=2, metavar=('MIN', 'MAX'),
                        help='Range for parameter sweep [min, max]')
    parser.add_argument('--sweep-steps', type=int, default=5,
                        help='Number of steps in parameter sweep (default: 5)')
    
    # Base MPC weights (override values from weights file)
    parser.add_argument('--q-pos', type=float, help='Override q_pos weight from file')
    parser.add_argument('--q-phi', type=float, help='Override q_phi weight from file')
    parser.add_argument('--qf-pos', type=float, help='Override qf_pos weight from file')
    parser.add_argument('--qf-phi', type=float, help='Override qf_phi weight from file')
    parser.add_argument('--r-vf', type=float, help='Override r_vf weight from file')
    parser.add_argument('--r-vs', type=float, help='Override r_vs weight from file')
    parser.add_argument('--r-omega', type=float, help='Override r_omega weight from file')
    parser.add_argument('--s-vf', type=float, help='Override s_vf weight from file')
    parser.add_argument('--s-vs', type=float, help='Override s_vs weight from file')
    parser.add_argument('--s-omega', type=float, help='Override s_omega weight from file')
    
    # Output options
    parser.add_argument('--output', type=str, default='comprehensive_batch_results.json',
                        help='Output JSON file for all results (default: comprehensive_batch_results.json)')
    parser.add_argument('--verbose', action='store_true',
                        help='Verbose output for each simulation')
    
    args = parser.parse_args()
    
    # Get project paths
    script_dir = Path(__file__).parent.absolute()
    project_root = script_dir.parent
    build_dir = project_root / "build"
    
    # Check if build directory and executable exist
    if not build_dir.exists():
        print("❌ Build directory not found. Please build the project first:")
        print("   cd build && cmake .. && make -j4")
        return 1
    
    demo_executable = build_dir / "demo_humanoid_mpc"
    if not demo_executable.exists():
        print("❌ Demo executable not found. Please build the project first:")
        print("   cd build && make demo_humanoid_mpc -j4")
        return 1
    
    # Load MPC weights from file
    weights_file = Path(args.weights_file)
    if not weights_file.is_absolute():
        weights_file = project_root / args.weights_file
    
    # Default weights (fallback)
    default_weights = {
        'q_pos': 1.0,
        'q_phi': 0.1,
        'qf_pos': 8.0,
        'qf_phi': 1.0,
        'r_vf': 0.1,
        'r_vs': 0.5,
        'r_omega': 0.2,
        's_vf': 0.2,
        's_vs': 0.8,
        's_omega': 0.4,
    }
    
    if weights_file.exists():
        print(f"📂 Loading MPC weights from: {weights_file}")
        with open(weights_file, 'r') as f:
            file_weights = json.load(f)
        base_mpc_weights = default_weights.copy()
        base_mpc_weights.update(file_weights)
    else:
        print(f"⚠️  Weights file not found: {weights_file}")
        print(f"   Using default weights")
        base_mpc_weights = default_weights.copy()
    
    # Apply command-line overrides
    if args.q_pos is not None: base_mpc_weights['q_pos'] = args.q_pos
    if args.q_phi is not None: base_mpc_weights['q_phi'] = args.q_phi
    if args.qf_pos is not None: base_mpc_weights['qf_pos'] = args.qf_pos
    if args.qf_phi is not None: base_mpc_weights['qf_phi'] = args.qf_phi
    if args.r_vf is not None: base_mpc_weights['r_vf'] = args.r_vf
    if args.r_vs is not None: base_mpc_weights['r_vs'] = args.r_vs
    if args.r_omega is not None: base_mpc_weights['r_omega'] = args.r_omega
    if args.s_vf is not None: base_mpc_weights['s_vf'] = args.s_vf
    if args.s_vs is not None: base_mpc_weights['s_vs'] = args.s_vs
    if args.s_omega is not None: base_mpc_weights['s_omega'] = args.s_omega
    
    # Define initial velocities (vf, vs, omega)
    # 0 m/s, 1 m/s forward (2 instances), 1 rad/s rotate (2 instances)
    initial_velocities = [
        (0.0, 0.0, 0.0),      # Stationary
        (1.0, 0.0, 0.0),      # 1 m/s forward
        (1.0, 0.0, 1.0),      # 1 m/s forward + rotate
        (0.0, 0.0, 1.0),      # 1 rad/s rotate
        (0.5, -0.4, 0.0),     # Diagonal forward-left
    ]
    
    # Define target positions (x, y, theta)
    target_positions = [
        (1.0, 0.0, 0.0),
        (4.0, 4.0, 0.0),
        (-4.0, 0.0, 0.0),
        (-2.0, 0.0, math.pi/2),      
        (0.0, -5.0, -math.pi),
        (0.0, 5.0, math.pi/2),
        (-4.0, 6.0, 0.0),
        (4.0, -6.0, -math.pi/2),
    ]
    
    # Generate parameter sweep values
    mpc_weight_sweeps = {}
    if args.sweep_param and args.sweep_range:
        min_val, max_val = args.sweep_range
        sweep_values = [min_val + (max_val - min_val) * i / (args.sweep_steps - 1) 
                       for i in range(args.sweep_steps)]
        mpc_weight_sweeps[args.sweep_param] = sweep_values
    
    # Generate all simulation configurations
    configs = generate_simulation_configs(
        initial_velocities=initial_velocities,
        target_positions=target_positions,
        mpc_weight_sweeps=mpc_weight_sweeps,
        base_mpc_weights=base_mpc_weights,
        horizon=args.horizon
    )
    
    # Print summary
    print("╔════════════════════════════════════════════════════════════════════╗")
    print("║      Comprehensive HumanoidMPC Batch Simulation Runner            ║")
    print("╚════════════════════════════════════════════════════════════════════╝")
    print()
    print(f"📊 Simulation Plan:")
    print(f"   Initial velocities: {len(initial_velocities)} configurations")
    print(f"   Target positions: {len(target_positions)} positions")
    print(f"   Base scenarios: {len(initial_velocities) * len(target_positions)}")
    
    if args.sweep_param:
        print(f"\n⚙️  Parameter Sweep:")
        print(f"   Parameter: {args.sweep_param}")
        print(f"   Range: [{args.sweep_range[0]:.3f}, {args.sweep_range[1]:.3f}]")
        print(f"   Steps: {args.sweep_steps}")
        print(f"   Values: {mpc_weight_sweeps[args.sweep_param]}")
        print(f"   Total simulations: {len(configs)}")
    else:
        print(f"\n   Mode: Baseline (no parameter sweep)")
        print(f"   Total simulations: {len(configs)}")
    
    print(f"\n📁 Output file: {args.output}")
    print(f"🔭 MPC Horizon: {args.horizon}")
    print()
    
    # Confirm before starting
    try:
        response = input("Press Enter to start batch simulation (Ctrl+C to cancel)... ")
    except KeyboardInterrupt:
        print("\n\nCancelled by user")
        return 0
    
    # Run all simulations
    all_results = {}
    start_time = time.time()
    
    print(f"\n{'='*70}")
    print(f"Starting {len(configs)} simulations...")
    print(f"{'='*70}\n")
    
    for idx, (run_id, config) in enumerate(configs):
        print(f"[{idx+1}/{len(configs)}] ", end='')
        result = run_single_simulation(build_dir, demo_executable, config, run_id, args.verbose)
        all_results[run_id] = result
    
    end_time = time.time()
    total_time = end_time - start_time
    
    # Save all results to output file
    output_path = Path(args.output)
    if not output_path.is_absolute():
        output_path = project_root / output_path
    
    with open(output_path, 'w') as f:
        json.dump(all_results, f, indent=2)
    
    # Print summary statistics
    print(f"\n{'='*70}")
    print("📊 BATCH SIMULATION COMPLETE")
    print(f"{'='*70}")
    print(f"\n⏱️  Total time: {total_time:.1f} seconds ({total_time/60:.1f} minutes)")
    print(f"   Average per simulation: {total_time/len(configs):.2f} seconds")
    
    # Count successful runs
    successful = sum(1 for r in all_results.values() if 'simulation_time_s' in r)
    failed = len(configs) - successful
    
    print(f"\n✅ Results:")
    print(f"   Successful: {successful}/{len(configs)}")
    if failed > 0:
        print(f"   Failed/Timeout: {failed}")
    
    # Calculate aggregate statistics for successful runs
    if successful > 0:
        sim_times = [r['simulation_time_s'] for r in all_results.values() if 'simulation_time_s' in r]
        avg_mpc_times = [r['avg_mpc_time_ms'] for r in all_results.values() if 'avg_mpc_time_ms' in r]
        position_errors = [r['final_position_error_m'] for r in all_results.values() if 'final_position_error_m' in r]
        
        print(f"\n📈 Aggregate Statistics:")
        print(f"   Simulation time: {min(sim_times):.2f}s - {max(sim_times):.2f}s (avg: {sum(sim_times)/len(sim_times):.2f}s)")
        print(f"   Avg MPC time: {min(avg_mpc_times):.3f}ms - {max(avg_mpc_times):.3f}ms (avg: {sum(avg_mpc_times)/len(avg_mpc_times):.3f}ms)")
        print(f"   Position error: {min(position_errors):.6f}m - {max(position_errors):.6f}m (avg: {sum(position_errors)/len(position_errors):.6f}m)")
    
    print(f"\n💾 Full results saved to: {output_path}")
    print(f"{'='*70}\n")
    
    # If sweeping, show best/worst performers
    if args.sweep_param and successful > 1:
        print(f"🏆 Parameter Sweep Analysis ({args.sweep_param}):")
        sweep_results = []
        for run_id, result in all_results.items():
            if 'avg_mpc_time_ms' in result and args.sweep_param in run_id:
                param_value = float(run_id.split('_')[1])
                sweep_results.append((param_value, result['avg_mpc_time_ms'], result['final_position_error_m']))
        
        if sweep_results:
            sweep_results.sort(key=lambda x: x[0])
            print(f"\n   {'Value':<10} {'Avg MPC (ms)':<15} {'Pos Error (m)':<15}")
            print(f"   {'-'*40}")
            for val, mpc_time, pos_err in sweep_results[:5]:  # Show first 5
                print(f"   {val:<10.3f} {mpc_time:<15.3f} {pos_err:<15.6f}")
        
    return 0

if __name__ == "__main__":
    sys.exit(main())
