#!/usr/bin/env python3
"""
Bayesian Optimization for HumanoidMPC tuning parameters
Uses scikit-optimize (install: pip install scikit-optimize)
"""

import subprocess
import json
import numpy as np
from pathlib import Path
from skopt import gp_minimize
from skopt.space import Real
from skopt.utils import use_named_args
from skopt import dump, load

# Define the search space for MPC weights
space = [
    Real(0.5, 3.0, name='q_pos'),      # Position tracking weight
    Real(0.05, 0.3, name='q_phi'),     # Orientation tracking weight
    Real(4.0, 20.0, name='qf_pos'),    # Terminal position weight
    Real(0.5, 3.0, name='qf_phi'),     # Terminal orientation weight
    Real(0.01, 0.3, name='r_vf'),      # Forward velocity effort
    Real(0.1, 1.0, name='r_vs'),       # Sideways velocity effort
    Real(0.05, 0.5, name='r_omega'),   # Angular velocity effort
    Real(0.05, 1.0, name='s_vf'),      # Forward smoothness
    Real(0.2, 2.0, name='s_vs'),       # Sideways smoothness
    Real(0.1, 1.0, name='s_omega'),    # Angular smoothness
]

# Project paths
script_dir = Path(__file__).parent.absolute()
project_root = script_dir.parent
build_dir = project_root / "build"
demo_executable = build_dir / "demo_humanoid_mpc"

# Track all evaluations
all_results = []

@use_named_args(space)
def objective(**params):
    """
    Objective function to minimize
    Returns: simulation_time_s (lower is better)
    """
    
    # Create config
    config = {
        "start_pos": {"x": 0.0, "y": 3.0, "theta": 0.0},
        "target_pos": {"x": 0.0, "y": -3.0, "theta": 0.0},
        "prediction_horizon": 10,
        "mpc_weights": params
    }
    
    # Write config
    run_id = len(all_results)
    config_file = build_dir / f"bo_config_{run_id}.json"
    with open(config_file, 'w') as f:
        json.dump(config, f, indent=2)
    
    print(f"\n{'='*70}")
    print(f"BO Iteration {run_id + 1}")
    print(f"{'='*70}")
    print(f"Testing weights: {params}")
    
    try:
        # Run simulation
        result = subprocess.run(
            [str(demo_executable), "--config", str(config_file)],
            cwd=build_dir,
            capture_output=True,
            text=True,
            timeout=120
        )
        
        # Read results
        timing_file = build_dir / "mpc_timing_results.json"
        if timing_file.exists():
            with open(timing_file, 'r') as f:
                data = json.load(f)
            
            # Objective: minimize completion time
            objective_value = data['simulation_time_s']
            
            # Add constraints as penalties if needed
            # e.g., penalize if MPC too slow or doesn't reach goal
            if data['final_position_error_m'] > 0.01:
                objective_value += 100.0  # Heavy penalty for not reaching goal
            
            if data['avg_mpc_time_ms'] > 5.0:
                objective_value += data['avg_mpc_time_ms'] * 0.1  # Penalty for slow MPC
            
            print(f"✓ Completion time: {data['simulation_time_s']:.3f} s")
            print(f"  Avg MPC time: {data['avg_mpc_time_ms']:.3f} ms")
            print(f"  Objective value: {objective_value:.3f}")
            
            # Save full results
            data['params'] = params
            data['objective'] = objective_value
            all_results.append(data)
            
            return objective_value
        else:
            print("✗ No timing data generated")
            return 999.0  # Large penalty
            
    except Exception as e:
        print(f"✗ Error: {e}")
        return 999.0

def main():
    print("\n" + "╔" + "="*68 + "╗")
    print("║" + " "*15 + "BAYESIAN OPTIMIZATION START" + " "*26 + "║")
    print("╚" + "="*68 + "╝\n")
    
    print("Search Space:")
    for dim in space:
        print(f"  {dim.name}: [{dim.low}, {dim.high}]")
    
    print("\nObjective: Minimize simulation completion time")
    print("Starting optimization...\n")
    
    # Run Bayesian Optimization
    result = gp_minimize(
        objective,
        space,
        n_calls=30,              # Number of evaluations
        n_random_starts=5,       # Random exploration first
        acq_func='EI',           # Expected Improvement
        random_state=42,
        verbose=True
    )
    
    # Save BO results
    dump(result, 'bo_results.pkl', store_objective=False)
    
    # Save all evaluations to JSON
    with open('bo_all_results.json', 'w') as f:
        json.dump(all_results, f, indent=2)
    
    # Print best results
    print("\n\n" + "╔" + "="*68 + "╗")
    print("║" + " "*20 + "OPTIMIZATION COMPLETE" + " "*27 + "║")
    print("╚" + "="*68 + "╝\n")
    
    print(f"Total evaluations: {len(result.func_vals)}")
    print(f"Best objective value: {result.fun:.3f}")
    print(f"\n🏆 Best Parameters Found:")
    for i, dim in enumerate(space):
        print(f"  {dim.name}: {result.x[i]:.4f}")
    
    # Find best run with full data
    best_run = min(all_results, key=lambda r: r['objective'])
    print(f"\n📊 Best Run Performance:")
    print(f"  Simulation time: {best_run['simulation_time_s']:.3f} s")
    print(f"  Avg MPC time: {best_run['avg_mpc_time_ms']:.3f} ms")
    print(f"  Position error: {best_run['final_position_error_m']:.6f} m")
    
    print(f"\n💾 Results saved to:")
    print(f"  - bo_results.pkl (BO state, can resume)")
    print(f"  - bo_all_results.json (all evaluations)")
    
    # Plot convergence (requires matplotlib)
    try:
        from skopt.plots import plot_convergence, plot_objective
        import matplotlib.pyplot as plt
        
        plot_convergence(result)
        plt.savefig('bo_convergence.png')
        print(f"  - bo_convergence.png (convergence plot)")
        
        # Plot objective function (first 2 dimensions)
        plot_objective(result, dimensions=['q_pos', 'qf_pos'])
        plt.savefig('bo_objective.png')
        print(f"  - bo_objective.png (objective landscape)")
    except ImportError:
        print("  (Install matplotlib for plots: pip install matplotlib)")
    
    print()

if __name__ == "__main__":
    main()
