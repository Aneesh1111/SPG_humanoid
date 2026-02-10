#!/usr/bin/env python3
"""
Bayesian Optimization for Integrated Obstacle Avoidance MPC Weights

This script tunes the weights for HumanoidObstacleAvoidanceMPC using Bayesian optimization.
Unlike run_bas_BO.py which tunes the full SPG pipeline, this focuses on the obstacle
avoidance MPC in isolation.

The weights being tuned:
- Q_pos: Position tracking cost
- Q_theta: Orientation tracking cost
- Q_vel: Velocity regularization
- R_accel: Control (acceleration) cost
- Q_term_pos: Terminal position cost
- Q_term_theta: Terminal orientation cost
- Q_term_vel: Terminal velocity cost
- collision_weight: Obstacle avoidance penalty

Note: These weights are baked into the acados solver at generation time,
so each iteration requires regenerating and recompiling the solver.
"""

import numpy as np
import subprocess
import sys
import json
import argparse
from pathlib import Path
from typing import List, Dict, Tuple
import time
import math
from bayes_opt import BayesianOptimization

# ============================================================================
# Weight Specification for Obstacle Avoidance MPC
# ============================================================================

OBSTACLE_MPC_WEIGHT_SPEC = {
    "Q_pos": {"init": 15.0, "min": 1.0, "max": 100.0},
    "Q_theta": {"init": 2.0, "min": 0.1, "max": 20.0},
    "Q_vel": {"init": 0.5, "min": 0.01, "max": 5.0},
    "R_accel": {"init": 0.1, "min": 0.01, "max": 2.0},
    "Q_term_pos": {"init": 100.0, "min": 10.0, "max": 500.0},
    "Q_term_theta": {"init": 10.0, "min": 1.0, "max": 100.0},
    "Q_term_vel": {"init": 5.0, "min": 0.5, "max": 50.0},
    "collision_weight": {"init": 1000.0, "min": 100.0, "max": 5000.0},
}

# ============================================================================
# Scenario Configuration
# ============================================================================

def generate_obstacle_scenarios(
    initial_velocities: List[Tuple[float, float, float]],
    target_positions: List[Tuple[float, float, float]],
    start_pos: Tuple[float, float, float] = (0.0, -3.0, 0.0),
) -> List[Tuple[str, Dict]]:
    """
    Generate obstacle avoidance scenarios.
    
    Each scenario has:
    - Starting position and velocity
    - Target position
    - Fixed set of obstacles (defined per scenario or globally)
    """
    sx, sy, stheta = start_pos
    configs = []
    
    for vel_idx, (vx, vy, omega) in enumerate(initial_velocities):
        for target_idx, (tx, ty, ttheta) in enumerate(target_positions):
            config = {
                "start_pos": {"x": float(sx), "y": float(sy), "theta": float(stheta)},
                "start_vel": {"vx": float(vx), "vy": float(vy), "omega": float(omega)},
                "target_pos": {"x": float(tx), "y": float(ty), "theta": float(ttheta)},
                # Default obstacles (corridor scenario)
                "obstacles": [
                    {"position": {"x": -1.0, "y": 0.0}, "radius": 0.3, "velocity": {"x": 0.5, "y": 0.0}},
                    {"position": {"x": 1.0, "y": 0.0}, "radius": 0.3, "velocity": {"x": -0.5, "y": 0.0}},
                    {"position": {"x": 0.0, "y": 1.5}, "radius": 0.25, "velocity": {"x": 0.0, "y": 0.3}},
                ],
                "dt": 0.15,
                "horizon": 20,
                "robot_radius": 0.3,
                "obstacle_margin": 0.2,
            }
            run_id = f"vel{vel_idx}_target{target_idx}"
            configs.append((run_id, config))
    
    return configs

def with_mpc_weights(scenario_config: Dict, **mpc_weights) -> Dict:
    """Add MPC weights to scenario config"""
    config = scenario_config.copy()
    config["mpc_weights"] = mpc_weights
    return config

# ============================================================================
# Acados Solver Generation
# ============================================================================

def regenerate_obstacle_mpc_solver_with_weights(
    script_path: Path,
    weights: Dict[str, float],
    project_root: Path
) -> bool:
    """
    Regenerate the obstacle avoidance MPC solver with new weights.
    
    This modifies the generation script temporarily, runs it, then restores it.
    """
    print(f"\n🔄 Regenerating obstacle MPC solver with weights:")
    for k, v in weights.items():
        print(f"   {k}: {v:.4f}")
    
    # Read the generation script
    try:
        with open(script_path, 'r') as f:
            script_content = f.read()
        
        # Backup original script
        backup_path = script_path.with_suffix('.py.backup')
        with open(backup_path, 'w') as f:
            f.write(script_content)
        
        # Replace weight values in the script
        # The generation script has lines like:
        #   Q[0, 0] = Q[1, 1] = 15.0  # Q_pos
        #   Q[2, 2] = 2.0             # Q_theta
        # etc.
        
        modified_content = script_content
        
        # Replace Q_pos (diagonal elements 0, 1)
        if "Q_pos" in weights:
            modified_content = modified_content.replace(
                "Q[0, 0] = Q[1, 1] = 15.0",
                f"Q[0, 0] = Q[1, 1] = {weights['Q_pos']}"
            )
        
        # Replace Q_theta
        if "Q_theta" in weights:
            modified_content = modified_content.replace(
                "Q[2, 2] = 2.0",
                f"Q[2, 2] = {weights['Q_theta']}"
            )
        
        # Replace Q_vel
        if "Q_vel" in weights:
            modified_content = modified_content.replace(
                "Q[3, 3] = Q[4, 4] = Q[5, 5] = 0.5",
                f"Q[3, 3] = Q[4, 4] = Q[5, 5] = {weights['Q_vel']}"
            )
        
        # Replace R_accel
        if "R_accel" in weights:
            modified_content = modified_content.replace(
                "R[0, 0] = R[1, 1] = R[2, 2] = 0.1",
                f"R[0, 0] = R[1, 1] = R[2, 2] = {weights['R_accel']}"
            )
        
        # Replace Q_term_pos
        if "Q_term_pos" in weights:
            modified_content = modified_content.replace(
                "Qe[0, 0] = Qe[1, 1] = 100.0",
                f"Qe[0, 0] = Qe[1, 1] = {weights['Q_term_pos']}"
            )
        
        # Replace Q_term_theta
        if "Q_term_theta" in weights:
            modified_content = modified_content.replace(
                "Qe[2, 2] = 10.0",
                f"Qe[2, 2] = {weights['Q_term_theta']}"
            )
        
        # Replace Q_term_vel
        if "Q_term_vel" in weights:
            modified_content = modified_content.replace(
                "Qe[3, 3] = Qe[4, 4] = Qe[5, 5] = 5.0",
                f"Qe[3, 3] = Qe[4, 4] = Qe[5, 5] = {weights['Q_term_vel']}"
            )
        
        # Write modified script
        with open(script_path, 'w') as f:
            f.write(modified_content)
        
        # Run the generation script
        result = subprocess.run(
            [sys.executable, str(script_path)],
            cwd=project_root,
            capture_output=True,
            text=True,
            timeout=60
        )
        
        # Restore original script
        with open(backup_path, 'r') as f:
            original = f.read()
        with open(script_path, 'w') as f:
            f.write(original)
        backup_path.unlink()
        
        if result.returncode != 0:
            print(f"❌ Solver generation failed: {result.stderr}")
            return False
        
        print("✅ Obstacle MPC solver generated")
        return True
        
    except Exception as e:
        print(f"❌ Error regenerating solver: {e}")
        # Try to restore backup
        if backup_path.exists():
            with open(backup_path, 'r') as f:
                original = f.read()
            with open(script_path, 'w') as f:
                f.write(original)
            backup_path.unlink()
        return False

# ============================================================================
# Compilation
# ============================================================================

def recompile_project(build_dir: Path) -> bool:
    """Recompile the C++ project after solver regeneration."""
    try:
        print("🔨 Recompiling project...")
        result = subprocess.run(
            ["make", "-j4"],
            cwd=build_dir,
            capture_output=True,
            text=True,
            timeout=300
        )
        if result.returncode != 0:
            print(f"❌ Compilation failed: {result.stderr}")
            return False
        print("✅ Compilation successful")
        return True
    except Exception as e:
        print(f"❌ Compilation error: {e}")
        return False

# ============================================================================
# Simulation Runner
# ============================================================================

def run_single_obstacle_simulation(
    build_dir: Path,
    demo_executable: Path,
    config: Dict,
    run_id: str,
    verbose: bool = False
) -> Dict:
    """Run a single obstacle avoidance simulation"""
    
    config_file = build_dir / f"obstacle_mpc_config_{run_id}.json"
    with open(config_file, 'w') as f:
        json.dump(config, f, indent=2)
    
    if verbose:
        print(f"\n{'='*70}")
        print(f"RUN: {run_id}")
        print(f"{'='*70}")
        print(f"Start: ({config['start_pos']['x']:.1f}, {config['start_pos']['y']:.1f}, {config['start_pos']['theta']:.2f})")
        print(f"Target: ({config['target_pos']['x']:.1f}, {config['target_pos']['y']:.1f}, {config['target_pos']['theta']:.2f})")
        print(f"Obstacles: {len(config['obstacles'])}")
    else:
        print(f"Running {run_id}...", end=' ')
    
    try:
        timing_file = build_dir / "mpc_obstacle_timing_results.json"
        if timing_file.exists():
            timing_file.unlink()
        
        result = subprocess.run(
            [str(demo_executable), "--config", str(config_file), "--headless", "--max-steps", "1000"],
            cwd=build_dir,
            capture_output=True,
            text=True,
            timeout=120
        )
        
        if timing_file.exists():
            with open(timing_file, 'r') as f:
                data = json.load(f)
            
            if verbose:
                print(f"\n✅ Completed:")
                print(f"   Simulation time: {data['simulation_time_s']:.2f} s")
                print(f"   Avg MPC time: {data['avg_mpc_time_ms']:.3f} ms")
                print(f"   Position error: {data['final_position_error_m']:.6f} m")
            else:
                print(f"✅ ({data['simulation_time_s']:.2f}s, {data['avg_mpc_time_ms']:.3f}ms)")
            
            data['run_id'] = run_id
            data['config'] = config
            return data
        else:
            if verbose:
                print("⚠️  No timing data")
            else:
                print("⚠️  No data")
            return {
                'run_id': run_id,
                'status': 'no_timing_data',
                'config': config
            }
    
    except subprocess.TimeoutExpired:
        print("❌ Timeout")
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

# ============================================================================
# Objective Function
# ============================================================================

def compute_objective_from_results(results: List[Dict]) -> float:
    """
    Compute BO objective from simulation results.
    
    Maximize: -simulation_time (want fast convergence)
    Penalize: high position error, timeouts
    """
    total_score = 0.0
    valid_runs = 0
    
    for res in results:
        if res.get('status') in ['timeout', 'error', 'no_timing_data']:
            # Heavy penalty
            total_score -= 100.0
            continue
        
        sim_time = res.get('simulation_time_s', 100.0)
        pos_error = res.get('final_position_error_m', 1.0)
        
        # Objective: minimize time + penalize error
        # We want negative score (lower is better for minimization)
        # But BayesOpt maximizes, so we negate
        score = -sim_time - 10.0 * pos_error
        
        # Bonus for very accurate results
        if pos_error < 0.01:
            score += 1.0
        
        total_score += score
        valid_runs += 1
    
    if valid_runs == 0:
        return -1000.0
    
    return total_score / len(results)

# ============================================================================
# Bayesian Optimization
# ============================================================================

def run_obstacle_mpc_bo(
    project_root: Path,
    build_dir: Path,
    generator_script: Path,
    demo_executable: Path,
    scenarios: List[Tuple[str, Dict]],
    weight_spec: Dict,
    opt_var: List[str],
    n_init: int = 5,
    n_iter: int = 20,
    output_file: Path = None
):
    """
    Run Bayesian Optimization for obstacle MPC weights.
    
    Each iteration:
    1. Select new weights to test
    2. Regenerate acados solver with those weights
    3. Recompile project
    4. Run all scenarios
    5. Compute aggregate objective
    """
    
    # Get bounds for BO
    _, pbounds = spec_to_init_and_bounds(weight_spec, opt_var)
    
    print(f"\n{'='*70}")
    print("OBSTACLE MPC BAYESIAN OPTIMIZATION")
    print(f"{'='*70}")
    print(f"Optimizing: {opt_var}")
    print(f"Scenarios: {len(scenarios)}")
    print(f"Initial samples: {n_init}")
    print(f"BO iterations: {n_iter}")
    print(f"{'='*70}\n")
    
    all_results = []
    iteration_count = [0]  # Mutable counter for closure
    
    def objective_function(**params):
        """Objective function for BO"""
        iteration_count[0] += 1
        print(f"\n{'#'*70}")
        print(f"# ITERATION {iteration_count[0]}/{n_init + n_iter}")
        print(f"{'#'*70}")
        
        # Get complete weight dict (defaults + optimized params)
        default_weights = {k: v["init"] for k, v in weight_spec.items()}
        weights = merge_optimized_into_defaults(default_weights, params)
        
        # Regenerate solver
        if not regenerate_obstacle_mpc_solver_with_weights(generator_script, weights, project_root):
            return -1000.0
        
        # Recompile
        if not recompile_project(build_dir):
            return -1000.0
        
        # Run all scenarios
        scenario_results = []
        for run_id, scenario_config in scenarios:
            config_with_weights = with_mpc_weights(scenario_config, **weights)
            result = run_single_obstacle_simulation(
                build_dir, demo_executable, config_with_weights, run_id, verbose=False
            )
            scenario_results.append(result)
        
        # Compute objective
        objective = compute_objective_from_results(scenario_results)
        
        print(f"\n📊 Iteration {iteration_count[0]} Summary:")
        print(f"   Objective: {objective:.3f}")
        print(f"   Weights: {params}")
        
        # Store results
        iter_data = {
            'iteration': iteration_count[0],
            'weights': weights,
            'objective': objective,
            'scenario_results': scenario_results
        }
        all_results.append(iter_data)
        
        return objective
    
    # Initialize BO
    optimizer = BayesianOptimization(
        f=objective_function,
        pbounds=pbounds,
        random_state=42,
        verbose=2
    )
    
    # Run optimization
    optimizer.maximize(init_points=n_init, n_iter=n_iter)
    
    # Print results
    print(f"\n{'='*70}")
    print("OPTIMIZATION COMPLETE")
    print(f"{'='*70}")
    print(f"Best objective: {optimizer.max['target']:.3f}")
    print(f"Best weights:")
    for k, v in optimizer.max['params'].items():
        print(f"   {k}: {v:.4f}")
    
    # Save results
    if output_file:
        output_data = {
            'best_objective': optimizer.max['target'],
            'best_params': optimizer.max['params'],
            'all_iterations': all_results,
            'weight_spec': weight_spec,
            'optimized_variables': opt_var
        }
        with open(output_file, 'w') as f:
            json.dump(output_data, f, indent=2)
        print(f"\n💾 Results saved to: {output_file}")
    
    return optimizer

def spec_to_init_and_bounds(spec: Dict, opt_var: List[str] = None) -> Tuple[Dict, Dict]:
    """Extract init and bounds for BO"""
    if opt_var is None:
        keys = list(spec.keys())
    else:
        keys = opt_var
    
    init = {k: float(spec[k]["init"]) for k in keys}
    pbounds = {k: (float(spec[k]["min"]), float(spec[k]["max"])) for k in keys}
    return init, pbounds

def merge_optimized_into_defaults(defaults: Dict, optimized: Dict) -> Dict:
    """Merge optimized params into defaults"""
    result = defaults.copy()
    result.update(optimized)
    return result

# ============================================================================
# Main
# ============================================================================

def main():
    parser = argparse.ArgumentParser(description="Bayesian Optimization for Obstacle MPC")
    parser.add_argument("--init", type=int, default=5, help="Initial random samples")
    parser.add_argument("--iter", type=int, default=20, help="BO iterations")
    parser.add_argument("--output", type=str, default="obstacle_mpc_bo_results.json", help="Output file")
    parser.add_argument("--weights", nargs='+', help="Weights to optimize (default: all)")
    args = parser.parse_args()
    
    # Setup paths
    project_root = Path(__file__).parent.parent
    build_dir = project_root / "build"
    generator_script = project_root / "scripts" / "generate_acados_obstacle_mpc.py"
    demo_executable = build_dir / "demo_obstacle_mpc"
    
    if not generator_script.exists():
        print(f"❌ Generator script not found: {generator_script}")
        return 1
    
    if not demo_executable.exists():
        print(f"❌ Demo executable not found: {demo_executable}")
        print("Build it first: cd build && make demo_obstacle_mpc")
        return 1
    
    # Define scenarios
    initial_velocities = [
        (0.0, 0.0, 0.0),  # Starting from rest
        (0.5, 0.0, 0.0),  # Moving forward
    ]
    
    target_positions = [
        (0.0, 3.0, 0.0),    # Straight ahead
        (2.0, 3.0, 0.0),    # Diagonal right
        (-2.0, 3.0, 0.0),   # Diagonal left
    ]
    
    scenarios = generate_obstacle_scenarios(initial_velocities, target_positions)
    
    # Determine which weights to optimize
    opt_var = args.weights if args.weights else list(OBSTACLE_MPC_WEIGHT_SPEC.keys())
    
    # Run BO
    output_file = project_root / args.output
    run_obstacle_mpc_bo(
        project_root=project_root,
        build_dir=build_dir,
        generator_script=generator_script,
        demo_executable=demo_executable,
        scenarios=scenarios,
        weight_spec=OBSTACLE_MPC_WEIGHT_SPEC,
        opt_var=opt_var,
        n_init=args.init,
        n_iter=args.iter,
        output_file=output_file
    )
    
    return 0

if __name__ == "__main__":
    sys.exit(main())
