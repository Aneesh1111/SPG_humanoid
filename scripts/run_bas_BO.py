#!/usr/bin/env python3
from typing import Dict, List, Tuple, Any
import numpy as np
import math
import subprocess
import sys
import json
import argparse
from pathlib import Path
from typing import List, Dict, Tuple
import time
import math
from itertools import product
from bayes_opt import BayesianOptimization

def spec_to_weights(spec: Dict[str, Dict[str, float]]) -> Dict[str, float]:
    """Extract plain {name: init} weights for normal runs."""
    return {k: float(v["init"]) for k, v in spec.items()}

def spec_to_init_and_bounds(
    spec: Dict[str, Dict[str, float]],
    opt_var: List[str] | None = None,
) -> Tuple[Dict[str, float], Dict[str, Tuple[float, float]]]:
    """
    Extract BayesOpt (init, pbounds) from spec.
    If opt_var is provided, only those variables are included.
    """
    if opt_var is None:
        keys = list(spec.keys())
    else:
        missing = [k for k in opt_var if k not in spec]
        if missing:
            raise KeyError(f"opt_var contains unknown keys: {missing}")
        keys = opt_var

    init = {k: float(spec[k]["init"]) for k in keys}
    pbounds = {k: (float(spec[k]["min"]), float(spec[k]["max"])) for k in keys}
    return init, pbounds


def merge_optimized_into_defaults(
    default_weights: Dict[str, float],
    optimized_params: Dict[str, float],
) -> Dict[str, float]:
    """
    Returns a complete weights dict:
    - start from defaults
    - overwrite only the optimized keys
    """
    w = dict(default_weights)
    for k, v in optimized_params.items():
        w[k] = float(v)
    return w

def generate_scenario_configs(
    initial_velocities: List[Tuple[float, float, float]],
    target_positions: List[Tuple[float, float, float]],
    start_pos: Tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> List[Tuple[str, Dict[str, Any]]]:
    """
    Generate scenario-only configs (no MPC tuning parameters such as weights or horizon).
    """
    sx, sy, stheta = start_pos
    configs: List[Tuple[str, Dict[str, Any]]] = []

    for vel_idx, (vf, vs, omega) in enumerate(initial_velocities):
        for target_idx, (tx, ty, ttheta) in enumerate(target_positions):
            config: Dict[str, Any] = {
                "start_pos": {"x": float(sx), "y": float(sy), "theta": float(stheta)},
                "target_pos": {"x": float(tx), "y": float(ty), "theta": float(ttheta)},
                "initial_velocity": {"vf": float(vf), "vs": float(vs), "omega": float(omega)},
            }
            run_id = f"vel{vel_idx}_target{target_idx}"
            configs.append((run_id, config))

    return configs

def with_mpc_params(
    scenario_config: Dict[str, Any],
    *,
    mpc_weights: Dict[str, float],
    horizon: int | None = None,
) -> Dict[str, Any]:
    """
    Return a new config dict where MPC parameters are injected.
    """
    cfg = dict(scenario_config)  # shallow copy ok (we overwrite added keys)
    cfg["mpc_weights"] = dict(mpc_weights)
    if horizon is not None:
        cfg["prediction_horizon"] = int(horizon)
    return cfg


def regenerate_acados_solver_with_weights(
    acados_gen_script: Path,
    weights: Dict[str, float],
    horizon: int = 20,
) -> bool:
    """
    Modify generate_acados_solver.py with new weights, run it, return success.
    
    Acados weights mapping:
    - Q_pos (position) 
    - Q_theta (heading)
    - Q_vel (velocity)
    - R_accel (acceleration effort)
    - Q_term_pos, Q_term_theta, Q_term_vel, Q_term_omega (terminal costs)
    """
    # Read the generation script
    with open(acados_gen_script, 'r') as f:
        lines = f.readlines()
    
    # Find and replace weight lines
    new_lines = []
    for line in lines:
        if line.strip().startswith('N ='):
            new_lines.append(f"    N = {horizon}\n")
        elif line.strip().startswith('Q_pos ='):
            new_lines.append(f"    Q_pos = {weights.get('Q_pos', 15.0)}\n")
        elif line.strip().startswith('Q_theta ='):
            new_lines.append(f"    Q_theta = {weights.get('Q_theta', 2.0)}\n")
        elif line.strip().startswith('Q_vel ='):
            new_lines.append(f"    Q_vel = {weights.get('Q_vel', 0.5)}\n")
        elif line.strip().startswith('R_accel ='):
            new_lines.append(f"    R_accel = {weights.get('R_accel', 0.1)}\n")
        elif 'Q_term = np.diag' in line:
            # Terminal cost: [pos_x, pos_y, theta, vx, vy, omega]
            qtp = weights.get('Q_term_pos', 100.0)
            qtt = weights.get('Q_term_theta', 10.0)
            qtv = weights.get('Q_term_vel', 5.0)
            qto = weights.get('Q_term_omega', 2.0)
            new_lines.append(f"    Q_term = np.diag([{qtp}, {qtp}, {qtt}, {qtv}, {qtv}, {qto}])\n")
        else:
            new_lines.append(line)
    
    # Write modified script
    with open(acados_gen_script, 'w') as f:
        f.writelines(new_lines)
    
    # Run the generation script
    try:
        result = subprocess.run(
            [sys.executable, str(acados_gen_script)],
            cwd=acados_gen_script.parent,
            capture_output=True,
            text=True,
            timeout=60
        )
        if result.returncode != 0:
            print(f"❌ Acados generation failed: {result.stderr}")
            return False
        print("✅ Acados solver generated")
        return True
    except Exception as e:
        print(f"❌ Error running acados generation: {e}")
        return False


def recompile_project(build_dir: Path) -> bool:
    """Recompile the C++ project after acados solver regeneration."""
    try:
        print("🔨 Recompiling project...")
        result = subprocess.run(
            ["make", "-j4"],
            cwd=build_dir,
            capture_output=True,
            text=True,
            timeout=300  # 5 min compile timeout
        )
        if result.returncode != 0:
            print(f"❌ Compilation failed: {result.stderr}")
            return False
        print("✅ Compilation successful")
        return True
    except Exception as e:
        print(f"❌ Compilation error: {e}")
        return False


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
        timing_file = build_dir / "mpc_timing_results.json"
        if timing_file.exists():
            timing_file.unlink()

        result = subprocess.run(
            [str(demo_executable), "--config", str(config_file)],
            cwd=build_dir,
            capture_output=True,
            text=True,
            timeout=120  # 2 minute timeout per run
        )
        
        # Check if timing data was saved
        if timing_file.exists():
            with open(timing_file, 'r') as f:
                data = json.load(f)
            
            if verbose:
                print(f"\nâœ… Completed:")
                print(f"   Simulation time: {data['simulation_time_s']:.2f} s")
                print(f"   Avg MPC time: {data['avg_mpc_time_ms']:.3f} ms")
                print(f"   MPC calls: {data['mpc_call_count']}")
                print(f"   Position error: {data['final_position_error_m']:.6f} m")
            else:
                print(f"âœ… ({data['simulation_time_s']:.2f}s, {data['avg_mpc_time_ms']:.3f}ms avg)")
            
            # Add config info to results
            data['run_id'] = run_id
            data['config'] = config
            
            return data
        else:
            if verbose:
                print(f"âš ï¸  No timing data generated")
            else:
                print("âš ï¸  No data")
            return {
                'run_id': run_id,
                'status': 'no_timing_data',
                'config': config
            }
            
    except subprocess.TimeoutExpired:
        print(f"âš ï¸  Timeout" if not verbose else f"âš ï¸  Timeout (>120s)")
        return {
            'run_id': run_id,
            'status': 'timeout',
            'config': config
        }
    except Exception as e:
        print(f"âŒ Error: {e}")
        return {
            'run_id': run_id,
            'status': 'error',
            'error': str(e),
            'config': config
        }
    
def run_all_configurations(
    build_dir: Path,
    demo_executable: Path,
    scenario_configs: List[Tuple[str, Dict[str, Any]]],
    *,
    mpc_weights: Dict[str, float],
    horizon: int | None = None,
    verbose: bool = False,
) -> Dict[str, Dict[str, Any]]:
    """
    Run all scenario configurations for a fixed set of MPC weights (and optional horizon).

    Returns:
        Dict keyed by run_id with each value being the result dict returned by run_single_simulation().
    """
    all_results: Dict[str, Dict[str, Any]] = {}

    for idx, (run_id, base_cfg) in enumerate(scenario_configs):
        full_cfg = with_mpc_params(base_cfg, mpc_weights=mpc_weights, horizon=horizon)

        if not verbose:
            print(f"[{idx+1}/{len(scenario_configs)}] ", end="")

        result = run_single_simulation(
            build_dir=build_dir,
            demo_executable=demo_executable,
            config=full_cfg,
            run_id=run_id,
            verbose=verbose,
        )
        all_results[run_id] = result

    return all_results


def run_all_configurations_with_acados_weights(
    build_dir: Path,
    demo_executable: Path,
    acados_gen_script: Path,
    scenario_configs: List[Tuple[str, Dict[str, Any]]],
    *,
    acados_weights: Dict[str, float],
    horizon: int = 20,
    verbose: bool = False,
) -> Dict[str, Dict[str, Any]]:
    """
    Regenerate acados solver with new weights, recompile, then run all scenarios.
    
    acados_weights keys:
    - Q_pos, Q_theta, Q_vel, R_accel
    - Q_term_pos, Q_term_theta, Q_term_vel, Q_term_omega
    """
    # Step 1: Regenerate acados solver
    if not regenerate_acados_solver_with_weights(acados_gen_script, acados_weights, horizon):
        return {}
    
    # Step 2: Recompile
    if not recompile_project(build_dir):
        return {}
    
    # Step 3: Run scenarios (no mpc_weights in config - weights are now baked in)
    all_results: Dict[str, Dict[str, Any]] = {}
    for idx, (run_id, base_cfg) in enumerate(scenario_configs):
        # Add horizon to config for npredict
        full_cfg = dict(base_cfg)
        full_cfg["prediction_horizon"] = horizon
        
        if not verbose:
            print(f"[{idx+1}/{len(scenario_configs)}] ", end="")
        
        result = run_single_simulation(
            build_dir=build_dir,
            demo_executable=demo_executable,
            config=full_cfg,
            run_id=run_id,
            verbose=verbose,
        )
        all_results[run_id] = result
    
    return all_results


def compute_metrics(results: Dict[str, Dict[str, Any]]):
    """
    Returns (avg_sim_time, success_rate, n_success, n_total)
    - avg_sim_time computed over runs that contain 'simulation_time_s'
    - success_rate considers those successful runs
    """
    times = [r["simulation_time_s"] for r in results.values() if "simulation_time_s" in r]
    n_total = len(results)
    n_success = len(times)
    success_rate = (n_success / n_total) if n_total > 0 else 0.0
    avg_time = float(np.mean(times)) if times else float("inf")
    return avg_time, success_rate, n_success, n_total

def make_objective(
    *,
    build_dir: Path,
    demo_executable: Path,
    scenario_configs,
    horizon: int | None,
    default_weights: Dict[str, float],
    min_success_rate: float,
    penalty_time_s: float,
    verbose: bool = False,
):
    def objective(**bo_params) -> float:
        # bo_params contains ONLY the optimized subset (keys in pbounds)
        optimized_subset = {k: float(v) for k, v in bo_params.items()}
        weights = merge_optimized_into_defaults(default_weights, optimized_subset)

        results = run_all_configurations(
            build_dir=build_dir,
            demo_executable=demo_executable,
            scenario_configs=scenario_configs,
            mpc_weights=weights,
            horizon=horizon,
            verbose=verbose,
        )

        avg_time, success_rate, n_success, n_total = compute_metrics(results)

        if not np.isfinite(avg_time):
            score = -1e6
        else:
            score = -avg_time
            if success_rate < min_success_rate:
                score -= (min_success_rate - success_rate) * penalty_time_s

        print(
            f"[BO] avg={avg_time:.3f}s "
            f"success={n_success}/{n_total} "
            f"score={score:.4f} "
            f"subset={optimized_subset}"
        )
        return float(score)

    return objective


def make_acados_objective(
    *,
    build_dir: Path,
    demo_executable: Path,
    acados_gen_script: Path,
    scenario_configs,
    horizon: int,
    default_weights: Dict[str, float],
    min_success_rate: float,
    penalty_time_s: float,
    verbose: bool = False,
):
    """Create objective function that regenerates acados solver for each BO iteration."""
    def objective(**bo_params) -> float:
        optimized_subset = {k: float(v) for k, v in bo_params.items()}
        weights = merge_optimized_into_defaults(default_weights, optimized_subset)
        
        print(f"\n{'='*70}")
        print(f"BO iteration: Testing weights {optimized_subset}")
        print(f"{'='*70}")
        
        results = run_all_configurations_with_acados_weights(
            build_dir=build_dir,
            demo_executable=demo_executable,
            acados_gen_script=acados_gen_script,
            scenario_configs=scenario_configs,
            acados_weights=weights,
            horizon=horizon,
            verbose=verbose,
        )
        
        if not results:
            return -1e6
        
        avg_time, success_rate, n_success, n_total = compute_metrics(results)
        
        if not np.isfinite(avg_time):
            score = -1e6
        else:
            score = -avg_time
            if success_rate < min_success_rate:
                score -= (min_success_rate - success_rate) * penalty_time_s
        
        print(
            f"[BO] avg={avg_time:.3f}s "
            f"success={n_success}/{n_total} "
            f"score={score:.4f}"
        )
        return float(score)
    
    return objective


def main() -> int:
    initial_velocities = [
        (0.0, 0.0, 0.0),      # Stationary
        (1.0, 0.0, 0.0),      # 1 m/s forward
        (1.0, 0.0, 1.0),      # 1 m/s forward + rotate
        (0.0, 0.0, 1.0),      # 1 rad/s rotate
        (0.5, -0.4, 0.0),     # Diagonal forward-left
    ]

    target_positions = [
        (1.0, 0.0, 0.0),
        (4.0, 4.0, 0.0),
        (-4.0, 0.0, 0.0),
        (-2.0, 0.0, math.pi / 2),
        (0.0, -5.0, -math.pi),
        (0.0, 5.0, math.pi / 2),
        (-4.0, 6.0, 0.0),
        (4.0, -6.0, -math.pi / 2),
    ]

    start_pos = (0.0, 0.0, 0.0)

    # Acados weight spec (different from qpOASES!)
    acados_weight_spec = {
        "Q_pos":         {"init": 15.0, "min": 1.0, "max": 100.0},
        "Q_theta":       {"init": 2.0,  "min": 0.1, "max": 20.0},
        "Q_vel":         {"init": 0.5,  "min": 0.01, "max": 10.0},
        "R_accel":       {"init": 0.1,  "min": 0.01, "max": 5.0},
        "Q_term_pos":    {"init": 100.0, "min": 10.0, "max": 500.0},
        "Q_term_theta":  {"init": 10.0, "min": 1.0, "max": 100.0},
        "Q_term_vel":    {"init": 5.0,  "min": 0.1, "max": 50.0},
        "Q_term_omega":  {"init": 2.0,  "min": 0.1, "max": 20.0},
    }

    configs = generate_scenario_configs(
        initial_velocities=initial_velocities,
        target_positions=target_positions,
        start_pos=start_pos,
    )

    print(f"Generated {len(configs)} configs.")
    print("First 2 configs:")
    for run_id, cfg in configs[:2]:
        print(run_id, cfg)

    # --- Paths ---
    script_dir = Path(__file__).parent.absolute()
    project_root = script_dir.parent
    build_dir = project_root / "build"
    demo_executable = build_dir / "demo_humanoid_mpc"
    acados_gen_script = script_dir / "generate_acados_solver.py"

    if not build_dir.exists():
        print("❌ Build directory not found. Build first: cd build && cmake .. && make -j4")
        return 1
    if not demo_executable.exists():
        print("❌ Executable not found. Build first: cd build && make demo_humanoid_mpc -j4")
        return 1
    if not acados_gen_script.exists():
        print(f"❌ Acados generation script not found: {acados_gen_script}")
        return 1

    default_weights = spec_to_weights(acados_weight_spec)
    
    # --- Test with default acados weights ---
    print("\n" + "="*70)
    print("Testing DEFAULT acados weights")
    print("="*70)
    print(json.dumps(default_weights, indent=2))
    
    results = run_all_configurations_with_acados_weights(
        build_dir=build_dir,
        demo_executable=demo_executable,
        acados_gen_script=acados_gen_script,
        scenario_configs=configs,
        acados_weights=default_weights,
        horizon=20,
        verbose=False,
    )

    # --- Save results ---
    out_file = project_root / "results_acados_default.json"
    with open(out_file, "w") as f:
        json.dump(results, f, indent=2)

    avg_time, success_rate, n_success, n_total = compute_metrics(results)
    print(f"\nDefault weights: avg={avg_time:.3f}s, success={n_success}/{n_total}")
    print(f"Saved results to: {out_file}")

    # --- BayesOpt (WARNING: Each iteration takes ~1-2 minutes!) ---
    print("\n" + "="*70)
    print("Starting Bayesian Optimization (SLOW - regen + compile each iter)")
    print("="*70)
    
    # Optimize subset of weights
    opt_var = ["Q_pos", "Q_term_pos", "R_accel"]  # Start with 3 variables

    init, pbounds = spec_to_init_and_bounds(acados_weight_spec, opt_var=opt_var)

    objective = make_acados_objective(
        build_dir=build_dir,
        demo_executable=demo_executable,
        acados_gen_script=acados_gen_script,
        scenario_configs=configs,
        horizon=20,
        default_weights=default_weights,
        min_success_rate=0.8,
        penalty_time_s=120.0,
    )

    optimizer = BayesianOptimization(
        f=objective,
        pbounds=pbounds,
        random_state=1,
        verbose=2,
    )

    # Fewer iterations due to slow regeneration+compilation
    optimizer.maximize(init_points=3, n_iter=10)

    best_subset = optimizer.max["params"]
    best_weights = merge_optimized_into_defaults(default_weights, best_subset)

    print("\n" + "="*70)
    print("BEST WEIGHTS FOUND:")
    print("="*70)
    print(json.dumps(best_weights, indent=2))

    out_file_bo = project_root / "results_acados_best.json"
    best_results = run_all_configurations_with_acados_weights(
        build_dir=build_dir,
        demo_executable=demo_executable,
        acados_gen_script=acados_gen_script,
        scenario_configs=configs,
        acados_weights=best_weights,
        horizon=20,
        verbose=False,
    )
    with open(out_file_bo, "w") as f:
        json.dump(best_results, f, indent=2)
    
    avg_time_best, success_rate_best, n_success_best, n_total_best = compute_metrics(best_results)
    print(f"\nBest weights: avg={avg_time_best:.3f}s, success={n_success_best}/{n_total_best}")
    print(f"Saved BO best results to: {out_file_bo}")

    return 0

if __name__ == "__main__":
    raise SystemExit(main())