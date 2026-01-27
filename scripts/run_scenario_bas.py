#!/usr/bin/env python3
from typing import Dict, List, Tuple, Any
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

    default_weights = {
        "q_pos": 1.0,
        "q_phi": 0.1,
        "qf_pos": 8.0,
        "qf_phi": 1.0,
        "r_vf": 0.1,
        "r_vs": 0.5,
        "r_omega": 0.2,
        "s_vf": 0.2,
        "s_vs": 0.8,
        "s_omega": 0.4,
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

    # --- Paths (same logic as original script) ---
    script_dir = Path(__file__).parent.absolute()
    project_root = script_dir.parent
    build_dir = project_root / "build"
    demo_executable = build_dir / "demo_humanoid_mpc"

    if not build_dir.exists():
        print("❌ Build directory not found. Build first: cd build && cmake .. && make -j4")
        return 1
    if not demo_executable.exists():
        print("❌ Executable not found. Build first: cd build && make demo_humanoid_mpc -j4")
        return 1

    # --- Run all scenarios with default weights ---
    results = run_all_configurations(
        build_dir=build_dir,
        demo_executable=demo_executable,
        scenario_configs=configs,
        mpc_weights=default_weights,
        horizon=10,     # use None if you want the C++ default instead
        verbose=False,
    )

    # --- Save results ---
    out_file = project_root / "results_default_weights.json"
    with open(out_file, "w") as f:
        json.dump(results, f, indent=2)

    successful = sum(1 for r in results.values() if "simulation_time_s" in r)
    print(f"\nCompleted {len(results)} runs. Successful: {successful}/{len(results)}")
    print(f"Saved results to: {out_file}")

    return 0

if __name__ == "__main__":
    raise SystemExit(main())