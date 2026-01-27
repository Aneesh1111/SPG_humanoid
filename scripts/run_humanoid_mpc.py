#!/usr/bin/env python3
"""
Script to run the HumanoidMPC simulator with configurable parameters
"""

import subprocess
import sys
import json
import argparse
from pathlib import Path

def main():
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='Run HumanoidMPC Simulator')
    parser.add_argument('--start-x', type=float, default=0.0, help='Start position X (default: 0)')
    parser.add_argument('--start-y', type=float, default=3.0, help='Start position Y (default: 3)')
    parser.add_argument('--start-theta', type=float, default=0.0, help='Start orientation theta (default: 0)')
    parser.add_argument('--target-x', type=float, default=0.0, help='Target position X (default: 0)')
    parser.add_argument('--target-y', type=float, default=-3.0, help='Target position Y (default: -3)')
    parser.add_argument('--target-theta', type=float, default=0.0, help='Target orientation theta (default: 0)')
    parser.add_argument('--horizon', type=int, default=10, help='MPC prediction horizon (default: 10)')
    parser.add_argument('--q-pos', type=float, help='MPC weight: position tracking (default: 1.0)')
    parser.add_argument('--q-phi', type=float, help='MPC weight: orientation tracking (default: 0.1)')
    parser.add_argument('--qf-pos', type=float, help='MPC weight: final position (default: 8.0)')
    parser.add_argument('--qf-phi', type=float, help='MPC weight: final orientation (default: 1.0)')
    parser.add_argument('--r-vf', type=float, help='MPC weight: forward velocity control (default: 0.1)')
    parser.add_argument('--r-vs', type=float, help='MPC weight: sideways velocity control (default: 0.5)')
    parser.add_argument('--r-omega', type=float, help='MPC weight: angular velocity control (default: 0.2)')
    parser.add_argument('--s-vf', type=float, help='MPC weight: forward velocity smoothness (default: 0.2)')
    parser.add_argument('--s-vs', type=float, help='MPC weight: sideways velocity smoothness (default: 0.8)')
    parser.add_argument('--s-omega', type=float, help='MPC weight: angular velocity smoothness (default: 0.4)')
    
    args = parser.parse_args()
    
    # Get project root
    script_dir = Path(__file__).parent.absolute()
    project_root = script_dir.parent
    build_dir = project_root / "build"
    
    # Check if build directory exists
    if not build_dir.exists():
        print("❌ Build directory not found. Please build the project first:")
        print("   cd build && cmake .. && make -j4")
        return 1
    
    # Check which demo to run
    demo_executable = build_dir / "demo_humanoid_mpc"
    
    if not demo_executable.exists():
        print("❌ Demo executable not found. Please build the project first:")
        print("   cd build && make demo_humanoid_mpc -j4")
        return 1
    
    # Create configuration file
    config = {
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
    
    # Add MPC weights if any are specified
    mpc_weights = {}
    if args.q_pos is not None: mpc_weights["q_pos"] = args.q_pos
    if args.q_phi is not None: mpc_weights["q_phi"] = args.q_phi
    if args.qf_pos is not None: mpc_weights["qf_pos"] = args.qf_pos
    if args.qf_phi is not None: mpc_weights["qf_phi"] = args.qf_phi
    if args.r_vf is not None: mpc_weights["r_vf"] = args.r_vf
    if args.r_vs is not None: mpc_weights["r_vs"] = args.r_vs
    if args.r_omega is not None: mpc_weights["r_omega"] = args.r_omega
    if args.s_vf is not None: mpc_weights["s_vf"] = args.s_vf
    if args.s_vs is not None: mpc_weights["s_vs"] = args.s_vs
    if args.s_omega is not None: mpc_weights["s_omega"] = args.s_omega
    
    if mpc_weights:
        config["mpc_weights"] = mpc_weights
    
    # Write config to file
    config_file = build_dir / "mpc_config.json"
    with open(config_file, 'w') as f:
        json.dump(config, f, indent=2)
    
    print("╔══════════════════════════════════════════════════════════════╗")
    print("║          Running HumanoidMPC Simulator Demo                 ║")
    print("╚══════════════════════════════════════════════════════════════╝")
    print()
    print(f"📍 Start: ({args.start_x:.1f}, {args.start_y:.1f}, {args.start_theta:.2f})")
    print(f"🎯 Target: ({args.target_x:.1f}, {args.target_y:.1f}, {args.target_theta:.2f})")
    print(f"🔭 Horizon: {args.horizon}")
    if mpc_weights:
        print(f"⚙️  Custom MPC weights: {len(mpc_weights)} parameters")
    print()
    print("Controls:")
    print("  M - Toggle between MSL and HumanoidMPC modes")
    print("  R - Reset simulation")
    print("  Click - Set new target position")
    print("  ESC - Exit")
    print()
    print("Starting simulator...")
    print()
    
    # Run the demo with config
    try:
        result = subprocess.run([str(demo_executable), "--config", str(config_file)], cwd=build_dir)
        
        # Check if timing data was saved
        timing_file = build_dir / "mpc_timing_results.json"
        if timing_file.exists():
            print("\n" + "="*62)
            print("📊 TIMING DATA SUMMARY")
            print("="*62)
            
            with open(timing_file, 'r') as f:
                data = json.load(f)
            
            print(f"\n⏱️  Completion Time:")
            print(f"   Simulation time: {data['simulation_time_s']:.2f} s")
            print(f"   Wall clock time: {data['wall_time_s']:.2f} s")
            print(f"   Total steps: {data['total_steps']}")
            
            print(f"\n🚀 MPC Performance:")
            print(f"   MPC calls: {data['mpc_call_count']}")
            print(f"   Average: {data['avg_mpc_time_ms']:.3f} ms/step")
            print(f"   Min: {data['min_mpc_time_ms']:.3f} ms")
            print(f"   Max: {data['max_mpc_time_ms']:.3f} ms")
            print(f"   Frequency: {data['mpc_frequency_hz']:.1f} Hz")
            print(f"   Total MPC time: {data['total_mpc_time_ms']:.1f} ms")
            
            print(f"\n✅ Final State:")
            print(f"   Position error: {data['final_position_error_m']:.6f} m")
            print(f"   Final velocity: {data['final_velocity_ms']:.6f} m/s")
            
            print(f"\n📈 MPC Times Array:")
            mpc_times = data['mpc_times_ms']
            print(f"   Length: {len(mpc_times)} samples")
            print(f"   First 5: {mpc_times[:5]}")
            print(f"   Last 5: {mpc_times[-5:]}")
            
            print("\n" + "="*62)
            print(f"💾 Full data available in: {timing_file}")
            print("="*62 + "\n")
        
        return result.returncode
    except KeyboardInterrupt:
        print("\n\nSimulation stopped by user")
        return 0
    except Exception as e:
        print(f"❌ Error running simulator: {e}")
        return 1

if __name__ == "__main__":
    sys.exit(main())
