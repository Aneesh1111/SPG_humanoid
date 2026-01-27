#!/usr/bin/env python3
"""
Simple MPC Benchmark Runner

Runs HumanoidMPC simulations and collects timing data by creating
custom C++ benchmark programs for each configuration.

Usage:
    python run_mpc_benchmarks.py --runs 100
    python run_mpc_benchmarks.py --runs 1000 --randomize --output results.csv
"""

import subprocess
import json
import csv
import argparse
import os
import re
from pathlib import Path
from typing import Dict, List
import numpy as np


class SimpleMPCBenchmark:
    """Simple benchmark runner that generates and runs C++ programs"""
    
    def __init__(self, project_root: Path):
        self.project_root = Path(project_root)
        self.build_dir = self.project_root / "build"
        self.demo_dir = self.project_root / "demo"
        self.set_cpp = self.project_root / "src" / "spg" / "setpoint" / "Set.cpp"
        
    def update_mpc_weights(self, weights: Dict):
        """Update MPC weights in Set.cpp"""
        
        with open(self.set_cpp, 'r') as f:
            content = f.read()
        
        # Find and replace weight values
        for key, value in weights.items():
            pattern = f"(mpc_params\\.weights\\.{key}\\s*=\\s*)([\\d.]+)"
            content = re.sub(pattern, f"\\g<1>{value}", content)
        
        with open(self.set_cpp, 'w') as f:
            f.write(content)
        
        print(f"✓ Updated MPC weights in Set.cpp")
    
    def create_benchmark_cpp(self, run_id: int, start: List, target: List, 
                           max_time: float = 60.0) -> Path:
        """Create a benchmark C++ file"""
        
        code = f'''// Auto-generated benchmark {run_id}
#include <iostream>
#include <fstream>
#include <chrono>
#include <Eigen/Dense>
#include "spg/Init.hpp"
#include "spg/setpoint/StateCorrection.hpp"
#include "spg/target/Target.hpp"
#include "spg/subtarget/SubtargetSet.hpp"
#include "spg/setpoint/Setpoint.hpp"

int main() {{
    // Configuration
    Eigen::Vector3d p_initial({start[0]}, {start[1]}, {start[2]});
    Eigen::Vector3d target({target[0]}, {target[1]}, {target[2]});
    
    // Initialize state
    auto state = spg::Init(p_initial, Eigen::Vector3d(0,0,0), 0, 20, 
                          Eigen::Vector2d(0,0), Eigen::Vector2d(0,0), 15);
    
    state.par.use_humanoid_mpc = true;
    state.input.robot.target = target;
    state.input.robot.p = p_initial;
    state.setpoint.p = p_initial;
    state.setpoint.v = Eigen::Vector3d::Zero();
    
    // Run simulation
    double dt = 0.02;
    int max_steps = static_cast<int>({max_time} / dt);
    double sim_time = 0;
    int steps = 0;
    double total_mpc_ms = 0;
    bool reached = false;
    
    std::vector<double> control_changes;
    Eigen::Vector3d prev_v = Eigen::Vector3d::Zero();
    
    for (int i = 0; i < max_steps; ++i) {{
        // SPG simulation step
        spg::setpoint::stateCorrection(state);
        spg::target::updateTarget(state);
        spg::subtarget::updateSubtarget(state);
        
        auto t1 = std::chrono::high_resolution_clock::now();
        spg::setpoint::updateSetpoint(state);
        auto t2 = std::chrono::high_resolution_clock::now();
        
        double step_ms = std::chrono::duration<double, std::milli>(t2 - t1).count();
        total_mpc_ms += step_ms;
        
        // Update robot position
        for (int j = 0; j < 3; ++j) {{
            state.input.robot.p[j] = state.setpoint.p[j];
            state.input.robot.v[j] = state.setpoint.v[j];
        }}
        
        steps++;
        sim_time += dt;
        
        // Check goal
        double dist = (state.setpoint.p.head<2>() - target.head<2>()).norm();
        if (dist < 0.15) {{
            reached = true;
            break;
        }}
        
        // Track smoothness
        double dv = (state.setpoint.v - prev_v).norm();
        control_changes.push_back(dv);
        prev_v = state.setpoint.v;
        
        sim_time += dt;
    }}
    
    // Compute stats
    double avg_mpc_ms = total_mpc_ms / steps;
    double avg_smoothness = 0;
    if (!control_changes.empty()) {{
        for (double dc : control_changes) avg_smoothness += dc;
        avg_smoothness /= control_changes.size();
    }}
    
    // Output JSON
    std::ofstream out("bench_result_{run_id}.json");
    out << "{{" << std::endl;
    out << "  \\"run_id\\": {run_id}," << std::endl;
    out << "  \\"reached\\": " << (reached ? "true" : "false") << "," << std::endl;
    out << "  \\"time\\": " << sim_time << "," << std::endl;
    out << "  \\"steps\\": " << steps << "," << std::endl;
    out << "  \\"avg_mpc_ms\\": " << avg_mpc_ms << "," << std::endl;
    out << "  \\"smoothness\\": " << avg_smoothness << "," << std::endl;
    out << "  \\"final_dist\\": " << (state.setpoint.p - target).head<2>().norm() << std::endl;
    out << "}}" << std::endl;
    
    return reached ? 0 : 1;
}}
'''
        
        cpp_file = self.demo_dir / f"benchmark_{run_id}.cpp"
        with open(cpp_file, 'w') as f:
            f.write(code)
        
        return cpp_file
    
    def compile(self) -> bool:
        """Compile the project"""
        
        print("Compiling...")
        result = subprocess.run(["cmake", "--build", ".", "--target", "spg", "-j4"],
                              cwd=self.build_dir,
                              capture_output=True,
                              text=True)
        
        if result.returncode != 0:
            print(f"❌ Compilation failed:\\n{result.stderr}")
            return False
        
        print("✓ Compilation successful")
        return True
    
    def compile_and_run_benchmark(self, run_id: int, cpp_file: Path) -> Dict:
        """Compile and run a single benchmark"""
        
        # Update CMakeLists to include this benchmark
        exec_name = f"benchmark_{run_id}"
        
        # Simple approach: compile directly
        compile_cmd = [
            "g++", "-std=c++17", "-O3",
            str(cpp_file),
            "-I", str(self.project_root / "include"),
            "-I", "/usr/include/eigen3",
            "-I", "/usr/local/include",
            "-o", str(self.build_dir / exec_name),
            "-L", str(self.build_dir),
            "-lspg", "-lqpOASES"
        ]
        
        result = subprocess.run(compile_cmd, capture_output=True, text=True)
        if result.returncode != 0:
            print(f"❌ Compile error for run {run_id}")
            print(result.stderr)
            return {"run_id": run_id, "status": "compile_error"}
        
        # Run the executable
        result_file = self.build_dir / f"bench_result_{run_id}.json"
        if result_file.exists():
            result_file.unlink()
        
        try:
            subprocess.run([self.build_dir / exec_name],
                          cwd=self.build_dir,
                          timeout=35,
                          capture_output=True)
        except subprocess.TimeoutExpired:
            return {"run_id": run_id, "status": "timeout"}
        
        # Read results
        if result_file.exists():
            with open(result_file, 'r') as f:
                return json.load(f)
        else:
            return {"run_id": run_id, "status": "no_output"}
    
    def run_batch(self, n_runs: int, weights: Dict = None, 
                  randomize: bool = False, output_csv: str = "results.csv"):
        """Run batch of benchmarks"""
        
        # Default weights
        if weights is None:
            weights = {
                "q_pos": 1.0, "q_phi": 0.1,
                "qf_pos": 8.0, "qf_phi": 1.0,
                "r_vf": 0.1, "r_vs": 0.5, "r_omega": 0.2,
                "s_vf": 0.2, "s_vs": 0.8, "s_omega": 0.4
            }
        
        # Update weights and recompile
        self.update_mpc_weights(weights)
        if not self.compile():
            return []
        
        results = []
        
        print(f"\\nRunning {n_runs} benchmarks...")
        print("=" * 60)
        
        for i in range(n_runs):
            # Generate random or fixed positions
            if randomize:
                start = [
                    np.random.uniform(-4, 4),
                    np.random.uniform(-6, 6),
                    np.random.uniform(-np.pi, np.pi)
                ]
                target = [
                    np.random.uniform(-4, 4),
                    np.random.uniform(-6, 6),
                    np.random.uniform(-np.pi, np.pi)
                ]
            else:
                # Fixed scenario: corner to corner
                start = [4.0, -6.0, 0.0]
                target = [0.0, 0.0, 0.0]
            
            # Create benchmark
            cpp_file = self.create_benchmark_cpp(i, start, target)
            
            # Run it
            result = self.compile_and_run_benchmark(i, cpp_file)
            result["weights"] = weights
            result["start"] = start
            result["target"] = target
            results.append(result)
            
            # Progress
            if result.get("reached"):
                print(f"[{i+1}/{n_runs}] ✓ {result['time']:.2f}s, "
                      f"MPC: {result['avg_mpc_ms']:.2f}ms")
            else:
                print(f"[{i+1}/{n_runs}] ✗ {result.get('status', 'failed')}")
        
        # Save results
        self.save_csv(results, output_csv)
        self.print_stats(results)
        
        return results
    
    def save_csv(self, results: List[Dict], filename: str):
        """Save results to CSV"""
        
        if not results:
            return
        
        fieldnames = ["run_id", "reached", "time", "steps", "avg_mpc_ms", 
                     "smoothness", "final_dist", "status"]
        
        with open(filename, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction='ignore')
            writer.writeheader()
            writer.writerows(results)
        
        print(f"\\n💾 Results saved to {filename}")
    
    def print_stats(self, results: List[Dict]):
        """Print summary statistics"""
        
        successful = [r for r in results if r.get("reached")]
        n_success = len(successful)
        n_total = len(results)
        
        print(f"\\n{'='*60}")
        print(f"RESULTS SUMMARY")
        print(f"{'='*60}")
        print(f"Success: {n_success}/{n_total} ({100*n_success/n_total:.1f}%)")
        
        if successful:
            times = [r["time"] for r in successful]
            mpc_times = [r["avg_mpc_ms"] for r in successful]
            
            print(f"\\nCompletion Time:")
            print(f"  Mean: {np.mean(times):.2f}s ± {np.std(times):.2f}s")
            print(f"  Range: [{np.min(times):.2f}s, {np.max(times):.2f}s]")
            
            print(f"\\nMPC Computation:")
            print(f"  Mean: {np.mean(mpc_times):.2f}ms ± {np.std(mpc_times):.2f}ms")
            print(f"  Range: [{np.min(mpc_times):.2f}ms, {np.max(mpc_times):.2f}ms]")
        
        print(f"{'='*60}\\n")


def main():
    parser = argparse.ArgumentParser(description="Simple MPC Benchmark Runner")
    parser.add_argument("--runs", type=int, default=10,
                       help="Number of runs (default: 10)")
    parser.add_argument("--randomize", action="store_true",
                       help="Randomize start/target positions")
    parser.add_argument("--output", type=str, default="mpc_results.csv",
                       help="Output CSV file")
    parser.add_argument("--weights", type=str,
                       help="JSON file with MPC weights")
    
    args = parser.parse_args()
    
    # Find project root
    script_dir = Path(__file__).parent
    project_root = script_dir.parent
    
    # Load weights if provided
    weights = None
    if args.weights:
        with open(args.weights, 'r') as f:
            weights = json.load(f)
    
    # Run benchmarks
    benchmark = SimpleMPCBenchmark(project_root)
    benchmark.run_batch(args.runs, weights=weights, 
                       randomize=args.randomize, output_csv=args.output)


if __name__ == "__main__":
    main()
