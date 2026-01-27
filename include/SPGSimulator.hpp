#pragma once
#include "spg/Init.hpp"
#include "visualization/SimulatorVisualizer.hpp"
#include <chrono>
#include <vector>

class SPGSimulator {
public:
    SPGSimulator(const spg::SPGState& initial_state);
    void run();
    void step();
    void runWithoutVisualization();
    void runWithoutVisualization(int max_steps, double dt);
    // ...other methods...
private:
    spg::SPGState state_;
    spg::SPGState initial_state_; // Store initial state for reset functionality
    SimulatorVisualizer visualizer;
    double simulation_time_; // Elapsed simulation time in seconds
    int step_count_; // Number of simulation steps
    bool simulation_completed_; // Flag to indicate if simulation reached target
    double total_mpc_time_ms_; // Total MPC computation time in milliseconds
    int mpc_call_count_; // Number of MPC calls
    std::vector<double> mpc_times_ms_; // Array of individual MPC computation times in milliseconds
    std::chrono::high_resolution_clock::time_point start_time_; // Simulation start time
    // ...other members...
};
