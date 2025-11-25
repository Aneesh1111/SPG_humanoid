#pragma once
#include "spg/Init.hpp"
#include "visualization/SimulatorVisualizer.hpp"

class SPGSimulator {
public:
    SPGSimulator(const spg::SPGState& initial_state);
    void run();
    void runHumanoid();  // Run simulation with humanoid constraints
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
    // ...other members...
};
