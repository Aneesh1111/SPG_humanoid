#pragma once
#include "spg/Init.hpp"
#include "visualization/SimulatorVisualizer.hpp"

class SPGSimulator {
public:
    SPGSimulator(const spg::SPGState& initial_state);
    void run();
    void step();
    void runWithoutVisualization();
    // ...other methods...
private:
    spg::SPGState state_;
    SimulatorVisualizer visualizer;
    // ...other members...
};
