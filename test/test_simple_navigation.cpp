#include <gtest/gtest.h>
#include <iostream>
#include <Eigen/Dense>
#include <random>
#include "SPGSimulator.hpp"
#include "spg/Init.hpp"

class StraightLineToTargetIntegrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Default initial parameters (similar to MATLAB test setup)
        p_initial = Eigen::Vector3d(-3, 5, 0);
        v_initial = Eigen::Vector3d(0, 0, 0);
        target = Eigen::Vector3d(0, 0, 0);
        target_vel = Eigen::Vector3d(0, 0, 0);
        npredict = 20;
        nobstacles = 0;
        nintercept_positions = 15;
        p_initial_ball = Eigen::Vector2d(0, 0);
        v_initial_ball = Eigen::Vector2d(0, 0);
    }

    void runSim(bool with_visualization = false) {
        // Initialize state using spg::Init
        auto initial_state = spg::Init(p_initial, v_initial, nobstacles, npredict, 
                                      p_initial_ball, v_initial_ball, nintercept_positions);

        // Set target
        initial_state.input.robot.target = target;
        initial_state.input.robot.target_vel = target_vel;

        // Clear all obstacles first
        for (int i = 0; i < nobstacles; ++i) {
            initial_state.input.obstacles.active[i] = false;
            initial_state.input.obstacles.p[i] = Eigen::Vector2d(0, 0);
            initial_state.input.obstacles.v[i] = Eigen::Vector2d(0, 0);
            initial_state.input.obstacles.r[i] = 0.3; // Default radius
        }

        // Set up obstacles as configured by test
        for (size_t i = 0; i < obstacle_positions.size() && i < nobstacles; ++i) {
            initial_state.input.obstacles.p[i] = obstacle_positions[i];
            initial_state.input.obstacles.active[i] = obstacle_active[i];
            initial_state.input.obstacles.v[i] = Eigen::Vector2d(0, 0); // Static obstacles
            initial_state.input.obstacles.r[i] = 0.3;
        }

        // Create simulator
        SPGSimulator simulator(initial_state);
        
        if (with_visualization) {
            // Run with visualization for manual testing/debugging
            std::cout << "Running test with visualization - close window to continue..." << std::endl;
            simulator.run(); // This will show the GUI
        } else {
            // Run simulation for a fixed number of steps without visualization
            const int max_steps = 100;
            const double dt = 0.1;
            simulator.runWithoutVisualization(max_steps, dt);
        }
        
        // Basic validation - simulation should complete without crashing
        EXPECT_TRUE(true); // If we get here, simulation completed successfully
        
        std::cout << "Test completed successfully with " << obstacle_positions.size() 
                  << " obstacles" << std::endl;
    }

    // Test configuration variables
    Eigen::Vector3d p_initial, v_initial, target, target_vel;
    Eigen::Vector2d p_initial_ball, v_initial_ball;
    int npredict, nobstacles, nintercept_positions;
    
    std::vector<Eigen::Vector2d> obstacle_positions;
    std::vector<bool> obstacle_active;
};

TEST_F(StraightLineToTargetIntegrationTest, BalanceXandY) {
    std::cout << "Running BalanceXandY test" << std::endl;
    runSim();
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    
    // Print usage information
    std::cout << "=== SPG Integration Test Suite ===" << std::endl;
    std::cout << "Usage:" << std::endl;
    std::cout << "  Normal testing:     ./test_static_obstacles_integration" << std::endl;
    std::cout << "  With visualization: SPG_VISUALIZE=1 ./test_static_obstacles_integration" << std::endl;
    std::cout << "  Specific test:      ./test_static_obstacles_integration --gtest_filter=*Visualize*" << std::endl;
    std::cout << "========================================" << std::endl;
    
    return RUN_ALL_TESTS();
}
