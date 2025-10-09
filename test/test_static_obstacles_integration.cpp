#include <gtest/gtest.h>
#include <iostream>
#include <Eigen/Dense>
#include <random>
#include "SPGSimulator.hpp"
#include "spg/Init.hpp"

class StaticObstaclesIntegrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Default initial parameters (similar to MATLAB test setup)
        p_initial = Eigen::Vector3d(0, 0, 0);
        v_initial = Eigen::Vector3d(0, 0, 0);
        target = Eigen::Vector3d(0, 0, 0);
        target_vel = Eigen::Vector3d(0, 0, 0);
        npredict = 20;
        nobstacles = 10;
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

TEST_F(StaticObstaclesIntegrationTest, ObstacleWall) {
    // Test: obstacle_wall - 5 obstacles in a line from (-1,0) to (1,0)
    const int m = 5;
    obstacle_positions.clear();
    obstacle_active.clear();
    
    for (int i = 0; i < m; ++i) {
        double x = -1.0 + 2.0 * i / (m - 1); // linspace(-1, 1, m)
        obstacle_positions.push_back(Eigen::Vector2d(x, 0));
        obstacle_active.push_back(true);
    }
    
    std::cout << "Running obstacle_wall test with " << m << " obstacles in a line" << std::endl;
    runSim();
}

TEST_F(StaticObstaclesIntegrationTest, ObstacleTwoWalls) {
    // Test: obstacle_two_walls - Two walls of obstacles
    const int m1 = 5;
    const int m2 = 3;
    obstacle_positions.clear();
    obstacle_active.clear();
    
    // First wall: 5 obstacles from (-1,-1) to (1,-1)
    for (int i = 0; i < m1; ++i) {
        double x = -1.0 + 2.0 * i / (m1 - 1); // linspace(-1, 1, m1)
        obstacle_positions.push_back(Eigen::Vector2d(x, -1));
        obstacle_active.push_back(true);
    }
    
    // Second wall: 3 obstacles from (-4,2) to (-2,2)
    for (int i = 0; i < m2; ++i) {
        double x = -4.0 + 2.0 * i / (m2 - 1); // linspace(-4, -2, m2)
        obstacle_positions.push_back(Eigen::Vector2d(x, 2));
        obstacle_active.push_back(true);
    }
    
    std::cout << "Running obstacle_two_walls test with " << (m1 + m2) << " obstacles in two walls" << std::endl;
    runSim();
}

TEST_F(StaticObstaclesIntegrationTest, NearFieldEdge) {
    // Test: near_field_edge - Target near field edge with obstacle
    target = Eigen::Vector3d(-4.1, -2.5, 0);
    
    obstacle_positions.clear();
    obstacle_active.clear();
    obstacle_positions.push_back(Eigen::Vector2d(0, -3.3));
    obstacle_active.push_back(true);
    
    std::cout << "Running near_field_edge test with target at field edge" << std::endl;
    runSim();
}

TEST_F(StaticObstaclesIntegrationTest, Random) {
    // Test: random - Random obstacle positions with seed 0
    const int m = 10;
    std::mt19937 gen(0); // Fixed seed for reproducibility
    std::normal_distribution<double> dist(0.0, 1.0);
    
    obstacle_positions.clear();
    obstacle_active.clear();
    
    for (int i = 0; i < m; ++i) {
        double x = dist(gen);
        double y = dist(gen);
        obstacle_positions.push_back(Eigen::Vector2d(x, y));
        obstacle_active.push_back(true);
    }
    
    std::cout << "Running random test with " << m << " randomly placed obstacles" << std::endl;
    runSim();
}

TEST_F(StaticObstaclesIntegrationTest, RandomScrum) {
    // Test: random_scrum - Random obstacles with robot at origin
    const int m = 10;
    std::mt19937 gen(1); // Different seed from random test
    std::normal_distribution<double> dist(0.0, 1.0);
    
    p_initial = Eigen::Vector3d(0, 0, 0); // Robot starts at origin
    obstacle_positions.clear();
    obstacle_active.clear();
    
    for (int i = 0; i < m; ++i) {
        double x = dist(gen);
        double y = dist(gen);
        obstacle_positions.push_back(Eigen::Vector2d(x, y));
        obstacle_active.push_back(true);
    }
    
    std::cout << "Running random_scrum test with robot at origin and " << m << " random obstacles" << std::endl;
    runSim();
}

TEST_F(StaticObstaclesIntegrationTest, Keeper) {
    // Test: keeper - Goalkeeper scenario near field edge
    // Note: field_size(2) in MATLAB is field width, assuming 6m wide field
    const double field_width = 6.0;
    
    p_initial = Eigen::Vector3d(-1, -field_width/2 - 0.1, 0.5);
    target = p_initial + Eigen::Vector3d(2, 0, -1);
    
    // No specific obstacles for this test, just the challenging position
    obstacle_positions.clear();
    obstacle_active.clear();
    
    std::cout << "Running keeper test with goalkeeper starting position" << std::endl;
    runSim();
}

// Additional test for moving obstacles (extension of static tests)
TEST_F(StaticObstaclesIntegrationTest, MovingObstaclesCircular) {
    // Test with moving obstacles in circular motion
    const int m = 5;
    obstacle_positions.clear();
    obstacle_active.clear();
    
    for (int i = 0; i < m; ++i) {
        double angle = 2.0 * M_PI * i / m;
        double radius = 1.5;
        obstacle_positions.push_back(Eigen::Vector2d(radius * cos(angle), radius * sin(angle)));
        obstacle_active.push_back(true);
    }
    
    // Override runSim to add velocities for this test
    auto initial_state = spg::Init(p_initial, v_initial, nobstacles, npredict, 
                                  p_initial_ball, v_initial_ball, nintercept_positions);
    
    initial_state.input.robot.target = target;
    initial_state.input.robot.target_vel = target_vel;
    
    // Set up moving obstacles
    for (int i = 0; i < m && i < nobstacles; ++i) {
        double angle = 2.0 * M_PI * i / m;
        initial_state.input.obstacles.p[i] = obstacle_positions[i];
        initial_state.input.obstacles.active[i] = true;
        // Give tangential velocity for circular motion
        initial_state.input.obstacles.v[i] = Eigen::Vector2d(-sin(angle), cos(angle)) * 0.5;
        initial_state.input.obstacles.r[i] = 0.3;
    }
    
    SPGSimulator simulator(initial_state);
    
    // Check if visualization is requested via environment variable
    const char* viz_env = std::getenv("SPG_VISUALIZE");
    bool with_viz = (viz_env != nullptr && std::string(viz_env) == "1");
    
    if (with_viz) {
        std::cout << "Running moving_obstacles_circular test with visualization..." << std::endl;
        simulator.run();
    } else {
        simulator.runWithoutVisualization(50, 0.1);
    }
    
    std::cout << "Running moving_obstacles_circular test with " << m << " moving obstacles" << std::endl;
    EXPECT_TRUE(true); // Test completion
}

// Visualization-specific test cases
TEST_F(StaticObstaclesIntegrationTest, VisualizeObstacleWall) {
    // Check if visualization is requested
    const char* viz_env = std::getenv("SPG_VISUALIZE");
    if (!viz_env || std::string(viz_env) != "1") {
        GTEST_SKIP() << "Skipping visualization test (set SPG_VISUALIZE=1 to enable)";
    }
    
    // Set robot initial position to (-4, 0, 0)
    p_initial = Eigen::Vector3d(-4.0, 0.0, 0.0);
    
    const int m = 5;
    obstacle_positions.clear();
    obstacle_active.clear();
    
    for (int i = 0; i < m; ++i) {
        double x = -1.0 + 2.0 * i / (m - 1);
        obstacle_positions.push_back(Eigen::Vector2d(x, 0));
        obstacle_active.push_back(true);
    }
    
    std::cout << "=== VISUALIZATION: Obstacle Wall Test ===" << std::endl;
    std::cout << "Robot starts at (-4, 0, 0). You should see 5 obstacles in a horizontal line blocking the robot's path." << std::endl;
    runSim(true); // Enable visualization
}

TEST_F(StaticObstaclesIntegrationTest, VisualizeMovingObstacles) {
    // Check if visualization is requested
    const char* viz_env = std::getenv("SPG_VISUALIZE");
    if (!viz_env || std::string(viz_env) != "1") {
        GTEST_SKIP() << "Skipping visualization test (set SPG_VISUALIZE=1 to enable)";
    }
    
    const int m = 8;
    obstacle_positions.clear();
    obstacle_active.clear();
    
    // Create obstacles in a more interesting pattern
    for (int i = 0; i < m; ++i) {
        double angle = 2.0 * M_PI * i / m;
        double radius = 1.0 + 0.5 * sin(3 * angle); // Vary radius
        obstacle_positions.push_back(Eigen::Vector2d(radius * cos(angle), radius * sin(angle)));
        obstacle_active.push_back(true);
    }
    
    // Override to add moving obstacles
    auto initial_state = spg::Init(p_initial, v_initial, nobstacles, npredict, 
                                  p_initial_ball, v_initial_ball, nintercept_positions);
    
    initial_state.input.robot.target = target;
    initial_state.input.robot.target_vel = target_vel;
    
    // Set up moving obstacles with different velocities
    for (int i = 0; i < m && i < nobstacles; ++i) {
        double angle = 2.0 * M_PI * i / m;
        initial_state.input.obstacles.p[i] = obstacle_positions[i];
        initial_state.input.obstacles.active[i] = true;
        // Give varying velocities
        double speed = 0.3 + 0.2 * (i % 3);
        initial_state.input.obstacles.v[i] = Eigen::Vector2d(-sin(angle), cos(angle)) * speed;
        initial_state.input.obstacles.r[i] = 0.2 + 0.1 * (i % 3); // Vary radius too
    }
    
    SPGSimulator simulator(initial_state);
    
    std::cout << "=== VISUALIZATION: Moving Obstacles Test ===" << std::endl;
    std::cout << "You should see " << m << " obstacles moving in circular patterns." << std::endl;
    std::cout << "Watch them bounce off the field boundaries with energy damping!" << std::endl;
    simulator.run();
    
    EXPECT_TRUE(true);
}

TEST_F(StaticObstaclesIntegrationTest, VisualizeNearFieldEdge) {
    // Check if visualization is requested  
    const char* viz_env = std::getenv("SPG_VISUALIZE");
    if (!viz_env || std::string(viz_env) != "1") {
        GTEST_SKIP() << "Skipping visualization test (set SPG_VISUALIZE=1 to enable)";
    }
    
    // Start robot near one corner
    p_initial = Eigen::Vector3d(-3.5, -2.0, 0);
    target = Eigen::Vector3d(-4.1, -2.5, 0); // Target near field edge
    
    obstacle_positions.clear();
    obstacle_active.clear();
    obstacle_positions.push_back(Eigen::Vector2d(0, -3.3)); // Obstacle near field edge
    obstacle_active.push_back(true);
    
    std::cout << "=== VISUALIZATION: Near Field Edge Test ===" << std::endl;
    std::cout << "Robot starts near field corner, target is at field edge." << std::endl;
    std::cout << "Watch how the robot navigates around the obstacle near the boundary!" << std::endl;
    runSim(true);
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
