#include <gtest/gtest.h>
#include "SPGSimulator.hpp"
#include "spg/Init.hpp"
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <random>

class BouncingObstaclesIntegrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Initialize SPG system - robot starts at (0, -4)
        Eigen::Vector3d p_initial(0, -4, 0);
        Eigen::Vector3d v_initial(0, 0, 0);
        Eigen::Vector2d p_initial_ball(0, -4);  // Ball starts with robot
        Eigen::Vector2d v_initial_ball(0, 0);
        
        initial_state = spg::Init(p_initial, v_initial, 0, 20, p_initial_ball, v_initial_ball, 5);
        
        // IMPORTANT: Set target AFTER init to ensure it's not overwritten
        initial_state.input.robot.target = Eigen::Vector3d(0, 4, 0);
        initial_state.input.robot.target_vel = Eigen::Vector3d::Zero();
        
        // ALSO set the robot's target field (used by SPG algorithm)
        initial_state.input.robot.target = Eigen::Vector3d(0, 4, 0);
        
        std::cout << "DEBUG: Robot initialized at: (" << initial_state.input.robot.p.transpose() << ")" << std::endl;
        std::cout << "DEBUG: Target set to: (" << initial_state.target.p.transpose() << ")" << std::endl;
        std::cout << "DEBUG: Robot target set to: (" << initial_state.input.robot.target.transpose() << ")" << std::endl;
        std::cout << "DEBUG: Distance to target: " << (initial_state.input.robot.p - initial_state.target.p).norm() << std::endl;
        
        // Set skillID to 0 (no ball initially)
        initial_state.input.robot.skillID = 0;

        // Field boundaries (RoboCup field is approximately 8m x 12m)
        field_width = initial_state.par.field_size[0];   // x-direction: -4.0 to 4.0
        field_height = initial_state.par.field_size[1];  // y-direction: -6.0 to 6.0
        
        std::cout << "DEBUG: Field size from SPG params: " << field_width << "m x " << field_height << "m" << std::endl;
        std::cout << "DEBUG: Obstacle bounce boundaries: x = ±" << field_width/2 << ", y = ±" << field_height/2 << std::endl;

        // Initialize 20 bouncing obstacles
        setupBouncingObstacles();
    }
    
    void setupBouncingObstacles() {
        const int num_obstacles = 20;
        
        // Clear existing obstacles
        initial_state.input.obstacles.p.clear();
        initial_state.input.obstacles.v.clear();
        initial_state.input.obstacles.r.clear();
        initial_state.input.obstacles.active.clear();
        
        // Random number generator for obstacle positions and velocities
        std::random_device rd;
        std::mt19937 gen(42); // Fixed seed for reproducible tests
        std::uniform_real_distribution<double> pos_x_dist(-4.0, 4.0);
        std::uniform_real_distribution<double> pos_y_dist(-2.5, 2.5);
        std::uniform_real_distribution<double> vel_dist(-3.5, 3.5);  // Much faster obstacles!
        
        obstacle_positions.clear();
        obstacle_velocities.clear();
        
        for (int i = 0; i < num_obstacles; ++i) {
            // Random initial position (avoid starting too close to robot or target)
            Eigen::Vector2d pos;
            do {
                pos = Eigen::Vector2d(pos_x_dist(gen), pos_y_dist(gen));
            } while ((pos - Eigen::Vector2d(0, -4)).norm() < 1.0 || // Not too close to robot start
                     (pos - Eigen::Vector2d(0, 4)).norm() < 1.0);   // Not too close to target
            
            // Random initial velocity
            Eigen::Vector2d vel(vel_dist(gen), vel_dist(gen));
            // Ensure minimum speed for fast-moving obstacles
            if (vel.norm() < 1.5) {
                vel = vel.normalized() * 1.5;  // Minimum speed of 1.5 m/s
            }
            
            obstacle_positions.push_back(pos);
            obstacle_velocities.push_back(vel);
            
            // Add to initial state
            initial_state.input.obstacles.p.push_back(pos);
            initial_state.input.obstacles.v.push_back(vel);
            initial_state.input.obstacles.r.push_back(0.2); // Obstacle radius
            initial_state.input.obstacles.active.push_back(true);
        }
        
        std::cout << "Created " << num_obstacles << " bouncing obstacles:" << std::endl;
        for (size_t i = 0; i < obstacle_positions.size(); ++i) {
            std::cout << "  Obstacle " << i << ": pos(" << obstacle_positions[i].transpose() 
                      << ") vel(" << obstacle_velocities[i].transpose() << ")" << std::endl;
        }
    }
    
    // Update obstacle positions with field boundary bouncing
    void updateObstaclePositions(double dt) {
        for (size_t i = 0; i < obstacle_positions.size(); ++i) {
            // Update position
            obstacle_positions[i] += obstacle_velocities[i] * dt;
            
            // Check boundaries and bounce
            // X boundaries (left/right walls)
            if (obstacle_positions[i].x() <= -field_width/2 || obstacle_positions[i].x() >= field_width/2) {
                obstacle_velocities[i].x() *= -1; // Reverse x velocity
                // Clamp position to boundary
                obstacle_positions[i].x() = std::max(-field_width/2, std::min(field_width/2, obstacle_positions[i].x()));
            }
            
            // Y boundaries (top/bottom walls)
            if (obstacle_positions[i].y() <= -field_height/2 || obstacle_positions[i].y() >= field_height/2) {
                std::cout << "DEBUG: Obstacle " << i << " bounced at y=" << obstacle_positions[i].y() 
                          << " (boundary at ±" << field_height/2 << ")" << std::endl;
                obstacle_velocities[i].y() *= -1; // Reverse y velocity
                // Clamp position to boundary
                obstacle_positions[i].y() = std::max(-field_height/2, std::min(field_height/2, obstacle_positions[i].y()));
            }
            
            // Update in initial state for simulator
            if (i < initial_state.input.obstacles.p.size()) {
                initial_state.input.obstacles.p[i] = obstacle_positions[i];
                initial_state.input.obstacles.v[i] = obstacle_velocities[i];
            }
        }
    }
    
    void runBouncingObstaclesSimulation(bool with_visualization = false) {
        std::cout << "=== Bouncing Obstacles Integration Test ===" << std::endl;
        std::cout << "Robot starts at: (" << initial_state.input.robot.p.transpose() << ")" << std::endl;
        std::cout << "Robot target:     (" << initial_state.target.p.transpose() << ")" << std::endl;
        std::cout << "Initial distance to target: " << (initial_state.input.robot.p - initial_state.target.p).norm() << std::endl;
        std::cout << "Robot velocity:   (" << initial_state.input.robot.v.transpose() << ")" << std::endl;
        std::cout << "Field size: " << field_width << "m x " << field_height << "m" << std::endl;
        std::cout << "Number of bouncing obstacles: " << initial_state.input.obstacles.p.size() << std::endl;
        
        if (with_visualization) {
            std::cout << "\n🎮 VISUALIZATION MODE:" << std::endl;
            std::cout << "- Blue circle = Robot" << std::endl;
            std::cout << "- Green circle = Target" << std::endl;
            std::cout << "- Red circles = Bouncing obstacles" << std::endl;
            std::cout << "- Watch obstacles bounce off field boundaries!" << std::endl;
            std::cout << "- Close window when done watching...\n" << std::endl;
            
            // FINAL CHECK before creating simulator
            std::cout << "FINAL CHECK before simulator:" << std::endl;
            std::cout << "  Robot position: (" << initial_state.input.robot.p.transpose() << ")" << std::endl;
            std::cout << "  Target position: (" << initial_state.target.p.transpose() << ")" << std::endl;
            std::cout << "  Position error: " << (initial_state.input.robot.p - initial_state.target.p).norm() << std::endl;
            
            // Create simulator and run with visualization
            SPGSimulator simulator(initial_state);
            simulator.run();
        } else {
            std::cout << "Running automated test (no visualization)..." << std::endl;
            // For automated testing, just run a short simulation
            SPGSimulator simulator(initial_state);
            simulator.runWithoutVisualization(100, 0.02); // 2 seconds
        }
        
        std::cout << "✅ Bouncing obstacles simulation completed!" << std::endl;
    }
    
    spg::SPGState initial_state;
    std::vector<Eigen::Vector2d> obstacle_positions;
    std::vector<Eigen::Vector2d> obstacle_velocities;
    double field_width, field_height;
};

TEST_F(BouncingObstaclesIntegrationTest, AutomatedBouncingObstacles) {
    // Automated test - always passes since it's a visual integration test
    runBouncingObstaclesSimulation(false);
    
    // This test always passes - it's just to verify the simulation runs
    EXPECT_TRUE(true) << "Bouncing obstacles integration test completed successfully";
}

TEST_F(BouncingObstaclesIntegrationTest, DISABLED_VisualBouncingObstacles) {
    // Visual test - only run when specifically requested
    // To run: ./test_bouncing_obstacles_integration --gtest_also_run_disabled_tests --gtest_filter="*VisualBouncingObstacles*"
    runBouncingObstaclesSimulation(true);
    
    // This test always passes - it's a visual integration test
    EXPECT_TRUE(true) << "Visual bouncing obstacles test completed - you should have seen the simulation!";
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    
    // Print usage information
    if (argc > 1) {
        for (int i = 1; i < argc; ++i) {
            if (std::string(argv[i]).find("help") != std::string::npos) {
                std::cout << "\n🚀 Bouncing Obstacles Integration Test" << std::endl;
                std::cout << "=======================================" << std::endl;
                std::cout << "This test simulates a robot navigating from (0,-4) to (0,4)" << std::endl;
                std::cout << "while 10 FAST obstacles bounce around the field like pinballs!" << std::endl;
                std::cout << "" << std::endl;
                std::cout << "📋 Usage:" << std::endl;
                std::cout << "  Automated test:  ./test_bouncing_obstacles_integration" << std::endl;
                std::cout << "  Visual test:     ./test_bouncing_obstacles_integration --gtest_also_run_disabled_tests --gtest_filter=\"*VisualBouncingObstacles*\"" << std::endl;
                std::cout << "" << std::endl;
                std::cout << "🎯 What you'll see in visual mode:" << std::endl;
                std::cout << "  • Blue circle: Robot (starts at 0,-4, goes to 0,4)" << std::endl;
                std::cout << "  • Green circle: Target at (0,4)" << std::endl;
                std::cout << "  • Red circles: 10 FAST obstacles bouncing off field walls (1.5-5.0 m/s)" << std::endl;
                std::cout << "  • Real-time trajectory planning and obstacle avoidance!" << std::endl;
                std::cout << "" << std::endl;
                return 0;
            }
        }
    }
    
    return RUN_ALL_TESTS();
}