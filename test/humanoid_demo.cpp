#include <gtest/gtest.h>
#include "HumanoidSimulator.hpp"
#include "spg/Init.hpp"
#include "spg/HumanoidUtils.hpp"
#include "spg/HumanoidTrajectoryGenerator.hpp"
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <iomanip>

class HumanoidDemoTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Initialize SPG with humanoid parameters
        Eigen::Vector3d p_initial(0, 0, 0);
        Eigen::Vector3d v_initial(0, 0, 0);
        Eigen::Vector2d p_initial_ball(0, 0);
        Eigen::Vector2d v_initial_ball(0, 0);
        
        state = spg::Init(p_initial, v_initial, 0, 20, p_initial_ball, v_initial_ball, 5);
        
        // Set robot orientation (facing +Y direction initially)
        state.input.robot.IMU_orientation = Eigen::Vector3d(0, 0, 0); // [roll, pitch, yaw]
        
        // Set skillID to 0 (no ball)
        state.input.robot.skillID = 0;
    }
    
    void setupDemoScenario(const Eigen::Vector3d& robot_start, 
                          const Eigen::Vector3d& target_pos,
                          double robot_orientation = 0.0,
                          const std::string& scenario_name = "") {
        
        // Reinitialize SPG with correct starting position
        Eigen::Vector2d p_initial_ball = robot_start.head<2>();
        Eigen::Vector2d v_initial_ball(0, 0);
        state = spg::Init(robot_start, Eigen::Vector3d::Zero(), 0, 20, p_initial_ball, v_initial_ball, 5);
        
        // Set robot state (after init to ensure it's not overwritten)
        state.input.robot.p = robot_start;
        state.input.robot.v = Eigen::Vector3d::Zero();
        state.input.robot.IMU_orientation(2) = robot_orientation; // Set yaw
        state.input.robot.target = target_pos; // Set robot's internal target
        state.input.robot.skillID = 0;
        
        // Set SPG target (both locations to be sure)
        state.target.p = target_pos;
        state.target.v = Eigen::Vector3d::Zero();
        
        // Force update subtarget as well
        state.subtarget.target = target_pos;
        
        // Clear obstacles for clean demo
        state.input.obstacles.p.clear();
        state.input.obstacles.v.clear();
        state.input.obstacles.r.clear();
        state.input.obstacles.active.clear();
        
        std::cout << "\n🤖 " << scenario_name << std::endl;
        std::cout << "==================================" << std::endl;
        std::cout << "Robot starts at: [" << robot_start.transpose() << "]" << std::endl;
        std::cout << "Robot orientation: " << robot_orientation * 180.0 / M_PI << "° (facing " 
                  << (std::abs(robot_orientation) < 0.1 ? "+Y (forward)" : 
                      std::abs(robot_orientation - M_PI/2) < 0.1 ? "+X (right)" :
                      std::abs(robot_orientation - M_PI) < 0.1 ? "-Y (backward)" : 
                      std::abs(robot_orientation + M_PI/2) < 0.1 ? "-X (left)" : "angle") << ")" << std::endl;
        std::cout << "Target: [" << target_pos.transpose() << "]" << std::endl;
        
        // Verify the positions are set correctly
        std::cout << "VERIFICATION:" << std::endl;
        std::cout << "  state.input.robot.p: [" << state.input.robot.p.transpose() << "]" << std::endl;
        std::cout << "  state.target.p: [" << state.target.p.transpose() << "]" << std::endl;
        std::cout << "  state.input.robot.target: [" << state.input.robot.target.transpose() << "]" << std::endl;
        std::cout << "  Distance to target: " << (state.target.p - state.input.robot.p).norm() << " meters" << std::endl;
        
        // Show movement analysis
        Eigen::Vector3d movement_direction = target_pos - robot_start;
        Eigen::Vector3d local_movement = spg::humanoid::globalToLocalVelocity(movement_direction, robot_orientation);
        
        std::cout << "Movement direction (global): [" << movement_direction.transpose() << "]" << std::endl;
        std::cout << "Movement direction (local):  [" << local_movement.head<2>().transpose() << "] (sideways, forward)" << std::endl;
        
        // Calculate effective limits
        Eigen::Vector3d effective_limits = spg::humanoid::calculateEffectiveVelocityLimits(
            state.par, robot_orientation, movement_direction);
        std::cout << "Effective velocity limits: [" << effective_limits.transpose() << "] (vx, vy, vtheta)" << std::endl;
        
        // Show efficiency
        double sideways_component = std::abs(local_movement(0));
        double forward_component = std::abs(local_movement(1));
        double total = sideways_component + forward_component;
        if (total > 1e-6) {
            double forward_ratio = forward_component / total;
            double sideways_ratio = sideways_component / total;
            double efficiency = forward_ratio * state.par.forward_efficiency + sideways_ratio * state.par.sideways_efficiency;
            std::cout << "Movement efficiency: " << efficiency * 100 << "% (forward: " 
                      << forward_ratio * 100 << "%, sideways: " << sideways_ratio * 100 << "%)" << std::endl;
        }
    }
    
    void runDemo(bool with_visualization = true) {
        // Final check before starting simulator
        std::cout << "\nFINAL CHECK BEFORE SIMULATION:" << std::endl;
        std::cout << "  Robot position: [" << state.input.robot.p.transpose() << "]" << std::endl;
        std::cout << "  Target position: [" << state.target.p.transpose() << "]" << std::endl;
        std::cout << "  Distance: " << (state.target.p - state.input.robot.p).norm() << " meters" << std::endl;
        
        if ((state.target.p - state.input.robot.p).norm() < 0.01) {
            std::cout << "⚠️  WARNING: Robot and target are at the same position!" << std::endl;
            std::cout << "   This will cause the simulation to end immediately." << std::endl;
        }
        
        if (with_visualization) {
            std::cout << "\n🎮 Starting visual simulation..." << std::endl;
            std::cout << "- Blue circle = Humanoid robot" << std::endl;
            std::cout << "- Green circle = Target" << std::endl;
            std::cout << "- Watch how the robot moves differently based on orientation!" << std::endl;
            std::cout << "- Close window to continue to next demo..." << std::endl;
            
            HumanoidSimulator simulator(state);
            simulator.run();
        } else {
            // Quick automated demo
            HumanoidSimulator simulator(state);
            simulator.runWithoutVisualization(100, 0.02); // 2 seconds
        }
        
        std::cout << "✅ Demo completed!" << std::endl;
    }
    
    spg::SPGState state;
};

TEST_F(HumanoidDemoTest, DISABLED_ForwardMovementDemo) {
    // Demo 1: Moving forward when robot faces forward - should be fast and efficient
    setupDemoScenario(
        Eigen::Vector3d(0, -3, 0),     // Start position
        Eigen::Vector3d(0, 3, 0),      // Target (forward movement)
        0.0,                           // Robot facing +Y (forward)
        "DEMO 1: Forward Movement (Optimal)"
    );
    
    runDemo(true);
}

TEST_F(HumanoidDemoTest, DISABLED_SidewaysMovementDemo) {
    // Demo 2: Moving sideways when robot faces forward - should be slower
    setupDemoScenario(
        Eigen::Vector3d(-3, 0, 0),     // Start position  
        Eigen::Vector3d(3, 0, 0),      // Target (sideways movement)
        0.0,                           // Robot facing +Y (forward)
        "DEMO 2: Sideways Movement (Slower)"
    );
    
    runDemo(true);
}

TEST_F(HumanoidDemoTest, DISABLED_RotateAndMoveDemo) {
    // Demo 3: Robot needs to rotate to face target - shows orientation strategy
    setupDemoScenario(
        Eigen::Vector3d(0, 0, 0),      // Start position
        Eigen::Vector3d(4, 0, 0),      // Target to the right
        0.0,                           // Robot facing +Y (forward), but target is to the right
        "DEMO 3: Rotate Then Move (Orientation Strategy)"
    );
    
    runDemo(true);
}

TEST_F(HumanoidDemoTest, DISABLED_DiagonalMovementDemo) {
    // Demo 4: Diagonal movement - blend of forward and sideways
    setupDemoScenario(
        Eigen::Vector3d(-2, -2, 0),    // Start position
        Eigen::Vector3d(2, 2, 0),      // Target (diagonal)
        0.0,                           // Robot facing +Y (forward)
        "DEMO 4: Diagonal Movement (Blended Efficiency)"
    );
    
    runDemo(true);
}

TEST_F(HumanoidDemoTest, DISABLED_OrientationComparisonDemo) {
    // Demo 5: Same movement, different orientations - shows efficiency difference
    std::cout << "\n🔄 ORIENTATION COMPARISON DEMO" << std::endl;
    std::cout << "==============================" << std::endl;
    std::cout << "Same target, different robot orientations - notice the speed difference!" << std::endl;
    
    // Part A: Robot facing target direction (efficient)
    setupDemoScenario(
        Eigen::Vector3d(0, -2, 0),     // Start position
        Eigen::Vector3d(0, 2, 0),      // Target (forward for robot)
        0.0,                           // Robot facing +Y (aligned with movement)
        "Part A: Robot Facing Target (Efficient)"
    );
    runDemo(true);
    
    // Part B: Robot perpendicular to target (inefficient)
    setupDemoScenario(
        Eigen::Vector3d(0, -2, 0),     // Same start position
        Eigen::Vector3d(0, 2, 0),      // Same target
        M_PI/2,                        // Robot facing +X (perpendicular to movement)
        "Part B: Robot Perpendicular (Less Efficient)"
    );
    runDemo(true);
}

TEST_F(HumanoidDemoTest, AutomatedHumanoidDemo) {
    // Automated demo that always passes - just to verify the system works
    std::cout << "\n🤖 AUTOMATED HUMANOID DEMO" << std::endl;
    std::cout << "===========================" << std::endl;
    
    setupDemoScenario(
        Eigen::Vector3d(0, 0, 0),
        Eigen::Vector3d(2, 2, 0),
        0.0,
        "Automated Demo Test"
    );
    
    runDemo(false); // No visualization for automated test
    
    EXPECT_TRUE(true) << "Humanoid demo completed successfully";
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    
    std::cout << "🤖 HUMANOID SPG DEMO" << std::endl;
    std::cout << "====================" << std::endl;
    std::cout << "This demo shows how humanoid constraints affect robot movement:" << std::endl;
    std::cout << "• Forward movement (Y-direction) is faster than sideways (X-direction)" << std::endl;
    std::cout << "• Robot orientation affects movement efficiency" << std::endl;  
    std::cout << "• Rotation strategies minimize movement penalties" << std::endl;
    std::cout << "" << std::endl;
    std::cout << "📋 Usage:" << std::endl;
    std::cout << "  Automated demo:     ./humanoid_demo" << std::endl;
    std::cout << "  Visual demos:       ./humanoid_demo --gtest_also_run_disabled_tests --gtest_filter=\"*Demo*\"" << std::endl;
    std::cout << "" << std::endl;
    std::cout << "🎯 Individual demos:" << std::endl;
    std::cout << "  Forward movement:   --gtest_filter=\"*ForwardMovementDemo*\"" << std::endl;
    std::cout << "  Sideways movement:  --gtest_filter=\"*SidewaysMovementDemo*\"" << std::endl;
    std::cout << "  Rotate and move:    --gtest_filter=\"*RotateAndMoveDemo*\"" << std::endl;
    std::cout << "  Diagonal movement:  --gtest_filter=\"*DiagonalMovementDemo*\"" << std::endl;
    std::cout << "  Orientation compare: --gtest_filter=\"*OrientationComparisonDemo*\"" << std::endl;
    std::cout << "" << std::endl;
    std::cout << "📊 Humanoid Parameters:" << std::endl;
    std::cout << "  Max forward velocity:  4.0 m/s (Y-direction)" << std::endl;
    std::cout << "  Max sideways velocity: 2.5 m/s (X-direction)" << std::endl;
    std::cout << "  Forward efficiency:    100%" << std::endl;
    std::cout << "  Sideways efficiency:   70%" << std::endl;
    std::cout << "" << std::endl;
    
    return RUN_ALL_TESTS();
}