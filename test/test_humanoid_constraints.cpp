#include <gtest/gtest.h>
#include "spg/Init.hpp"
#include "spg/HumanoidUtils.hpp"
#include "spg/HumanoidTrajectoryGenerator.hpp"
#include <Eigen/Dense>
#include <iostream>
#include <iomanip>

class HumanoidConstraintsTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Initialize SPG with humanoid parameters
        Eigen::Vector3d p_initial(0, 0, 0);
        Eigen::Vector3d v_initial(0, 0, 0);
        Eigen::Vector2d p_initial_ball(0, 0);
        Eigen::Vector2d v_initial_ball(0, 0);
        
        state = spg::Init(p_initial, v_initial, 0, 20, p_initial_ball, v_initial_ball, 5);
        
        // Set robot orientation (facing +x direction initially)
        state.input.robot.IMU_orientation = Eigen::Vector3d(0, 0, 0); // [roll, pitch, yaw]
    }
    
    void printVelocityComparison(const std::string& scenario, 
                                const Eigen::Vector3d& global_desired,
                                const Eigen::Vector3d& effective_limits,
                                double orientation) {
        std::cout << "\n=== " << scenario << " ===" << std::endl;
        std::cout << std::fixed << std::setprecision(3);
        std::cout << "Robot orientation: " << orientation * 180.0 / M_PI << "° (facing " 
                  << (std::abs(orientation) < 0.1 ? "forward" : 
                      std::abs(orientation - M_PI/2) < 0.1 ? "left" :
                      std::abs(orientation - M_PI) < 0.1 ? "backward" : "right") << ")" << std::endl;
        std::cout << "Desired global velocity: [" << global_desired.transpose() << "]" << std::endl;
        std::cout << "Effective limits:        [" << effective_limits.transpose() << "]" << std::endl;
        
        // Transform to show local frame
        Eigen::Vector3d local_desired = spg::humanoid::globalToLocalVelocity(global_desired, orientation);
        std::cout << "Local frame (forward/sideways/rotation): [" << local_desired.transpose() << "]" << std::endl;
        
        // Show efficiency
        double forward_eff = std::abs(local_desired(0)) > 1e-6 ? effective_limits.head<2>().norm() / state.par.vmax_move_x : 1.0;
        double sideways_eff = std::abs(local_desired(1)) > 1e-6 ? effective_limits.head<2>().norm() / state.par.vmax_move_y : 1.0;
        std::cout << "Movement efficiency: forward=" << forward_eff*100 << "%, sideways=" << sideways_eff*100 << "%" << std::endl;
    }
    
    spg::SPGState state;
};

TEST_F(HumanoidConstraintsTest, DirectionalVelocityLimits) {
    std::cout << "\n🤖 HUMANOID VELOCITY CONSTRAINTS TEST" << std::endl;
    std::cout << "=====================================" << std::endl;
    
    // Print parameter summary
    std::cout << "Humanoid Parameters:" << std::endl;
    std::cout << "  vmax_move_x (sideways):     " << state.par.vmax_move_x << " m/s" << std::endl;
    std::cout << "  vmax_move_y (forward/back): " << state.par.vmax_move_y << " m/s" << std::endl;
    std::cout << "  forward_efficiency:         " << state.par.forward_efficiency * 100 << "%" << std::endl;
    std::cout << "  sideways_efficiency:        " << state.par.sideways_efficiency * 100 << "%" << std::endl;
    std::cout << "  rotation_penalty:           " << state.par.rotation_while_moving_penalty * 100 << "%" << std::endl;
    
    // Test Case 1: Moving forward when robot faces forward (+Y direction)
    double robot_orientation = 0.0; // Facing +Y
    Eigen::Vector3d desired_forward(0.0, 3.0, 0.0);
    Eigen::Vector3d limits_forward = spg::humanoid::calculateEffectiveVelocityLimits(
        state.par, robot_orientation, desired_forward);
    printVelocityComparison("Moving Forward (aligned)", desired_forward, limits_forward, robot_orientation);
    
    // Test Case 2: Moving sideways when robot faces forward (+X direction)
    Eigen::Vector3d desired_sideways(2.0, 0.0, 0.0);
    Eigen::Vector3d limits_sideways = spg::humanoid::calculateEffectiveVelocityLimits(
        state.par, robot_orientation, desired_sideways);
    printVelocityComparison("Moving Sideways", desired_sideways, limits_sideways, robot_orientation);
    
    // Test Case 3: Diagonal movement
    Eigen::Vector3d desired_diagonal(2.0, 2.0, 0.0);
    Eigen::Vector3d limits_diagonal = spg::humanoid::calculateEffectiveVelocityLimits(
        state.par, robot_orientation, desired_diagonal);
    printVelocityComparison("Diagonal Movement", desired_diagonal, limits_diagonal, robot_orientation);
    
    // Test Case 4: Robot rotated 90° (facing +X), moving in +Y direction  
    robot_orientation = M_PI/2; // Facing +X direction
    Eigen::Vector3d desired_rotated(0.0, 3.0, 0.0);
    Eigen::Vector3d limits_rotated = spg::humanoid::calculateEffectiveVelocityLimits(
        state.par, robot_orientation, desired_rotated);
    printVelocityComparison("Forward motion when rotated 90°", desired_rotated, limits_rotated, robot_orientation);
    
    // Verify that forward motion (in robot frame) gets higher limits than sideways
    EXPECT_GT(limits_forward.head<2>().norm(), limits_sideways.head<2>().norm()) 
        << "Forward movement should be faster than sideways movement";
    
    // Verify that diagonal movement has balanced constraints
    EXPECT_LT(limits_diagonal.head<2>().norm(), limits_forward.head<2>().norm())
        << "Diagonal movement should be constrained compared to pure forward";
        
    std::cout << "\n✅ All humanoid constraint tests passed!" << std::endl;
}

TEST_F(HumanoidConstraintsTest, OrientationStrategy) {
    std::cout << "\n🔄 ORIENTATION STRATEGY TEST" << std::endl;
    std::cout << "============================" << std::endl;
    
    // Test different target positions and optimal orientation strategies
    std::vector<std::pair<Eigen::Vector3d, std::string>> test_targets = {
        {Eigen::Vector3d(0, 5, 0), "Forward"},
        {Eigen::Vector3d(-5, 0, 0), "Left"},  
        {Eigen::Vector3d(0, -5, 0), "Backward"},
        {Eigen::Vector3d(5, 0, 0), "Right"},
        {Eigen::Vector3d(3, 3, 0), "Diagonal (45°)"},
        {Eigen::Vector3d(0, 1, 0), "Close Forward"}
    };
    
    for (const auto& test : test_targets) {
        Eigen::Vector3d target = test.first;
        std::string description = test.second;
        
        spg::humanoid::OrientationStrategy strategy = 
            spg::humanoid::calculateOptimalOrientation(state, target, 0.0);
        
        std::cout << "\nTarget: " << description << " [" << target.transpose() << "]" << std::endl;
        std::cout << "  Optimal orientation: " << strategy.target_orientation * 180.0 / M_PI << "°" << std::endl;
        std::cout << "  Rotate first: " << (strategy.should_rotate_first ? "Yes" : "No") << std::endl;
        std::cout << "  Movement blend: " << strategy.rotation_movement_blend * 100 << "% simultaneous" << std::endl;
        
        // Verify that forward targets don't require rotation
        if (target(1) == 0 && target(0) > 0) {
            EXPECT_NEAR(strategy.target_orientation, 0.0, 0.1) << "Forward target should have 0° orientation";
        }
    }
    
    std::cout << "\n✅ Orientation strategy tests completed!" << std::endl;
}

TEST_F(HumanoidConstraintsTest, MovementCommandGeneration) {
    std::cout << "\n🎯 MOVEMENT COMMAND GENERATION TEST" << std::endl;
    std::cout << "===================================" << std::endl;
    
    double dt = 0.02; // 20ms timestep
    
    // Test movement to different targets
    std::vector<Eigen::Vector3d> targets = {
        Eigen::Vector3d(0, 2, 0),    // Forward
        Eigen::Vector3d(2, 0, 0),    // Sideways  
        Eigen::Vector3d(1, -1, 0),   // Backward-diagonal
        Eigen::Vector3d(0, 0.5, 0)   // Close target
    };
    
    for (size_t i = 0; i < targets.size(); ++i) {
        Eigen::Vector3d target = targets[i];
        
        Eigen::Vector3d movement_cmd = spg::humanoid::generateHumanoidMovementCommand(
            state, target, dt);
        
        std::cout << "\nTarget " << i+1 << ": [" << target.transpose() << "]" << std::endl;
        std::cout << "  Generated command: [" << movement_cmd.transpose() << "] (vx, vy, vtheta)" << std::endl;
        
        // Transform to local frame to analyze
        Eigen::Vector3d local_cmd = spg::humanoid::globalToLocalVelocity(movement_cmd, 0.0);
        std::cout << "  Local frame: [" << local_cmd.transpose() << "] (forward, sideways, rotation)" << std::endl;
        
        // Check that commands respect velocity limits
        EXPECT_LE(std::abs(local_cmd(0)), state.par.vmax_move_x + 0.01) 
            << "Sideways velocity should not exceed limit";
        EXPECT_LE(std::abs(local_cmd(1)), state.par.vmax_move_y + 0.01) 
            << "Forward velocity should not exceed limit";
        EXPECT_LE(std::abs(local_cmd(2)), state.par.vmax_rotate + 0.01) 
            << "Rotational velocity should not exceed limit";
    }
    
    std::cout << "\n✅ Movement command generation tests passed!" << std::endl;
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    
    std::cout << "🤖 HUMANOID SPG CONSTRAINTS TEST SUITE" << std::endl;
    std::cout << "=======================================" << std::endl;
    std::cout << "Testing directional velocity constraints, orientation strategies," << std::endl;
    std::cout << "and movement command generation for humanoid robots." << std::endl;
    
    return RUN_ALL_TESTS();
}