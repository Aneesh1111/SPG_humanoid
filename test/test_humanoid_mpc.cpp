#include <gtest/gtest.h>
#include "spg/setpoint/HumanoidMPC.hpp"
#include <cmath>

using namespace spg::setpoint;

// Test fixture for HumanoidMPC tests
class HumanoidMPCTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Default parameters
        params.dt = 0.1;
        params.horizon = 10;
        params.vf_max = 1.0;
        params.vs_max = 0.3;
        params.omega_max = 1.0;
        
        // Set BO-ready weights
        params.weights.q_pos = 1.0;
        params.weights.q_phi = 0.1;
        params.weights.qf_pos = 3.0;
        params.weights.qf_phi = 0.5;
        params.weights.r_vf = 1.0;
        params.weights.r_vs = 4.0;  // Prefer forward over sideways
        params.weights.r_omega = 2.0;
        params.weights.s_vf = 0.2;
        params.weights.s_vs = 0.8;
        params.weights.s_omega = 0.4;
    }

    MPCParams params;
};

// Test basic construction
TEST_F(HumanoidMPCTest, Construction) {
    QPOasesSolver solver;
    EXPECT_NO_THROW({
        HumanoidMPC mpc(params, &solver);
    });
}

// Test that null solver throws
TEST_F(HumanoidMPCTest, NullSolverThrows) {
    EXPECT_THROW({
        HumanoidMPC mpc(params, nullptr);
    }, std::runtime_error);
}

// Test straight line forward motion
TEST_F(HumanoidMPCTest, StraightLineForward) {
    QPOasesSolver solver;
    HumanoidMPC mpc(params, &solver);

    // Robot at origin, facing forward (phi=0)
    MPCState x0{0.0, 0.0, 0.0};
    
    // Goal: move to (1.0, 0.0, 0.0)
    MPCState x_goal{1.0, 0.0, 0.0};
    
    // Currently at rest
    MPCControl u_meas{0.0, 0.0, 0.0};

    MPCControl u;
    ASSERT_TRUE(mpc.computeControl(x0, x_goal, u_meas, u));

    // Should produce forward velocity, minimal sideways and rotation
    EXPECT_GT(u.vf, 0.0) << "Forward velocity should be positive";
    EXPECT_NEAR(u.vs, 0.0, 0.1) << "Sideways velocity should be near zero";
    EXPECT_NEAR(u.omega, 0.0, 0.1) << "Rotation should be near zero";
    
    // Check velocity limits
    EXPECT_LE(std::abs(u.vf), params.vf_max);
    EXPECT_LE(std::abs(u.vs), params.vs_max);
    EXPECT_LE(std::abs(u.omega), params.omega_max);
}

// Test with measured velocity (smoothness)
TEST_F(HumanoidMPCTest, SmoothTransition) {
    QPOasesSolver solver;
    HumanoidMPC mpc(params, &solver);

    MPCState x0{0.0, 0.0, 0.0};
    MPCState x_goal{1.0, 0.0, 0.0};
    
    // Already moving forward at 0.5 m/s
    MPCControl u_meas{0.5, 0.0, 0.0};

    MPCControl u;
    ASSERT_TRUE(mpc.computeControl(x0, x_goal, u_meas, u));

    // Should continue roughly at current velocity (smooth)
    EXPECT_NEAR(u.vf, 0.5, 0.3) << "Should maintain roughly current forward velocity";
}

// Test rotation requirement
TEST_F(HumanoidMPCTest, RotationRequired) {
    QPOasesSolver solver;
    HumanoidMPC mpc(params, &solver);

    // Robot at origin, facing right (phi=pi/2)
    MPCState x0{0.0, 0.0, M_PI / 2.0};
    
    // Goal: move forward in body frame (which is sideways in world)
    MPCState x_goal{0.0, 1.0, M_PI / 2.0};
    
    MPCControl u_meas{0.0, 0.0, 0.0};

    MPCControl u;
    ASSERT_TRUE(mpc.computeControl(x0, x_goal, u_meas, u));

    // Should produce forward velocity (not sideways)
    EXPECT_GT(u.vf, 0.0) << "Should move forward in body frame";
}

// Test trajectory prediction
TEST_F(HumanoidMPCTest, TrajectoryPrediction) {
    QPOasesSolver solver;
    HumanoidMPC mpc(params, &solver);

    MPCState x0{0.0, 0.0, 0.0};
    MPCState x_goal{1.0, 0.0, 0.0};
    MPCControl u_meas{0.0, 0.0, 0.0};

    MPCControl u;
    std::vector<MPCState> predicted_states;
    std::vector<MPCControl> predicted_controls;
    
    ASSERT_TRUE(mpc.computeControlAndTrajectory(x0, x_goal, u_meas, u,
                                                 predicted_states, predicted_controls));

    // Check sizes
    EXPECT_EQ(predicted_states.size(), static_cast<size_t>(params.horizon + 1));
    EXPECT_EQ(predicted_controls.size(), static_cast<size_t>(params.horizon));
    
    // First state should be initial state
    EXPECT_NEAR(predicted_states[0].x, x0.x, 1e-6);
    EXPECT_NEAR(predicted_states[0].y, x0.y, 1e-6);
    EXPECT_NEAR(predicted_states[0].phi, x0.phi, 1e-6);
    
    // Should make progress toward goal
    EXPECT_GT(predicted_states[params.horizon].x, x0.x);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
