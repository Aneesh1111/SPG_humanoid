#include <gtest/gtest.h>
#include "spg/subtarget/replan/ReplanUtils.hpp"
#include "spg/Init.hpp"

class ReplanUtilsTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Initialize a basic SPGState for testing
        Eigen::Vector3d p_initial(0, 0, 0);
        Eigen::Vector3d v_initial(0, 0, 0);
        Eigen::Vector2d p_initial_ball(0, 0);
        Eigen::Vector2d v_initial_ball(0, 0);
        d = spg::Init(p_initial, v_initial, 0, 20, p_initial_ball, v_initial_ball, 5);
        
        // Set up basic parameters
        d.par.vmax_rotate = 6.0;
        d.par.vmax_move = 3.0;
        d.par.amax_rotate = 10.0;
        d.par.amax_move = 2.0;
        d.par.scale_angle = 0.5;
        d.par.Ts = 0.02;
        
        // Set up basic state
        d.setpoint.p = Eigen::Vector3d(0, 0, 0);
        d.setpoint.v = Eigen::Vector3d(0, 0, 0);
        d.target.p = Eigen::Vector3d(5, 0, 0);
        d.subtarget.p = Eigen::Vector3d(2, 0, 0);
        d.subtarget.v = Eigen::Vector3d(1, 0, 0);
        d.subtarget.eta = 2.0;
        d.subtarget.age = 0;
        d.subtarget.collisionfree = true;
        d.subtarget.violation_count = 0;
        d.subtarget.amax = Eigen::Vector3d(2, 2, 10);
        d.subtarget.vmax = Eigen::Vector3d(3, 3, 6);
        d.subtarget.segment_id = Eigen::Vector3i(0, 0, 0);
        
        // Set skill ID (no ball initially)
        d.input.robot.skillID = 0;
    }
    
    spg::SPGState d;
};

TEST_F(ReplanUtilsTest, GetChecksBasicFunctionality) {
    auto checks = spg::subtarget::replan::getChecks(d);
    
    // Basic state should have most checks as false
    EXPECT_FALSE(checks.is_colliding);  // collision free
    EXPECT_FALSE(checks.age_threshold_reached);  // age = 0
    EXPECT_FALSE(checks.hasball);  // skillID = 0
    EXPECT_FALSE(checks.starts_with_violation);  // violation_count = 0
    EXPECT_FALSE(checks.is_at_subtarget_soon);  // eta = 2.0 > 0.5
}

TEST_F(ReplanUtilsTest, NewSubtargetDesiredNoBall) {
    // With no ball and normal conditions, should not need replanning
    EXPECT_FALSE(spg::subtarget::replan::newSubtargetDesired(d));
    
    // Test collision case
    d.subtarget.collisionfree = false;
    EXPECT_TRUE(spg::subtarget::replan::newSubtargetDesired(d));
    d.subtarget.collisionfree = true;
    
    // Test age threshold
    d.subtarget.age = 200;  // 200 * 0.02 = 4 seconds > 3 second threshold
    EXPECT_TRUE(spg::subtarget::replan::newSubtargetDesired(d));
    d.subtarget.age = 0;
    
    // Test approaching subtarget soon
    d.subtarget.eta = 0.3;  // < 0.5 second threshold
    EXPECT_TRUE(spg::subtarget::replan::newSubtargetDesired(d));
}

TEST_F(ReplanUtilsTest, NewSubtargetDesiredWithBall) {
    // Set has ball state
    d.input.robot.skillID = 2;  // Has ball
    
    // With ball and normal conditions, should not need replanning
    EXPECT_FALSE(spg::subtarget::replan::newSubtargetDesired(d));
    
    // Test collision case
    d.subtarget.collisionfree = false;
    EXPECT_TRUE(spg::subtarget::replan::newSubtargetDesired(d));
    d.subtarget.collisionfree = true;
    
    // Test starts with violation
    d.subtarget.violation_count = 1;
    EXPECT_TRUE(spg::subtarget::replan::newSubtargetDesired(d));
    d.subtarget.violation_count = 0;
    
    // Test age threshold  
    d.subtarget.age = 200;
    EXPECT_TRUE(spg::subtarget::replan::newSubtargetDesired(d));
    d.subtarget.age = 0;
    
    // Test approaching subtarget soon
    d.subtarget.eta = 0.3;
    EXPECT_TRUE(spg::subtarget::replan::newSubtargetDesired(d));
}

TEST_F(ReplanUtilsTest, ChecksAngleDifference) {
    auto checks = spg::subtarget::replan::getChecks(d);
    
    // Small angle difference initially (both at 0)
    EXPECT_TRUE(checks.small_angle_diff);
    
    // Large angle difference
    d.subtarget.p(2) = 2.0;  // Large angle difference
    checks = spg::subtarget::replan::getChecks(d);
    EXPECT_FALSE(checks.small_angle_diff);
}

TEST_F(ReplanUtilsTest, ChecksDistanceMeasurements) {
    auto checks = spg::subtarget::replan::getChecks(d);
    
    // Not close to subtarget initially (2m distance > 1.5m threshold)
    EXPECT_FALSE(checks.is_close_to_subtarget);
    
    // Move close to subtarget
    d.subtarget.p = Eigen::Vector3d(1.0, 0, 0);  // 1m distance < 1.5m threshold
    checks = spg::subtarget::replan::getChecks(d);
    EXPECT_TRUE(checks.is_close_to_subtarget);
    
    // Test subtarget at target
    d.subtarget.p = d.target.p;  // Same position
    checks = spg::subtarget::replan::getChecks(d);
    EXPECT_TRUE(checks.subtarget_at_target);
}

TEST_F(ReplanUtilsTest, GetDistanceInsidePenaltyArea) {
    // Set up penalty area parameters
    d.par.field_penalty_area = {3.0, 2.0};  // 3m x 2m penalty area
    d.par.field_size = {12.0, 8.0};  // 12m x 8m field
    
    // Test point outside penalty area
    Eigen::Vector3d pos_outside(0, 0, 0);
    double dist = spg::subtarget::replan::getDistanceInsidePenaltyArea(d, pos_outside);
    EXPECT_EQ(dist, 0.0);
    
    // Test point at penalty area boundary  
    Eigen::Vector3d pos_boundary(1.5, 3.0, 0);  // At x boundary
    dist = spg::subtarget::replan::getDistanceInsidePenaltyArea(d, pos_boundary);
    EXPECT_GE(dist, 0.0);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
