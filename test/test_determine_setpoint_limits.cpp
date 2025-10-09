#include <gtest/gtest.h>
#include "spg/subtarget/replan/ReplanUtils.hpp"
#include "spg/Init.hpp"

class DetermineSetpointLimitsTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Initialize a basic SPGState for testing
        Eigen::Vector3d p_initial(0, 0, 0);
        Eigen::Vector3d v_initial(0, 0, 0);
        Eigen::Vector2d p_initial_ball(0, 0);
        Eigen::Vector2d v_initial_ball(0, 0);
        d = spg::Init(p_initial, v_initial, 0, 20, p_initial_ball, v_initial_ball, 5);
        
        // Set up basic parameters
        d.par.vmax_move = 3.0;
        d.par.amax_move = 2.0;
        d.par.dmax_move = 2.0;
        d.par.vmax_rotate = 6.0;
        d.par.amax_rotate = 10.0;
        d.par.dmax_rotate = 10.0;
        d.par.scale_rotate = 0.5;
        d.par.scale_angle = 0.5;
        d.par.Ts = 0.02;
        
        // Set up basic setpoint
        d.setpoint.p = Eigen::Vector3d(1.0, 2.0, 0.5);
        d.setpoint.v = Eigen::Vector3d(0.5, 0.3, 0.1);
        
        // Set up basic subtarget
        subtarget.p = Eigen::Vector3d(3.0, 4.0, 1.0);
        subtarget.v = Eigen::Vector3d(1.0, 1.5, 0.2);
        subtarget.vmax = Eigen::Vector3d(2.5, 2.5, 5.0);
        subtarget.amax = Eigen::Vector3d(1.8, 1.8, 8.0);
        subtarget.dmax = Eigen::Vector3d(1.8, 1.8, 8.0);
        subtarget.eta = 2.0;
        subtarget.age = 10;
        subtarget.collisionfree = true;
        subtarget.violation_count = 0;
        
        // Initialize segments (basic 4-segment structure)
        subtarget.segment.resize(4);
        for (int i = 0; i < 4; ++i) {
            subtarget.segment[i].dt = Eigen::Vector3d(0.5, 0.5, 0.5);
            subtarget.segment[i].t = Eigen::Vector3d(0.5 * (i + 1), 0.5 * (i + 1), 0.5 * (i + 1));
            subtarget.segment[i].p = Eigen::Vector3d(i * 0.5, i * 0.5, i * 0.1);
            subtarget.segment[i].v = Eigen::Vector3d(1.0, 1.0, 0.1);
            subtarget.segment[i].a = Eigen::Vector3d(0.5, 0.5, 0.1);
        }
    }
    
    spg::SPGState d;
    spg::Subtarget subtarget;
};

TEST_F(DetermineSetpointLimitsTest, MatlabValidationTest) {
    // MATLAB validation test with specific inputs and expected outputs
    
    // Set up MATLAB input parameters
    d.setpoint.p = Eigen::Vector3d(3.9950, -5.9925, 0);
    d.setpoint.v = Eigen::Vector3d(-0.0998, 0.1498, 0);
    
    // Set up subtarget input
    subtarget.p = Eigen::Vector3d(0, 0, 0);
    subtarget.v = Eigen::Vector3d(0, 0, 0);
    
    // Set up MATLAB parameter values
    d.par.vmax_move = 4.0;
    d.par.scale_rotate = 0.3000;
    d.par.amax_move = 1.8000;
    d.par.dmax_move = 1.8000;
    d.par.dmax_rotate = 13.0;
    d.par.amax_rotate = 13.0;
    d.par.vmax_rotate = 13.0;
    
    // Initialize subtarget segments to ensure proper structure
    subtarget.segment.resize(4);
    for (int i = 0; i < 4; ++i) {
        subtarget.segment[i].dt = Eigen::Vector3d::Zero();
        subtarget.segment[i].t = Eigen::Vector3d::Zero();
        subtarget.segment[i].p = Eigen::Vector3d::Zero();
        subtarget.segment[i].v = Eigen::Vector3d::Zero();
        subtarget.segment[i].a = Eigen::Vector3d::Zero();
    }
    
    // Call the function under test
    spg::Subtarget result = spg::subtarget::replan::determineSetpointLimits(d, subtarget);
    
    // Validate MATLAB expected outputs
    
    // Expected velocity (should remain [0, 0, 0])
    EXPECT_NEAR(result.v[0], 0.0, 1e-6) << "Velocity X should be 0";
    EXPECT_NEAR(result.v[1], 0.0, 1e-6) << "Velocity Y should be 0";
    EXPECT_NEAR(result.v[2], 0.0, 1e-6) << "Velocity Z should be 0";
    
    // Expected vmax: [2.2185, 3.3284, 13.0000]
    EXPECT_NEAR(result.vmax[0], 2.2185, 1e-3) << "vmax X mismatch";
    EXPECT_NEAR(result.vmax[1], 3.3284, 1e-3) << "vmax Y mismatch";
    EXPECT_NEAR(result.vmax[2], 13.0000, 1e-3) << "vmax Z mismatch";
    
    // Expected amax: [0.9983, 1.4978, 13.0000]
    EXPECT_NEAR(result.amax[0], 0.9983, 1e-3) << "amax X mismatch";
    EXPECT_NEAR(result.amax[1], 1.4978, 1e-3) << "amax Y mismatch";
    EXPECT_NEAR(result.amax[2], 13.0000, 1e-3) << "amax Z mismatch";
    
    // Expected dmax: [0.9983, 1.4978, 13.0000]
    EXPECT_NEAR(result.dmax[0], 0.9983, 1e-3) << "dmax X mismatch";
    EXPECT_NEAR(result.dmax[1], 1.4978, 1e-3) << "dmax Y mismatch";
    EXPECT_NEAR(result.dmax[2], 13.0000, 1e-3) << "dmax Z mismatch";
    
    // Validate segment structure
    EXPECT_EQ(result.segment.size(), 4) << "Should have 4 segments";
    
    // Expected segment 0 dt: [1.9017, 1.9015, 0]
    EXPECT_NEAR(result.segment[0].dt[0], 1.9017, 1e-3) << "Segment 0 dt X mismatch";
    EXPECT_NEAR(result.segment[0].dt[1], 1.9015, 1e-3) << "Segment 0 dt Y mismatch";
    EXPECT_NEAR(result.segment[0].dt[2], 0.0, 1e-6) << "Segment 0 dt Z mismatch";
    
    // Expected segment 0 t: [1.9017, 1.9015, 0]
    EXPECT_NEAR(result.segment[0].t[0], 1.9017, 1e-3) << "Segment 0 t X mismatch";
    EXPECT_NEAR(result.segment[0].t[1], 1.9015, 1e-3) << "Segment 0 t Y mismatch";
    EXPECT_NEAR(result.segment[0].t[2], 0.0, 1e-6) << "Segment 0 t Z mismatch";
    
    // Expected segment 0 p: [2, -3, 0]
    EXPECT_NEAR(result.segment[0].p[0], 2.0, 1e-3) << "Segment 0 p X mismatch";
    EXPECT_NEAR(result.segment[0].p[1], -3.0, 1e-3) << "Segment 0 p Y mismatch";
    EXPECT_NEAR(result.segment[0].p[2], 0.0, 1e-6) << "Segment 0 p Z mismatch";
    
    // Expected segment 0 v: [-1.9983, 2.9978, 0]
    EXPECT_NEAR(result.segment[0].v[0], -1.9983, 1e-3) << "Segment 0 v X mismatch";
    EXPECT_NEAR(result.segment[0].v[1], 2.9978, 1e-3) << "Segment 0 v Y mismatch";
    EXPECT_NEAR(result.segment[0].v[2], 0.0, 1e-6) << "Segment 0 v Z mismatch";
    
    // Expected segment 0 a: [-0.9983, 1.4978, 13.0000]
    EXPECT_NEAR(result.segment[0].a[0], -0.9983, 1e-3) << "Segment 0 a X mismatch";
    EXPECT_NEAR(result.segment[0].a[1], 1.4978, 1e-3) << "Segment 0 a Y mismatch";
    EXPECT_NEAR(result.segment[0].a[2], 13.0000, 1e-3) << "Segment 0 a Z mismatch";
    
    // Expected segment 1 dt: [0, 0, 0]
    EXPECT_NEAR(result.segment[1].dt[0], 0.0, 1e-6) << "Segment 1 dt X mismatch";
    EXPECT_NEAR(result.segment[1].dt[1], 0.0, 1e-6) << "Segment 1 dt Y mismatch";
    EXPECT_NEAR(result.segment[1].dt[2], 0.0, 1e-6) << "Segment 1 dt Z mismatch";
    
    // Expected segment 1 t: [1.9017, 1.9015, 0]
    EXPECT_NEAR(result.segment[1].t[0], 1.9017, 1e-3) << "Segment 1 t X mismatch";
    EXPECT_NEAR(result.segment[1].t[1], 1.9015, 1e-3) << "Segment 1 t Y mismatch";
    EXPECT_NEAR(result.segment[1].t[2], 0.0, 1e-6) << "Segment 1 t Z mismatch";
    
    // Expected segment 1 p: [2, -3, 0]
    EXPECT_NEAR(result.segment[1].p[0], 2.0, 1e-3) << "Segment 1 p X mismatch";
    EXPECT_NEAR(result.segment[1].p[1], -3.0, 1e-3) << "Segment 1 p Y mismatch";
    EXPECT_NEAR(result.segment[1].p[2], 0.0, 1e-6) << "Segment 1 p Z mismatch";
    
    // Expected segment 1 v: [-1.9983, 2.9978, 0]
    EXPECT_NEAR(result.segment[1].v[0], -1.9983, 1e-3) << "Segment 1 v X mismatch";
    EXPECT_NEAR(result.segment[1].v[1], 2.9978, 1e-3) << "Segment 1 v Y mismatch";
    EXPECT_NEAR(result.segment[1].v[2], 0.0, 1e-6) << "Segment 1 v Z mismatch";
    
    // Expected segment 1 a: [0, 0, 0]
    EXPECT_NEAR(result.segment[1].a[0], 0.0, 1e-6) << "Segment 1 a X mismatch";
    EXPECT_NEAR(result.segment[1].a[1], 0.0, 1e-6) << "Segment 1 a Y mismatch";
    EXPECT_NEAR(result.segment[1].a[2], 0.0, 1e-6) << "Segment 1 a Z mismatch";
    
    // Expected segment 2 dt: [2.0017, 2.0015, 0]
    EXPECT_NEAR(result.segment[2].dt[0], 2.0017, 1e-3) << "Segment 2 dt X mismatch";
    EXPECT_NEAR(result.segment[2].dt[1], 2.0015, 1e-3) << "Segment 2 dt Y mismatch";
    EXPECT_NEAR(result.segment[2].dt[2], 0.0, 1e-6) << "Segment 2 dt Z mismatch";
    
    // Expected segment 2 t: [3.9034, 3.9029, 0] 
    EXPECT_NEAR(result.segment[2].t[0], 3.9034, 1e-3) << "Segment 2 t X mismatch";
    EXPECT_NEAR(result.segment[2].t[1], 3.9029, 1e-3) << "Segment 2 t Y mismatch";
    EXPECT_NEAR(result.segment[2].t[2], 0.0, 1e-6) << "Segment 2 t Z mismatch";
    
    // Expected segment 2 p: [0.2220e-15, 0, 0] (essentially zero)
    EXPECT_NEAR(result.segment[2].p[0], 0.0, 1e-12) << "Segment 2 p X mismatch";
    EXPECT_NEAR(result.segment[2].p[1], 0.0, 1e-12) << "Segment 2 p Y mismatch";
    EXPECT_NEAR(result.segment[2].p[2], 0.0, 1e-6) << "Segment 2 p Z mismatch";
    
    // Expected segment 2 v: [0, 0, 0]
    EXPECT_NEAR(result.segment[2].v[0], 0.0, 1e-6) << "Segment 2 v X mismatch";
    EXPECT_NEAR(result.segment[2].v[1], 0.0, 1e-6) << "Segment 2 v Y mismatch";
    EXPECT_NEAR(result.segment[2].v[2], 0.0, 1e-6) << "Segment 2 v Z mismatch";
    
    // Expected segment 2 a: [0.9983, -1.4978, 13.0000]
    EXPECT_NEAR(result.segment[2].a[0], 0.9983, 1e-3) << "Segment 2 a X mismatch";
    EXPECT_NEAR(result.segment[2].a[1], -1.4978, 1e-3) << "Segment 2 a Y mismatch";
    EXPECT_NEAR(result.segment[2].a[2], 13.0000, 1e-3) << "Segment 2 a Z mismatch";
    
    // Expected segment 3 dt: [1e10, 1e10, 1e10]
    EXPECT_NEAR(result.segment[3].dt[0], 1e10, 1e6) << "Segment 3 dt X mismatch";
    EXPECT_NEAR(result.segment[3].dt[1], 1e10, 1e6) << "Segment 3 dt Y mismatch";
    EXPECT_NEAR(result.segment[3].dt[2], 1e10, 1e6) << "Segment 3 dt Z mismatch";
    
    // Expected segment 3 t: [1e10, 1e10, 1e10]
    EXPECT_NEAR(result.segment[3].t[0], 1e10, 1e6) << "Segment 3 t X mismatch";
    EXPECT_NEAR(result.segment[3].t[1], 1e10, 1e6) << "Segment 3 t Y mismatch";
    EXPECT_NEAR(result.segment[3].t[2], 1e10, 1e6) << "Segment 3 t Z mismatch";
    
    // Expected segment 3 p: [0.2220e-15, 0, 0] (essentially zero)
    EXPECT_NEAR(result.segment[3].p[0], 0.0, 1e-12) << "Segment 3 p X mismatch";
    EXPECT_NEAR(result.segment[3].p[1], 0.0, 1e-12) << "Segment 3 p Y mismatch";
    EXPECT_NEAR(result.segment[3].p[2], 0.0, 1e-6) << "Segment 3 p Z mismatch";
    
    // Expected segment 3 v: [0, 0, 0]
    EXPECT_NEAR(result.segment[3].v[0], 0.0, 1e-6) << "Segment 3 v X mismatch";
    EXPECT_NEAR(result.segment[3].v[1], 0.0, 1e-6) << "Segment 3 v Y mismatch";
    EXPECT_NEAR(result.segment[3].v[2], 0.0, 1e-6) << "Segment 3 v Z mismatch";
    
    // Expected segment 3 a: [0, 0, 0]
    EXPECT_NEAR(result.segment[3].a[0], 0.0, 1e-6) << "Segment 3 a X mismatch";
    EXPECT_NEAR(result.segment[3].a[1], 0.0, 1e-6) << "Segment 3 a Y mismatch";
    EXPECT_NEAR(result.segment[3].a[2], 0.0, 1e-6) << "Segment 3 a Z mismatch";
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}