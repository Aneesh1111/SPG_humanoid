#include <gtest/gtest.h>
#include <vector>
#include <Eigen/Dense>
#include "spg/setpoint/Traj1.hpp"
#include "spg/setpoint/Segment.hpp"
#include "spg/setpoint/CombineSegmentData.hpp"
#include "spg/Init.hpp"

class Traj1Test : public ::testing::Test {
protected:
    void SetUp() override {
        // Create a simple 3-segment trajectory
        segments.resize(3);
        
        // Segment 0: X dimension
        segments[0].t = Eigen::Vector3d(0.5, 1.0, 1.5);  // time intervals
        segments[0].p = Eigen::Vector3d(0, 1, 2);         // positions
        segments[0].v = Eigen::Vector3d(0, 1, 0);         // velocities
        segments[0].a = Eigen::Vector3d(2, -2, 0);        // accelerations
        segments[0].dt = Eigen::Vector3d(0.5, 0.5, 0.5);  // time deltas
        
        // Segment 1: Y dimension
        segments[1].t = Eigen::Vector3d(0.3, 0.8, 1.2);
        segments[1].p = Eigen::Vector3d(0, 0.5, 1);
        segments[1].v = Eigen::Vector3d(0, 0.5, 0);
        segments[1].a = Eigen::Vector3d(1, -1, 0);
        segments[1].dt = Eigen::Vector3d(0.3, 0.5, 0.4);
        
        // Segment 2: Z dimension (rotation)
        segments[2].t = Eigen::Vector3d(0.4, 0.9, 1.3);
        segments[2].p = Eigen::Vector3d(0, 0.2, 0.4);
        segments[2].v = Eigen::Vector3d(0, 0.2, 0);
        segments[2].a = Eigen::Vector3d(0.5, -0.5, 0);
        segments[2].dt = Eigen::Vector3d(0.4, 0.5, 0.4);
    }
    
    std::vector<spg::setpoint::Segment> segments;
    spg::Traject traject;
};

TEST_F(Traj1Test, BasicFunctionality) {
    double Ts = 0.5;  // Sample at 0.5 seconds

    spg::setpoint::Traj1(traject, segments, Ts);
    
    // Check that trajectory has been populated
    EXPECT_EQ(traject.t.size(), 1);
    EXPECT_EQ(traject.p.size(), 1);
    EXPECT_EQ(traject.v.size(), 1);
    EXPECT_EQ(traject.a.size(), 1);
    EXPECT_EQ(traject.segment_id.size(), 1);
    
    // Check time value
    EXPECT_DOUBLE_EQ(traject.t[0], Ts);
    
    // Check that position, velocity, and acceleration vectors are 3D
    EXPECT_EQ(traject.p[0].size(), 3);
    EXPECT_EQ(traject.v[0].size(), 3);
    EXPECT_EQ(traject.a[0].size(), 3);
    EXPECT_EQ(traject.segment_id[0].size(), 3);
}

TEST_F(Traj1Test, MatlabSegmentData) {
    // Clear existing segments and create segments with MATLAB data
    segments.clear();
    segments.resize(4);
    
    // Segment 0 (from d.aux.segment(1))
    segments[0].dt = Eigen::Vector3d(1.1750, 0, 0);
    segments[0].t = Eigen::Vector3d(1.1750, 0, 0);
    segments[0].p = Eigen::Vector3d(1.7574, 0, 0);
    segments[0].v = Eigen::Vector3d(-2.1151, 0, 0);
    segments[0].a = Eigen::Vector3d(-1.8000, 1.8000, 13.0000);
    
    // Segment 1 (from d.aux.segment(2))
    segments[1].dt = Eigen::Vector3d(0, 0, 0);
    segments[1].t = Eigen::Vector3d(1.1750, 0, 0);
    segments[1].p = Eigen::Vector3d(1.7574, 0, 0);
    segments[1].v = Eigen::Vector3d(-2.1151, 0, 0);
    segments[1].a = Eigen::Vector3d(0, 0, 0);
    
    // Segment 2 (from d.aux.segment(3))
    segments[2].dt = Eigen::Vector3d(1.1750, 0, 0);
    segments[2].t = Eigen::Vector3d(2.3501, 0, 0);
    segments[2].p = Eigen::Vector3d(0.5147, 0, 0);
    segments[2].v = Eigen::Vector3d(0, 0, 0);
    segments[2].a = Eigen::Vector3d(1.8000, 1.8000, 13.0000);
    
    // Segment 3 (from d.aux.segment(4))
    segments[3].dt = Eigen::Vector3d(1.0e+10, 1.0e+10, 1.0e+10);
    segments[3].t = Eigen::Vector3d(1.0e+10, 1.0e+10, 1.0e+10);
    segments[3].p = Eigen::Vector3d(0.5147, 0, 0);
    segments[3].v = Eigen::Vector3d(0, 0, 0);
    segments[3].a = Eigen::Vector3d(0, 0, 0);
    
    // Test at time Ts = 0.02 seconds (matching MATLAB d.traj.t(1))
    double Ts = 0.02;
    spg::setpoint::Traj1(traject, segments, Ts);
    
    // Verify trajectory was populated
    EXPECT_EQ(traject.t.size(), 1);
    EXPECT_EQ(traject.p.size(), 1);
    EXPECT_EQ(traject.v.size(), 1);
    EXPECT_EQ(traject.a.size(), 1);
    EXPECT_EQ(traject.segment_id.size(), 1);
    
    // Validate specific output values against MATLAB results
    // Time should match input
    EXPECT_NEAR(traject.t[0], 0.02, 1e-4);
    
    // Position should match d.traj.p(1,:) = [2.9996, 0, 0]
    EXPECT_NEAR(traject.p[0][0], 2.9996, 1e-3);
    EXPECT_NEAR(traject.p[0][1], 0.0, 1e-3);
    EXPECT_NEAR(traject.p[0][2], 0.0, 1e-3);
    
    // Velocity should match d.traj.v(1,:) = [-0.0360, 0, 0]
    EXPECT_NEAR(traject.v[0][0], -0.0360, 1e-3);
    EXPECT_NEAR(traject.v[0][1], 0.0, 1e-3);
    EXPECT_NEAR(traject.v[0][2], 0.0, 1e-3);

    // Acceleration should match d.traj.a(1,:) = [-1.8000, 0, 0]
    EXPECT_NEAR(traject.a[0][0], -1.8000, 1e-3);
    EXPECT_NEAR(traject.a[0][1], 0.0, 1e-3);
    EXPECT_NEAR(traject.a[0][2], 0.0, 1e-3);

    // Segment ID should match d.traj.segment_id(1,:) = [0, 3, 3]
    EXPECT_NEAR(traject.segment_id[0][0], 0, 1e-4);
    EXPECT_NEAR(traject.segment_id[0][1], 3, 1e-4);
    EXPECT_NEAR(traject.segment_id[0][2], 3, 1e-4);

}

TEST_F(Traj1Test, DebugMatlabSegmentData) {
    // Create the exact MATLAB segment data
    segments.resize(4);
    
    // From MATLAB d.aux.segment data
    // Segment 0: dt=[1.1750 0 0], t=[1.1750 0 0], p=[1.7574 0 0], v=[-2.1151 0 0], a=[-1.8000 1.8000 13.0000]
    segments[0].dt = Eigen::Vector3d(1.1750, 0, 0);
    segments[0].t = Eigen::Vector3d(1.1750, 0, 0);
    segments[0].p = Eigen::Vector3d(1.7574, 0, 0);
    segments[0].v = Eigen::Vector3d(-2.1151, 0, 0);
    segments[0].a = Eigen::Vector3d(-1.8000, 1.8000, 13.0000);
    
    // Segment 1: dt=[0 0 0], t=[1.1750 0 0], p=[1.7574 0 0], v=[-2.1151 0 0], a=[0 0 0]
    segments[1].dt = Eigen::Vector3d(0, 0, 0);
    segments[1].t = Eigen::Vector3d(1.1750, 0, 0);
    segments[1].p = Eigen::Vector3d(1.7574, 0, 0);
    segments[1].v = Eigen::Vector3d(-2.1151, 0, 0);
    segments[1].a = Eigen::Vector3d(0, 0, 0);
    
    // Segment 2: dt=[1.1750 0 0], t=[2.3501 0 0], p=[0.5147 0 0], v=[0 0 0], a=[1.8000 1.8000 13.0000]
    segments[2].dt = Eigen::Vector3d(1.1750, 0, 0);
    segments[2].t = Eigen::Vector3d(2.3501, 0, 0);
    segments[2].p = Eigen::Vector3d(0.5147, 0, 0);
    segments[2].v = Eigen::Vector3d(0, 0, 0);
    segments[2].a = Eigen::Vector3d(1.8000, 1.8000, 13.0000);
    
    // Segment 3: dt=[1e10 1e10 1e10], t=[1e10 1e10 1e10], p=[0.5147 0 0], v=[0 0 0], a=[0 0 0]
    segments[3].dt = Eigen::Vector3d(1e10, 1e10, 1e10);
    segments[3].t = Eigen::Vector3d(1e10, 1e10, 1e10);
    segments[3].p = Eigen::Vector3d(0.5147, 0, 0);
    segments[3].v = Eigen::Vector3d(0, 0, 0);
    segments[3].a = Eigen::Vector3d(0, 0, 0);
    
    double Ts = 0.02;  // MATLAB time sample
    
    spg::setpoint::Traj1(traject, segments, Ts);
    
    // Basic checks to ensure it runs without crashing
    EXPECT_EQ(traject.segment_id.size(), 1);
    
    // Expected from MATLAB:
    std::cout << "Position: [2.9996, 0, 0]" << std::endl;
    std::cout << "Velocity: [-0.0360, 0, 0]" << std::endl;
    std::cout << "Acceleration: [-1.8000, 0, 0]" << std::endl;
    std::cout << "Segment ID: [0, 3, 3]" << std::endl;
    
    // Basic checks to ensure it runs without crashing
    EXPECT_EQ(traject.t.size(), 1);
    EXPECT_EQ(traject.p.size(), 1);
    EXPECT_EQ(traject.v.size(), 1);
    EXPECT_EQ(traject.a.size(), 1);
    EXPECT_EQ(traject.segment_id.size(), 1);
}

TEST_F(Traj1Test, MatlabSegmentData2) {
    // Clear existing segments and create segments with MATLAB data
    segments.clear();
    segments.resize(4);
    
    // Segment 0 (from d.aux.segment(1))
    segments[0].dt = Eigen::Vector3d(0.0016, 0, 0);
    segments[0].t = Eigen::Vector3d(0.0016, 0, 0);
    segments[0].p = Eigen::Vector3d(1.4756, 0, 0);
    segments[0].v = Eigen::Vector3d(-2.3036, 0, 0);
    segments[0].a = Eigen::Vector3d(-1.8000, 1.8000, 13.0000);
    
    // Segment 1 (from d.aux.segment(2))
    segments[1].dt = Eigen::Vector3d(0, 0, 0);
    segments[1].t = Eigen::Vector3d(0.0016, 0, 0);
    segments[1].p = Eigen::Vector3d(1.4756, 0, 0);
    segments[1].v = Eigen::Vector3d(-2.3036, 0, 0);
    segments[1].a = Eigen::Vector3d(0, 0, 0);
    
    // Segment 2 (from d.aux.segment(3))
    segments[2].dt = Eigen::Vector3d(1.2798, 0, 0);
    segments[2].t = Eigen::Vector3d(1.2814, 0, 0);
    segments[2].p = Eigen::Vector3d(0.0015, 0, 0);
    segments[2].v = Eigen::Vector3d(0, 0, 0);
    segments[2].a = Eigen::Vector3d(1.8000, 1.8000, 13.0000);
    
    // Segment 3 (from d.aux.segment(4))
    segments[3].dt = Eigen::Vector3d(1.0e+10, 1.0e+10, 1.0e+10);
    segments[3].t = Eigen::Vector3d(1.0e+10, 1.0e+10, 1.0e+10);
    segments[3].p = Eigen::Vector3d(0.0015, 0, 0);
    segments[3].v = Eigen::Vector3d(0, 0, 0);
    segments[3].a = Eigen::Vector3d(0, 0, 0);
    
    // Test at time Ts = 0.02 seconds (matching MATLAB d.traj.t(1))
    double Ts = 0.02;
    spg::setpoint::Traj1(traject, segments, Ts);
    
    // Verify trajectory was populated
    EXPECT_EQ(traject.t.size(), 1);
    EXPECT_EQ(traject.p.size(), 1);
    EXPECT_EQ(traject.v.size(), 1);
    EXPECT_EQ(traject.a.size(), 1);
    EXPECT_EQ(traject.segment_id.size(), 1);
    
    // Validate specific output values against MATLAB results
    // Time should match input
    EXPECT_NEAR(traject.t[0], 0.02, 1e-4);
    
    // Position should match d.traj.p(1,:) = [1.4334, 0, 0]
    EXPECT_NEAR(traject.p[0][0], 1.4334, 1e-3);
    EXPECT_NEAR(traject.p[0][1], 0.0, 1e-3);
    EXPECT_NEAR(traject.p[0][2], 0.0, 1e-3);
    
    // Velocity should match d.traj.v(1,:) = [-2.2705, 0, 0]
    EXPECT_NEAR(traject.v[0][0], -2.2705, 1e-3);
    EXPECT_NEAR(traject.v[0][1], 0.0, 1e-3);
    EXPECT_NEAR(traject.v[0][2], 0.0, 1e-3);

    // Acceleration should match d.traj.a(1,:) = [1.8000, 0, 0]
    EXPECT_NEAR(traject.a[0][0], 1.8000, 1e-3);
    EXPECT_NEAR(traject.a[0][1], 0.0, 1e-3);
    EXPECT_NEAR(traject.a[0][2], 0.0, 1e-3);

    // Segment ID should match d.traj.segment_id(1,:) = [2, 3, 3]
    EXPECT_NEAR(traject.segment_id[0][0], 2, 1e-4);
    EXPECT_NEAR(traject.segment_id[0][1], 3, 1e-4);
    EXPECT_NEAR(traject.segment_id[0][2], 3, 1e-4);

}

TEST_F(Traj1Test, MatlabBoundaryConditionsTest) {
    // Test edge cases and boundary conditions
    std::vector<spg::setpoint::Segment> segments(4);
    
    // Very short first segment
    segments[0].dt = Eigen::Vector3d(0.001, 0.002, 0.003);
    segments[0].t = Eigen::Vector3d(0.001, 0.002, 0.003);
    segments[0].p = Eigen::Vector3d(0.5, -0.3, 0.1);
    segments[0].v = Eigen::Vector3d(-2.0, 1.5, -0.5);
    segments[0].a = Eigen::Vector3d(-3.0, 2.0, 10.0);
    
    // Zero duration segment
    segments[1].dt = Eigen::Vector3d(0, 0, 0);
    segments[1].t = Eigen::Vector3d(0.001, 0.002, 0.003);
    segments[1].p = Eigen::Vector3d(0.5, -0.3, 0.1);
    segments[1].v = Eigen::Vector3d(-2.0, 1.5, -0.5);
    segments[1].a = Eigen::Vector3d(0, 0, 0);
    
    // Normal second segment
    segments[2].dt = Eigen::Vector3d(1.0, 1.0, 1.0);
    segments[2].t = Eigen::Vector3d(1.001, 1.002, 1.003);
    segments[2].p = Eigen::Vector3d(0.0, 0.0, 0.0);
    segments[2].v = Eigen::Vector3d(0, 0, 0);
    segments[2].a = Eigen::Vector3d(2.0, -1.5, -10.0);
    
    // Final segment
    segments[3].dt = Eigen::Vector3d(1e10, 1e10, 1e10);
    segments[3].t = Eigen::Vector3d(1e10, 1e10, 1e10);
    segments[3].p = Eigen::Vector3d(0.0, 0.0, 0.0);
    segments[3].v = Eigen::Vector3d(0, 0, 0);
    segments[3].a = Eigen::Vector3d(0, 0, 0);
    
    spg::Traject traject;
    
    // Test at very early time (should be in segment 0)
    spg::setpoint::Traj1(traject, segments, 0.0005);
    EXPECT_EQ(traject.segment_id[0][0], 0);
    EXPECT_EQ(traject.segment_id[0][1], 0);
    EXPECT_EQ(traject.segment_id[0][2], 0);
    
    // Test at time just after first segment ends for X
    spg::setpoint::Traj1(traject, segments, 0.0015);
    EXPECT_EQ(traject.segment_id[0][0], 2);  // X should be in segment 2
    EXPECT_EQ(traject.segment_id[0][1], 0);  // Y still in segment 0
    EXPECT_EQ(traject.segment_id[0][2], 0);  // Z still in segment 0
    
    // Test at time when all dimensions have moved to segment 2
    spg::setpoint::Traj1(traject, segments, 0.005);
    EXPECT_EQ(traject.segment_id[0][0], 2);  // X in segment 2
    EXPECT_EQ(traject.segment_id[0][1], 2);  // Y in segment 2
    EXPECT_EQ(traject.segment_id[0][2], 2);  // Z in segment 2
    
    // Test at very large time (should be in final segment)
    spg::setpoint::Traj1(traject, segments, 10.0);
    EXPECT_EQ(traject.segment_id[0][0], 3);  // All dimensions in final segment
    EXPECT_EQ(traject.segment_id[0][1], 3);
    EXPECT_EQ(traject.segment_id[0][2], 3);
}

TEST_F(Traj1Test, MatlabInputsOutputsTest2) {
    // Second MATLAB test case: d.aux.segment with different values
    std::vector<spg::setpoint::Segment> segments(4);
    
    // Input segment[0]: dt=[1.7217, 1.7215, 0], t=[1.7217, 1.7215, 0], p=[2.0000, -3.0000, 0], v=[-1.9983, 2.9978, 0], a=[-0.9983, 1.4978, 13.0000]
    segments[0].dt = Eigen::Vector3d(1.7217, 1.7215, 0);
    segments[0].t = Eigen::Vector3d(1.7217, 1.7215, 0);
    segments[0].p = Eigen::Vector3d(2.0000, -3.0000, 0);
    segments[0].v = Eigen::Vector3d(-1.9983, 2.9978, 0);
    segments[0].a = Eigen::Vector3d(-0.9983, 1.4978, 13.0000);
    
    // Input segment[1]: dt=[0, 0, 0], t=[1.7217, 1.7215, 0], p=[2.0000, -3.0000, 0], v=[-1.9983, 2.9978, 0], a=[0, 0, 0]
    segments[1].dt = Eigen::Vector3d(0, 0, 0);
    segments[1].t = Eigen::Vector3d(1.7217, 1.7215, 0);
    segments[1].p = Eigen::Vector3d(2.0000, -3.0000, 0);
    segments[1].v = Eigen::Vector3d(-1.9983, 2.9978, 0);
    segments[1].a = Eigen::Vector3d(0, 0, 0);
    
    // Input segment[2]: dt=[2.0017, 2.0015, 0], t=[3.7234, 3.7229, 0], p=[-0.0888e-14, -0.1332e-14, 0], v=[0, 0, 0], a=[0.9983, -1.4978, 13.0000]
    segments[2].dt = Eigen::Vector3d(2.0017, 2.0015, 0);
    segments[2].t = Eigen::Vector3d(3.7234, 3.7229, 0);
    segments[2].p = Eigen::Vector3d(-0.0888e-14, -0.1332e-14, 0);
    segments[2].v = Eigen::Vector3d(0, 0, 0);
    segments[2].a = Eigen::Vector3d(0.9983, -1.4978, 13.0000);
    
    // Input segment[3]: dt=[1e10, 1e10, 1e10], t=[1e10, 1e10, 1e10], p=[-0.0888e-14, -0.1332e-14, 0], v=[0, 0, 0], a=[0, 0, 0]
    segments[3].dt = Eigen::Vector3d(1e10, 1e10, 1e10);
    segments[3].t = Eigen::Vector3d(1e10, 1e10, 1e10);
    segments[3].p = Eigen::Vector3d(-0.0888e-14, -0.1332e-14, 0);
    segments[3].v = Eigen::Vector3d(0, 0, 0);
    segments[3].a = Eigen::Vector3d(0, 0, 0);
    
    // Initialize trajectory with 20 timesteps (npredict=20)
    spg::Traject traject;
    traject.p = std::vector<Eigen::Vector3d>(20, Eigen::Vector3d::Zero());
    traject.v = std::vector<Eigen::Vector3d>(20, Eigen::Vector3d::Zero());
    traject.a = std::vector<Eigen::Vector3d>(20, Eigen::Vector3d::Zero());
    traject.t = std::vector<double>(20, 0.0);
    traject.segment_id = std::vector<Eigen::Vector3i>(20, Eigen::Vector3i::Zero());
    
    double Ts = 0.02; // Sampling time from MATLAB (t[0] = 0.02)
    
    // Call Traj1 function
    spg::setpoint::Traj1(traject, segments, Ts);
    
    // Expected outputs from MATLAB:
    // d.traj.p[0] = [3.9551, -5.9326, 0], rest are [0, 0, 0]
    EXPECT_NEAR(traject.p[0][0], 3.9551, 1e-3);
    EXPECT_NEAR(traject.p[0][1], -5.9326, 1e-3);
    EXPECT_NEAR(traject.p[0][2], 0.0, 1e-3);
    
    // All other positions should be [0, 0, 0]
    for (int i = 1; i < 20; ++i) {
        EXPECT_NEAR(traject.p[i][0], 0.0, 1e-10);
        EXPECT_NEAR(traject.p[i][1], 0.0, 1e-10);
        EXPECT_NEAR(traject.p[i][2], 0.0, 1e-10);
    }
    
    // d.traj.v[0] = [-0.2995, 0.4493, 0], rest are [0, 0, 0]
    EXPECT_NEAR(traject.v[0][0], -0.2995, 1e-3);
    EXPECT_NEAR(traject.v[0][1], 0.4493, 1e-3);
    EXPECT_NEAR(traject.v[0][2], 0.0, 1e-3);
    
    // All other velocities should be [0, 0, 0]
    for (int i = 1; i < 20; ++i) {
        EXPECT_NEAR(traject.v[i][0], 0.0, 1e-10);
        EXPECT_NEAR(traject.v[i][1], 0.0, 1e-10);
        EXPECT_NEAR(traject.v[i][2], 0.0, 1e-10);
    }
    
    // d.traj.a[0] = [-0.9983, 1.4978, 0], rest are [0, 0, 0]
    EXPECT_NEAR(traject.a[0][0], -0.9983, 1e-3);
    EXPECT_NEAR(traject.a[0][1], 1.4978, 1e-3);
    EXPECT_NEAR(traject.a[0][2], 0.0, 1e-3);
    
    // All other accelerations should be [0, 0, 0]
    for (int i = 1; i < 20; ++i) {
        EXPECT_NEAR(traject.a[i][0], 0.0, 1e-10);
        EXPECT_NEAR(traject.a[i][1], 0.0, 1e-10);
        EXPECT_NEAR(traject.a[i][2], 0.0, 1e-10);
    }
    
    // d.traj.t[0] = 0.0200, rest are 0
    EXPECT_NEAR(traject.t[0], 0.0200, 1e-3);
    for (int i = 1; i < 20; ++i) {
        EXPECT_NEAR(traject.t[i], 0.0, 1e-10);
    }
    
    // d.traj.segment_id[0] = [0, 0, 3], rest are [0, 0, 0]
    EXPECT_EQ(traject.segment_id[0][0], 0);
    EXPECT_EQ(traject.segment_id[0][1], 0);
    EXPECT_EQ(traject.segment_id[0][2], 3);
    
    // All other segment IDs should be [0, 0, 0]
    for (int i = 1; i < 20; ++i) {
        EXPECT_EQ(traject.segment_id[i][0], 0);
        EXPECT_EQ(traject.segment_id[i][1], 0);
        EXPECT_EQ(traject.segment_id[i][2], 0);
    }
}

TEST_F(Traj1Test, MatlabInputsOutputsTest3) {
    // MATLAB inputs: d.aux.segment with new timing values
    std::vector<spg::setpoint::Segment> segments(4);
    
    // Input segment[0]: dt=[0.0217, 0.0215, 0], t=[0.0217, 0.0215, 0], p=[2.0000, -3.0000, 0], v=[-1.9983, 2.9978, 0], a=[-0.9983, 1.4978, 13.0000]
    segments[0].dt = Eigen::Vector3d(0.0217, 0.0215, 0);
    segments[0].t = Eigen::Vector3d(0.0217, 0.0215, 0);
    segments[0].p = Eigen::Vector3d(2.0000, -3.0000, 0);
    segments[0].v = Eigen::Vector3d(-1.9983, 2.9978, 0);
    segments[0].a = Eigen::Vector3d(-0.9983, 1.4978, 13.0000);
    
    // Input segment[1]: dt=[0, 0, 0], t=[0.0217, 0.0215, 0], p=[2.0000, -3.0000, 0], v=[-1.9983, 2.9978, 0], a=[0, 0, 0]
    segments[1].dt = Eigen::Vector3d(0, 0, 0);
    segments[1].t = Eigen::Vector3d(0.0217, 0.0215, 0);
    segments[1].p = Eigen::Vector3d(2.0000, -3.0000, 0);
    segments[1].v = Eigen::Vector3d(-1.9983, 2.9978, 0);
    segments[1].a = Eigen::Vector3d(0, 0, 0);
    
    // Input segment[2]: dt=[2.0017, 2.0015, 0], t=[2.0234, 2.0229, 0], p=[0.2220e-15, -0.8882e-15, 0], v=[0, 0, 0], a=[0.9983, -1.4978, 13.0000]
    segments[2].dt = Eigen::Vector3d(2.0017, 2.0015, 0);
    segments[2].t = Eigen::Vector3d(2.0234, 2.0229, 0);
    segments[2].p = Eigen::Vector3d(0.2220e-15, -0.8882e-15, 0);
    segments[2].v = Eigen::Vector3d(0, 0, 0);
    segments[2].a = Eigen::Vector3d(0.9983, -1.4978, 13.0000);
    
    // Input segment[3]: dt=[1e10, 1e10, 1e10], t=[1e10, 1e10, 1e10], p=[0.2220e-15, -0.8882e-15, 0], v=[0, 0, 0], a=[0, 0, 0]
    segments[3].dt = Eigen::Vector3d(1e10, 1e10, 1e10);
    segments[3].t = Eigen::Vector3d(1e10, 1e10, 1e10);
    segments[3].p = Eigen::Vector3d(0.2220e-15, -0.8882e-15, 0);
    segments[3].v = Eigen::Vector3d(0, 0, 0);
    segments[3].a = Eigen::Vector3d(0, 0, 0);
    
    // Initialize trajectory with 20 timesteps (npredict=20)
    spg::Traject traject;
    traject.p = std::vector<Eigen::Vector3d>(20, Eigen::Vector3d::Zero());
    traject.v = std::vector<Eigen::Vector3d>(20, Eigen::Vector3d::Zero());
    traject.a = std::vector<Eigen::Vector3d>(20, Eigen::Vector3d::Zero());
    traject.t = std::vector<double>(20, 0.0);
    traject.segment_id = std::vector<Eigen::Vector3i>(20, Eigen::Vector3i::Zero());
    
    double Ts = 0.02; // Sampling time from MATLAB (t[0] = 0.02)
    
    // Call Traj1 function
    spg::setpoint::Traj1(traject, segments, Ts);
    
    // Expected outputs from MATLAB:
    // d.traj.p[0] = [2.0034, -3.0044, 0], rest are [0, 0, 0]
    EXPECT_NEAR(traject.p[0][0], 2.0034, 1e-3);
    EXPECT_NEAR(traject.p[0][1], -3.0044, 1e-3);
    EXPECT_NEAR(traject.p[0][2], 0.0, 1e-3);
    
    // All other positions should be [0, 0, 0]
    for (int i = 1; i < 20; ++i) {
        EXPECT_NEAR(traject.p[i][0], 0.0, 1e-10);
        EXPECT_NEAR(traject.p[i][1], 0.0, 1e-10);
        EXPECT_NEAR(traject.p[i][2], 0.0, 1e-10);
    }
    
    // d.traj.v[0] = [-1.9966, 2.9956, 0], rest are [0, 0, 0]
    EXPECT_NEAR(traject.v[0][0], -1.9966, 1e-3);
    EXPECT_NEAR(traject.v[0][1], 2.9956, 1e-3);
    EXPECT_NEAR(traject.v[0][2], 0.0, 1e-3);
    
    // All other velocities should be [0, 0, 0]
    for (int i = 1; i < 20; ++i) {
        EXPECT_NEAR(traject.v[i][0], 0.0, 1e-10);
        EXPECT_NEAR(traject.v[i][1], 0.0, 1e-10);
        EXPECT_NEAR(traject.v[i][2], 0.0, 1e-10);
    }
    
    // d.traj.a[0] = [-0.9983, 1.4978, 0], rest are [0, 0, 0]
    EXPECT_NEAR(traject.a[0][0], -0.9983, 1e-3);
    EXPECT_NEAR(traject.a[0][1], 1.4978, 1e-3);
    EXPECT_NEAR(traject.a[0][2], 0.0, 1e-3);
    
    // All other accelerations should be [0, 0, 0]
    for (int i = 1; i < 20; ++i) {
        EXPECT_NEAR(traject.a[i][0], 0.0, 1e-10);
        EXPECT_NEAR(traject.a[i][1], 0.0, 1e-10);
        EXPECT_NEAR(traject.a[i][2], 0.0, 1e-10);
    }
    
    // d.traj.t[0] = 0.0200, rest are 0
    EXPECT_NEAR(traject.t[0], 0.0200, 1e-3);
    for (int i = 1; i < 20; ++i) {
        EXPECT_NEAR(traject.t[i], 0.0, 1e-10);
    }
    
    // d.traj.segment_id[0] = [0, 0, 3], rest are [0, 0, 0]
    EXPECT_EQ(traject.segment_id[0][0], 0);
    EXPECT_EQ(traject.segment_id[0][1], 0);
    EXPECT_EQ(traject.segment_id[0][2], 3);
    
    // All other segment IDs should be [0, 0, 0]
    for (int i = 1; i < 20; ++i) {
        EXPECT_EQ(traject.segment_id[i][0], 0);
        EXPECT_EQ(traject.segment_id[i][1], 0);
        EXPECT_EQ(traject.segment_id[i][2], 0);
    }
}


