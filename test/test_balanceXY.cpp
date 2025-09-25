#include <gtest/gtest.h>
#include <vector>
#include <Eigen/Dense>
#include "spg/setpoint/BalanceXY.hpp"
#include "spg/setpoint/Segment.hpp"

class BalanceXYTest : public ::testing::Test {
protected:
    void SetUp() override {
        segments.resize(4);
    }
    
    std::vector<spg::setpoint::Segment> segments;
};

TEST_F(BalanceXYTest, OutputVectorsAreSet) {
    Eigen::Vector3d p0(0, 0, 0), v0(0, 0, 0), pe(1, 1, 0), ve(0, 0, 0);
    Eigen::Vector3d vm(1, 1, 1), am(1, 1, 1), dm(1, 1, 1);
    Eigen::Vector3d vmax, amax, dmax;

    spg::setpoint::balanceXY(segments, p0, v0, pe, ve, vm, am, dm, vmax, amax, dmax);

    // Check that output vectors are not all zeros
    EXPECT_NE(vmax.norm(), 0);
    EXPECT_NE(amax.norm(), 0);
    EXPECT_NE(dmax.norm(), 0);
}

// TEST_F(BalanceXYTest, ZeroDistanceCase) {
//     // Test when pe == p0 (no movement needed)
//     Eigen::Vector3d p0(5, 5, 0), v0(0, 0, 0), pe(5, 5, 0), ve(0, 0, 0);
//     Eigen::Vector3d vm(2, 2, 2), am(1, 1, 1), dm(1, 1, 1);
//     Eigen::Vector3d vmax, amax, dmax;

//     spg::setpoint::balanceXY(segments, p0, v0, pe, ve, vm, am, dm, vmax, amax, dmax);

//     // When distance is very small, should return original values
//     EXPECT_NEAR(vmax[0], vm[0], 1e-6);
//     EXPECT_NEAR(vmax[1], vm[1], 1e-6);
//     EXPECT_NEAR(vmax[2], vm[2], 1e-6);
//     EXPECT_NEAR(amax[0], am[0], 1e-6);
//     EXPECT_NEAR(amax[1], am[1], 1e-6);
//     EXPECT_NEAR(amax[2], am[2], 1e-6);
//     EXPECT_NEAR(dmax[0], dm[0], 1e-6);
//     EXPECT_NEAR(dmax[1], dm[1], 1e-6);
//     EXPECT_NEAR(dmax[2], dm[2], 1e-6);
// }

// TEST_F(BalanceXYTest, SimpleMovement) {
//     // Test simple movement from origin to (2, 2, 0)
//     Eigen::Vector3d p0(0, 0, 0), v0(0, 0, 0), pe(2, 2, 0), ve(0, 0, 0);
//     Eigen::Vector3d vm(3, 3, 3), am(2, 2, 2), dm(2, 2, 2);
//     Eigen::Vector3d vmax, amax, dmax;

//     spg::setpoint::balanceXY(segments, p0, v0, pe, ve, vm, am, dm, vmax, amax, dmax);

//     // Check that output values are reasonable
//     EXPECT_GT(vmax[0], 0);  // Should have positive X velocity limit
//     EXPECT_GT(vmax[1], 0);  // Should have positive Y velocity limit
//     EXPECT_EQ(vmax[2], vm[2]);  // Z should remain unchanged
    
//     EXPECT_GT(amax[0], 0);  // Should have positive X acceleration limit
//     EXPECT_GT(amax[1], 0);  // Should have positive Y acceleration limit
//     EXPECT_EQ(amax[2], am[2]);  // Z should remain unchanged
    
//     EXPECT_GT(dmax[0], 0);  // Should have positive X deceleration limit
//     EXPECT_GT(dmax[1], 0);  // Should have positive Y deceleration limit
//     EXPECT_EQ(dmax[2], dm[2]);  // Z should remain unchanged
    
//     // The balanced values should not exceed the original maximums
//     EXPECT_LE(vmax[0], vm[0]);
//     EXPECT_LE(vmax[1], vm[1]);
//     EXPECT_LE(amax[0], am[0]);
//     EXPECT_LE(amax[1], am[1]);
//     EXPECT_LE(dmax[0], dm[0]);
//     EXPECT_LE(dmax[1], dm[1]);
// }

// TEST_F(BalanceXYTest, AsymmetricMovement) {
//     // Test movement where X and Y distances are very different
//     Eigen::Vector3d p0(0, 0, 0), v0(0, 0, 0), pe(10, 1, 0), ve(0, 0, 0);
//     Eigen::Vector3d vm(5, 5, 5), am(3, 3, 3), dm(3, 3, 3);
//     Eigen::Vector3d vmax, amax, dmax;

//     spg::setpoint::balanceXY(segments, p0, v0, pe, ve, vm, am, dm, vmax, amax, dmax);

//     // For asymmetric movement, the balancing should adjust X and Y differently
//     // X has larger distance, so might need higher limits
//     EXPECT_GT(vmax[0], 0);
//     EXPECT_GT(vmax[1], 0);
    
//     // The ratio of vmax should roughly correspond to the distance ratio
//     double distance_ratio = std::abs(pe[0]) / std::abs(pe[1]);  // 10/1 = 10
//     double vmax_ratio = vmax[0] / vmax[1];
    
//     // The velocity ratio should be related to distance ratio (not necessarily equal due to balancing)
//     EXPECT_GT(vmax_ratio, 1.0);  // X should get higher velocity limit than Y
// }

// TEST_F(BalanceXYTest, WithInitialVelocity) {
//     // Test with non-zero initial velocity
//     Eigen::Vector3d p0(0, 0, 0), v0(1, -0.5, 0), pe(3, 3, 0), ve(0, 0, 0);
//     Eigen::Vector3d vm(4, 4, 4), am(2, 2, 2), dm(2, 2, 2);
//     Eigen::Vector3d vmax, amax, dmax;

//     spg::setpoint::balanceXY(segments, p0, v0, pe, ve, vm, am, dm, vmax, amax, dmax);

//     // Should still produce valid outputs
//     EXPECT_GT(vmax.norm(), 0);
//     EXPECT_GT(amax.norm(), 0);
//     EXPECT_GT(dmax.norm(), 0);
    
//     // Values should not exceed input maximums
//     EXPECT_LE(vmax[0], vm[0]);
//     EXPECT_LE(vmax[1], vm[1]);
//     EXPECT_EQ(vmax[2], vm[2]);
// }

TEST_F(BalanceXYTest, SpecificExpectedValues) {
    // Test with specific inputs and expected outputs from MATLAB
    Eigen::Vector3d p0(0, 0, 0), v0(1, 1, 0), pe(0, 4, 0), ve(0, 0, 0);
    Eigen::Vector3d vm(3, 3, 3), am(2, 2, 2), dm(2, 2, 2);
    Eigen::Vector3d vmax, amax, dmax;

    // Initialize 4 segments with all zero vectors
    std::vector<spg::setpoint::Segment> segments(4, spg::setpoint::Segment{
        Eigen::Vector3d::Zero(), // dt
        Eigen::Vector3d::Zero(), // t
        Eigen::Vector3d::Zero(), // p
        Eigen::Vector3d::Zero(), // v
        Eigen::Vector3d::Zero()  // a
    });

    spg::setpoint::balanceXY(segments, p0, v0, pe, ve, vm, am, dm, vmax, amax, dmax);

    // Expected output values vmax, amax, dmax from MATLAB
    EXPECT_NEAR(vmax[0], 1.6790, 1e-4);
    EXPECT_NEAR(vmax[1], 3.8963, 1e-4);
    EXPECT_NEAR(vmax[2], 3.0000, 1e-4);
    EXPECT_NEAR(amax[0], 1.1194, 1e-4);
    EXPECT_NEAR(amax[1], 2.5975, 1e-4);
    EXPECT_NEAR(amax[2], 2.0000, 1e-4);
    EXPECT_NEAR(dmax[0], 1.1194, 1e-4);
    EXPECT_NEAR(dmax[1], 2.5975, 1e-4);
    EXPECT_NEAR(dmax[2], 2.0000, 1e-4);

    // Expected segment values from MATLAB
    // Segment 1
    EXPECT_NEAR(segments[0].dt[0], 1.5251, 1e-4);
    EXPECT_NEAR(segments[0].dt[1], 0.8855, 1e-4);
    EXPECT_NEAR(segments[0].dt[2], 0, 1e-6);
    EXPECT_NEAR(segments[0].t[0], 1.5251, 1e-4);
    EXPECT_NEAR(segments[0].t[1], 0.8855, 1e-4);
    EXPECT_NEAR(segments[0].t[2], 0, 1e-6);
    EXPECT_NEAR(segments[0].p[0], 0.2233, 1e-4);
    EXPECT_NEAR(segments[0].p[1], 1.9038, 1e-4);
    EXPECT_NEAR(segments[0].p[2], 0, 1e-6);
    EXPECT_NEAR(segments[0].v[0], -0.7071, 1e-4);
    EXPECT_NEAR(segments[0].v[1], 3.3000, 1e-4);
    EXPECT_NEAR(segments[0].v[2], 0, 1e-6);
    EXPECT_NEAR(segments[0].a[0], -1.1194, 1e-4);
    EXPECT_NEAR(segments[0].a[1], 2.5975, 1e-4);
    EXPECT_NEAR(segments[0].a[2], 2, 1e-4);

    // Segment 2
    EXPECT_NEAR(segments[1].dt[0], 0, 1e-6);
    EXPECT_NEAR(segments[1].dt[1], 0, 1e-6);
    EXPECT_NEAR(segments[1].dt[2], 0, 1e-6);
    EXPECT_NEAR(segments[1].t[0], 1.5251, 1e-4);
    EXPECT_NEAR(segments[1].t[1], 0.8855, 1e-4);
    EXPECT_NEAR(segments[1].t[2], 0, 1e-6);
    EXPECT_NEAR(segments[1].p[0], 0.2233, 1e-4);
    EXPECT_NEAR(segments[1].p[1], 1.9038, 1e-4);
    EXPECT_NEAR(segments[1].p[2], 0, 1e-6);
    EXPECT_NEAR(segments[1].v[0], -0.7071, 1e-4);
    EXPECT_NEAR(segments[1].v[1], 3.3000, 1e-4);
    EXPECT_NEAR(segments[1].v[2], 0, 1e-6);
    EXPECT_NEAR(segments[1].a[0], 0, 1e-6);
    EXPECT_NEAR(segments[1].a[1], 0, 1e-6);
    EXPECT_NEAR(segments[1].a[2], 0, 1e-6);

    // Segment 3
    EXPECT_NEAR(segments[2].dt[0], 0.6317, 1e-4);
    EXPECT_NEAR(segments[2].dt[1], 1.2705, 1e-4);
    EXPECT_NEAR(segments[2].dt[2], 0, 1e-6);
    EXPECT_NEAR(segments[2].t[0], 2.1568, 1e-4);
    EXPECT_NEAR(segments[2].t[1], 2.1559, 1e-4);
    EXPECT_NEAR(segments[2].t[2], 0, 1e-6);
    EXPECT_NEAR(segments[2].p[0], 5.5511e-17, 1e-12);
    EXPECT_NEAR(segments[2].p[1], 4.0000, 1e-4);
    EXPECT_NEAR(segments[2].p[2], 0, 1e-6);
    EXPECT_NEAR(segments[2].v[0], 0, 1e-6);
    EXPECT_NEAR(segments[2].v[1], 0, 1e-6);
    EXPECT_NEAR(segments[2].v[2], 0, 1e-6);
    EXPECT_NEAR(segments[2].a[0], 1.1194, 1e-4);
    EXPECT_NEAR(segments[2].a[1], -2.5975, 1e-4);
    EXPECT_NEAR(segments[2].a[2], 2, 1e-4);

    // Segment 4
    EXPECT_NEAR(segments[3].dt[0], 1.0000e+10, 1e6);
    EXPECT_NEAR(segments[3].dt[1], 1.0000e+10, 1e6);
    EXPECT_NEAR(segments[3].dt[2], 1.0000e+10, 1e6);
    EXPECT_NEAR(segments[3].t[0], 1.0000e+10, 1e6);
    EXPECT_NEAR(segments[3].t[1], 1.0000e+10, 1e6);
    EXPECT_NEAR(segments[3].t[2], 1.0000e+10, 1e6);
    EXPECT_NEAR(segments[3].p[0], 5.5511e-17, 1e-12);
    EXPECT_NEAR(segments[3].p[1], 4.0000, 1e-4);
    EXPECT_NEAR(segments[3].p[2], 0, 1e-6);
    EXPECT_NEAR(segments[3].v[0], 0, 1e-6);
    EXPECT_NEAR(segments[3].v[1], 0, 1e-6);
    EXPECT_NEAR(segments[3].v[2], 0, 1e-6);
    EXPECT_NEAR(segments[3].a[0], 0, 1e-6);
    EXPECT_NEAR(segments[3].a[1], 0, 1e-6);
    EXPECT_NEAR(segments[3].a[2], 0, 1e-6);
}

TEST_F(BalanceXYTest, SegmentsAreModified) {
    // Test that the segments vector is actually modified
    Eigen::Vector3d p0(0, 0, 0), v0(0, 0, 0), pe(1, 1, 0), ve(0, 0, 0);
    Eigen::Vector3d vm(2, 2, 2), am(1, 1, 1), dm(1, 1, 1);
    Eigen::Vector3d vmax, amax, dmax;

    // Store initial state of segments
    auto initial_segments = segments;

    spg::setpoint::balanceXY(segments, p0, v0, pe, ve, vm, am, dm, vmax, amax, dmax);

    // At least one segment should be different from initial state
    bool segments_modified = false;
    for (size_t i = 0; i < segments.size(); ++i) {
        if ((segments[i].p - initial_segments[i].p).norm() > 1e-10 ||
            (segments[i].v - initial_segments[i].v).norm() > 1e-10 ||
            (segments[i].a - initial_segments[i].a).norm() > 1e-10) {
            segments_modified = true;
            break;
        }
    }
    EXPECT_TRUE(segments_modified);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
