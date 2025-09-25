#include <gtest/gtest.h>
#include <vector>
#include <Eigen/Dense>
#include "spg/setpoint/GetSegments.hpp"
#include "spg/setpoint/Segment.hpp"

TEST(GetSegmentsTest, ReturnsSegmentsWithExpectedSize) {
    std::vector<spg::setpoint::Segment> segments(4);
    Eigen::Vector3d p0(0, 0, 0), v0(0, 0, 0), pe(1, 1, 0), ve(0, 0, 0);
    Eigen::Vector3d vm(1, 1, 1), am(1, 1, 1), dm(1, 1, 1);

    auto result = spg::setpoint::getSegments(segments, p0, v0, pe, ve, vm, am, dm);

    // Check that the result has the expected number of segments
    EXPECT_EQ(result.size(), 4);
    // Optionally check that the segment values are reasonable
    for (const auto& seg : result) {
        EXPECT_TRUE(seg.p.allFinite());
        EXPECT_TRUE(seg.v.allFinite());
        EXPECT_TRUE(seg.a.allFinite());
    }
}

TEST(GetSegmentsTest, ReturnsExpectedValues) {
    std::vector<spg::setpoint::Segment> segments(4);
    Eigen::Vector3d p0(0, 0, 0), v0(1, 1, 0), pe(0, 4, 0), ve(0, 0, 0);
    Eigen::Vector3d vm(3, 3, 3), am(2, 2, 2), dm(2, 2, 2);

    auto result = spg::setpoint::getSegments(segments, p0, v0, pe, ve, vm, am, dm);

    // Expected values for each segment (dt, t, p, v, a)
    std::vector<Eigen::Vector3d> expected_dt = {        
        Eigen::Vector3d(0.853553390593274, 0.957737973711325, 0),
        Eigen::Vector3d(0, 0, 0),
        Eigen::Vector3d(0.353553390593274, 1.45773797371133, 0),
        Eigen::Vector3d(10000000000.0, 10000000000.0, 10000000000.0)
    };
    std::vector<Eigen::Vector3d> expected_time = {
        Eigen::Vector3d(0.853553390593274, 0.957737973711325, 0),
        Eigen::Vector3d(0.853553390593274, 0.957737973711325, 0),
        Eigen::Vector3d(1.20710678118655, 2.41547594742265, 0),
        Eigen::Vector3d(10000000001.2071, 10000000002.4155, 10000000000.0)
    };
    std::vector<Eigen::Vector3d> expected_p = {
        Eigen::Vector3d(0.125, 1.875, 0),
        Eigen::Vector3d(0.125, 1.875, 0),
        Eigen::Vector3d(-2.77555756156289e-17, 4, 0),
        Eigen::Vector3d(-2.77555756156289e-17, 4, 0)
    };
    std::vector<Eigen::Vector3d> expected_v = {
        Eigen::Vector3d(-0.707106781186548, 2.91547594742265, 0),
        Eigen::Vector3d(-0.707106781186548, 2.91547594742265, 0),
        Eigen::Vector3d(0, 0, 0),
        Eigen::Vector3d(0, 0, 0)
    };
    std::vector<Eigen::Vector3d> expected_a = {
        Eigen::Vector3d(-2, 2, 2),
        Eigen::Vector3d(0, 0, 0),
        Eigen::Vector3d(2, -2, 2),
        Eigen::Vector3d(0, 0, 0)
    };

    for (size_t i = 0; i < result.size(); ++i) {
        for (int j = 0; j < 3; ++j) {
            EXPECT_NEAR(result[i].dt[j], expected_dt[i][j], 1e-4);
            EXPECT_NEAR(result[i].t[j], expected_time[i][j], 1e-4);
            EXPECT_NEAR(result[i].p[j], expected_p[i][j], 1e-4);
            EXPECT_NEAR(result[i].v[j], expected_v[i][j], 1e-4);
            EXPECT_NEAR(result[i].a[j], expected_a[i][j], 1e-4);
        }
    }
}

TEST(GetSegmentsTest, ReturnsExpectedValues_2) {
    std::vector<spg::setpoint::Segment> segments(4);
    Eigen::Vector3d p0(0, 0, 1), v0(1, 1, 1), pe(-2, -4, -1), ve(0, 0, 0);
    Eigen::Vector3d vm(3, 3, 3), am(2, 2, 2), dm(2, 2, 2);

    auto result = spg::setpoint::getSegments(segments, p0, v0, pe, ve, vm, am, dm);

    // Expected values for each segment (dt, t, p, v, a) from MATLAB
    std::vector<Eigen::Vector3d> expected_dt = {        
        Eigen::Vector3d(1.5607, 1.9577, 1.5607),
        Eigen::Vector3d(0, 0, 0),
        Eigen::Vector3d(1.0607, 1.4577, 1.0607),
        Eigen::Vector3d(1.0000e+10, 1.0000e+10, 1.0000e+10)
    };
    std::vector<Eigen::Vector3d> expected_time = {
        Eigen::Vector3d(1.5607, 1.9577, 1.5607),
        Eigen::Vector3d(1.5607, 1.9577, 1.5607),
        Eigen::Vector3d(2.6213, 3.4155, 2.6213),
        Eigen::Vector3d(1.0000e+10, 1.0000e+10, 1.0000e+10)
    };
    std::vector<Eigen::Vector3d> expected_p = {
        Eigen::Vector3d(-0.8750, -1.8750, 0.1250),
        Eigen::Vector3d(-0.8750, -1.8750, 0.1250),
        Eigen::Vector3d(-2.0000, -4, -1.0000),
        Eigen::Vector3d(-2.0000, -4, -1.0000)
    };
    std::vector<Eigen::Vector3d> expected_v = {
        Eigen::Vector3d(-2.1213, -2.9155, -2.1213),
        Eigen::Vector3d(-2.1213, -2.9155, -2.1213),
        Eigen::Vector3d(0, 0, 0),
        Eigen::Vector3d(0, 0, 0)
    };
    std::vector<Eigen::Vector3d> expected_a = {
        Eigen::Vector3d(-2, -2, -2),
        Eigen::Vector3d(0, 0, 0),
        Eigen::Vector3d(2, 2, 2),
        Eigen::Vector3d(0, 0, 0)
    };

    for (size_t i = 0; i < result.size(); ++i) {
        for (int j = 0; j < 3; ++j) {
            if (i == 3) {
                EXPECT_NEAR(result[i].dt[j], expected_dt[i][j], 1e1);
                EXPECT_NEAR(result[i].t[j], expected_time[i][j], 1e1);
            } else {
                EXPECT_NEAR(result[i].dt[j], expected_dt[i][j], 1e-4);
                EXPECT_NEAR(result[i].t[j], expected_time[i][j], 1e-4);
            }
            EXPECT_NEAR(result[i].p[j], expected_p[i][j], 1e-4);
            EXPECT_NEAR(result[i].v[j], expected_v[i][j], 1e-4);
            EXPECT_NEAR(result[i].a[j], expected_a[i][j], 1e-4);
        }
    }
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
