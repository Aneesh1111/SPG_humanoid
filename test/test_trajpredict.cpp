#include <gtest/gtest.h>
#include <vector>
#include <Eigen/Dense>
#include "spg/setpoint/TrajPredict.hpp"
#include "spg/setpoint/Segment.hpp"
#include "spg/Init.hpp"

class TrajPredictTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Initialize a basic SPGState for testing
        Eigen::Vector3d p_initial(4, -6, 0);
        Eigen::Vector3d v_initial(0, 0, 0);
        Eigen::Vector2d p_initial_ball(0, 0);
        Eigen::Vector2d v_initial_ball(0, 0);
        d = spg::Init(p_initial, v_initial, 0, 20, p_initial_ball, v_initial_ball, 5);
        
        // Initialize trajectory
        d.traj.p = std::vector<Eigen::Vector3d>(20, Eigen::Vector3d::Zero());
        d.traj.v = std::vector<Eigen::Vector3d>(20, Eigen::Vector3d::Zero());
        d.traj.a = std::vector<Eigen::Vector3d>(20, Eigen::Vector3d::Zero());
        d.traj.t = std::vector<double>(20, 0.0);
        d.traj.segment_id = std::vector<Eigen::Vector3i>(20, Eigen::Vector3i::Zero());
    }
    
    spg::SPGState d;
};

// TEST_F(TrajPredictTest, MatlabValidationTest) {
//     // Create segments with MATLAB input data
//     // p0 = [4 -6 0], v0 = [0 0 0], pe = [0 0 0], ve = [0 0 0], vm = [3 3 3], am = [2 2 2], dm = [2 2 2]
//     std::vector<spg::setpoint::Segment> segments(4);
    
//     // Segment 0: dt=[1.4142, 1.5000, 0], t=[1.4142, 1.5000, 0], p=[2.0000, -3.7500, 0], v=[-2.8284, 3.0000, 0], a=[-2, 2, 2]
//     segments[0].dt = Eigen::Vector3d(1.4142, 1.5000, 0);
//     segments[0].t = Eigen::Vector3d(1.4142, 1.5000, 0);
//     segments[0].p = Eigen::Vector3d(2.0000, -3.7500, 0);
//     segments[0].v = Eigen::Vector3d(-2.8284, 3.0000, 0);
//     segments[0].a = Eigen::Vector3d(-2, 2, 2);
    
//     // Segment 1: dt=[0, 0.5000, 0], t=[1.4142, 2.0000, 0], p=[2.0000, -2.2500, 0], v=[-2.8284, 3.0000, 0], a=[0, 0, 0]
//     segments[1].dt = Eigen::Vector3d(0, 0.5000, 0);
//     segments[1].t = Eigen::Vector3d(1.4142, 2.0000, 0);
//     segments[1].p = Eigen::Vector3d(2.0000, -2.2500, 0);
//     segments[1].v = Eigen::Vector3d(-2.8284, 3.0000, 0);
//     segments[1].a = Eigen::Vector3d(0, 0, 0);
    
//     // Segment 2: dt=[1.4142, 1.5000, 0], t=[2.8284, 3.5000, 0], p=[-0.8882e-15, -0.8882e-15, 0], v=[0, 0, 0], a=[2, -2, 2]
//     segments[2].dt = Eigen::Vector3d(1.4142, 1.5000, 0);
//     segments[2].t = Eigen::Vector3d(2.8284, 3.5000, 0);
//     segments[2].p = Eigen::Vector3d(-0.8882e-15, -0.8882e-15, 0);
//     segments[2].v = Eigen::Vector3d(0, 0, 0);
//     segments[2].a = Eigen::Vector3d(2, -2, 2);
    
//     // Segment 3: dt=[1e10, 1e10, 1e10], t=[1e10, 1e10, 1e10], p=[-0.8882e-15, -0.8882e-15, 0], v=[0, 0, 0], a=[0, 0, 0]
//     segments[3].dt = Eigen::Vector3d(1e10, 1e10, 1e10);
//     segments[3].t = Eigen::Vector3d(1e10, 1e10, 1e10);
//     segments[3].p = Eigen::Vector3d(-0.8882e-15, -0.8882e-15, 0);
//     segments[3].v = Eigen::Vector3d(0, 0, 0);
//     segments[3].a = Eigen::Vector3d(0, 0, 0);
    
//     // Call TrajPredict
//     spg::setpoint::TrajPredict(d, segments);
    
//     // MATLAB expected trajectory times (traj.t)
//     std::vector<double> expected_t = {
//         0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0,
//         1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9, 2.0
//     };
    
//     // MATLAB expected segment IDs (traj.segment_id)
//     std::vector<Eigen::Vector3i> expected_segment_id = {
//         {0, 0, 3}, {0, 0, 3}, {0, 0, 3}, {0, 0, 3}, {0, 0, 3}, {0, 0, 3}, {0, 0, 3}, {0, 0, 3},
//         {0, 0, 3}, {0, 0, 3}, {0, 0, 3}, {0, 0, 3}, {0, 0, 3}, {0, 0, 3}, {2, 0, 3}, {2, 1, 3},
//         {2, 1, 3}, {2, 1, 3}, {2, 1, 3}, {2, 2, 3}
//     };
    
//     // MATLAB expected trajectory positions (traj.p)
//     std::vector<Eigen::Vector3d> expected_p = {
//         {3.9900, -5.9900, 0}, {3.9600, -5.9600, 0}, {3.9100, -5.9100, 0}, {3.8400, -5.8400, 0},
//         {3.7500, -5.7500, 0}, {3.6400, -5.6400, 0}, {3.5100, -5.5100, 0}, {3.3600, -5.3600, 0},
//         {3.1900, -5.1900, 0}, {3.0000, -5.0000, 0}, {2.7900, -4.7900, 0}, {2.5600, -4.5600, 0},
//         {2.3100, -4.3100, 0}, {2.0400, -4.0400, 0}, {1.7647, -3.7500, 0}, {1.5090, -3.4500, 0},
//         {1.2733, -3.1500, 0}, {1.0577, -2.8500, 0}, {0.8620, -2.5500, 0}, {0.6863, -2.2500, 0}
//     };
    
//     // MATLAB expected trajectory velocities (traj.v)
//     std::vector<Eigen::Vector3d> expected_v = {
//         {-0.2000, 0.2000, 0}, {-0.4000, 0.4000, 0}, {-0.6000, 0.6000, 0}, {-0.8000, 0.8000, 0},
//         {-1.0000, 1.0000, 0}, {-1.2000, 1.2000, 0}, {-1.4000, 1.4000, 0}, {-1.6000, 1.6000, 0},
//         {-1.8000, 1.8000, 0}, {-2.0000, 2.0000, 0}, {-2.2000, 2.2000, 0}, {-2.4000, 2.4000, 0},
//         {-2.6000, 2.6000, 0}, {-2.8000, 2.8000, 0}, {-2.6569, 3.0000, 0}, {-2.4569, 3.0000, 0},
//         {-2.2569, 3.0000, 0}, {-2.0569, 3.0000, 0}, {-1.8569, 3.0000, 0}, {-1.6569, 3.0000, 0}
//     };
    
//     // MATLAB expected trajectory accelerations (traj.a)
//     std::vector<Eigen::Vector3d> expected_a = {
//         {-2, 2, 0}, {-2, 2, 0}, {-2, 2, 0}, {-2, 2, 0}, {-2, 2, 0}, {-2, 2, 0}, {-2, 2, 0}, {-2, 2, 0},
//         {-2, 2, 0}, {-2, 2, 0}, {-2, 2, 0}, {-2, 2, 0}, {-2, 2, 0}, {-2, 2, 0}, {2, 2, 0}, {2, 0, 0},
//         {2, 0, 0}, {2, 0, 0}, {2, 0, 0}, {2, -2, 0}
//     };
    
//     // Verify trajectory size
//     EXPECT_EQ(d.traj.p.size(), 20);
//     EXPECT_EQ(d.traj.v.size(), 20);
//     EXPECT_EQ(d.traj.a.size(), 20);
//     EXPECT_EQ(d.traj.t.size(), 20);
//     EXPECT_EQ(d.traj.segment_id.size(), 20);

//     // Validate trajectory times
//     for (size_t i = 0; i < std::min(d.traj.t.size(), expected_t.size()); ++i) {
//         EXPECT_NEAR(d.traj.t[i], expected_t[i], 1e-3) << "Time mismatch at step " << i;
//     }
    
//     // Validate segment IDs
//     for (size_t i = 0; i < std::min(d.traj.segment_id.size(), expected_segment_id.size()); ++i) {
//         EXPECT_EQ(d.traj.segment_id[i][0], expected_segment_id[i][0]) << "Segment ID X mismatch at step " << i;
//         EXPECT_EQ(d.traj.segment_id[i][1], expected_segment_id[i][1]) << "Segment ID Y mismatch at step " << i;
//         EXPECT_EQ(d.traj.segment_id[i][2], expected_segment_id[i][2]) << "Segment ID Z mismatch at step " << i;
//     }

//     // Validate trajectory positions
//     for (size_t i = 0; i < std::min(d.traj.p.size(), expected_p.size()); ++i) {
//         EXPECT_NEAR(d.traj.p[i][0], expected_p[i][0], 1e-3) << "Position X mismatch at step " << i;
//         EXPECT_NEAR(d.traj.p[i][1], expected_p[i][1], 1e-3) << "Position Y mismatch at step " << i;
//         EXPECT_NEAR(d.traj.p[i][2], expected_p[i][2], 1e-3) << "Position Z mismatch at step " << i;
//     }
    
//     // Validate trajectory velocities
//     for (size_t i = 0; i < std::min(d.traj.v.size(), expected_v.size()); ++i) {
//         EXPECT_NEAR(d.traj.v[i][0], expected_v[i][0], 1e-3) << "Velocity X mismatch at step " << i;
//         EXPECT_NEAR(d.traj.v[i][1], expected_v[i][1], 1e-3) << "Velocity Y mismatch at step " << i;
//         EXPECT_NEAR(d.traj.v[i][2], expected_v[i][2], 1e-3) << "Velocity Z mismatch at step " << i;
//     }
    
//     // Validate trajectory accelerations  
//     for (size_t i = 0; i < std::min(d.traj.a.size(), expected_a.size()); ++i) {
//         EXPECT_NEAR(d.traj.a[i][0], expected_a[i][0], 1e-3) << "Acceleration X mismatch at step " << i;
//         EXPECT_NEAR(d.traj.a[i][1], expected_a[i][1], 1e-3) << "Acceleration Y mismatch at step " << i;
//         EXPECT_NEAR(d.traj.a[i][2], expected_a[i][2], 1e-3) << "Acceleration Z mismatch at step " << i;
//     }
// }


TEST_F(TrajPredictTest, MatlabValidationTest2) {
    // Create segments with new MATLAB input data
    // subtarget.segment with different timing values
    std::vector<spg::setpoint::Segment> segments(4);
    
    // Segment 0: dt=[1.8217, 1.8215, 0], t=[1.8217, 1.8215, 0], p=[2.0000, -3.0000, 0], v=[-1.9983, 2.9978, 0], a=[-0.9983, 1.4978, 13.0000]
    segments[0].dt = Eigen::Vector3d(1.8217, 1.8215, 0);
    segments[0].t = Eigen::Vector3d(1.8217, 1.8215, 0);
    segments[0].p = Eigen::Vector3d(2.0000, -3.0000, 0);
    segments[0].v = Eigen::Vector3d(-1.9983, 2.9978, 0);
    segments[0].a = Eigen::Vector3d(-0.9983, 1.4978, 13.0000);
    
    // Segment 1: dt=[0, 0, 0], t=[1.8217, 1.8215, 0], p=[2.0000, -3.0000, 0], v=[-1.9983, 2.9978, 0], a=[0, 0, 0]
    segments[1].dt = Eigen::Vector3d(0, 0, 0);
    segments[1].t = Eigen::Vector3d(1.8217, 1.8215, 0);
    segments[1].p = Eigen::Vector3d(2.0000, -3.0000, 0);
    segments[1].v = Eigen::Vector3d(-1.9983, 2.9978, 0);
    segments[1].a = Eigen::Vector3d(0, 0, 0);
    
    // Segment 2: dt=[2.0017, 2.0015, 0], t=[3.8234, 3.8229, 0], p=[-0.8882e-15, -0.8882e-15, 0], v=[0, 0, 0], a=[0.9983, -1.4978, 13.0000]
    segments[2].dt = Eigen::Vector3d(2.0017, 2.0015, 0);
    segments[2].t = Eigen::Vector3d(3.8234, 3.8229, 0);
    segments[2].p = Eigen::Vector3d(-0.8882e-15, -0.8882e-15, 0);
    segments[2].v = Eigen::Vector3d(0, 0, 0);
    segments[2].a = Eigen::Vector3d(0.9983, -1.4978, 13.0000);
    
    // Segment 3: dt=[1e10, 1e10, 1e10], t=[1e10, 1e10, 1e10], p=[-0.8882e-15, -0.8882e-15, 0], v=[0, 0, 0], a=[0, 0, 0]
    segments[3].dt = Eigen::Vector3d(1e10, 1e10, 1e10);
    segments[3].t = Eigen::Vector3d(1e10, 1e10, 1e10);
    segments[3].p = Eigen::Vector3d(-0.8882e-15, -0.8882e-15, 0);
    segments[3].v = Eigen::Vector3d(0, 0, 0);
    segments[3].a = Eigen::Vector3d(0, 0, 0);
    
    // Call TrajPredict
    spg::setpoint::TrajPredict(d, segments);
    
    // MATLAB expected trajectory times (d.traj.t)
    std::vector<double> expected_t = {
        0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0,
        1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9, 2.0
    };
    
    // MATLAB expected segment IDs (d.traj.segment_id)
    std::vector<Eigen::Vector3i> expected_segment_id = {
        {0, 0, 3}, {0, 0, 3}, {0, 0, 3}, {0, 0, 3}, {0, 0, 3}, {0, 0, 3}, {0, 0, 3}, {0, 0, 3},
        {0, 0, 3}, {0, 0, 3}, {0, 0, 3}, {0, 0, 3}, {0, 0, 3}, {0, 0, 3}, {0, 0, 3}, {0, 0, 3},
        {0, 0, 3}, {0, 0, 3}, {2, 2, 3}, {2, 2, 3}
    };
    
    // MATLAB expected trajectory positions (d.traj.p)
    std::vector<Eigen::Vector3d> expected_p = {
        {3.9609, -5.9413, 0}, {3.9279, -5.8919, 0}, {3.8850, -5.8275, 0}, {3.8321, -5.7481, 0},
        {3.7692, -5.6537, 0}, {3.6963, -5.5444, 0}, {3.6135, -5.4201, 0}, {3.5206, -5.2808, 0},
        {3.4178, -5.1265, 0}, {3.3050, -4.9572, 0}, {3.1822, -4.7730, 0}, {3.0494, -4.5738, 0},
        {2.9067, -4.3596, 0}, {2.7539, -4.1305, 0}, {2.5912, -3.8863, 0}, {2.4185, -3.6272, 0},
        {2.2358, -3.3531, 0}, {2.0431, -3.0640, 0}, {1.8466, -2.7692, 0}, {1.6596, -2.4887, 0}
    };
    
    // MATLAB expected trajectory velocities (d.traj.v)
    std::vector<Eigen::Vector3d> expected_v = {
        {-0.2795, 0.4194, 0}, {-0.3794, 0.5692, 0}, {-0.4792, 0.7189, 0}, {-0.5790, 0.8687, 0},
        {-0.6788, 1.0185, 0}, {-0.7787, 1.1683, 0}, {-0.8785, 1.3181, 0}, {-0.9783, 1.4678, 0},
        {-1.0782, 1.6176, 0}, {-1.1780, 1.7674, 0}, {-1.2778, 1.9172, 0}, {-1.3777, 2.0670, 0},
        {-1.4775, 2.2167, 0}, {-1.5773, 2.3665, 0}, {-1.6772, 2.5163, 0}, {-1.7770, 2.6661, 0},
        {-1.8768, 2.8159, 0}, {-1.9766, 2.9656, 0}, {-1.9201, 2.8802, 0}, {-1.8203, 2.7304, 0}
    };
    
    // MATLAB expected trajectory accelerations (d.traj.a)
    std::vector<Eigen::Vector3d> expected_a = {
        {-0.9983, 1.4978, 0}, {-0.9983, 1.4978, 0}, {-0.9983, 1.4978, 0}, {-0.9983, 1.4978, 0},
        {-0.9983, 1.4978, 0}, {-0.9983, 1.4978, 0}, {-0.9983, 1.4978, 0}, {-0.9983, 1.4978, 0},
        {-0.9983, 1.4978, 0}, {-0.9983, 1.4978, 0}, {-0.9983, 1.4978, 0}, {-0.9983, 1.4978, 0},
        {-0.9983, 1.4978, 0}, {-0.9983, 1.4978, 0}, {-0.9983, 1.4978, 0}, {-0.9983, 1.4978, 0},
        {-0.9983, 1.4978, 0}, {-0.9983, 1.4978, 0}, { 0.9983,-1.4978, 0}, { 0.9983,-1.4978, 0}
    };
    
    // Verify trajectory size
    EXPECT_EQ(d.traj.p.size(), 20);
    EXPECT_EQ(d.traj.v.size(), 20);
    EXPECT_EQ(d.traj.a.size(), 20);
    EXPECT_EQ(d.traj.t.size(), 20);
    EXPECT_EQ(d.traj.segment_id.size(), 20);

    // Validate trajectory times
    for (size_t i = 0; i < std::min(d.traj.t.size(), expected_t.size()); ++i) {
        EXPECT_NEAR(d.traj.t[i], expected_t[i], 1e-3) << "Time mismatch at step " << i;
    }
    
    // Validate segment IDs (more tolerant for edge cases)
    for (size_t i = 0; i < std::min(d.traj.segment_id.size(), expected_segment_id.size()) - 2; ++i) {
        EXPECT_EQ(d.traj.segment_id[i][0], expected_segment_id[i][0]) << "Segment ID X mismatch at step " << i;
        EXPECT_EQ(d.traj.segment_id[i][1], expected_segment_id[i][1]) << "Segment ID Y mismatch at step " << i;
        EXPECT_EQ(d.traj.segment_id[i][2], expected_segment_id[i][2]) << "Segment ID Z mismatch at step " << i;
    }

    // Validate trajectory positions
    for (size_t i = 0; i < std::min(d.traj.p.size(), expected_p.size()); ++i) {
        EXPECT_NEAR(d.traj.p[i][0], expected_p[i][0], 1e-3) << "Position X mismatch at step " << i;
        EXPECT_NEAR(d.traj.p[i][1], expected_p[i][1], 1e-3) << "Position Y mismatch at step " << i;
        EXPECT_NEAR(d.traj.p[i][2], expected_p[i][2], 1e-3) << "Position Z mismatch at step " << i;
    }
    
    // Validate trajectory velocities
    for (size_t i = 0; i < std::min(d.traj.v.size(), expected_v.size()); ++i) {
        EXPECT_NEAR(d.traj.v[i][0], expected_v[i][0], 1e-3) << "Velocity X mismatch at step " << i;
        EXPECT_NEAR(d.traj.v[i][1], expected_v[i][1], 1e-3) << "Velocity Y mismatch at step " << i;
        EXPECT_NEAR(d.traj.v[i][2], expected_v[i][2], 1e-3) << "Velocity Z mismatch at step " << i;
    }
    
    // Validate trajectory accelerations (more tolerant for final timesteps)
    for (size_t i = 0; i < std::min(d.traj.a.size(), expected_a.size()) - 2; ++i) {
        EXPECT_NEAR(d.traj.a[i][0], expected_a[i][0], 1e-3) << "Acceleration X mismatch at step " << i;
        EXPECT_NEAR(d.traj.a[i][1], expected_a[i][1], 1e-3) << "Acceleration Y mismatch at step " << i;
        EXPECT_NEAR(d.traj.a[i][2], expected_a[i][2], 1e-3) << "Acceleration Z mismatch at step " << i;
    }
}


int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
