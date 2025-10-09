#include <gtest/gtest.h>
#include "spg/setpoint/Set.hpp"
#include "spg/Init.hpp"
#include <Eigen/Dense>
#include <cmath>

class SetpointSetTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Initialize a basic SPGState for testing
        Eigen::Vector3d p_initial(0, 0, 0);
        Eigen::Vector3d v_initial(0, 0, 0);
        Eigen::Vector2d p_initial_ball(0, 0);
        Eigen::Vector2d v_initial_ball(0, 0);
        d = spg::Init(p_initial, v_initial, 0, 20, p_initial_ball, v_initial_ball, 5);
        
        // // Set up standard field parameters
        // d.par.field_size = {8.0, 12.0};  // 8m x 12m field
        // d.par.field_border_margin = 0.5;  // 0.5m border margin
        // d.par.technical_area_width = 1.0;  // 1m technical area
        // d.par.dmax_move = 1.8;
        // d.par.dmax_rotate = 13.0;
        // d.par.Ts = 0.02;  // 20ms sampling time
        
        // Set up robot state - inside field initially
        d.input.robot.p = Eigen::Vector3d(2.0, 1.0, 0.0);
        d.input.robot.v = Eigen::Vector3d(1.0, 0.5, 0.0);
        d.input.robot.IMU_orientation = Eigen::Vector3d(0.0, 0.0, 0.0);  // No tipping
        
        // Set up setpoint state
        d.setpoint.p = Eigen::Vector3d(2.0, 1.0, 0.0);
        d.setpoint.v = Eigen::Vector3d(1.0, 0.5, 0.0);
        d.setpoint.a = Eigen::Vector3d(0.0, 0.0, 0.0);
        
        // Set up subtarget
        d.subtarget.p = Eigen::Vector3d(4.0, 2.0, 0.0);
        d.subtarget.v = Eigen::Vector3d(0.0, 0.0, 0.0);
        d.subtarget.vmax = Eigen::Vector3d(3.0, 3.0, 6.0);
        d.subtarget.amax = Eigen::Vector3d(2.0, 2.0, 10.0);
        d.subtarget.automatic_substitution_flag = 0;
        
        // Initialize auxiliary segments (4 segments as typical)
        d.aux.segment.resize(4);
        for (int i = 0; i < 4; ++i) {
            d.aux.segment[i].dt = Eigen::Vector3d::Zero();
            d.aux.segment[i].t = Eigen::Vector3d::Zero();
            d.aux.segment[i].p = Eigen::Vector3d::Zero();
            d.aux.segment[i].v = Eigen::Vector3d::Zero();
            d.aux.segment[i].a = Eigen::Vector3d::Zero();
        }
        
        // Initialize trajectory
        d.traj.p = std::vector<Eigen::Vector3d>(20, Eigen::Vector3d::Zero());
        d.traj.v = std::vector<Eigen::Vector3d>(20, Eigen::Vector3d::Zero());
        d.traj.a = std::vector<Eigen::Vector3d>(20, Eigen::Vector3d::Zero());
        d.traj.t = std::vector<double>(20, 0.0);
        d.traj.segment_id = std::vector<Eigen::Vector3i>(20, Eigen::Vector3i::Zero());
    }
    
    spg::SPGState d;
};

TEST_F(SetpointSetTest, BasicFunctionality) {

    d.input.robot.p = Eigen::Vector3d(4.0, -6.0, 0.0);
    d.input.robot.v = Eigen::Vector3d(0.0, 0.0, 0.0);

    d.setpoint.p = Eigen::Vector3d(3.9998, -5.9997, 0.0);
    d.setpoint.v = Eigen::Vector3d(-0.0200, 0.0300, 0.0);
    d.setpoint.a = Eigen::Vector3d(-0.9983, 1.4978, 0.0);

    d.subtarget.p = Eigen::Vector3d(0.0, 0.0, 0.0);
    d.subtarget.v = Eigen::Vector3d(0.0, 0.0, 0.0);
    d.subtarget.vmax = Eigen::Vector3d(2.2185, 3.3284, 13.0);
    d.subtarget.amax = Eigen::Vector3d(0.9983, 1.4978, 13.0);

    // Set up the trajectory with MATLAB expected values
    d.traj.p[0] = Eigen::Vector3d(3.9998, -5.9997, 0.0);
    d.traj.v[0] = Eigen::Vector3d(-0.0200, 0.0300, 0.0);
    d.traj.a[0] = Eigen::Vector3d(-0.9983, 1.4978, 0.0);
    d.traj.t[0] = 0.0200;
    d.traj.segment_id[0] = Eigen::Vector3i(0, 0, 3);

    // Set up auxiliary segments with MATLAB data
    // Segment 0: dt=[2.0017, 2.0015, 0], t=[2.0017, 2.0015, 0], p=[2.0000, -3.0000, 0], v=[-1.9983, 2.9978, 0], a=[-0.9983, 1.4978, 13.0000]
    d.aux.segment[0].dt = Eigen::Vector3d(2.0017, 2.0015, 0);
    d.aux.segment[0].t = Eigen::Vector3d(2.0017, 2.0015, 0);
    d.aux.segment[0].p = Eigen::Vector3d(2.0000, -3.0000, 0);
    d.aux.segment[0].v = Eigen::Vector3d(-1.9983, 2.9978, 0);
    d.aux.segment[0].a = Eigen::Vector3d(-0.9983, 1.4978, 13.0000);
    
    // Segment 1: dt=[0, 0, 0], t=[2.0017, 2.0015, 0], p=[2.0000, -3.0000, 0], v=[-1.9983, 2.9978, 0], a=[0, 0, 0]
    d.aux.segment[1].dt = Eigen::Vector3d(0, 0, 0);
    d.aux.segment[1].t = Eigen::Vector3d(2.0017, 2.0015, 0);
    d.aux.segment[1].p = Eigen::Vector3d(2.0000, -3.0000, 0);
    d.aux.segment[1].v = Eigen::Vector3d(-1.9983, 2.9978, 0);
    d.aux.segment[1].a = Eigen::Vector3d(0, 0, 0);
    
    // Segment 2: dt=[2.0017, 2.0015, 0], t=[4.0034, 4.0029, 0], p=[-0.8882e-15, 0.8882e-15, 0], v=[0, 0, 0], a=[0.9983, -1.4978, 13.0000]
    d.aux.segment[2].dt = Eigen::Vector3d(2.0017, 2.0015, 0);
    d.aux.segment[2].t = Eigen::Vector3d(4.0034, 4.0029, 0);
    d.aux.segment[2].p = Eigen::Vector3d(-0.8882e-15, 0.8882e-15, 0);
    d.aux.segment[2].v = Eigen::Vector3d(0, 0, 0);
    d.aux.segment[2].a = Eigen::Vector3d(0.9983, -1.4978, 13.0000);
    
    // Segment 3: dt=[1e10, 1e10, 1e10], t=[1e10, 1e10, 1e10], p=[-0.8882e-15, 0.8882e-15, 0], v=[0, 0, 0], a=[0, 0, 0]
    d.aux.segment[3].dt = Eigen::Vector3d(1e10, 1e10, 1e10);
    d.aux.segment[3].t = Eigen::Vector3d(1e10, 1e10, 1e10);
    d.aux.segment[3].p = Eigen::Vector3d(-0.8882e-15, 0.8882e-15, 0);
    d.aux.segment[3].v = Eigen::Vector3d(0, 0, 0);
    d.aux.segment[3].a = Eigen::Vector3d(0, 0, 0);

    // Test basic setpoint generation
    spg::SPGState result = spg::setpoint::Set(d);
    
    // The function should have processed the trajectory and updated setpoint values
    
    // Verify that setpoint was updated from trajectory (first timestep) - expected MATLAB values
    EXPECT_NEAR(result.setpoint.p[0], 3.9992, 1e-3);
    EXPECT_NEAR(result.setpoint.p[1], -5.9988, 1e-3);
    EXPECT_NEAR(result.setpoint.p[2], 0.0, 1e-3);

    EXPECT_NEAR(result.setpoint.v[0], -0.0399, 1e-3);
    EXPECT_NEAR(result.setpoint.v[1], 0.0599, 1e-3);
    EXPECT_NEAR(result.setpoint.v[2], 0.0, 1e-3);

    EXPECT_NEAR(result.setpoint.a[0], -0.9983, 1e-3);
    EXPECT_NEAR(result.setpoint.a[1], 1.4978, 1e-3);
    EXPECT_NEAR(result.setpoint.a[2], 0.0, 1e-3);
}

TEST_F(SetpointSetTest, BasicFunctionality2) {

    d.input.robot.p = Eigen::Vector3d(4.0, -6.0, 0.0);
    d.input.robot.v = Eigen::Vector3d(0.0, 0.0, 0.0);

    d.setpoint.p = Eigen::Vector3d(3.9928, -5.9892, 0.0);
    d.setpoint.v = Eigen::Vector3d(-0.1198, 0.1797, 0.0);

    d.subtarget.p = Eigen::Vector3d(0.0, 0.0, 0.0);
    d.subtarget.v = Eigen::Vector3d(0.0, 0.0, 0.0);
    d.subtarget.vmax = Eigen::Vector3d(2.2185, 3.3284, 13.0);
    d.subtarget.amax = Eigen::Vector3d(0.9983, 1.4978, 13.0);

    // MATLAB expected trajectory positions (d.traj.p)
    std::vector<Eigen::Vector3d> expected_p = {
        {3.9712, -5.9569, 0}, {3.9423, -5.9134, 0}, {3.9034, -5.8550, 0}, {3.8544, -5.7816, 0},
        {3.7955, -5.6933, 0}, {3.7267, -5.5899, 0}, {3.6478, -5.4716, 0}, {3.5589, -5.3383, 0},
        {3.4601, -5.1900, 0}, {3.3513, -5.0267, 0}, {3.2325, -4.8485, 0}, {3.1037, -4.6553, 0},
        {2.9650, -4.4471, 0}, {2.8162, -4.2239, 0}, {2.6575, -3.9858, 0}, {2.4888, -3.7326, 0},
        {2.3101, -3.4645, 0}, {2.1214, -3.1814, 0}, {1.9242, -2.8856, 0}, {1.7332, -2.5991, 0}
    };
    
    // MATLAB expected trajectory velocities (d.traj.v)
    std::vector<Eigen::Vector3d> expected_v = {
        {-0.2396, 0.3595, 0}, {-0.3394, 0.5093, 0}, {-0.4393, 0.6590, 0}, {-0.5391, 0.8088, 0},
        {-0.6389, 0.9586, 0}, {-0.7387, 1.1084, 0}, {-0.8386, 1.2581, 0}, {-0.9384, 1.4079, 0},
        {-1.0382, 1.5577, 0}, {-1.1381, 1.7075, 0}, {-1.2379, 1.8573, 0}, {-1.3377, 2.0070, 0},
        {-1.4376, 2.1568, 0}, {-1.5374, 2.3066, 0}, {-1.6372, 2.4564, 0}, {-1.7370, 2.6062, 0},
        {-1.8369, 2.7559, 0}, {-1.9367, 2.9057, 0}, {-1.9601, 2.9401, 0}, {-1.8602, 2.7903, 0}
    };
    
    // MATLAB expected trajectory accelerations (d.traj.a)
    std::vector<Eigen::Vector3d> expected_a = {
        {-0.9983, 1.4978, 0}, {-0.9983, 1.4978, 0}, {-0.9983, 1.4978, 0}, {-0.9983, 1.4978, 0},
        {-0.9983, 1.4978, 0}, {-0.9983, 1.4978, 0}, {-0.9983, 1.4978, 0}, {-0.9983, 1.4978, 0},
        {-0.9983, 1.4978, 0}, {-0.9983, 1.4978, 0}, {-0.9983, 1.4978, 0}, {-0.9983, 1.4978, 0},
        {-0.9983, 1.4978, 0}, {-0.9983, 1.4978, 0}, {-0.9983, 1.4978, 0}, {-0.9983, 1.4978, 0},
        {-0.9983, 1.4978, 0}, {-0.9983, 1.4978, 0}, { 0.9983,-1.4978, 0}, { 0.9983,-1.4978, 0}
    };
    
    // MATLAB expected segment IDs (d.traj.segment_id)
    std::vector<Eigen::Vector3i> expected_segment_id = {
        {0, 0, 3}, {0, 0, 3}, {0, 0, 3}, {0, 0, 3}, {0, 0, 3}, {0, 0, 3}, {0, 0, 3}, {0, 0, 3},
        {0, 0, 3}, {0, 0, 3}, {0, 0, 3}, {0, 0, 3}, {0, 0, 3}, {0, 0, 3}, {0, 0, 3}, {0, 0, 3},
        {0, 0, 3}, {0, 0, 3}, {2, 2, 3}, {2, 2, 3}
    };
    
    // Expected time values (d.traj.t)
    std::vector<double> expected_t = {
        0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0,
        1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9, 2.0
    };

    // Test basic setpoint generation
    spg::SPGState result = spg::setpoint::Set(d);
    
    // The function should have processed the trajectory and updated setpoint values
    
    // Verify that setpoint was updated from trajectory (first timestep) - expected MATLAB values
    EXPECT_NEAR(result.setpoint.p[0], 3.9902, 1e-3);
    EXPECT_NEAR(result.setpoint.p[1], -5.9853, 1e-3);
    EXPECT_NEAR(result.setpoint.p[2], 0.0, 1e-3);

    EXPECT_NEAR(result.setpoint.v[0], -0.1398, 1e-3);
    EXPECT_NEAR(result.setpoint.v[1], 0.2097, 1e-3);
    EXPECT_NEAR(result.setpoint.v[2], 0.0, 1e-3);

    EXPECT_NEAR(result.setpoint.a[0], -0.9983, 1e-3);
    EXPECT_NEAR(result.setpoint.a[1], 1.4978, 1e-3);
    EXPECT_NEAR(result.setpoint.a[2], 0.0, 1e-3);

    // Validate trajectory positions
    for (size_t i = 1; i < std::min(result.traj.p.size(), expected_p.size()); ++i) {
        EXPECT_NEAR(result.traj.p[i][0], expected_p[i][0], 6e-2) << "Position X mismatch at step " << i;
        EXPECT_NEAR(result.traj.p[i][1], expected_p[i][1], 6e-2) << "Position Y mismatch at step " << i;
        EXPECT_NEAR(result.traj.p[i][2], expected_p[i][2], 6e-2) << "Position Z mismatch at step " << i;
    }
    
    // Validate trajectory velocities
    for (size_t i = 1; i < std::min(result.traj.v.size(), expected_v.size()); ++i) {
        EXPECT_NEAR(result.traj.v[i][0], expected_v[i][0], 6e-2) << "Velocity X mismatch at step " << i;
        EXPECT_NEAR(result.traj.v[i][1], expected_v[i][1], 6e-2) << "Velocity Y mismatch at step " << i;
        EXPECT_NEAR(result.traj.v[i][2], expected_v[i][2], 6e-2) << "Velocity Z mismatch at step " << i;
    }
    
    // Validate trajectory accelerations  
    for (size_t i = 1; i < std::min(result.traj.a.size(), expected_a.size()); ++i) {
        EXPECT_NEAR(result.traj.a[i][0], expected_a[i][0], 1e-3) << "Acceleration X mismatch at step " << i;
        EXPECT_NEAR(result.traj.a[i][1], expected_a[i][1], 1e-3) << "Acceleration Y mismatch at step " << i;
        EXPECT_NEAR(result.traj.a[i][2], expected_a[i][2], 1e-3) << "Acceleration Z mismatch at step " << i;
    }
    
    // Validate trajectory times
    for (size_t i = 1; i < std::min(result.traj.t.size(), expected_t.size()); ++i) {
        EXPECT_NEAR(result.traj.t[i], expected_t[i], 1e-3) << "Time mismatch at step " << i;
    }
    
    // Validate segment IDs
    for (size_t i = 1; i < std::min(result.traj.segment_id.size(), expected_segment_id.size()); ++i) {
        EXPECT_EQ(result.traj.segment_id[i][0], expected_segment_id[i][0]) << "Segment ID X mismatch at step " << i;
        EXPECT_EQ(result.traj.segment_id[i][1], expected_segment_id[i][1]) << "Segment ID Y mismatch at step " << i;
        EXPECT_EQ(result.traj.segment_id[i][2], expected_segment_id[i][2]) << "Segment ID Z mismatch at step " << i;
    }
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
