#include <vector>
#include <Eigen/Dense>
#include "spg/setpoint/TrajPredict.hpp"
#include "spg/setpoint/Traj1.hpp"

namespace spg {
namespace setpoint {

void TrajPredict(SPGState& d, const std::vector<Segment>& segment) {
    // Get the number of prediction steps from the parameter
    size_t npredict = static_cast<size_t>(d.par.npredict);
    double Ts = d.par.Ts_predict;  // Use main sampling time for consistency with Traj1
    
    // Ensure trajectory vectors are properly sized
    d.traj.p.resize(npredict);
    d.traj.v.resize(npredict);
    d.traj.a.resize(npredict);
    d.traj.t.resize(npredict);
    d.traj.segment_id.resize(npredict);
    
    // Call Traj1 multiple times for different time steps: Ts, 2*Ts, 3*Ts, ..., npredict*Ts
    for (size_t i = 0; i < npredict; ++i) {
        double time_step = (i + 1) * Ts;
        
        // Create a temporary trajectory structure for this time step
        Traject temp_traj;
        
        // Call Traj1 to get the predicted state at this time step
        Traj1(temp_traj, segment, time_step);
        
        // Store the results in the main trajectory
        if (!temp_traj.p.empty()) {
            d.traj.p[i] = temp_traj.p[0];
            d.traj.v[i] = temp_traj.v[0];
            d.traj.a[i] = temp_traj.a[0];
            d.traj.t[i] = time_step;
            d.traj.segment_id[i] = temp_traj.segment_id[0];
        } else {
            // Fallback: use zeros if Traj1 fails
            d.traj.p[i] = Eigen::Vector3d::Zero();
            d.traj.v[i] = Eigen::Vector3d::Zero();
            d.traj.a[i] = Eigen::Vector3d::Zero();
            d.traj.t[i] = time_step;
            d.traj.segment_id[i] = Eigen::Vector3i::Zero();
        }
    }
    
    // Dribble orientation update if needed
    if (d.input.robot.skillID == 1) {
        for (size_t i = 0; i < d.traj.p.size(); ++i) {
            double vnorm = d.traj.v[i].head(2).squaredNorm();
            if (vnorm > 1e-12) {
                d.traj.p[i][2] = std::atan2(-d.traj.v[i][0], d.traj.v[i][1]);
            }
        }
    }
}

} // namespace setpoint
} // namespace spg