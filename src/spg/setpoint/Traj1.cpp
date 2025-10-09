#include <vector>
#include <Eigen/Dense>
#include "spg/setpoint/Traj1.hpp"
#include "spg/setpoint/TrajSegment.hpp"
#include "spg/setpoint/CombineSegmentData.hpp"

namespace spg {
namespace setpoint {

void Traj1(Traject& traject, const std::vector<Segment>& segment, double Ts) {
    // Equivalent to MATLAB: n = 1; nvec = 1; t = nvec*Ts;
    size_t n = 1;
    std::vector<double> t = {Ts};
    
    // Preparations - equivalent to MATLAB: ndof = length(segment(1).p); nseg = length(segment);
    size_t ndof = segment[0].p.size(); // should be 3 for x,y,theta
    size_t nseg = segment.size();       // number of segments (should be 4 from test data)
    
    // Implement trajectory calculation directly like MATLAB traj_segment
    // Instead of using the existing TrajSegment function, implement the logic here
    // to match MATLAB behavior exactly
    
    // Get the combined segment data
    std::vector<double> t_combined, p_combined, v_combined, a_combined;
    CombineSegmentData(segment, t_combined, p_combined, v_combined, a_combined);
    
    // Calculate P, V, A for each segment and DOF combination
    std::vector<double> P(nseg * ndof), V(nseg * ndof), A(nseg * ndof);
    std::vector<bool> tseg(nseg * ndof);
    
    for (size_t i = 0; i < nseg; ++i) {
        for (size_t j = 0; j < ndof; ++j) {
            size_t idx = i * ndof + j; // segment i, DOF j
            
            double t_seg = t_combined[idx];
            double p_seg = p_combined[idx];
            double v_seg = v_combined[idx];
            double a_seg = a_combined[idx];
            
            // Calculate trajectory at time Ts relative to segment start time t_seg
            double t_rel = Ts - t_seg;
            
            P[idx] = p_seg + v_seg * t_rel + 0.5 * a_seg * t_rel * t_rel;
            V[idx] = v_seg + a_seg * t_rel;
            A[idx] = a_seg;
            
            tseg[idx] = (Ts > t_seg);
        }
    }
    
    // Determine active segment at each time instance
    // MATLAB: segment_id = sum(reshape(tseg,[n ndof nseg]),3);
    Eigen::Vector3i segment_id = Eigen::Vector3i::Zero();
    for (size_t j = 0; j < ndof; ++j) {
        for (size_t i = 0; i < nseg; ++i) {
            size_t idx = i * ndof + j;
            if (tseg[idx]) {
                segment_id[j]++;
            }
        }
    }
    
    // Determine correct segment sample selection
    // MATLAB: ind = bsxfun(@plus, nvec, (0:ndof-1)*n); ind = ind + segment_id*n*ndof;
    // With nvec=1 (but 0-based): ind = [0, 1, 2] + segment_id * 1 * 3
    std::vector<size_t> ind(ndof);
    for (size_t j = 0; j < ndof; ++j) {
        ind[j] = j + segment_id[j] * ndof;
        
        // Ensure index is within bounds
        if (ind[j] >= P.size()) {
            ind[j] = j + (nseg - 1) * ndof; // use last segment
        }
    }
    
    // Set output - equivalent to MATLAB setpoint assignments
    traject.t = t;
    traject.p.clear();
    traject.v.clear();
    traject.a.clear();
    traject.segment_id.clear();
    
    // MATLAB: traject.p(nvec,:) = P(ind); etc.
    Eigen::Vector3d pos, vel, acc;
    for (size_t j = 0; j < ndof; ++j) {
        pos[j] = P[ind[j]];
        vel[j] = V[ind[j]];
        acc[j] = A[ind[j]];
    }
    
    traject.p.push_back(pos);
    traject.v.push_back(vel);
    traject.a.push_back(acc);
    traject.segment_id.push_back(segment_id);
}

} // namespace setpoint
} // namespace spg