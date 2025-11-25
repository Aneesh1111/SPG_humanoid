#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include "spg/setpoint/BalanceXY.hpp"
#include "spg/setpoint/GetSegments.hpp"
#include "spg/HumanoidUtils.hpp"

namespace spg {
namespace setpoint {

void balanceXY(std::vector<Segment>& segment,
    const Eigen::Vector3d& p0, const Eigen::Vector3d& v0,
    const Eigen::Vector3d& pe, const Eigen::Vector3d& ve,
    const Eigen::Vector3d& vm, const Eigen::Vector3d& am, const Eigen::Vector3d& dm,
    Eigen::Vector3d& vmax_out, Eigen::Vector3d& amax_out, Eigen::Vector3d& dmax_out) {
    double vmax_move = vm.head<2>().norm();
    double amax_move = am.head<2>().norm();
    double dmax_move = dm.head<2>().norm();
    Eigen::Vector3d amax, vmax, dmax;
    // Check if trajectory requires X-Y balancing (matches MATLAB exactly)
    Eigen::Vector2d pos_diff = (pe.head<2>() - p0.head<2>()).cwiseAbs();
    Eigen::Vector2d vel_diff = (ve.head<2>() - v0.head<2>()).cwiseAbs();
    bool needs_xy_balancing = 
        (pos_diff.array() > 1e-8).all() || (vel_diff.array() > 1e-2).all();

    if (needs_xy_balancing) {
        // Run balancing algorithm
        double a = 45.0;
        double stepsize = a / 2.0;
        int niter = 12;
        for (int i = 0; i < niter; ++i) {
            double max_downscale = 0.01;
            Eigen::Vector2d A(std::max(max_downscale, std::cos(a * M_PI / 180.0)), std::max(max_downscale, std::sin(a * M_PI / 180.0)));
            amax = Eigen::Vector3d(amax_move * A(0), amax_move * A(1), am(2));
            vmax = Eigen::Vector3d(vmax_move * A(0), vmax_move * A(1), vm(2));
            dmax = Eigen::Vector3d(dmax_move * A(0), dmax_move * A(1), dm(2));
            segment = getSegments(segment, p0, v0, pe, ve, vmax, amax, dmax);
            if (segment[2].t[1] > segment[2].t[0]) {
                a += stepsize;
            } else {
                a -= stepsize;
            }
            stepsize /= 2.0;
        }
    } else {
        amax = am;
        vmax = vm;
        dmax = dm;
    }
    vmax_out = vmax;
    amax_out = amax;
    dmax_out = dmax;
}

void balanceXY_humanoid(std::vector<Segment>& segment,
    const Eigen::Vector3d& p0, const Eigen::Vector3d& v0,
    const Eigen::Vector3d& pe, const Eigen::Vector3d& ve,
    const FieldParams& par, double robot_orientation,
    Eigen::Vector3d& vmax_out, Eigen::Vector3d& amax_out, Eigen::Vector3d& dmax_out) {
    
    // Calculate desired movement direction
    Eigen::Vector3d desired_direction = pe - p0;
    desired_direction(2) = 0; // Only consider x-y movement for direction
    
    // Get effective limits based on robot orientation and movement direction
    Eigen::Vector3d effective_vmax = humanoid::calculateEffectiveVelocityLimits(par, robot_orientation, desired_direction);
    Eigen::Vector3d effective_amax = humanoid::calculateEffectiveAccelerationLimits(par, robot_orientation, desired_direction);
    Eigen::Vector3d effective_dmax = effective_amax; // Use same limits for deceleration
    
    // Check if trajectory requires X-Y balancing
    Eigen::Vector2d pos_diff = (pe.head<2>() - p0.head<2>()).cwiseAbs();
    Eigen::Vector2d vel_diff = (ve.head<2>() - v0.head<2>()).cwiseAbs();
    bool needs_xy_balancing = 
        (pos_diff.array() > 1e-8).all() || (vel_diff.array() > 1e-2).all();

    if (needs_xy_balancing) {
        // Enhanced balancing that considers humanoid directional constraints
        double a = 45.0;
        double stepsize = a / 2.0;
        int niter = 12;
        
        for (int i = 0; i < niter; ++i) {
            double max_downscale = 0.01;
            Eigen::Vector2d A(std::max(max_downscale, std::cos(a * M_PI / 180.0)), 
                             std::max(max_downscale, std::sin(a * M_PI / 180.0)));
            
            // Apply humanoid-specific scaling factors to effective limits
            // Use the pre-calculated effective limits that already account for orientation
            Eigen::Vector3d scaled_vmax(effective_vmax(0) * A(0), effective_vmax(1) * A(1), effective_vmax(2));
            Eigen::Vector3d scaled_amax(effective_amax(0) * A(0), effective_amax(1) * A(1), effective_amax(2));
            
            segment = getSegments(segment, p0, v0, pe, ve, scaled_vmax, scaled_amax, scaled_amax);
            
            if (segment[2].t[1] > segment[2].t[0]) {
                a += stepsize;
            } else {
                a -= stepsize;
            }
            stepsize /= 2.0;
        }
        
        // Set final output values
        vmax_out = effective_vmax;
        amax_out = effective_amax;
        dmax_out = effective_dmax;
    } else {
        // No balancing needed, use effective limits directly to generate segments
        segment = getSegments(segment, p0, v0, pe, ve, effective_vmax, effective_amax, effective_dmax);
        vmax_out = effective_vmax;
        amax_out = effective_amax;
        dmax_out = effective_dmax;
    }
}

} // namespace setpoint
} // namespace spg
