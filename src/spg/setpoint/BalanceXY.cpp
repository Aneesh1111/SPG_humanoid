#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include "spg/setpoint/BalanceXY.hpp"
#include "spg/setpoint/GetSegments.hpp"

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

} // namespace setpoint
} // namespace spg
