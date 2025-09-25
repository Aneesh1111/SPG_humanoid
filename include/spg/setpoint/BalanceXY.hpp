#pragma once
#include <vector>
#include <array>
#include "Segment.hpp"
#include <Eigen/Dense>

namespace spg {
namespace setpoint {

void balanceXY(std::vector<Segment>& segments,
               const Eigen::Vector3d& p0,
               const Eigen::Vector3d& v0,
               const Eigen::Vector3d& pe,
               const Eigen::Vector3d& ve,
               const Eigen::Vector3d& vm,
               const Eigen::Vector3d& am,
               const Eigen::Vector3d& dm,
               Eigen::Vector3d& vmax,
               Eigen::Vector3d& amax,
               Eigen::Vector3d& dmax);

}}
