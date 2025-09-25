#pragma once
#include <vector>
#include <array>
#include "Segment.hpp"
#include "Wrap.hpp"

namespace spg {
namespace setpoint {

std::vector<Segment> getSegments(std::vector<Segment> segment, 
                const Eigen::Vector3d& p0, 
                const Eigen::Vector3d& v0, 
                Eigen::Vector3d pe, 
                const Eigen::Vector3d& ve, 
                const Eigen::Vector3d& vm, 
                const Eigen::Vector3d& am, 
                const Eigen::Vector3d& dm);

// Individual functions for testing
Segment move_to_vel(Segment segment, const Eigen::Vector3d& p0, const Eigen::Vector3d& v0, const Eigen::Vector3d& t0, const Eigen::Vector3d& ve, const Eigen::Vector3d& am, const Eigen::Vector3d& dm);

Segment move_at_constant_vel(Segment segment, const Eigen::Vector3d& p0, const Eigen::Vector3d& v0, const Eigen::Vector3d& t0, const Eigen::Vector3d& dt);

std::pair<Eigen::Vector3d, Eigen::Vector3d> get_max_speed(Segment segment, const Eigen::Vector3d& p0, const Eigen::Vector3d& v0, const Eigen::Vector3d& pe, const Eigen::Vector3d& ve, const Eigen::Vector3d& vm, const Eigen::Vector3d& am, const Eigen::Vector3d& dm);

}}
