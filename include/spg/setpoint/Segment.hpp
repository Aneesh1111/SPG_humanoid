#pragma once
#include <Eigen/Dense>

namespace spg {
namespace setpoint {

struct Segment {
    Eigen::Vector3d t;
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    Eigen::Vector3d a;
    Eigen::Vector3d dt;
};

}}