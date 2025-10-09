#pragma once
#include "spg/Init.hpp"
#include <vector>
#include <Eigen/Dense>

namespace spg {
namespace subtarget {

Subtarget checkCollisionFree(SPGState& d, Subtarget subtarget, double obstacle_margin, std::vector<Eigen::Vector2d>* p_robot_out = nullptr, std::vector<std::vector<Eigen::Vector2d>>* p_obstacles_traj_out = nullptr);

}}
