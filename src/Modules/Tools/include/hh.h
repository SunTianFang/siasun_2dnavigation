#pragma once

#include <vector>
#include <map>
#include <types.h>

namespace ceres
{
namespace examples
{
    bool BuildOptimizationProblem1(const std::vector<Constraint2d> &constraints, std::map<int, Pose2d> *poses, ceres::Problem *problem, std::vector<int> &FixedPoses);
    bool SolveOptimizationProblem1(ceres::Problem *problem);
}
}
