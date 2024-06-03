#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "angle_local_parameterization.h"
#include "ceres/ceres.h"
#include "pose_graph_2d_error_term.h"
#include "types.h"
#include "hh.h"

#include "SlamDataSet.h"

namespace ceres
{
namespace examples
{

bool BuildOptimizationProblem1(const std::vector<Constraint2d> &constraints, std::map<int, Pose2d> *poses, ceres::Problem *problem,
                              std::vector<int> &FixedPoses)
{
    if (poses == NULL || problem == NULL)
        return false;

    if (constraints.empty())
        return false;

    ceres::LossFunction *loss_function = NULL;
    ceres::LocalParameterization *angle_local_parameterization = AngleLocalParameterization::Create();

    for (std::vector<Constraint2d>::const_iterator constraints_iter = constraints.begin(); constraints_iter != constraints.end();
         ++constraints_iter)
    {
        const Constraint2d &constraint = *constraints_iter;

        std::map<int, Pose2d>::iterator pose_begin_iter = poses->find(constraint.id_begin);
        if (pose_begin_iter == poses->end())
            return false;

        std::map<int, Pose2d>::iterator pose_end_iter = poses->find(constraint.id_end);
        if (pose_end_iter == poses->end())
            return false;

        const Eigen::Matrix3d sqrt_information = constraint.information.llt().matrixL();

        // Ceres will take ownership of the pointer.
        ceres::CostFunction *cost_function =
            PoseGraph2dErrorTerm::Create(constraint.x, constraint.y, constraint.yaw_radians, sqrt_information);

        problem->AddResidualBlock(cost_function, loss_function, &pose_begin_iter->second.x, &pose_begin_iter->second.y,
                                  &pose_begin_iter->second.yaw_radians, &pose_end_iter->second.x, &pose_end_iter->second.y,
                                  &pose_end_iter->second.yaw_radians);

        problem->SetParameterization(&pose_begin_iter->second.yaw_radians, angle_local_parameterization);
        problem->SetParameterization(&pose_end_iter->second.yaw_radians, angle_local_parameterization);
    }

    // The pose graph optimization problem has three DOFs that are not fully
    // constrained. This is typically referred to as gauge freedom. You can apply
    // a rigid body transformation to all the nodes and the optimization problem
    // will still have the exact same cost. The Levenberg-Marquardt algorithm has
    // internal damping which mitigate this issue, but it is better to properly
    // constrain the gauge freedom. This can be done by setting one of the poses
    // as constant so the optimizer cannot change it.

    std::map<int, Pose2d>::iterator pose_start_iter = poses->begin();
    if (pose_start_iter == poses->end())
        return false;

    problem->SetParameterBlockConstant(&pose_start_iter->second.x);
    problem->SetParameterBlockConstant(&pose_start_iter->second.y);
    problem->SetParameterBlockConstant(&pose_start_iter->second.yaw_radians);

    for (int i = 0; i < (int)FixedPoses.size(); i++)
    {
        int j = FixedPoses[i];

        problem->SetParameterBlockConstant(&(poses->at(j).x));
        problem->SetParameterBlockConstant(&(poses->at(j).y));
        problem->SetParameterBlockConstant(&(poses->at(j).yaw_radians));
    }

    return true;
}

// Returns true if the solve was successful.
bool SolveOptimizationProblem1(ceres::Problem *problem)
{
    if (problem == NULL)
        return false;

    ceres::Solver::Options options;
    options.max_num_iterations = 100;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

    ceres::Solver::Summary summary;
    ceres::Solve(options, problem, &summary);

    std::cout << summary.FullReport() << '\n';

    return summary.IsSolutionUsable();
}

}    // namespace examples
}    // namespace ceres
