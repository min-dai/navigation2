#include "constrained_smoother_core/smoother.hpp"

#include <algorithm>
#include <deque>
#include <limits>
#include <stdexcept>
#include <utility>

#include <Eigen/Core>

#include "constrained_smoother_core/smoother_cost_function.hpp"

namespace constrained_smoother_core
{

ConstrainedSmoother::ConstrainedSmoother(OptimizerParams optimizer_params)
: optimizer_params_(optimizer_params)
{
  solver_options_.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  solver_options_.max_num_iterations = optimizer_params_.max_iterations;
  solver_options_.function_tolerance = optimizer_params_.fn_tol;
  solver_options_.gradient_tolerance = optimizer_params_.gradient_tol;
  solver_options_.parameter_tolerance = optimizer_params_.param_tol;
  solver_options_.logging_type = optimizer_params_.debug_optimizer ?
    ceres::PER_MINIMIZER_ITERATION : ceres::SILENT;
}

bool ConstrainedSmoother::smooth(
  std::vector<PathPoint> & path,
  const CostmapGrid & costmap,
  const SmootherParams & params) const
{
  if (!costmap.valid()) {
    throw std::invalid_argument("costmap must be valid");
  }
  if (path.size() < 3) {
    return false;
  }

  auto sampler = std::make_shared<CostmapSampler>(costmap);

  std::vector<Eigen::Vector2d> points;
  points.reserve(path.size());
  for (const auto & p : path) {
    points.emplace_back(p.x, p.y);
  }

  ceres::Problem problem;
  const double cusp_half_length = params.cusp_zone_length / 2.0;
  int prelast_i = -1;
  int last_i = 0;
  double last_direction = path[0].direction_sign;
  bool last_was_cusp = false;
  bool last_is_reversing = false;
  double last_segment_len = kEpsilon;
  double len_since_cusp = std::numeric_limits<double>::infinity();
  std::deque<std::pair<double, SmootherCostFunction *>> potential_cusp_funcs;
  double potential_cusp_funcs_len = 0.0;

  for (size_t i = 1; i < points.size(); ++i) {
    bool is_cusp = false;
    if (i != points.size() - 1) {
      is_cusp = path[i].direction_sign * last_direction < 0.0;
      last_direction = path[i].direction_sign;

      if (!is_cusp && i > (params.keep_start_orientation ? 1U : 0U) &&
        i < points.size() - (params.keep_goal_orientation ? 2U : 1U) &&
        static_cast<int>(i - last_i) < params.path_downsampling_factor)
      {
        continue;
      }
    }

    const double current_segment_len = (points[i] - points[last_i]).norm();

    potential_cusp_funcs_len += current_segment_len;
    while (!potential_cusp_funcs.empty() && potential_cusp_funcs_len > cusp_half_length) {
      potential_cusp_funcs_len -= potential_cusp_funcs.front().first;
      potential_cusp_funcs.pop_front();
    }

    if (is_cusp) {
      double len_to_cusp = current_segment_len;
      for (int ci = static_cast<int>(potential_cusp_funcs.size()) - 1; ci >= 0; --ci) {
        auto & f = potential_cusp_funcs[ci];
        const double new_weight =
          params.cusp_cost_weight_sqrt() * (1.0 - len_to_cusp / cusp_half_length) +
          params.cost_weight_sqrt() * len_to_cusp / cusp_half_length;
        if (std::abs(new_weight - params.cusp_cost_weight_sqrt()) <
          std::abs(f.second->costWeightSqrt() - params.cusp_cost_weight_sqrt()))
        {
          f.second->setCostWeightSqrt(new_weight);
        }
        len_to_cusp += f.first;
      }
      potential_cusp_funcs_len = 0.0;
      potential_cusp_funcs.clear();
      len_since_cusp = 0.0;
    }

    if (prelast_i != -1) {
      double cost_w = params.cost_weight_sqrt();
      if (len_since_cusp <= cusp_half_length) {
        cost_w = params.cusp_cost_weight_sqrt() * (1.0 - len_since_cusp / cusp_half_length) +
          params.cost_weight_sqrt() * len_since_cusp / cusp_half_length;
      }

      auto * cost_fn = new SmootherCostFunction(
        points[last_i],
        (last_was_cusp ? -1.0 : 1.0) * last_segment_len / std::max(kEpsilon, current_segment_len),
        last_is_reversing,
        sampler,
        params,
        cost_w);
      problem.AddResidualBlock(cost_fn->autoDiff(), nullptr,
        points[last_i].data(), points[i].data(), points[prelast_i].data());
      potential_cusp_funcs.emplace_back(current_segment_len, cost_fn);
    }

    last_was_cusp = is_cusp;
    last_is_reversing = last_direction < 0.0;
    prelast_i = last_i;
    last_i = static_cast<int>(i);
    len_since_cusp += current_segment_len;
    last_segment_len = std::max(kEpsilon, current_segment_len);
  }

  if (problem.NumParameterBlocks() < 3) {
    return false;
  }

  problem.SetParameterBlockConstant(points.front().data());
  if (params.keep_start_orientation && points.size() > 2) {
    problem.SetParameterBlockConstant(points[1].data());
  }
  if (params.keep_goal_orientation && points.size() > 2) {
    problem.SetParameterBlockConstant(points[points.size() - 2].data());
  }
  problem.SetParameterBlockConstant(points.back().data());

  ceres::Solver::Summary summary;
  ceres::Solve(solver_options_, &problem, &summary);
  if (!summary.IsSolutionUsable() || summary.final_cost > summary.initial_cost) {
    return false;
  }

  for (size_t i = 0; i < path.size(); ++i) {
    path[i].x = points[i].x();
    path[i].y = points[i].y();
  }
  return true;
}

}  // namespace constrained_smoother_core
