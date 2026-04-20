#pragma once

#include <cmath>
#include <vector>

namespace constrained_smoother_core
{

struct PathPoint
{
  double x{0.0};
  double y{0.0};
  double direction_sign{1.0};  // +1 forward, -1 reverse
};

struct SmootherParams
{
  double smooth_weight{3.0e3};
  double cost_weight{4.5e-5};
  double cusp_cost_multiplier{3.0};
  double cusp_zone_length{2.5};
  double distance_weight{0.0};
  double curve_weight{0.5};
  double minimum_turning_radius{0.4};
  int path_downsampling_factor{1};
  bool keep_goal_orientation{true};
  bool keep_start_orientation{true};
  bool reversing_enabled{true};
  std::vector<double> cost_check_points{};  // [x, y, weight, ...]

  double smooth_weight_sqrt() const {return std::sqrt(smooth_weight);}  // NOLINT
  double cost_weight_sqrt() const {return std::sqrt(cost_weight);}  // NOLINT
  double cusp_cost_weight_sqrt() const {return std::sqrt(cost_weight * cusp_cost_multiplier);}  // NOLINT
  double distance_weight_sqrt() const {return std::sqrt(distance_weight);}  // NOLINT
  double curve_weight_sqrt() const {return std::sqrt(curve_weight);}  // NOLINT
  double max_curvature() const {return 1.0 / minimum_turning_radius;}  // NOLINT
};

struct OptimizerParams
{
  bool debug_optimizer{false};
  int max_iterations{70};
  double gradient_tol{50.0};
  double fn_tol{1.0e-15};
  double param_tol{1.0e-20};
};

}  // namespace constrained_smoother_core
