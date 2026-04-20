#pragma once

#include <vector>

#include <ceres/ceres.h>

#include "constrained_smoother_core/costmap.hpp"
#include "constrained_smoother_core/types.hpp"

namespace constrained_smoother_core
{

class ConstrainedSmoother
{
public:
  explicit ConstrainedSmoother(OptimizerParams optimizer_params = {});

  bool smooth(
    std::vector<PathPoint> & path,
    const CostmapGrid & costmap,
    const SmootherParams & params) const;

private:
  OptimizerParams optimizer_params_;
  ceres::Solver::Options solver_options_;
};

}  // namespace constrained_smoother_core
