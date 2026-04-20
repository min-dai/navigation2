#include <gtest/gtest.h>

#include <vector>

#include "constrained_smoother_core/smoother.hpp"

using constrained_smoother_core::ConstrainedSmoother;
using constrained_smoother_core::CostmapGrid;
using constrained_smoother_core::OptimizerParams;
using constrained_smoother_core::PathPoint;
using constrained_smoother_core::SmootherParams;

TEST(Smoother, RunsAndMovesInteriorPoints)
{
  CostmapGrid grid;
  grid.width = 40;
  grid.height = 40;
  grid.resolution = 0.1;
  grid.values.assign(static_cast<size_t>(grid.width * grid.height), 0);
  for (int y = 15; y < 30; ++y) {
    for (int x = 20; x < 25; ++x) {
      grid.values[static_cast<size_t>(y * grid.width + x)] = 250;
    }
  }

  std::vector<PathPoint> path = {
    {0.0, 0.0, 1.0}, {0.6, 0.3, 1.0}, {1.1, 1.0, 1.0}, {1.8, 1.7, 1.0},
    {2.4, 2.4, 1.0}, {3.0, 3.1, 1.0}, {3.8, 3.3, 1.0}, {4.5, 3.5, 1.0}
  };

  const auto original = path;

  SmootherParams sparams;
  sparams.smooth_weight = 3000.0;
  sparams.cost_weight = 1e-3;
  sparams.curve_weight = 0.5;

  OptimizerParams oparams;
  oparams.max_iterations = 40;

  ConstrainedSmoother smoother(oparams);
  const bool ok = smoother.smooth(path, grid, sparams);

  EXPECT_TRUE(ok);
  EXPECT_DOUBLE_EQ(path.front().x, original.front().x);
  EXPECT_DOUBLE_EQ(path.back().x, original.back().x);

  double displacement = 0.0;
  for (size_t i = 1; i + 1 < path.size(); ++i) {
    displacement += std::abs(path[i].x - original[i].x) + std::abs(path[i].y - original[i].y);
  }
  EXPECT_GT(displacement, 1e-6);
}
