#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "constrained_smoother_core/smoother.hpp"
#include "constrained_smoother_core/toml_config.hpp"

using constrained_smoother_core::ConstrainedSmoother;
using constrained_smoother_core::CostmapGrid;
using constrained_smoother_core::PathPoint;

namespace
{

double pathLength(const std::vector<PathPoint> & path)
{
  double len = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    const auto dx = path[i].x - path[i - 1].x;
    const auto dy = path[i].y - path[i - 1].y;
    len += std::hypot(dx, dy);
  }
  return len;
}

}  // namespace

int main(int argc, char ** argv)
{
  const std::string config = (argc > 1) ? argv[1] : "examples/weights.toml";

  try {
    auto cfg = constrained_smoother_core::loadTomlConfig(config);

    CostmapGrid grid;
    grid.width = 60;
    grid.height = 60;
    grid.resolution = 0.1;
    grid.values.assign(static_cast<size_t>(grid.width * grid.height), 0);
    for (int y = 20; y < 40; ++y) {
      for (int x = 28; x < 35; ++x) {
        grid.values[static_cast<size_t>(y * grid.width + x)] = 220;
      }
    }

    std::vector<PathPoint> path = {
      {0.0, 0.0, 1.0}, {0.5, 0.2, 1.0}, {1.2, 0.9, 1.0}, {1.9, 1.8, 1.0},
      {2.5, 2.4, 1.0}, {3.0, 2.9, 1.0}, {3.6, 3.1, 1.0}, {4.2, 3.2, 1.0},
      {4.8, 3.4, 1.0}, {5.4, 3.8, 1.0}
    };

    const double before = pathLength(path);
    ConstrainedSmoother smoother(cfg.optimizer);
    const bool ok = smoother.smooth(path, grid, cfg.smoother);
    const double after = pathLength(path);

    std::cout << "Smoothing success: " << std::boolalpha << ok << "\n";
    std::cout << "Path length before: " << before << "\n";
    std::cout << "Path length after : " << after << "\n";
    std::cout << "Smoothed points:\n";
    for (const auto & p : path) {
      std::cout << "  (" << p.x << ", " << p.y << ") dir=" << p.direction_sign << "\n";
    }
  } catch (const std::exception & e) {
    std::cerr << "Error: " << e.what() << "\n";
    return 1;
  }

  return 0;
}
