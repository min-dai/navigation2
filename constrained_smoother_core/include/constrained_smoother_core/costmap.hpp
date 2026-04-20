#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include <ceres/cubic_interpolation.h>
#include <Eigen/Core>

namespace constrained_smoother_core
{

struct CostmapGrid
{
  int width{0};
  int height{0};
  double resolution{1.0};
  double origin_x{0.0};
  double origin_y{0.0};
  std::vector<std::uint8_t> values;

  [[nodiscard]] bool valid() const
  {
    return width > 1 && height > 1 && static_cast<int>(values.size()) == width * height;
  }
};

class CostmapSampler
{
public:
  explicit CostmapSampler(const CostmapGrid & grid)
  : grid_(grid),
    ceres_grid_(std::make_shared<ceres::Grid2D<std::uint8_t>>(
        grid_.values.data(), 0, grid_.height, 0, grid_.width)),
    interpolator_(std::make_shared<ceres::BiCubicInterpolator<ceres::Grid2D<std::uint8_t>>>(*ceres_grid_))
  {
  }

  template<typename T>
  T sample(const Eigen::Matrix<T, 2, 1> & world_xy) const
  {
    const Eigen::Matrix<T, 2, 1> interp_pos =
      (world_xy - Eigen::Matrix<T, 2, 1>(static_cast<T>(grid_.origin_x), static_cast<T>(grid_.origin_y))) /
      static_cast<T>(grid_.resolution);
    T value;
    interpolator_->Evaluate(interp_pos[1] - static_cast<T>(0.5), interp_pos[0] - static_cast<T>(0.5), &value);
    return value;
  }

private:
  const CostmapGrid & grid_;
  std::shared_ptr<ceres::Grid2D<std::uint8_t>> ceres_grid_;
  std::shared_ptr<ceres::BiCubicInterpolator<ceres::Grid2D<std::uint8_t>>> interpolator_;
};

}  // namespace constrained_smoother_core
