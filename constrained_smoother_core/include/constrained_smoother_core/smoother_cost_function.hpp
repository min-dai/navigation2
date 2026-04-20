#pragma once

#include <cstddef>
#include <memory>

#include <ceres/ceres.h>
#include <Eigen/Core>

#include "constrained_smoother_core/costmap.hpp"
#include "constrained_smoother_core/geometry.hpp"
#include "constrained_smoother_core/types.hpp"

namespace constrained_smoother_core
{

class SmootherCostFunction
{
public:
  SmootherCostFunction(
    const Eigen::Vector2d & original_pos,
    double next_to_last_length_ratio,
    bool reversing,
    const std::shared_ptr<CostmapSampler> & sampler,
    const SmootherParams & params,
    double cost_weight_sqrt)
  : original_pos_(original_pos),
    next_to_last_length_ratio_(next_to_last_length_ratio),
    reversing_(reversing),
    sampler_(sampler),
    params_(params),
    cost_weight_sqrt_(cost_weight_sqrt)
  {
  }

  ceres::CostFunction * autoDiff()
  {
    return new ceres::AutoDiffCostFunction<SmootherCostFunction, 6, 2, 2, 2>(this);
  }

  void setCostWeightSqrt(double weight) {cost_weight_sqrt_ = weight;}
  double costWeightSqrt() const {return cost_weight_sqrt_;}

  template<typename T>
  bool operator()(const T * const pt, const T * const pt_next, const T * const pt_prev, T * residuals) const
  {
    Eigen::Map<const Eigen::Matrix<T, 2, 1>> xi(pt);
    Eigen::Map<const Eigen::Matrix<T, 2, 1>> xi_next(pt_next);
    Eigen::Map<const Eigen::Matrix<T, 2, 1>> xi_prev(pt_prev);
    Eigen::Map<Eigen::Matrix<T, 6, 1>> residual(residuals);
    residual.setZero();

    addSmoothingResidual(params_.smooth_weight_sqrt(), xi, xi_next, xi_prev, residual[0], residual[1]);
    addCurvatureResidual(params_.curve_weight_sqrt(), xi, xi_next, xi_prev, residual[2]);
    addDistanceResidual(params_.distance_weight_sqrt(), xi, original_pos_.template cast<T>(), residual[3], residual[4]);
    addCostResidual(cost_weight_sqrt_, xi, xi_next, xi_prev, residual[5]);
    return true;
  }

private:
  template<typename T>
  void addSmoothingResidual(
    double weight_sqrt, const Eigen::Matrix<T, 2, 1> & pt,
    const Eigen::Matrix<T, 2, 1> & pt_next, const Eigen::Matrix<T, 2, 1> & pt_prev,
    T & r1, T & r2) const
  {
    const Eigen::Matrix<T, 2, 1> d_next = pt_next - pt;
    const Eigen::Matrix<T, 2, 1> d_prev = pt - pt_prev;
    const Eigen::Matrix<T, 2, 1> d_diff = static_cast<T>(next_to_last_length_ratio_) * d_next - d_prev;
    r1 += static_cast<T>(weight_sqrt) * d_diff[0];
    r2 += static_cast<T>(weight_sqrt) * d_diff[1];
  }

  template<typename T>
  void addCurvatureResidual(
    double weight_sqrt, const Eigen::Matrix<T, 2, 1> & pt,
    const Eigen::Matrix<T, 2, 1> & pt_next, const Eigen::Matrix<T, 2, 1> & pt_prev,
    T & r) const
  {
    const Eigen::Matrix<T, 2, 1> center = arcCenter(pt_prev, pt, pt_next, next_to_last_length_ratio_ < 0.0);
    if (ceres::isinf(center[0])) {
      return;
    }

    const T turning_radius = (pt - center).norm();
    const T curvature_violation = static_cast<T>(1.0) / turning_radius - static_cast<T>(params_.max_curvature());
    if (curvature_violation <= static_cast<T>(kEpsilon)) {
      return;
    }

    r += static_cast<T>(weight_sqrt) * curvature_violation;
  }

  template<typename T>
  void addDistanceResidual(
    double weight_sqrt, const Eigen::Matrix<T, 2, 1> & xi,
    const Eigen::Matrix<T, 2, 1> & xi_original, T & r1, T & r2) const
  {
    const Eigen::Matrix<T, 2, 1> diff = xi - xi_original;
    r1 += static_cast<T>(weight_sqrt) * diff[0];
    r2 += static_cast<T>(weight_sqrt) * diff[1];
  }

  template<typename T>
  void addCostResidual(
    double weight_sqrt, const Eigen::Matrix<T, 2, 1> & pt,
    const Eigen::Matrix<T, 2, 1> & pt_next, const Eigen::Matrix<T, 2, 1> & pt_prev,
    T & r) const
  {
    if (params_.cost_check_points.empty()) {
      r += static_cast<T>(weight_sqrt) * sampler_->sample(pt);
      return;
    }

    Eigen::Matrix<T, 2, 1> dir = tangentDir(pt_prev, pt, pt_next, next_to_last_length_ratio_ < 0.0);
    dir.normalize();
    if (((pt_next - pt).dot(dir) < static_cast<T>(0.0)) != reversing_) {
      dir = -dir;
    }

    Eigen::Matrix<T, 3, 3> transform;
    transform << dir[0], -dir[1], pt[0], dir[1], dir[0], pt[1], static_cast<T>(0.0), static_cast<T>(0.0), static_cast<T>(1.0);

    for (size_t i = 0; i < params_.cost_check_points.size(); i += 3) {
      Eigen::Matrix<T, 3, 1> local_pt(
        static_cast<T>(params_.cost_check_points[i]),
        static_cast<T>(params_.cost_check_points[i + 1]),
        static_cast<T>(1.0));
      const auto world_pt = (transform * local_pt).template block<2, 1>(0, 0);
      r += static_cast<T>(weight_sqrt) * static_cast<T>(params_.cost_check_points[i + 2]) * sampler_->sample(world_pt);
    }
  }

  Eigen::Vector2d original_pos_;
  double next_to_last_length_ratio_{1.0};
  bool reversing_{false};
  std::shared_ptr<CostmapSampler> sampler_;
  SmootherParams params_;
  double cost_weight_sqrt_{0.0};
};

}  // namespace constrained_smoother_core
