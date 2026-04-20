#pragma once

#include <cmath>
#include <limits>

#include <Eigen/Core>
#include <ceres/jet.h>

namespace constrained_smoother_core
{

constexpr double kEpsilon = 1e-4;

template<typename T>
inline Eigen::Matrix<T, 2, 1> arcCenter(
  Eigen::Matrix<T, 2, 1> pt_prev,
  Eigen::Matrix<T, 2, 1> pt,
  Eigen::Matrix<T, 2, 1> pt_next,
  bool is_cusp)
{
  Eigen::Matrix<T, 2, 1> d1 = pt - pt_prev;
  Eigen::Matrix<T, 2, 1> d2 = pt_next - pt;

  if (is_cusp) {
    d2 = -d2;
    pt_next = pt + d2;
  }

  T det = d1[0] * d2[1] - d1[1] * d2[0];
  if (ceres::abs(det) < static_cast<T>(kEpsilon)) {
    const auto inf = static_cast<T>(std::numeric_limits<double>::infinity());
    return Eigen::Matrix<T, 2, 1>(inf, inf);
  }

  Eigen::Matrix<T, 2, 1> mid1 = (pt_prev + pt) / static_cast<T>(2.0);
  Eigen::Matrix<T, 2, 1> mid2 = (pt + pt_next) / static_cast<T>(2.0);
  Eigen::Matrix<T, 2, 1> n1(-d1[1], d1[0]);
  Eigen::Matrix<T, 2, 1> n2(-d2[1], d2[0]);
  T det1 = (mid1[0] + n1[0]) * mid1[1] - (mid1[1] + n1[1]) * mid1[0];
  T det2 = (mid2[0] + n2[0]) * mid2[1] - (mid2[1] + n2[1]) * mid2[0];
  return Eigen::Matrix<T, 2, 1>((det1 * n2[0] - det2 * n1[0]) / det, (det1 * n2[1] - det2 * n1[1]) / det);
}

template<typename T>
inline Eigen::Matrix<T, 2, 1> tangentDir(
  Eigen::Matrix<T, 2, 1> pt_prev,
  Eigen::Matrix<T, 2, 1> pt,
  Eigen::Matrix<T, 2, 1> pt_next,
  bool is_cusp)
{
  Eigen::Matrix<T, 2, 1> center = arcCenter(pt_prev, pt, pt_next, is_cusp);
  if (ceres::isinf(center[0])) {
    Eigen::Matrix<T, 2, 1> d1 = pt - pt_prev;
    Eigen::Matrix<T, 2, 1> d2 = pt_next - pt;

    if (is_cusp) {
      d2 = -d2;
      pt_next = pt + d2;
    }

    Eigen::Matrix<T, 2, 1> result(pt_next[0] - pt_prev[0], pt_next[1] - pt_prev[1]);
    if (result[0] == static_cast<T>(0.0) && result[1] == static_cast<T>(0.0)) {
      return Eigen::Matrix<T, 2, 1>(d1[1], -d1[0]);
    }
    return result;
  }

  return Eigen::Matrix<T, 2, 1>(center[1] - pt[1], pt[0] - center[0]);
}

}  // namespace constrained_smoother_core
