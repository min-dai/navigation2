#include <gtest/gtest.h>

#include <Eigen/Core>

#include "constrained_smoother_core/geometry.hpp"

using constrained_smoother_core::arcCenter;
using constrained_smoother_core::tangentDir;

TEST(Geometry, ArcCenterCircle)
{
  const Eigen::Vector2d p0(1.0, 0.0);
  const Eigen::Vector2d p1(0.0, 1.0);
  const Eigen::Vector2d p2(-1.0, 0.0);
  const auto c = arcCenter(p0, p1, p2, false);

  EXPECT_NEAR(c.x(), 0.0, 1e-6);
  EXPECT_NEAR(c.y(), 0.0, 1e-6);
}

TEST(Geometry, TangentDirectionStraight)
{
  const Eigen::Vector2d p0(0.0, 0.0);
  const Eigen::Vector2d p1(1.0, 0.0);
  const Eigen::Vector2d p2(2.0, 0.0);
  const auto t = tangentDir(p0, p1, p2, false);

  EXPECT_NEAR(t.y(), 0.0, 1e-8);
  EXPECT_GT(t.x(), 0.0);
}
