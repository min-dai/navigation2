# Constrained Smoother Optimization Math

This document derives the optimization objective implemented from local Navigation2 source files:
- `nav2_constrained_smoother/include/nav2_constrained_smoother/smoother_cost_function.hpp`
- `nav2_constrained_smoother/include/nav2_constrained_smoother/smoother.hpp`
- `nav2_constrained_smoother/include/nav2_constrained_smoother/utils.hpp`

## 1) Decision variables

Given a path with points \(\{\mathbf{x}_i\}_{i=0}^{N-1}\), each interior point is
\(\mathbf{x}_i = [x_i, y_i]^\top\). Start/end points (and optionally neighbors)
can be held constant.

Each residual block optimizes a triple \((\mathbf{x}_{i-1}, \mathbf{x}_i, \mathbf{x}_{i+1})\)
and contributes 6 scalar residuals.

## 2) Total objective

Ceres minimizes a least-squares objective:

\[
\min_{\{\mathbf{x}_i\}} \sum_i
\left( r_{\text{smooth},x,i}^2 + r_{\text{smooth},y,i}^2
+ r_{\text{curve},i}^2
+ r_{\text{dist},x,i}^2 + r_{\text{dist},y,i}^2
+ r_{\text{cost},i}^2 \right)
\]

where each residual is already weighted by the square-root of its corresponding
weight, matching the implementation pattern.

## 3) Smoothness residual

Define
\[
\Delta^+_i = \mathbf{x}_{i+1} - \mathbf{x}_i,
\quad
\Delta^-_i = \mathbf{x}_i - \mathbf{x}_{i-1},
\quad
\rho_i = \frac{\|\mathbf{x}_i-\mathbf{x}_{i-1}\|}{\|\mathbf{x}_{i+1}-\mathbf{x}_i\|}
\]
(with sign flip near cusps).

Then
\[
\mathbf{r}_{\text{smooth},i} = \sqrt{w_{\text{smooth}}}\,(\rho_i\Delta^+_i - \Delta^-_i)
\]
with x/y components emitted separately.

## 4) Curvature residual (minimum turning radius)

From three consecutive points, construct the osculating circle center
\(\mathbf{c}_i\) using intersection of perpendicular bisectors.
If points are (near) collinear, residual is zero.

Turning radius:
\[
R_i = \|\mathbf{x}_i - \mathbf{c}_i\|,
\quad
\kappa_i = \frac{1}{R_i}
\]

Maximum allowed curvature from minimum turning radius \(R_{\min}\):
\[
\kappa_{\max} = \frac{1}{R_{\min}}
\]

Hinge-style residual:
\[
r_{\text{curve},i} =
\sqrt{w_{\text{curve}}}\,\max(0, \kappa_i - \kappa_{\max})
\]

## 5) Distance-to-original residual

Let original point be \(\mathbf{x}_i^0\). Then
\[
\mathbf{r}_{\text{dist},i} = \sqrt{w_{\text{dist}}}\,(\mathbf{x}_i - \mathbf{x}_i^0)
\]
(x/y components used as two residuals).

## 6) Costmap residual

Let \(C(\mathbf{x})\) be bicubically interpolated costmap value at world position
\(\mathbf{x}\).

### Single-point mode
\[
r_{\text{cost},i} = \sqrt{w_{\text{cost},i}}\,C(\mathbf{x}_i)
\]

### Multi-checkpoint mode
Given body-frame checkpoints \((u_k, v_k, \alpha_k)\), build local tangent-frame
transform at \(\mathbf{x}_i\), map each checkpoint to world \(\mathbf{p}_{ik}\), then
\[
r_{\text{cost},i} = \sqrt{w_{\text{cost},i}}\sum_k \alpha_k\,C(\mathbf{p}_{ik})
\]

## 7) Cusp-zone weighting

Around forward/reverse cusps, cost weight is linearly ramped from base to cusp
multiplied weight within half-length \(L_c/2\):
\[
\sqrt{w_{\text{cost},i}} =
\sqrt{w_{\text{cost}}}\cdot\frac{d_i}{L_c/2} +
\sqrt{w_{\text{cost}}\,m_{\text{cusp}}}\cdot\left(1-\frac{d_i}{L_c/2}\right)
\]
for distance \(d_i\in[0,L_c/2]\), else base weight.

## 8) Notes on constraints and fixed points

The solver uses unconstrained nonlinear least squares, while endpoint behavior
is enforced by setting selected parameter blocks constant:
- point 0 and point \(N-1\) are fixed,
- optionally point 1 and point \(N-2\) are fixed to preserve endpoint orientation.
