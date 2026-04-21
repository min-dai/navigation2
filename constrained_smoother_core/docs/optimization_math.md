# Constrained Smoother Optimization Math

This document derives the optimization objective implemented from local Navigation2 source files:
- `nav2_constrained_smoother/include/nav2_constrained_smoother/smoother_cost_function.hpp`
- `nav2_constrained_smoother/include/nav2_constrained_smoother/smoother.hpp`
- `nav2_constrained_smoother/include/nav2_constrained_smoother/utils.hpp`

## 1) Inputs, labels, and decision variables

Given a path with points \(\{\mathbf{x}_i\}_{i=0}^{N-1}\), each interior point is
\(\mathbf{x}_i = [x_i, y_i]^\top\). Start/end points (and optionally neighbors)
can be held constant.

The original input path also provides:
- original point positions \(\mathbf{x}_i^0\), used by the distance residual,
- fixed direction labels \(s_i\), where \(s_i=+1\) means forward and
  \(s_i=-1\) means reverse.

The direction labels are not optimized. They are metadata attached to the input
path indices. Thus saying "\(\mathbf{x}_i\) is a cusp" means "index \(i\) is
pre-labeled as a forward/reverse transition." The optimizer may move the
coordinate value \(\mathbf{x}_i\), but it does not decide whether index \(i\)
is a cusp.

An index \(i\) is treated as a cusp when the direction label changes:
\[
\chi_i =
\begin{cases}
1, & s_i s_{i^-} < 0,\\
0, & \text{otherwise},
\end{cases}
\]
where \(i^-\) is the previous retained path index in the smoother traversal.

The smoother may downsample non-cusp interior points. In the formulas below,
\((\mathbf{x}_{i-1}, \mathbf{x}_i, \mathbf{x}_{i+1})\) denotes a consecutive
triple in the retained sequence used by one residual block, not necessarily
three adjacent samples in the original dense input path.

Each residual block optimizes a triple
\((\mathbf{x}_{i-1}, \mathbf{x}_i, \mathbf{x}_{i+1})\) and contributes 6 scalar
residuals:
\[
\left[
r_{\text{smooth},x,i},
r_{\text{smooth},y,i},
r_{\text{curve},i},
r_{\text{dist},x,i},
r_{\text{dist},y,i},
r_{\text{cost},i}
\right]^\top.
\]

## 2) Total objective

Ceres minimizes a least-squares objective:


$$\min_{\{\mathbf{x}_i\}} \sum_i  r_{\text{smooth},x,i}^2 + r_{\text{smooth},y,i}^2+ r_{\text{curve},i}^2 + r_{\text{dist},x,i}^2 + r_{\text{dist},y,i}^2 + r_{\text{cost},i}^2 $$


where each residual is already weighted by the square-root of its corresponding
weight, matching the implementation pattern.

## 3) Smoothness residual

Define
\[
\Delta^+_i = \mathbf{x}_{i+1} - \mathbf{x}_i,
\quad
\Delta^-_i = \mathbf{x}_i - \mathbf{x}_{i-1}.
\]

The smoother uses a signed length ratio:
\[
\rho_i =
\sigma_i
\frac{\|\Delta^-_i\|}{\max(\epsilon,\|\Delta^+_i\|)},
\quad
\sigma_i =
\begin{cases}
-1, & \chi_i = 1,\\
+1, & \chi_i = 0,
\end{cases}
\]
where \(\epsilon\) prevents division by zero.

Then
\[
\mathbf{r}_{\text{smooth},i}
= \sqrt{w_{\text{smooth}}}\,(\rho_i\Delta^+_i - \Delta^-_i)
\in \mathbb{R}^2.
\]

Since Ceres stores residual blocks as scalar arrays, this vector residual is
written as two scalar residual entries:
\[
r_{\text{smooth},x,i}
= \sqrt{w_{\text{smooth}}}\,(\rho_i\Delta^+_{x,i}-\Delta^-_{x,i}),
\quad
r_{\text{smooth},y,i}
= \sqrt{w_{\text{smooth}}}\,(\rho_i\Delta^+_{y,i}-\Delta^-_{y,i}).
\]
Its contribution to the least-squares objective is therefore
\[
r_{\text{smooth},x,i}^2+r_{\text{smooth},y,i}^2
= \|\mathbf{r}_{\text{smooth},i}\|^2.
\]

Away from cusps, \(\sigma_i=+1\), so the residual encourages the incoming and
outgoing segment directions to match after normalizing for unequal segment
lengths:
\[
\rho_i\Delta^+_i \approx \Delta^-_i.
\]
For equal spacing, \(\rho_i=1\), and this becomes the standard second finite
difference:
\[
\mathbf{r}_{\text{smooth},i}
= \sqrt{w_{\text{smooth}}}\,
(\mathbf{x}_{i+1}-2\mathbf{x}_i+\mathbf{x}_{i-1}).
\]
Thus the smoothness term penalizes local second derivative / acceleration of
the path with respect to path distance.

At a cusp, \(\sigma_i=-1\), so the residual instead allows the tangent to flip:
\[
-|\rho_i|\Delta^+_i \approx \Delta^-_i.
\]
This preserves a forward/reverse transition at the pre-labeled cusp index rather
than smoothing it into an ordinary continuous-direction bend.

## 4) Curvature residual (minimum turning radius)

From three consecutive retained points, construct the osculating circle center
\(\mathbf{c}_i\) using intersection of perpendicular bisectors.
If points are (near) collinear, residual is zero.

At a cusp, the next segment is reflected before constructing the circle:
\[
\widetilde{\Delta}^+_i =
\begin{cases}
-\Delta^+_i, & \chi_i=1,\\
\Delta^+_i, & \chi_i=0.
\end{cases}
\]
This makes the curvature calculation treat a forward/reverse cusp as a tangent
reversal rather than as an extremely sharp same-direction turn.

For the circle-center calculation, define:
\[
\mathbf{d}_1 = \Delta^-_i,
\quad
\mathbf{d}_2 = \widetilde{\Delta}^+_i,
\quad
\widetilde{\mathbf{x}}_{i+1} = \mathbf{x}_i + \mathbf{d}_2.
\]

The two segment midpoints are:
\[
\mathbf{m}_1 = \frac{\mathbf{x}_{i-1}+\mathbf{x}_i}{2},
\quad
\mathbf{m}_2 = \frac{\mathbf{x}_i+\widetilde{\mathbf{x}}_{i+1}}{2}.
\]

Perpendicular directions to the two segments are:
\[
\mathbf{n}_1 =
\begin{bmatrix}
-d_{1y}\\
d_{1x}
\end{bmatrix},
\quad
\mathbf{n}_2 =
\begin{bmatrix}
-d_{2y}\\
d_{2x}
\end{bmatrix}.
\]

The perpendicular bisectors are the two lines:
\[
\ell_1(t) = \mathbf{m}_1 + t\mathbf{n}_1,
\quad
\ell_2(u) = \mathbf{m}_2 + u\mathbf{n}_2.
\]

The circle center is their intersection:
\[
\mathbf{c}_i = \ell_1(t^*) = \ell_2(u^*).
\]

Equivalently, the implementation uses the 2D line-intersection determinant
form. Let
\[
D = d_{1x}d_{2y} - d_{1y}d_{2x}.
\]
If \(|D| < \epsilon\), the points are treated as collinear and no curvature
penalty is applied. Otherwise define
\[
q_1 = (m_{1x}+n_{1x})m_{1y} - (m_{1y}+n_{1y})m_{1x},
\quad
q_2 = (m_{2x}+n_{2x})m_{2y} - (m_{2y}+n_{2y})m_{2x}.
\]
Then
\[
\mathbf{c}_i =
\begin{bmatrix}
\dfrac{q_1 n_{2x} - q_2 n_{1x}}{D}\\[6pt]
\dfrac{q_1 n_{2y} - q_2 n_{1y}}{D}
\end{bmatrix}.
\]

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
Given body-frame checkpoints \((u_k, v_k, \alpha_k)\), build a local tangent-frame
transform at \(\mathbf{x}_i\), map each checkpoint to world \(\mathbf{p}_{ik}\), then
\[
r_{\text{cost},i} = \sqrt{w_{\text{cost},i}}\sum_k \alpha_k\,C(\mathbf{p}_{ik})
\]

The local tangent direction is also computed using the cusp-aware geometry
above. It is then oriented using the fixed reverse/forward label for the segment,
so that body-frame checkpoints stay aligned with the vehicle body direction
rather than only with the geometric chord.

## 7) Cusp-zone weighting

Around forward/reverse cusps, cost weight is linearly ramped from base to cusp
multiplied weight within half-length \(L_c/2\). This cusp zone affects the
costmap residual weight near a cusp. It does not change which indices are cusps;
those come from the fixed direction labels.
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
