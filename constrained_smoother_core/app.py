#!/usr/bin/env python3
"""Interactive web tuner for constrained smoother parameters and obstacle settings.

Run with either:
  python app.py
or:
  streamlit run app.py
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from typing import List, Tuple

import matplotlib.pyplot as plt
import numpy as np
import streamlit as st
from scipy.optimize import least_squares


@dataclass
class Params:
    smooth_weight: float = 3000.0
    cost_weight: float = 4.5e-5
    distance_weight: float = 0.0
    curve_weight: float = 0.5
    minimum_turning_radius: float = 0.4
    keep_start_orientation: bool = True
    keep_goal_orientation: bool = True


def arc_center(p_prev: np.ndarray, p: np.ndarray, p_next: np.ndarray) -> np.ndarray:
    d1 = p - p_prev
    d2 = p_next - p
    det = d1[0] * d2[1] - d1[1] * d2[0]
    if abs(det) < 1e-4:
        return np.array([np.inf, np.inf])
    mid1 = (p_prev + p) / 2.0
    mid2 = (p + p_next) / 2.0
    n1 = np.array([-d1[1], d1[0]])
    n2 = np.array([-d2[1], d2[0]])
    det1 = (mid1[0] + n1[0]) * mid1[1] - (mid1[1] + n1[1]) * mid1[0]
    det2 = (mid2[0] + n2[0]) * mid2[1] - (mid2[1] + n2[1]) * mid2[0]
    return np.array([(det1 * n2[0] - det2 * n1[0]) / det, (det1 * n2[1] - det2 * n1[1]) / det])


def bilinear_sample(costmap: np.ndarray, xy: np.ndarray, origin: Tuple[float, float], resolution: float) -> float:
    gx = (xy[0] - origin[0]) / resolution
    gy = (xy[1] - origin[1]) / resolution
    if gx < 0 or gy < 0 or gx > costmap.shape[1] - 1 or gy > costmap.shape[0] - 1:
        return 0.0
    x0, y0 = int(np.floor(gx)), int(np.floor(gy))
    x1, y1 = min(x0 + 1, costmap.shape[1] - 1), min(y0 + 1, costmap.shape[0] - 1)
    dx, dy = gx - x0, gy - y0
    v00 = costmap[y0, x0]
    v10 = costmap[y0, x1]
    v01 = costmap[y1, x0]
    v11 = costmap[y1, x1]
    return float((1 - dx) * (1 - dy) * v00 + dx * (1 - dy) * v10 + (1 - dx) * dy * v01 + dx * dy * v11)


def build_costmap(size: int, obstacle_x: float, obstacle_y: float, obstacle_w: float, obstacle_h: float, obstacle_cost: float):
    resolution = 0.1
    origin = (0.0, 0.0)
    grid = np.zeros((size, size), dtype=np.float64)

    x0 = int(max(0, obstacle_x / resolution))
    y0 = int(max(0, obstacle_y / resolution))
    x1 = int(min(size, (obstacle_x + obstacle_w) / resolution))
    y1 = int(min(size, (obstacle_y + obstacle_h) / resolution))
    grid[y0:y1, x0:x1] = obstacle_cost
    return grid, origin, resolution


def smooth_path(path: np.ndarray, params: Params, costmap: np.ndarray, origin: Tuple[float, float], resolution: float):
    original = path.copy()
    n = len(path)

    fixed = {0, n - 1}
    if params.keep_start_orientation and n > 2:
        fixed.add(1)
    if params.keep_goal_orientation and n > 2:
        fixed.add(n - 2)

    variable_idx = [i for i in range(n) if i not in fixed]
    if not variable_idx:
        return path, 0.0

    x0 = path[variable_idx, :2].reshape(-1)

    def unpack(xvec: np.ndarray) -> np.ndarray:
        p = path.copy()
        p[variable_idx, :2] = xvec.reshape(-1, 2)
        return p

    max_curvature = 1.0 / max(params.minimum_turning_radius, 1e-6)
    ws = math.sqrt(params.smooth_weight)
    wc = math.sqrt(params.cost_weight)
    wd = math.sqrt(params.distance_weight)
    wcur = math.sqrt(params.curve_weight)

    def residuals(xvec: np.ndarray) -> np.ndarray:
        p = unpack(xvec)
        r: List[float] = []
        for i in range(1, n - 1):
            pi = p[i, :2]
            pprev = p[i - 1, :2]
            pnext = p[i + 1, :2]

            len_prev = np.linalg.norm(pi - pprev)
            len_next = np.linalg.norm(pnext - pi)
            ratio = len_prev / max(len_next, 1e-4)

            d_diff = ratio * (pnext - pi) - (pi - pprev)
            r.extend((ws * d_diff).tolist())

            center = arc_center(pprev, pi, pnext)
            if not np.isinf(center[0]):
                turn_radius = np.linalg.norm(pi - center)
                k_violation = 1.0 / max(turn_radius, 1e-6) - max_curvature
                r.append(wcur * max(0.0, k_violation))
            else:
                r.append(0.0)

            r.extend((wd * (pi - original[i, :2])).tolist())
            r.append(wc * bilinear_sample(costmap, pi, origin, resolution))

        return np.array(r, dtype=np.float64)

    result = least_squares(residuals, x0, max_nfev=60)
    return unpack(result.x), float(np.linalg.norm(residuals(result.x)))


def default_path() -> np.ndarray:
    pts = np.array(
        [
            [0.0, 0.0, 1.0],
            [0.5, 0.2, 1.0],
            [1.2, 0.9, 1.0],
            [1.9, 1.8, 1.0],
            [2.5, 2.4, 1.0],
            [3.0, 2.9, 1.0],
            [3.6, 3.1, 1.0],
            [4.2, 3.2, 1.0],
            [4.8, 3.4, 1.0],
            [5.4, 3.8, 1.0],
        ],
        dtype=np.float64,
    )
    return pts


def render_app() -> None:
    st.set_page_config(page_title="Constrained Smoother Live Tuner", layout="wide")
    st.title("Constrained Smoother — Live Tuning Session")
    st.caption("Tune optimization weights and obstacle settings, then inspect the solved path.")

    c1, c2 = st.columns(2)

    with c1:
        st.subheader("Optimization Parameters")
        params = Params(
            smooth_weight=st.slider("smooth_weight", 10.0, 20000.0, 3000.0, 10.0),
            cost_weight=st.slider("cost_weight", 0.0, 0.01, 4.5e-5, 1e-5, format="%.6f"),
            distance_weight=st.slider("distance_weight", 0.0, 20.0, 0.0, 0.1),
            curve_weight=st.slider("curve_weight", 0.0, 20.0, 0.5, 0.1),
            minimum_turning_radius=st.slider("minimum_turning_radius", 0.05, 2.0, 0.4, 0.01),
            keep_start_orientation=st.checkbox("keep_start_orientation", value=True),
            keep_goal_orientation=st.checkbox("keep_goal_orientation", value=True),
        )

    with c2:
        st.subheader("Obstacle Settings")
        obstacle_x = st.slider("obstacle_x", 0.0, 5.0, 2.8, 0.1)
        obstacle_y = st.slider("obstacle_y", 0.0, 5.0, 2.0, 0.1)
        obstacle_w = st.slider("obstacle_width", 0.1, 3.0, 0.7, 0.1)
        obstacle_h = st.slider("obstacle_height", 0.1, 3.0, 2.0, 0.1)
        obstacle_cost = st.slider("obstacle_cost", 0.0, 255.0, 220.0, 1.0)

    path = default_path()
    costmap, origin, resolution = build_costmap(70, obstacle_x, obstacle_y, obstacle_w, obstacle_h, obstacle_cost)
    solved, residual_norm = smooth_path(path, params, costmap, origin, resolution)

    fig, ax = plt.subplots(figsize=(8, 6))
    ax.plot(path[:, 0], path[:, 1], "o--", label="original", color="gray")
    ax.plot(solved[:, 0], solved[:, 1], "o-", label="smoothed", color="tab:blue")
    rect = plt.Rectangle((obstacle_x, obstacle_y), obstacle_w, obstacle_h, color="tab:red", alpha=0.25)
    ax.add_patch(rect)
    ax.set_xlim(-0.2, 6.8)
    ax.set_ylim(-0.2, 6.8)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.3)
    ax.legend()
    st.pyplot(fig)

    st.metric("Residual norm", f"{residual_norm:.3f}")
    st.dataframe(
        {
            "x_original": path[:, 0],
            "y_original": path[:, 1],
            "x_solved": solved[:, 0],
            "y_solved": solved[:, 1],
        },
        use_container_width=True,
    )


def run() -> None:
    if getattr(st, "_is_running_with_streamlit", False):
        render_app()
        return

    from streamlit.web import cli as stcli

    sys.argv = ["streamlit", "run", __file__, "--server.headless=false", "--server.port=8501"]
    raise SystemExit(stcli.main())


if __name__ == "__main__":
    run()
