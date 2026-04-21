#!/usr/bin/env python3
"""Plain WebSocket server for the constrained smoother demo.

Run with:
  python app.py

Then open dashboard.html in a browser.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import List

import numpy as np
from scipy.optimize import least_squares
import websockets


OBSTACLE_CLEARANCE = 0.25
OBSTACLE_WEIGHT_SCALE = 5.0
SEGMENT_COST_SAMPLES = (0.25, 0.5, 0.75)


@dataclass
class Params:
    smooth_weight: float = 3000.0
    cost_weight: float = 4.5e-5
    distance_weight: float = 0.0
    curve_weight: float = 0.5
    minimum_turning_radius: float = 0.4
    keep_start_orientation: bool = True
    keep_goal_orientation: bool = True


@dataclass
class Obstacle:
    x: float = 2.8
    y: float = 2.0
    width: float = 0.7
    height: float = 2.0
    cost: float = 220.0


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


def rectangle_signed_distance(xy: np.ndarray, obstacle: Obstacle) -> float:
    left = obstacle.x
    right = obstacle.x + obstacle.width
    bottom = obstacle.y
    top = obstacle.y + obstacle.height
    x = xy[0]
    y = xy[1]

    outside_dx = max(left - x, 0.0, x - right)
    outside_dy = max(bottom - y, 0.0, y - top)
    outside_distance = math.hypot(outside_dx, outside_dy)
    if outside_distance > 0.0:
        return outside_distance

    return -min(x - left, right - x, y - bottom, top - y)


def obstacle_residual(xy: np.ndarray, obstacle: Obstacle, weight: float) -> float:
    clearance_error = OBSTACLE_CLEARANCE - rectangle_signed_distance(xy, obstacle)
    if clearance_error <= 0.0:
        return 0.0
    return weight * clearance_error / OBSTACLE_CLEARANCE


def smooth_path(path: np.ndarray, params: Params, obstacle: Obstacle):
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

    def unpack(base_path: np.ndarray, xvec: np.ndarray) -> np.ndarray:
        p = base_path.copy()
        p[variable_idx, :2] = xvec.reshape(-1, 2)
        return p

    max_curvature = 1.0 / max(params.minimum_turning_radius, 1e-6)
    ws = math.sqrt(params.smooth_weight)
    wc = math.sqrt(params.cost_weight)
    wd = math.sqrt(params.distance_weight)
    wcur = math.sqrt(params.curve_weight)

    def residuals(base_path: np.ndarray, xvec: np.ndarray) -> np.ndarray:
        p = unpack(base_path, xvec)
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
            r.append(obstacle_residual(pi, obstacle, wc * obstacle.cost * OBSTACLE_WEIGHT_SCALE))

        for i in range(n - 1):
            p0 = p[i, :2]
            p1 = p[i + 1, :2]
            for t in SEGMENT_COST_SAMPLES:
                sample = (1.0 - t) * p0 + t * p1
                r.append(obstacle_residual(sample, obstacle, wc * obstacle.cost * OBSTACLE_WEIGHT_SCALE))

        return np.array(r, dtype=np.float64)

    def seeded_path(side: str) -> np.ndarray:
        seed = path.copy()
        left = obstacle.x - OBSTACLE_CLEARANCE
        right = obstacle.x + obstacle.width + OBSTACLE_CLEARANCE
        target_y = obstacle.y - OBSTACLE_CLEARANCE if side == "below" else obstacle.y + obstacle.height + OBSTACLE_CLEARANCE
        for i in variable_idx:
            x = seed[i, 0]
            if left <= x <= right:
                seed[i, 1] = target_y
        return seed

    def clearance_violation(candidate: np.ndarray) -> float:
        violation = 0.0
        for i in range(n - 1):
            p0 = candidate[i, :2]
            p1 = candidate[i + 1, :2]
            for t in np.linspace(0.0, 1.0, 11):
                sample = (1.0 - t) * p0 + t * p1
                violation += max(0.0, OBSTACLE_CLEARANCE - rectangle_signed_distance(sample, obstacle))
        return violation

    best_path = path
    best_norm = math.inf
    best_violation = math.inf
    for seed in (path, seeded_path("below"), seeded_path("above")):
        x0 = seed[variable_idx, :2].reshape(-1)
        result = least_squares(lambda xvec: residuals(seed, xvec), x0, max_nfev=120)
        candidate = unpack(seed, result.x)
        norm = float(np.linalg.norm(residuals(seed, result.x)))
        violation = clearance_violation(candidate)
        if (violation, norm) < (best_violation, best_norm):
            best_path = candidate
            best_norm = norm
            best_violation = violation

    return best_path, best_norm


def default_path() -> np.ndarray:
    return np.array(
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


def obstacle_outline(obstacle: Obstacle) -> np.ndarray:
    return np.array(
        [
            [obstacle.x, obstacle.y],
            [obstacle.x + obstacle.width, obstacle.y],
            [obstacle.x + obstacle.width, obstacle.y + obstacle.height],
            [obstacle.x, obstacle.y + obstacle.height],
            [obstacle.x, obstacle.y],
        ],
        dtype=np.float64,
    )


def make_state(path: np.ndarray, solved: np.ndarray, params: Params, obstacle: Obstacle, residual_norm: float):
    return {
        "type": "state",
        "residual_norm": residual_norm,
        "params": params.__dict__,
        "obstacle": obstacle.__dict__,
        "original_path": [{"x": float(p[0]), "y": float(p[1]), "direction_sign": float(p[2])} for p in path],
        "smoothed_path": [{"x": float(p[0]), "y": float(p[1]), "direction_sign": float(p[2])} for p in solved],
        "obstacle_outline": [{"x": float(p[0]), "y": float(p[1])} for p in obstacle_outline(obstacle)],
    }


def apply_updates(target, values: dict, allowed: dict[str, tuple[float | None, float | None]]) -> None:
    for key, value in values.items():
        if key not in allowed:
            continue
        low, high = allowed[key]
        if isinstance(getattr(target, key), bool):
            setattr(target, key, bool(value))
            continue
        try:
            number = float(value)
        except (TypeError, ValueError):
            continue
        if low is not None:
            number = max(low, number)
        if high is not None:
            number = min(high, number)
        setattr(target, key, number)


def solve_state(params: Params, obstacle: Obstacle) -> dict:
    path = default_path()
    solved, residual_norm = smooth_path(path, params, obstacle)
    return make_state(path, solved, params, obstacle, residual_norm)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Serve constrained smoother data over a plain local WebSocket.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--smooth-weight", type=float, default=3000.0)
    parser.add_argument("--cost-weight", type=float, default=4.5e-5)
    parser.add_argument("--distance-weight", type=float, default=0.0)
    parser.add_argument("--curve-weight", type=float, default=0.5)
    parser.add_argument("--minimum-turning-radius", type=float, default=0.4)
    parser.add_argument("--allow-start-orientation", action="store_true", help="Allow the second path point to move.")
    parser.add_argument("--allow-goal-orientation", action="store_true", help="Allow the penultimate path point to move.")
    parser.add_argument("--obstacle-x", type=float, default=2.8)
    parser.add_argument("--obstacle-y", type=float, default=2.0)
    parser.add_argument("--obstacle-width", type=float, default=0.7)
    parser.add_argument("--obstacle-height", type=float, default=2.0)
    parser.add_argument("--obstacle-cost", type=float, default=220.0)
    return parser.parse_args()


async def run_server(args: argparse.Namespace) -> None:
    params = Params(
        smooth_weight=args.smooth_weight,
        cost_weight=args.cost_weight,
        distance_weight=args.distance_weight,
        curve_weight=args.curve_weight,
        minimum_turning_radius=args.minimum_turning_radius,
        keep_start_orientation=not args.allow_start_orientation,
        keep_goal_orientation=not args.allow_goal_orientation,
    )
    obstacle = Obstacle(
        x=args.obstacle_x,
        y=args.obstacle_y,
        width=args.obstacle_width,
        height=args.obstacle_height,
        cost=args.obstacle_cost,
    )

    param_limits = {
        "smooth_weight": (0.0, 20000.0),
        "cost_weight": (0.0, 0.01),
        "distance_weight": (0.0, 20.0),
        "curve_weight": (0.0, 20.0),
        "minimum_turning_radius": (0.05, 2.0),
        "keep_start_orientation": (None, None),
        "keep_goal_orientation": (None, None),
    }
    obstacle_limits = {
        "x": (0.0, 6.5),
        "y": (0.0, 6.5),
        "width": (0.1, 3.0),
        "height": (0.1, 3.0),
        "cost": (0.0, 255.0),
    }

    async def send_state(websocket) -> None:
        state = await asyncio.to_thread(solve_state, params, obstacle)
        await websocket.send(json.dumps(state))

    async def handler(websocket) -> None:
        await send_state(websocket)
        async for raw_message in websocket:
            try:
                message = json.loads(raw_message)
            except json.JSONDecodeError:
                await websocket.send(json.dumps({"type": "error", "message": "invalid json"}))
                continue

            if message.get("type") == "update":
                apply_updates(params, message.get("params", {}), param_limits)
                apply_updates(obstacle, message.get("obstacle", {}), obstacle_limits)
                await send_state(websocket)
            elif message.get("type") == "reset":
                params.__dict__.update(Params().__dict__)
                obstacle.__dict__.update(Obstacle().__dict__)
                await send_state(websocket)
            else:
                await websocket.send(json.dumps({"type": "error", "message": "unknown message type"}))

    dashboard = Path(__file__).with_name("dashboard.html")
    print(f"WebSocket server listening on ws://{args.host}:{args.port}")
    print(f"Dashboard: {dashboard}")
    async with websockets.serve(handler, args.host, args.port):
        await asyncio.Future()


def run() -> None:
    asyncio.run(run_server(parse_args()))


if __name__ == "__main__":
    run()
