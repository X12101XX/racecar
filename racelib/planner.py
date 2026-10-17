"""路径规划：封锁线缝隙分析 + A* + 路径平滑 + 车辆足迹校验。"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from . import geometry as G
from .field import Cone, Field
from .fusion import ConeTrack
from .gridmap import GridMap


# --------------------------------------------------------------------------- #
# 封锁线缝隙
# --------------------------------------------------------------------------- #
@dataclass
class Gap:
    """一条封锁线上的可通行缝隙。"""

    line: int
    y: float
    width: float
    risk: float = 0.0

    @property
    def center(self) -> float:
        return self.y


def gap_candidates(cones_y: Sequence[float], y_lo: float, y_hi: float,
                   min_gap: float, cone_radius: float = 0.11,
                   margin: float = 0.20) -> List[Gap]:
    """把一条封锁线上的锥桶 y 坐标转成可通行缝隙列表（按宽度降序）。"""
    ys = sorted(float(y) for y in cones_y if y_lo - 0.5 <= y <= y_hi + 0.5)
    edges = [y_lo - cone_radius] + ys + [y_hi + cone_radius]
    out: List[Gap] = []
    for k in range(len(edges) - 1):
        a, b = edges[k], edges[k + 1]
        # 缝隙净宽 = 相邻锥桶边缘间距
        if k == 0:
            a = y_lo
        elif k == len(edges) - 2:
            b = y_hi
        else:
            a = a + cone_radius
            b = b - cone_radius
        w = b - a
        if w <= 1e-6:
            continue
        out.append(Gap(line=-1, y=0.5 * (a + b), width=w))
    usable = [g for g in out if g.width >= min_gap]
    usable.sort(key=lambda g: -g.width)
    return usable


def plan_gap_waypoints(field: Field, tracks: Sequence[ConeTrack],
                       entry: np.ndarray, exit_: np.ndarray,
                       y_lo: float, y_hi: float,
                       min_gap: float, cone_radius: float = 0.11,
                       line_tol: float = 0.40, narrow_penalty: float = 3.0,
                       prefer_gap: float = 0.90) -> List[np.ndarray]:
    """三层封锁线的缝隙选择：动态规划求总代价最小的"入口->缝隙->出口"序列。

    代价 = 路径长度 + 窄缝惩罚 + 与疑似地雷的接近惩罚。
    """
    lines: List[Tuple[float, List[Gap]]] = []
    for li, lx in enumerate(field.blockade_lines_x):
        ys = [t.y for t in tracks if abs(t.x - lx) <= line_tol and y_lo - 0.6 <= t.y <= y_hi + 0.6]
        if not ys:
            continue
        gaps = gap_candidates(ys, y_lo, y_hi, min_gap, cone_radius)
        gaps = [Gap(li, g.y, g.width) for g in gaps]
        if not gaps:
            continue
        # 风险：缝隙中心与疑似地雷（红/未知）距离过近
        for g in gaps:
            r = 0.0
            for t in tracks:
                if t.color in ("red", "unknown") and abs(t.x - lx) <= 1.2:
                    d = math.hypot(lx - t.x, g.y - t.y)
                    if d < 0.9:
                        r += (0.9 - d) * 2.0
            g.risk = r
        lines.append((lx, gaps))

    if not lines:
        return []

    # 动态规划
    best: List[List[Tuple[float, List[np.ndarray]]]] = []
    for li, (lx, gaps) in enumerate(lines):
        entries: List[Tuple[float, List[np.ndarray]]] = []
        for g in gaps:
            node = np.array([lx, g.y])
            if li == 0:
                prev_pt = entry
                cost = float(np.linalg.norm(node - prev_pt))
                path = [node]
            else:
                best_cost, best_path = math.inf, []
                for c, p in best[li - 1]:
                    prev_pt = p[-1]
                    cc = c + float(np.linalg.norm(node - prev_pt))
                    if cc < best_cost:
                        best_cost, best_path = cc, p + [node]
                cost, path = best_cost, best_path
            cost += narrow_penalty * max(0.0, prefer_gap - g.width) + g.risk
            entries.append((cost, path))
        best.append(entries)

    final_cost, final_path = math.inf, []
    for c, p in best[-1]:
        cc = c + float(np.linalg.norm(exit_ - p[-1]))
        if cc < final_cost:
            final_cost, final_path = cc, p
    return final_path


# --------------------------------------------------------------------------- #
# A*
# --------------------------------------------------------------------------- #
def _octile(dx: float, dy: float) -> float:
    a, b = abs(dx), abs(dy)
    return (a + b) + (math.sqrt(2.0) - 2.0) * min(a, b)


def astar(grid: GridMap, start: Sequence[float], goal: Sequence[float],
          max_expand: int = 60000) -> Optional[np.ndarray]:
    """8 邻域 A*。返回世界坐标折线 (N,2)，失败返回 None。"""
    s = grid.best_free(start, 0.80)
    gpt = grid.best_free(goal, 0.60)
    if s is None or gpt is None:
        return None
    si = grid.world_to_idx(s)
    gi = grid.world_to_idx(gpt)
    if si == gi:
        return np.array([s, gpt], dtype=float)

    nx, ny = grid.nx, grid.ny
    blocked = grid.blocked
    cost = grid.cost
    gscore = np.full((nx, ny), math.inf, dtype=np.float64)
    came = np.full((nx, ny), -1, dtype=np.int64)
    gscore[si] = 0.0
    open_heap: List[Tuple[float, int, int]] = [(_octile(gi[0] - si[0], gi[1] - si[1]), si[0], si[1])]
    neighbors = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
                 (1, 1, math.sqrt(2.0)), (1, -1, math.sqrt(2.0)),
                 (-1, 1, math.sqrt(2.0)), (-1, -1, math.sqrt(2.0))]
    expanded = 0
    found = False
    while open_heap:
        f, i, j = heapq.heappop(open_heap)
        cur_g = gscore[i, j]
        if (i, j) == gi:
            found = True
            break
        expanded += 1
        if expanded > max_expand:
            break
        for di, dj, step in neighbors:
            ni, nj = i + di, j + dj
            if not (0 <= ni < nx and 0 <= nj < ny) or blocked[ni, nj]:
                continue
            if di != 0 and dj != 0:
                if blocked[i + di, j] or blocked[i, j + dj]:
                    continue
            ng = cur_g + step * (1.0 + 0.35 * float(cost[ni, nj]))
            if ng < gscore[ni, nj] - 1e-9:
                gscore[ni, nj] = ng
                came[ni, nj] = i * ny + j
                h = _octile(gi[0] - ni, gi[1] - nj)
                heapq.heappush(open_heap, (ng + h, ni, nj))
    if not found:
        return None

    cells: List[Tuple[int, int]] = []
    ci, cj = gi
    while ci >= 0 and cj >= 0:
        cells.append((ci, cj))
        prev = came[ci, cj]
        if prev < 0:
            break
        ci, cj = int(prev // ny), int(prev % ny)
    cells.reverse()
    pts = [grid.idx_to_world(i, j) for i, j in cells]
    pts[0] = np.asarray(s, dtype=float)
    pts[-1] = np.asarray(gpt, dtype=float)
    return np.asarray(pts, dtype=float)


def simplify_path(grid: GridMap, path: np.ndarray, start: Sequence[float]
                  ) -> np.ndarray:
    """视线法平滑：能直连就直连。"""
    if path is None or len(path) <= 2:
        return path
    pts = [np.asarray(start, dtype=float)] + list(np.asarray(path, dtype=float))
    out = [pts[0]]
    i = 0
    while i < len(pts) - 1:
        j = len(pts) - 1
        while j > i + 1:
            if grid.los_clear(pts[i], pts[j]):
                break
            j -= 1
        out.append(pts[j])
        i = j
    return np.asarray(out, dtype=float)


# --------------------------------------------------------------------------- #
# 车辆足迹校验
# --------------------------------------------------------------------------- #
@dataclass
class PathCheck:
    ok: bool
    min_clearance: float
    worst_index: int


def check_footprint(path: np.ndarray, cones: Sequence[Cone], car_length: float,
                    car_width: float, margin: float = 0.02,
                    step: float = 0.05, skip_start_m: float = 0.0) -> PathCheck:
    """沿路径按行进方向摆放车辆矩形，检查与锥桶的最小间距。

    skip_start_m：跳过路径起始的一小段（车已经在那里，无法再"检查"），
    用于从"贴着一个目标"的位姿规划脱困路径。
    """
    if path is None or len(path) < 2:
        return PathCheck(False, 0.0, 0)
    pts = G.resample_polyline(path, step)
    if len(pts) < 2:
        pts = np.asarray(path, dtype=float)
    t = np.gradient(pts, axis=0)
    yaw = np.arctan2(t[:, 1], t[:, 0])
    keep = np.ones(len(pts), dtype=bool)
    if skip_start_m > 0.0:
        seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        s = np.concatenate([[0.0], np.cumsum(seg)])
        keep = s >= skip_start_m
        if not keep.any():
            keep[-1] = True
    if not len(cones):
        return PathCheck(True, math.inf, 0)
    C = np.array([[c.x, c.y] for c in cones], dtype=float)      # (M,2)
    R = np.array([c.radius for c in cones], dtype=float)
    rel = C[None, :, :] - pts[:, None, :]                       # (N,M,2)
    c, s = np.cos(yaw)[:, None], np.sin(yaw)[:, None]
    vx = c * rel[:, :, 0] + s * rel[:, :, 1]
    vy = -s * rel[:, :, 0] + c * rel[:, :, 1]
    qx = np.clip(vx, -car_length / 2.0, car_length / 2.0)
    qy = np.clip(vy, -car_width / 2.0, car_width / 2.0)
    d = np.hypot(vx - qx, vy - qy) - R[None, :]                 # (N,M)
    d_masked = d[keep]
    k_local = int(np.unravel_index(np.argmin(d_masked), d_masked.shape)[0])
    k = int(np.flatnonzero(keep)[k_local])
    worst = float(d_masked.min())
    return PathCheck(bool(worst >= margin), float(max(0.0, worst)), k)


def plan_path(grid: GridMap, start: Sequence[float], goal: Sequence[float],
              cones: Sequence[Cone], car_length: float, car_width: float,
              clearances: Optional[Sequence[float]] = None,
              margin: float = 0.02, min_clearance: float = 0.14,
              skip_start_m: float = 0.0,
              penalties: Optional[Sequence[Tuple[float, float, float, float]]] = None
              ) -> Tuple[Optional[np.ndarray], float]:
    """规划 + 平滑 + 车辆足迹校验；失败时逐级放宽安全余量重试。

    返回 (路径, 实际使用的安全余量[米])；路径为 None 表示彻底失败。
    """
    base = float(grid.hard_clearance)
    if clearances is None:
        clearances = [base, base + 0.05, base - 0.03, base - 0.06, base - 0.08]
    used = base
    for c in clearances:
        if c < min_clearance:
            break
        for cc in ([c] if c > base - 1e-9 else [c + 0.04, c]):
            used = cc
            grid.build(cones, hard_clearance=cc, penalties=penalties)
            raw = astar(grid, start, goal)
            if raw is None:
                continue
            simple = simplify_path(grid, raw, start)
            chk = check_footprint(simple, cones, car_length, car_width, margin,
                                  skip_start_m=skip_start_m)
            if chk.ok:
                return simple, cc
    return None, used
