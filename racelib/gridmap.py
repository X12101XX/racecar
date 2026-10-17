"""局部栅格代价地图：锥桶/墙面 -> 硬障碍 + 软代价（距离场）。"""

from __future__ import annotations

import math
from typing import Iterable, List, Optional, Sequence, Tuple

import numpy as np

from . import geometry as G
from .field import Cone, Field

try:  # pragma: no cover - 环境相关
    from scipy import ndimage as _ndi  # type: ignore
except Exception:  # noqa: BLE001
    _ndi = None  # type: ignore

try:  # pragma: no cover - 环境相关
    import cv2  # type: ignore
except Exception:  # noqa: BLE001
    cv2 = None  # type: ignore


def _chamfer_distance(mask: np.ndarray, res: float) -> np.ndarray:
    """两趟倒角距离变换（numpy 兜底实现），返回"到最近障碍的距离(米)"。"""
    h, w = mask.shape
    big = 1e6
    d = np.where(mask, 0.0, big).astype(np.float64)
    a, b = 1.0, math.sqrt(2.0)
    for j in range(h):
        for i in range(w):
            m = d[j, i]
            if j > 0:
                m = min(m, d[j - 1, i] + a)
                if i > 0:
                    m = min(m, d[j - 1, i - 1] + b)
                if i < w - 1:
                    m = min(m, d[j - 1, i + 1] + b)
            if i > 0:
                m = min(m, d[j, i - 1] + a)
            d[j, i] = m
    for j in range(h - 1, -1, -1):
        for i in range(w - 1, -1, -1):
            m = d[j, i]
            if j < h - 1:
                m = min(m, d[j + 1, i] + a)
                if i > 0:
                    m = min(m, d[j + 1, i - 1] + b)
                if i < w - 1:
                    m = min(m, d[j + 1, i + 1] + b)
            if i < w - 1:
                m = min(m, d[j, i + 1] + a)
            d[j, i] = m
    return d * res


def distance_to_obstacles(mask: np.ndarray, res: float) -> np.ndarray:
    """距离场（米）。mask=True 表示障碍。"""
    if _ndi is not None:
        return _ndi.distance_transform_edt(~mask, sampling=(res, res))
    if cv2 is not None:  # pragma: no cover
        return cv2.distanceTransform((~mask).astype(np.uint8), cv2.DIST_L2, 5) * res
    return _chamfer_distance(mask, res)


class GridMap:
    """以 0.05m 分辨率的栅格地图（覆盖整个赛场）。"""

    def __init__(self, field: Field, res: float = 0.05,
                 hard_clearance: float = 0.22, wall_clearance: float = 0.20,
                 soft_inflate: float = 0.45, soft_weight: float = 2.5):
        self.field = field
        self.res = float(res)
        self.nx = int(math.ceil(field.length / res)) + 1
        self.ny = int(math.ceil(field.width / res)) + 1
        self.hard_clearance = float(hard_clearance)
        self.wall_clearance = float(wall_clearance)
        self.soft_inflate = float(soft_inflate)
        self.soft_weight = float(soft_weight)
        self.obstacle = np.zeros((self.nx, self.ny), dtype=bool)
        self.dist = np.zeros((self.nx, self.ny), dtype=float)
        self.cost = np.zeros((self.nx, self.ny), dtype=float)
        self.blocked = np.zeros((self.nx, self.ny), dtype=bool)
        self._xs = np.arange(self.nx) * res
        self._ys = np.arange(self.ny) * res

    # ---------------------------------------------------------------- #
    def build(self, cones: Sequence[Cone], inflate_scale: float = 1.0,
              hard_clearance: Optional[float] = None,
              wall_clearance: Optional[float] = None,
              penalties: Optional[Sequence[Tuple[float, float, float, float]]] = None) -> None:
        """根据锥桶与墙面重建障碍/代价。

        hard_clearance / wall_clearance 可覆盖默认安全余量（求解失败时逐级放宽）。
        penalties 是"软禁区"列表 [(x, y, 半径, 权重)]：只加代价、不封通道，
        用于记住刚才蹭到的位置，避免反复在同一个点打转。
        """
        f = self.field
        self.obstacle[:] = False
        # 墙面作为障碍（用墙外一圈单元格表示）
        xs = self._xs[:, None] * np.ones((1, self.ny))
        ys = np.ones((self.nx, 1)) * self._ys[None, :]
        self.obstacle |= (xs < 0.0) | (xs > f.length) | (ys < 0.0) | (ys > f.width)
        # 锥桶
        for c in cones:
            r = c.radius
            i0 = max(0, int((c.x - r - 0.05) / self.res) - 1)
            i1 = min(self.nx - 1, int((c.x + r + 0.05) / self.res) + 1)
            j0 = max(0, int((c.y - r - 0.05) / self.res) - 1)
            j1 = min(self.ny - 1, int((c.y + r + 0.05) / self.res) + 1)
            if i1 < i0 or j1 < j0:
                continue
            sub_x = self._xs[i0:i1 + 1][:, None] * np.ones((1, j1 - j0 + 1))
            sub_y = np.ones((i1 - i0 + 1, 1)) * self._ys[j0:j1 + 1][None, :]
            self.obstacle[i0:i1 + 1, j0:j1 + 1] |= (np.hypot(sub_x - c.x, sub_y - c.y) <= r)

        self.dist = distance_to_obstacles(self.obstacle, self.res)
        hc = self.hard_clearance if hard_clearance is None else float(hard_clearance)
        wc = self.wall_clearance if wall_clearance is None else float(wall_clearance)
        hard_clear = max(0.12, hc * inflate_scale)
        wall_clear = max(0.12, wc * inflate_scale)
        # 墙面安全距离单独处理（离墙太近同样不允许）
        wall_d = np.minimum.reduce([xs, f.length - xs, ys, f.width - ys])
        self.blocked = (self.dist < hard_clear) | (wall_d < wall_clear)
        soft = np.clip((self.soft_inflate - self.dist) / max(self.soft_inflate, 1e-3), 0.0, 1.0)
        self.cost = self.soft_weight * soft
        if penalties:
            for (px, py, pr, pw) in penalties:
                i0 = max(0, int((px - pr) / self.res) - 1)
                i1 = min(self.nx - 1, int((px + pr) / self.res) + 1)
                j0 = max(0, int((py - pr) / self.res) - 1)
                j1 = min(self.ny - 1, int((py + pr) / self.res) + 1)
                if i1 < i0 or j1 < j0:
                    continue
                sub_x = self._xs[i0:i1 + 1][:, None] * np.ones((1, j1 - j0 + 1))
                sub_y = np.ones((i1 - i0 + 1, 1)) * self._ys[j0:j1 + 1][None, :]
                dd = np.hypot(sub_x - px, sub_y - py)
                self.cost[i0:i1 + 1, j0:j1 + 1] += float(pw) * np.clip(1.0 - dd / max(pr, 1e-3),
                                                                      0.0, 1.0)

    # ---------------------------------------------------------------- #
    def world_to_idx(self, p: Sequence[float]) -> Tuple[int, int]:
        i = int(round(float(p[0]) / self.res))
        j = int(round(float(p[1]) / self.res))
        return (max(0, min(self.nx - 1, i)), max(0, min(self.ny - 1, j)))

    def idx_to_world(self, i: int, j: int) -> np.ndarray:
        return np.array([i * self.res, j * self.res], dtype=float)

    def is_blocked(self, p: Sequence[float]) -> bool:
        i, j = self.world_to_idx(p)
        return bool(self.blocked[i, j])

    def clearance(self, p: Sequence[float]) -> float:
        i, j = self.world_to_idx(p)
        return float(self.dist[i, j])

    def clearance_cells(self, i: int, j: int) -> float:
        return float(self.dist[i, j])

    # ---------------------------------------------------------------- #
    def los_clear(self, a: Sequence[float], b: Sequence[float]) -> bool:
        """两点连线是否全程在可通行区域（按半分辨率采样）。"""
        a = np.asarray(a, dtype=float)
        b = np.asarray(b, dtype=float)
        d = float(np.linalg.norm(b - a))
        n = max(2, int(d / (self.res * 0.5)) + 1)
        for t in np.linspace(0.0, 1.0, n):
            p = a + t * (b - a)
            if self.is_blocked(p):
                return False
        return True

    def nearest_free(self, p: Sequence[float], max_radius: float = 0.40) -> Optional[np.ndarray]:
        """找最近的可通行点（车被"困"在膨胀区里时用）。"""
        if not self.is_blocked(p):
            return np.asarray(p, dtype=float)
        i0, j0 = self.world_to_idx(p)
        r = int(max_radius / self.res)
        best, best_d = None, math.inf
        for di in range(-r, r + 1):
            for dj in range(-r, r + 1):
                i, j = i0 + di, j0 + dj
                if not (0 <= i < self.nx and 0 <= j < self.ny):
                    continue
                if self.blocked[i, j]:
                    continue
                d = math.hypot(di, dj)
                if d < best_d:
                    best, best_d = (i, j), d
        return self.idx_to_world(*best) if best else None

    def best_free(self, p: Sequence[float], max_radius: float = 0.75,
                  clearance_weight: float = 0.25) -> Optional[np.ndarray]:
        """找"最值得去"的可通行点：优先间隙大，其次离车近。

        车深陷在膨胀区里时（例如被慢速推入、或地图刚补上远处的锥桶），
        单纯找"最近自由格"可能因为被隔离而让 A* 直接失败；这里改为在半径内
        用 (间隙 - 权重*距离) 打分，保证总能选到一个真正连通的落脚点。
        """
        if not self.is_blocked(p):
            return np.asarray(p, dtype=float)
        i0, j0 = self.world_to_idx(p)
        r = int(max_radius / self.res)
        best, best_score = None, -math.inf
        for di in range(-r, r + 1):
            for dj in range(-r, r + 1):
                i, j = i0 + di, j0 + dj
                if not (0 <= i < self.nx and 0 <= j < self.ny):
                    continue
                if self.blocked[i, j]:
                    continue
                d_m = math.hypot(di, dj) * self.res
                score = float(self.dist[i, j]) - clearance_weight * d_m
                if score > best_score:
                    best, best_score = (i, j), score
        return self.idx_to_world(*best) if best else None
