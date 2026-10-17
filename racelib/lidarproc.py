"""激光雷达处理：点云聚类 -> 锥桶（圆拟合）/ 墙面（直线拟合）提取。

适配任意束数的 2D 激光雷达（车上为 LSX10，360 度 10Hz）。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from . import geometry as G


@dataclass
class Scan:
    """一帧 2D 扫描（车体坐标系）。"""

    angles: np.ndarray
    ranges: np.ndarray
    t: float = 0.0

    def valid(self, range_min: float, range_max: float) -> np.ndarray:
        r = self.ranges
        return np.isfinite(r) & (r >= range_min) & (r <= range_max)

    def points_body(self, range_min: float = 0.05,
                    range_max: float = 50.0) -> np.ndarray:
        m = self.valid(range_min, range_max)
        a = self.angles[m]
        r = self.ranges[m]
        return np.stack([r * np.cos(a), r * np.sin(a)], axis=-1)

    def points_world(self, pose: Sequence[float], range_min: float = 0.05,
                     range_max: float = 50.0) -> np.ndarray:
        p = self.points_body(range_min, range_max)
        if len(p) == 0:
            return p
        c, s = math.cos(pose[2]), math.sin(pose[2])
        out = np.empty_like(p)
        out[:, 0] = pose[0] + c * p[:, 0] - s * p[:, 1]
        out[:, 1] = pose[1] + s * p[:, 0] + c * p[:, 1]
        return out


def cluster_points(pts: np.ndarray, jump_m: float = 0.16,
                   min_pts: int = 3) -> List[np.ndarray]:
    """按相邻点欧氏距离断裂聚类（扫描序）。返回每簇的点索引。"""
    if len(pts) == 0:
        return []
    if len(pts) == 1:
        return [np.array([0])] if min_pts <= 1 else []
    d = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    breaks = np.flatnonzero(d > jump_m)
    clusters: List[np.ndarray] = []
    start = 0
    for b in list(breaks) + [len(pts) - 1]:
        idx = np.arange(start, b + 1)
        if len(idx) >= min_pts:
            clusters.append(idx)
        start = b + 1
    return clusters


def fit_circle(pts: np.ndarray) -> Tuple[float, float, float, float]:
    """Kasa 代数圆拟合。返回 (cx, cy, r, rms)。"""
    x = pts[:, 0]
    y = pts[:, 1]
    a = np.stack([x, y, np.ones_like(x)], axis=1)
    b = -(x * x + y * y)
    try:
        sol, *_ = np.linalg.lstsq(a, b, rcond=None)
    except np.linalg.LinAlgError:  # pragma: no cover
        return 0.0, 0.0, -1.0, math.inf
    cx, cy = -sol[0] / 2.0, -sol[1] / 2.0
    val = cx * cx + cy * cy - sol[2]
    r = math.sqrt(val) if val > 0 else -1.0
    rms = float(np.sqrt(np.mean((np.hypot(x - cx, y - cy) - r) ** 2))) if r > 0 else math.inf
    return float(cx), float(cy), float(r), rms


@dataclass
class ConeDetection:
    """雷达单帧检测到的锥桶。"""

    x: float
    y: float
    radius: float
    n_points: int
    rms: float
    frame: str = "body"      # body / world


def extract_cones(pts: np.ndarray, r_min: float = 0.02, r_max: float = 0.30,
                  max_rms: float = 0.035, max_points: int = 90,
                  range_min: float = 0.25, min_points: int = 4,
                  max_center_offset: float = 3.2,
                  min_axis_ratio: float = 0.18) -> List[ConeDetection]:
    """从点云中提取锥桶：形状近似圆、半径合理、拟合残差小、几何自洽。

    额外的几何自洽检查用于剔除"墙角"这类假目标：拟合圆心必须靠近点簇质心。
    （注意：近距离锥桶的可见弧约为前半圈 ~180 度，不能用张角来过滤。）
    """
    out: List[ConeDetection] = []
    for idx in cluster_points(pts):
        p = pts[idx]
        if len(p) > max_points or len(p) < min_points:
            continue
        if float(np.linalg.norm(p.mean(axis=0))) < range_min:
            continue
        cx, cy, r, rms = fit_circle(p)
        if not (r_min <= r <= r_max):
            continue
        if rms > max_rms:
            continue
        # 点簇跨度不能超过直径的 1.35 倍（否则不是同一个圆上的点）
        dmax = float(np.linalg.norm(p[:, None, :] - p[None, :, :], axis=-1).max())
        if dmax > 2.0 * r * 1.35 + 0.02:
            continue
        centroid = p.mean(axis=0)
        if float(np.linalg.norm(centroid - np.array([cx, cy]))) > \
                max(max_center_offset * r, 0.06):
            continue
        # 形状检查：锥桶可见的是接近半圆的圆弧（PCA 主轴/次轴比约 0.2~0.5），
        # 而贴墙的掠射点簇近似直线（轴比很小），据此剔除墙面伪目标。
        q = p - centroid
        cov = q.T @ q / max(1, len(q))
        ev = np.linalg.eigvalsh(cov)
        axis_ratio = math.sqrt(max(ev[0], 1e-12)) / math.sqrt(max(ev[1], 1e-12))
        if axis_ratio < min_axis_ratio:
            continue
        out.append(ConeDetection(cx, cy, r, int(len(p)), rms))
    return out


@dataclass
class WallDetection:
    """墙面/围挡的一段直线。"""

    p0: np.ndarray
    p1: np.ndarray

    @property
    def length(self) -> float:
        return float(np.linalg.norm(self.p1 - self.p0))

    def distance_to(self, p: Sequence[float]) -> float:
        return G.dist_point_segment(p, self.p0, self.p1)


def extract_walls(pts: np.ndarray, min_points: int = 20,
                  max_rms: float = 0.03) -> List[WallDetection]:
    """长直线段拟合为墙面。"""
    out: List[WallDetection] = []
    def fit_segment(p, depth=0):
        if len(p)<min_points or depth>16:return
        c=p.mean(axis=0);q=p-c
        _,_,vt=np.linalg.svd(q,full_matrices=False)
        direction=vt[0];t=q@direction
        residual=np.abs(q@vt[1])
        rms=float(np.sqrt(np.mean(residual**2)))
        if rms>max_rms:
            # Continuous returns from adjacent walls form one connected cluster.
            # Split at the corner rather than discarding the whole perimeter.
            chord=p[-1]-p[0];norm=float(np.linalg.norm(chord))
            if norm>1e-6:
                normal=np.array([-chord[1],chord[0]])/norm
                error=np.abs((p-p[0])@normal)
                split=int(np.argmax(error))
            else:split=len(p)//2
            if split<min_points or split>len(p)-min_points:split=len(p)//2
            fit_segment(p[:split+1],depth+1);fit_segment(p[split:],depth+1)
            return
        p0=c+direction*t.min();p1=c+direction*t.max()
        if float(np.linalg.norm(p1-p0))>=.25:out.append(WallDetection(p0,p1))
    for idx in cluster_points(pts):fit_segment(pts[idx])

    return out


def wall_ranges(scan: Scan, pose: Sequence[float], field=None,
                half_angle_deg: float = 15.0) -> Dict[str, Optional[Tuple[float, float]]]:
    """沿车体 +x/-x/+y/-y 方向找最近墙面。

    返回 {方向: (中值距离, 该方向在世界坐标系下的角度)}；
    调用方需要做"这一束确实打在已知墙面上"的一致性门限检查（见 localization.py）。
    """
    out: Dict[str, Optional[Tuple[float, float]]] = {}
    a = scan.angles
    r = scan.ranges
    half = math.radians(half_angle_deg)
    dirs = {"front": 0.0, "back": math.pi, "left": math.pi / 2.0, "right": -math.pi / 2.0}
    valid = np.isfinite(r) & (r > 0.05)
    for name, ang in dirs.items():
        d = np.abs(np.arctan2(np.sin(a - ang), np.cos(a - ang)))
        m = valid & (d < half)
        if not m.any():
            out[name] = None
            continue
        rr = float(np.median(r[m]))
        out[name] = (rr, G.wrap_pi(float(pose[2]) + ang))
    return out
