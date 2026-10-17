"""基础几何工具：角度、位姿变换、矩形/圆碰撞、射线求交、折线操作。

坐标约定：世界坐标系 x 沿场地长边（起点在 x 小的一侧，车向 +x 行驶），
+y 指向场地左侧（俯视图逆时针 90 度），角度为弧度，逆时针为正。
车体坐标系：x 向前，y 向左，yaw 为车头朝向。
"""

from __future__ import annotations

import math
from typing import Iterable, List, Optional, Sequence, Tuple

import numpy as np

EPS = 1e-9


# --------------------------------------------------------------------------- #
# 角度
# --------------------------------------------------------------------------- #
def wrap_pi(a: float) -> float:
    """把角度规整到 (-pi, pi]。"""
    return (float(a) + math.pi) % (2.0 * math.pi) - math.pi


def angle_diff(a: float, b: float) -> float:
    """a - b 的最短角差。"""
    return wrap_pi(float(a) - float(b))


def clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else (hi if v > hi else v)


def lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


# --------------------------------------------------------------------------- #
# 位姿与坐标变换
# --------------------------------------------------------------------------- #
def rot2(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]], dtype=float)


def world_to_body(pose: Sequence[float], point: Sequence[float]) -> np.ndarray:
    """世界点 -> 车体点。pose = (x, y, yaw)。"""
    dx = float(point[0]) - float(pose[0])
    dy = float(point[1]) - float(pose[1])
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return np.array([c * dx + s * dy, -s * dx + c * dy], dtype=float)


def body_to_world(pose: Sequence[float], point: Sequence[float]) -> np.ndarray:
    """车体点 -> 世界点。"""
    c, s = math.cos(pose[2]), math.sin(pose[2])
    x = pose[0] + c * float(point[0]) - s * float(point[1])
    y = pose[1] + s * float(point[0]) + c * float(point[1])
    return np.array([x, y], dtype=float)


def local_to_polar(x: float, y: float) -> Tuple[float, float]:
    """车体点 -> (方位角, 距离)，方位角在 (-pi, pi]，0 为正前方。"""
    return math.atan2(y, x), math.hypot(x, y)


# --------------------------------------------------------------------------- #
# 矩形（车辆、区域）
# --------------------------------------------------------------------------- #
def rect_corners(cx: float, cy: float, length: float, width: float,
                 yaw: float = 0.0) -> np.ndarray:
    """返回矩形四角（世界坐标），顺序：右前、左前、左后、右后。"""
    hl, hw = length / 2.0, width / 2.0
    local = np.array([[hl, -hw], [hl, hw], [-hl, hw], [-hl, -hw]], dtype=float)
    c, s = math.cos(yaw), math.sin(yaw)
    r = np.array([[c, -s], [s, c]], dtype=float)
    return local @ r.T + np.array([cx, cy], dtype=float)


def point_in_rect(p: Sequence[float], cx: float, cy: float,
                  length: float, width: float, yaw: float = 0.0) -> bool:
    """点是否在矩形内（矩形中心、尺寸、朝向）。"""
    v = world_to_body((cx, cy, yaw), p)
    return abs(v[0]) <= length / 2.0 + EPS and abs(v[1]) <= width / 2.0 + EPS


def rect_rect_overlap(a: np.ndarray, b: np.ndarray) -> bool:
    """两个凸多边形（按角点顺序）是否相交：分离轴定理。"""
    for poly in (a, b):
        n = len(poly)
        for i in range(n):
            edge = poly[(i + 1) % n] - poly[i]
            axis = np.array([-edge[1], edge[0]], dtype=float)
            norm = np.linalg.norm(axis)
            if norm < EPS:
                continue
            axis = axis / norm
            pa, pb = a @ axis, b @ axis
            if pa.max() < pb.min() - EPS or pb.max() < pa.min() - EPS:
                return False
    return True


def rect_circle_overlap(cx: float, cy: float, length: float, width: float,
                        yaw: float, cc: Sequence[float], radius: float) -> bool:
    """矩形与圆是否相交。"""
    v = world_to_body((cx, cy, yaw), cc)
    qx = clamp(v[0], -length / 2.0, length / 2.0)
    qy = clamp(v[1], -width / 2.0, width / 2.0)
    return math.hypot(v[0] - qx, v[1] - qy) <= radius + EPS


# --------------------------------------------------------------------------- #
# 距离与射线
# --------------------------------------------------------------------------- #
def dist_point_segment(p: Sequence[float], a: Sequence[float],
                       b: Sequence[float]) -> float:
    p = np.asarray(p, dtype=float)
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    ab = b - a
    denom = float(ab @ ab)
    t = 0.0 if denom < EPS else float(np.clip((p - a) @ ab / denom, 0.0, 1.0))
    return float(np.linalg.norm(p - (a + t * ab)))


def ray_circle_first_hit(origin: Sequence[float], direction: Sequence[float],
                         center: Sequence[float], radius: float) -> Optional[float]:
    """射线与圆的首个交点距离（沿 direction 的 t>=0），无交点返回 None。"""
    o = np.asarray(origin, dtype=float)[:2]
    d = np.asarray(direction, dtype=float)[:2]
    c = np.asarray(center, dtype=float)[:2]
    dn = float(np.linalg.norm(d))
    if dn < EPS:
        return None
    d = d / dn
    m = o - c
    b = float(m @ d)
    cc = float(m @ m) - radius * radius
    disc = b * b - cc
    if disc < 0.0:
        return None
    sq = math.sqrt(disc)
    t1, t2 = -b - sq, -b + sq
    if t2 < 0.0:
        return None
    return t1 if t1 >= 0.0 else t2


def ray_box_exit_distance(origin: Sequence[float], direction: Sequence[float],
                          x_min: float, y_min: float, x_max: float,
                          y_max: float) -> float:
    """从盒内部出发的射线到边界的距离（用于墙面）。"""
    o = np.asarray(origin, dtype=float)[:2]
    d = np.asarray(direction, dtype=float)[:2]
    dn = float(np.linalg.norm(d))
    if dn < EPS:
        return math.inf
    d = d / dn
    ts: List[float] = []
    for axis, lo, hi in ((0, x_min, x_max), (1, y_min, y_max)):
        if abs(d[axis]) < 1e-12:
            continue
        for bound in (lo, hi):
            t = (bound - o[axis]) / d[axis]
            if t > 0.0:
                ts.append(float(t))
    return min(ts) if ts else math.inf


# --------------------------------------------------------------------------- #
# 折线
# --------------------------------------------------------------------------- #
def polyline_length(pts: np.ndarray) -> float:
    if len(pts) < 2:
        return 0.0
    return float(np.sum(np.linalg.norm(np.diff(np.asarray(pts, dtype=float), axis=0), axis=1)))


def resample_polyline(pts: Iterable[Sequence[float]], step: float) -> np.ndarray:
    """按固定步长重采样折线（含首尾点）。"""
    p = np.asarray(list(pts), dtype=float)
    if len(p) < 2:
        return p.reshape(-1, 2)
    seg = np.linalg.norm(np.diff(p, axis=0), axis=1)
    total = float(seg.sum())
    if total < EPS:
        return p[:1]
    s = np.concatenate([[0.0], np.cumsum(seg)])
    n = max(2, int(round(total / max(step, 1e-3))) + 1)
    targets = np.linspace(0.0, total, n)
    out = []
    j = 0
    for t in targets:
        while j < len(seg) - 1 and s[j + 1] < t:
            j += 1
        denom = seg[j] if seg[j] > EPS else 1.0
        u = (t - s[j]) / denom
        out.append(p[j] + u * (p[j + 1] - p[j]))
    return np.asarray(out, dtype=float)


def closest_point_on_polyline(p: Sequence[float], pts: np.ndarray,
                              ) -> Tuple[float, float, int]:
    """点到折线的最近点。返回 (距离, 弧长位置, 线段索引)。"""
    pts = np.asarray(pts, dtype=float)
    p = np.asarray(p, dtype=float)
    if len(pts) == 1:
        return float(np.linalg.norm(p - pts[0])), 0.0, 0
    best = (math.inf, 0.0, 0)
    s = 0.0
    for i in range(len(pts) - 1):
        a, b = pts[i], pts[i + 1]
        ab = b - a
        denom = float(ab @ ab)
        t = 0.0 if denom < EPS else float(np.clip((p - a) @ ab / denom, 0.0, 1.0))
        q = a + t * ab
        d = float(np.linalg.norm(p - q))
        if d < best[0]:
            best = (d, s + t * float(np.linalg.norm(ab)), i)
        s += float(np.linalg.norm(ab))
    return best


def point_at_arc_length(pts: np.ndarray, arc: float) -> np.ndarray:
    """折线上距起点弧长为 arc 的点（超出则返回末点）。"""
    pts = np.asarray(pts, dtype=float)
    if len(pts) < 2:
        return pts[0] if len(pts) else np.zeros(2)
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    if arc <= 0.0:
        return pts[0]
    if arc >= s[-1]:
        return pts[-1]
    i = int(np.searchsorted(s, arc) - 1)
    i = max(0, min(i, len(seg) - 1))
    denom = seg[i] if seg[i] > EPS else 1.0
    return pts[i] + (arc - s[i]) / denom * (pts[i + 1] - pts[i])


def curvature_at(pts: np.ndarray, arc: float) -> float:
    """折线在给定弧长处的离散曲率（1/R，逆时针为正）。"""
    ds = 0.10
    a = point_at_arc_length(pts, max(0.0, arc - ds))
    b = point_at_arc_length(pts, arc)
    c = point_at_arc_length(pts, arc + ds)
    ab = b - a
    bc = c - b
    cross = float(ab[0] * bc[1] - ab[1] * bc[0])
    la, lb = float(np.linalg.norm(ab)), float(np.linalg.norm(bc))
    if la < EPS or lb < EPS:
        return 0.0
    return 2.0 * cross / (la * lb * (la + lb) + EPS)


def segment_lengths(pts: np.ndarray) -> np.ndarray:
    pts = np.asarray(pts, dtype=float)
    if len(pts) < 2:
        return np.zeros(0)
    return np.linalg.norm(np.diff(pts, axis=0), axis=1)
