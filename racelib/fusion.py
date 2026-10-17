"""锥桶地图：多帧雷达观测融合 + 相机颜色标注。"""

from __future__ import annotations

import math
from dataclasses import dataclass, field as dc_field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from . import geometry as G
from .field import Cone, Field
from .lidarproc import ConeDetection
from .vision import ConeObs


@dataclass
class ConeTrack:
    """稳定跟踪的一个锥桶。"""

    x: float
    y: float
    radius: float = 0.11
    color: str = "unknown"
    color_conf: float = 0.0
    hits: int = 1
    misses: int = 0
    t_created: float = 0.0
    t_seen: float = 0.0
    from_prior: bool = False
    on_line: Optional[int] = None       # 属于第几条封锁线
    spread: float = 0.0                 # 最近若干次观测的世界坐标离散度
    _recent: list = dc_field(default_factory=list)

    @property
    def xy(self) -> np.ndarray:
        return np.array([self.x, self.y], dtype=float)

    def to_cone(self) -> Cone:
        return Cone(self.x, self.y, self.color, self.radius, "fused",
                    max(0.3, self.color_conf if self.color != "unknown" else 0.5))


@dataclass
class ConeMapConfig:
    assoc_dist_m: float = 0.28
    forget_seconds: float = 3.0
    prior_trust: bool = True
    color_persist: bool = True
    bearing_tol_deg: float = 6.0
    bearing_tol_dist_ratio: float = 0.06
    base_radius_bonus_m: float = 0.030      # 雷达看到的是锥桶上部，补偿到底面半径
    planning_radius_bonus: float = 0.02     # 规划时额外放大半径，吸收地图 3~5cm 误差
    mine_extra_margin_m: float = 0.07       # 疑似地雷(红/未标色)再放大：碰蓝锥罚 5 秒，碰地雷判失败
    min_track_hits: int = 2
    wall_reject_margin_m: float = 0.06      # 场地边界附近的检测视为墙面伪目标


class ConeMap:
    """把逐帧观测融合成稳定的锥桶地图，并用相机观测给锥桶上色。"""

    def __init__(self, field: Field, cfg: Optional[ConeMapConfig] = None):
        self.field = field
        self.cfg = cfg or ConeMapConfig()
        self.tracks: List[ConeTrack] = []
        self.new_since_last_query = 0
        self.last_added: List[ConeTrack] = []

    # ---------------------------------------------------------------- #
    def seed_prior(self, t: float = 0.0) -> int:
        """把赛前公布的锥桶位置作为先验注入（封闭赛线下会用）。"""
        if not self.cfg.prior_trust:
            return 0
        n = 0
        for c in self.field.prior_cones():
            tr = ConeTrack(c.x, c.y, c.radius, c.color, 1.0, hits=3,
                           t_created=t, t_seen=t, from_prior=True)
            for i, lx in enumerate(self.field.blockade_lines_x):
                if abs(c.x - lx) < 0.25:
                    tr.on_line = i
            self.tracks.append(tr)
            n += 1
        return n

    # ---------------------------------------------------------------- #
    def update(self, detections: Sequence[ConeDetection], pose: Sequence[float],
               t: float) -> int:
        """融合一帧雷达检测（车体坐标）到世界坐标地图。返回新增数量。"""
        added = 0
        self.last_added = []
        for det in detections:
            p = G.body_to_world(pose, [det.x, det.y])
            # 贴墙的"圆形目标"多为墙面/墙角伪影，直接丢弃
            if (p[0] < self.cfg.wall_reject_margin_m
                    or p[0] > self.field.length - self.cfg.wall_reject_margin_m
                    or p[1] < self.cfg.wall_reject_margin_m
                    or p[1] > self.field.width - self.cfg.wall_reject_margin_m):
                continue
            tr = self._nearest(p)
            if tr is None:
                self.tracks.append(ConeTrack(
                    float(p[0]), float(p[1]),
                    max(det.radius + self.cfg.base_radius_bonus_m, 0.10),
                    t_created=t, t_seen=t))
                added += 1
                self.last_added.append(self.tracks[-1])
            else:
                alpha = 0.35
                tr.x = (1 - alpha) * tr.x + alpha * float(p[0])
                tr.y = (1 - alpha) * tr.y + alpha * float(p[1])
                tr.radius = (0.7 * tr.radius
                             + 0.3 * max(det.radius + self.cfg.base_radius_bonus_m, 0.10))
                tr.hits += 1
                tr.misses = 0
                tr.t_seen = t
                tr._recent.append((float(p[0]), float(p[1])))
                if len(tr._recent) > 5:
                    tr._recent.pop(0)
                if len(tr._recent) >= 3:
                    arr = np.asarray(tr._recent, dtype=float)
                    tr.spread = float(np.max(np.linalg.norm(arr - arr.mean(axis=0), axis=1)))
        for tr in self.tracks:
            if tr.from_prior:
                continue
            if (t - tr.t_seen) > self.cfg.forget_seconds:
                tr.misses += 1
        self.tracks = [tr for tr in self.tracks
                       if tr.from_prior or (t - tr.t_seen) <= self.cfg.forget_seconds * 2]
        # 地图规模上限：优先保留"看得到的、确认次数多的"目标，避免长时间游走时地图膨胀
        if len(self.tracks) > 60:
            self.tracks.sort(key=lambda tr: (tr.from_prior, tr.t_seen, tr.hits), reverse=True)
            self.tracks = self.tracks[:60]
        self.new_since_last_query += added
        return added

    def _nearest(self, p: np.ndarray) -> Optional[ConeTrack]:
        best, best_d = None, self.cfg.assoc_dist_m
        px, py = float(p[0]), float(p[1])
        for tr in self.tracks:
            d = math.hypot(px - tr.x, py - tr.y)
            if d < best_d:
                best, best_d = tr, d
        return best

    # ---------------------------------------------------------------- #
    def annotate_colors(self, cone_obs: Sequence[ConeObs], pose: Sequence[float]) -> int:
        """用相机观测的方位角给地图中的锥桶上色。返回新标注数量。"""
        n = 0
        for obs in cone_obs:
            if obs.color not in ("blue", "red"):
                continue
            scores = []
            for tr in self.tracks:
                b, d = G.local_to_polar(*G.world_to_body(pose, tr.xy))
                if d > 4.5 or obs.dist_est > 4.5:
                    continue
                err = abs(G.angle_diff(b, obs.bearing))
                tol = math.radians(self.cfg.bearing_tol_deg) + \
                    self.cfg.bearing_tol_dist_ratio * abs(d - obs.dist_est)
                if err > tol:
                    continue
                if abs(d - obs.dist_est) > 0.35 + 0.30 * obs.dist_est:
                    continue
                # 方位 + 距离都要吻合，用"方位差 + 距离差"打分
                scores.append((err / max(tol, 1e-6) + 0.5 * abs(d - obs.dist_est), tr))
            if not scores:
                continue
            scores.sort(key=lambda s: s[0])
            # 唯一性检查：第二名明显更差时才接受，避免"一排锥桶同方位"时标错颜色
            if len(scores) > 1 and scores[0][0] > 0.65 * scores[1][0]:
                continue
            best = scores[0][1]
            if best.color != obs.color:
                if best.color == "unknown" or obs.confidence > best.color_conf:
                    if best.color != "unknown":
                        n += 1  # 颜色被改写，计入统计
                    best.color = obs.color
                    best.color_conf = obs.confidence
                else:
                    best.color_conf = max(best.color_conf, obs.confidence)
            else:
                best.color_conf = max(best.color_conf, obs.confidence)
        return n

    # ---------------------------------------------------------------- #
    def snapshot(self) -> List[ConeTrack]:
        return list(self.tracks)

    def cones(self, min_hits: int = 2) -> List[ConeTrack]:
        return [t for t in self.tracks if t.from_prior or t.hits >= min_hits]

    def mines(self, include_unknown: bool = True) -> List[ConeTrack]:
        """疑似地雷：明确红色的锥桶 + （可选）颜色未知的锥桶（保守处理）。"""
        out = []
        for t in self.tracks:
            if t.color == "red" or (include_unknown and t.color == "unknown"):
                out.append(t)
        return out

    def blockers(self) -> List[ConeTrack]:
        """规划时当作障碍的锥桶（红蓝都避，蓝锥碰了要罚 5 秒）。

        只使用被观测到 2 次以上的稳定目标，避免单帧偶发伪目标把通道堵死。
        """
        return [t for t in self.tracks
                if t.from_prior or (t.hits >= self.cfg.min_track_hits
                                    and (t.hits < 3 or t.spread <= 0.12))]

    def nearest_distance(self, p: Sequence[float]) -> float:
        if not self.tracks:
            return math.inf
        px, py = float(p[0]), float(p[1])
        return min(math.hypot(px - t.x, py - t.y) for t in self.tracks)

    def stats(self) -> Dict[str, int]:
        c = {"blue": 0, "red": 0, "unknown": 0}
        for t in self.tracks:
            c[t.color if t.color in c else "unknown"] += 1
        return c
