"""场地模型：赛场尺寸、起点、据点/营地、雷区封锁线、先验锥桶。"""

from __future__ import annotations

import math
from dataclasses import dataclass, field as dc_field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from .config import Config
from . import geometry as G


@dataclass
class Zone:
    """一块 1m x 1m（或起点区）矩形区域。"""

    name: str
    cx: float
    cy: float
    length: float
    width: float
    color: str = ""
    yaw: float = 0.0

    @property
    def center(self) -> np.ndarray:
        return np.array([self.cx, self.cy], dtype=float)

    def corners(self) -> np.ndarray:
        return G.rect_corners(self.cx, self.cy, self.length, self.width, self.yaw)

    def contains(self, p: Sequence[float], tol: float = 0.0) -> bool:
        return G.point_in_rect(p, self.cx, self.cy,
                               self.length + 2 * tol, self.width + 2 * tol, self.yaw)

    def distance_to_center(self, p: Sequence[float]) -> float:
        return float(np.linalg.norm(np.asarray(p, dtype=float)[:2] - self.center))

    def clamp_inside(self, p: Sequence[float], margin: float = 0.0) -> np.ndarray:
        """把点限制在区域内（留 margin 边距），用于生成区域内目标点。"""
        v = G.world_to_body((self.cx, self.cy, self.yaw), p)
        hl = max(0.0, self.length / 2.0 - margin)
        hw = max(0.0, self.width / 2.0 - margin)
        v[0] = G.clamp(v[0], -hl, hl)
        v[1] = G.clamp(v[1], -hw, hw)
        return G.body_to_world((self.cx, self.cy, self.yaw), v)


@dataclass
class Cone:
    """一个锥桶（先验或实测）。radius 为底面半径，用于安全间距计算。"""

    x: float
    y: float
    color: str = "unknown"      # blue / red / unknown
    radius: float = 0.11
    source: str = "prior"       # prior / lidar / camera / fused
    confidence: float = 1.0

    @property
    def xy(self) -> np.ndarray:
        return np.array([self.x, self.y], dtype=float)


@dataclass
class Field:
    """赛场几何。默认值取自赛题 PDF，赛前公布实际尺寸后改 config/field.json。"""

    length: float = 7.0
    width: float = 4.0
    wall_height: float = 0.30
    start: Zone = dc_field(default_factory=lambda: Zone("start", 0.45, 2.0, 0.9, 0.8))
    zones: Dict[str, Zone] = dc_field(default_factory=dict)
    minefield_x: Tuple[float, float] = (1.5, 5.5)
    blockade_lines_x: List[float] = dc_field(default_factory=lambda: [2.0, 3.5, 5.0])
    min_cone_gap: float = 0.40
    prior_blue: List[Cone] = dc_field(default_factory=list)
    prior_red: List[Cone] = dc_field(default_factory=list)
    start_heading: float = 0.0
    boundary: Optional[List[List[float]]] = None

    # ---------------------------------------------------------------- #
    # 构建
    # ---------------------------------------------------------------- #
    @classmethod
    def from_config(cls, cfg: Config) -> "Field":
        f = cls()
        f.length = float(cfg.get("arena.length_m", 7.0))
        f.width = float(cfg.get("arena.width_m", 4.0))
        f.wall_height = float(cfg.get("arena.wall_height_m", 0.30))
        f.boundary = cfg.get("arena.boundary") or None

        sc = cfg.get("start.center", [0.45, 2.0])
        ss = cfg.get("start.size", [0.9, 0.8])
        f.start = Zone("start", float(sc[0]), float(sc[1]), float(ss[0]), float(ss[1]))
        f.start_heading = math.radians(float(cfg.get("start.heading_deg", 0.0)))

        f.zones = {}
        for name, z in (cfg.get("zones", {}) or {}).items():
            c = z.get("center", [1.0, 1.0])
            s = z.get("size", [1.0, 1.0])
            f.zones[name] = Zone(name, float(c[0]), float(c[1]),
                                 float(s[0]), float(s[1]), str(z.get("color", "")), math.radians(float(z.get("heading_deg",0.))))

        mf = cfg.get("minefield", {}) or {}
        f.minefield_x = (float(mf.get("x_min", 1.5)), float(mf.get("x_max", 5.5)))
        f.blockade_lines_x = [float(v) for v in mf.get("blockade_lines_x", [2.0, 3.5, 5.0])]
        f.min_cone_gap = float(mf.get("min_cone_gap_m", 0.40))

        prior = cfg.get("prior", {}) or {}
        f.prior_blue = []
        for i, line_y in enumerate(prior.get("blue_lines_y", []) or []):
            x = f.blockade_lines_x[i] if i < len(f.blockade_lines_x) else None
            if x is None:
                continue
            for y in line_y:
                f.prior_blue.append(Cone(x, float(y), "blue", 0.11, "prior"))
        f.prior_red = []
        for p in prior.get("red_mines", []) or []:
            f.prior_red.append(Cone(float(p[0]), float(p[1]), "red", 0.11, "prior"))
        for p in prior.get("obstacles", []):
            f.prior_red.append(Cone(float(p["x"]),float(p["y"]),"unknown",float(p.get("radius",.115)),"prior"))
        return f

    # ---------------------------------------------------------------- #
    # 查询
    # ---------------------------------------------------------------- #
    def zone(self, name: str) -> Zone:
        if name in self.zones:
            return self.zones[name]
        if name == "start":
            return self.start
        raise KeyError(f"未知区域: {name}")

    def inside_arena(self, p: Sequence[float], margin: float = 0.0) -> bool:
        if self.boundary:
            from .settings import inside
            return inside(p,self.boundary,margin)
        return (margin <= float(p[0]) <= self.length - margin
                and margin <= float(p[1]) <= self.width - margin)

    def clamp_to_arena(self, p: Sequence[float], margin: float = 0.0) -> np.ndarray:
        return np.array([G.clamp(float(p[0]), margin, self.length - margin),
                         G.clamp(float(p[1]), margin, self.width - margin)], dtype=float)

    def wall_clearance(self, p: Sequence[float]) -> float:
        """点到最近墙面的距离。"""
        if self.boundary:
            return min(G.dist_point_segment(p,a,b) for a,b in zip(self.boundary,self.boundary[1:]+self.boundary[:1]))
        x, y = float(p[0]), float(p[1])
        return min(x, self.length - x, y, self.width - y)

    def in_minefield(self, p: Sequence[float], tol: float = 0.0) -> bool:
        return (self.minefield_x[0] - tol) <= float(p[0]) <= (self.minefield_x[1] + tol)

    def prior_cones(self) -> List[Cone]:
        return list(self.prior_blue) + list(self.prior_red)

    def prior_line_y(self, line_index: int) -> List[float]:
        x = self.blockade_lines_x[line_index]
        return [c.y for c in self.prior_blue if abs(c.x - x) < 1e-6]

    def summary(self) -> str:
        return (f"赛场 {self.length:.2f}m x {self.width:.2f}m | 封锁线 x={self.blockade_lines_x} "
                f"| 先验蓝锥 {len(self.prior_blue)} 个 / 先验红锥 {len(self.prior_red)} 个")
