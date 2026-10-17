"""定位：航迹推算 + IMU 角速度/航向 + 场地墙面距离修正。

场地是一个已知尺寸的矩形围栏，激光雷达在开阔处（离锥桶较远）能稳定看到
四面墙，因此可以用"墙在已知坐标上"这一先验周期性消除累计漂移。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Optional, Sequence, Tuple

import numpy as np

from . import geometry as G
from .lidarproc import Scan, extract_walls


@dataclass
class PoseEstimatorConfig:
    imu_yaw_gain: float = 0.05
    wall_gain: float = 0.25
    wall_gate_m: float = 0.25      # 只接受与当前估计相差 25cm 以内的墙面观测
    wall_min_clear_m: float = 0.55
    accel_gate: float = 0.35


class PoseEstimator:
    """轻量状态估计：状态 (x, y, yaw, v, w)。"""

    def __init__(self, field, cfg: Optional[PoseEstimatorConfig] = None):
        self.field = field
        self.cfg = cfg or PoseEstimatorConfig()
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.v = 0.0
        self.w = 0.0
        self.t = 0.0
        self._last_wall_fix: Optional[float] = None

    # ---------------------------------------------------------------- #
    def set_pose(self, x: float, y: float, yaw: float) -> None:
        self.x, self.y, self.yaw = float(x), float(y), float(yaw)

    @property
    def pose(self) -> Tuple[float, float, float]:
        return self.x, self.y, self.yaw

    # ---------------------------------------------------------------- #
    def predict(self, dt: float, v_meas: float, w_meas: float) -> None:
        """用编码器线速度与 IMU 角速度做航迹推算（圆弧运动）。"""
        if dt <= 0:
            return
        self.v = float(v_meas)
        self.w = float(w_meas)
        yaw_mid = self.yaw + 0.5 * self.w * dt
        self.x += self.v * math.cos(yaw_mid) * dt
        self.y += self.v * math.sin(yaw_mid) * dt
        self.yaw = G.wrap_pi(self.yaw + self.w * dt)
        self.t += dt

    def update_imu_yaw(self, yaw: float, gate: float = 0.6) -> None:
        """IMU 绝对航向（缓慢修正，角速度大时不信任）。"""
        if abs(self.w) > gate:
            return
        d = G.angle_diff(yaw, self.yaw)
        if abs(d) < math.radians(30.0):
            self.yaw = G.wrap_pi(self.yaw + self.cfg.imu_yaw_gain * d)

    def update_walls(self, scan: Scan, nearest_cone_dist: float = math.inf,
                     min_period: float = 0.30) -> Dict[str, float]:
        """用长直墙面段做位置修正。

        在车体坐标系里，正对/背对的前后墙近似平行于车体 y 轴，左右墙近似平行于
        车体 x 轴；因此可以直接用"墙段到车体轴的垂距"反推世界坐标，不需要知道
        当前朝向是否与场地对齐。长墙段（>=0.6m）本身很难被锥桶伪装，因此这种方法
        在雷区内部也能持续工作，抑制累计漂移。
        """
        stats = {"corrected": 0.0, "dx": 0.0, "dy": 0.0}
        if self.field.boundary:
            return self._update_measured_walls(scan,stats,min_period)
        if self._last_wall_fix is not None and (self.t - self._last_wall_fix) < min_period:
            return stats
        pts = scan.points_body(0.20, 12.0)
        if len(pts) < 20:
            return stats
        walls = extract_walls(pts, min_points=15, max_rms=0.035)
        f = self.field
        x_meas: List[float] = []
        y_meas: List[float] = []
        for w in walls:
            # Rotate observed endpoints into the arena before choosing a known wall.
            # Body-axis tests were invalid after an Ackermann turn.
            p0=G.body_to_world(self.pose,w.p0)
            p1=G.body_to_world(self.pose,w.p1)
            d=p1-p0
            if w.length < .8:continue
            if abs(d[1]) < .18*abs(d[0]):
                off=float(.5*(p0[1]+p1[1]))
                bound=0. if off<f.width/2 else f.width
                if abs(off-bound)<self.cfg.wall_gate_m:
                    y_meas.append(self.y+bound-off)
            elif abs(d[0]) < .18*abs(d[1]):
                off=float(.5*(p0[0]+p1[0]))
                bound=0. if off<f.length/2 else f.length
                if abs(off-bound)<self.cfg.wall_gate_m:
                    x_meas.append(self.x+bound-off)
        if x_meas or y_meas:
            g = self.cfg.wall_gain
            corr_x = G.clamp(g * (float(np.mean(x_meas)) - self.x), -0.04, 0.04) if x_meas else 0.0
            corr_y = G.clamp(g * (float(np.mean(y_meas)) - self.y), -0.04, 0.04) if y_meas else 0.0
            self.x += corr_x
            self.y += corr_y
            stats.update(corrected=1.0, dx=corr_x, dy=corr_y)
            self._last_wall_fix = self.t
        return stats

    def clamp_inside_field(self, margin: float = 0.0) -> None:
        p = self.field.clamp_to_arena([self.x, self.y], margin)
        self.x, self.y = float(p[0]), float(p[1])

    def _update_measured_walls(self,scan,stats,min_period):
        if self._last_wall_fix is not None and self.t-self._last_wall_fix<min_period:return stats
        walls=extract_walls(scan.points_body(.2,12.),min_points=15,max_rms=.035)
        poly=self.field.boundary;normals=[];offsets=[]
        for w in walls:
            if w.length<.8:continue
            a=G.body_to_world(self.pose,w.p0);b=G.body_to_world(self.pose,w.p1)
            direction=(b-a)/w.length;mid=(a+b)/2;best=None
            for p,q in zip(poly,poly[1:]+poly[:1]):
                p,q=np.asarray(p),np.asarray(q);edge=q-p;length=float(np.linalg.norm(edge));unit=edge/length
                if abs(float(direction@unit))<.98:continue
                along=float((mid-p)@unit)
                if not -.1<=along<=length+.1:continue
                normal=np.array([-unit[1],unit[0]]);residual=float((p-mid)@normal)
                if abs(residual)<self.cfg.wall_gate_m and (best is None or abs(residual)<abs(best[1])):best=(normal,residual)
            if best is not None:normals.append(best[0]);offsets.append(best[1])
        if normals:
            delta=np.linalg.lstsq(np.asarray(normals),np.asarray(offsets),rcond=None)[0]*self.cfg.wall_gain
            delta=np.clip(delta,-.04,.04);self.x+=float(delta[0]);self.y+=float(delta[1]);self._last_wall_fix=self.t
            stats.update(corrected=1.,dx=float(delta[0]),dy=float(delta[1]))
        return stats
