"""仿真环境：场地与锥桶布局、车辆运动学、激光雷达/IMU/里程计模拟。"""

from __future__ import annotations

import math
from dataclasses import dataclass, field as dc_field
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

from racelib import geometry as G
from racelib.field import Field
from racelib.lidarproc import Scan
from racelib.io import ImuSample, OdomSample


@dataclass
class SimCone:
    """仿真中的锥桶：碰撞用底面半径，雷达看到的是锥桶上部截面半径。"""

    x: float
    y: float
    color: str                     # blue / red
    line: int = -1                 # 属于第几条封锁线（红锥为 -1）
    radius_base: float = 0.115
    radius_lidar: float = 0.085
    height: float = 0.40
    hit: bool = False
    hit_t: Optional[float] = None

    @property
    def xy(self) -> np.ndarray:
        return np.array([self.x, self.y], dtype=float)


def generate_cones(field: Field, rng: np.random.Generator,
                   blue_range: Tuple[int, int] = (3, 6),
                   red_range: Tuple[int, int] = (5, 8),
                   line_jitter_m: float = 0.0,
                   red_min_clear_m: float = 0.34,
                   min_usable_gap_m: float = 0.60) -> List[SimCone]:
    """按赛题规则随机生成锥桶布局。

    - 3 条蓝色封锁线，相邻线间隔 1.5m（x 方向加少量抖动）
    - 每条线 3~6 个蓝锥，线内相邻间距 >= 0.40m
    - 雷区 5~8 个红锥，随机摆放，且不与蓝锥重叠

    另外保证每条封锁线至少留一条 >= min_usable_gap_m 的可通行缝：
    组委会布置场地时必然要留出通道（否则任何车队都无法通过），
    这里显式模拟这一点，避免生成"规则允许但现实不可能"的对抗性布局。
    """
    cones: List[SimCone] = []
    y_margin = 0.35
    total = int(rng.integers(12,17))
    counts = [3,3,3]
    for _ in range(total-9):
        options = [i for i in range(3) if counts[i]<6]
        counts[int(rng.choice(options))] += 1
    for li, lx in enumerate(field.blockade_lines_x):
        x = lx + float(rng.uniform(-line_jitter_m, line_jitter_m))
        n = counts[li]
        # Spacing construction with randomized slack guarantees exact counts.
        # Official >=.40m center spacing; gaps also depend on cone bases/vehicle.
        span = field.width-2*y_margin-(n-1)*field.min_cone_gap
        if span < 0:
            raise ValueError("场地太窄，无法满足锥桶间距")
        for attempt in range(2000):
            slack = np.sort(rng.uniform(0.,span,n))
            ys = [y_margin+i*field.min_cone_gap+float(slack[i]) for i in range(n)]
            gaps = [ys[0]-.115,field.width-ys[-1]-.115]
            gaps += [ys[i+1]-ys[i]-.23 for i in range(n-1)]
            if max(gaps)>=min_usable_gap_m:
                break
        for y in ys:
            cones.append(SimCone(x,y,"blue",line=li))

    n_red = int(rng.integers(red_range[0], red_range[1] + 1))
    placed = 0
    for _ in range(4000):
        if placed >= n_red:
            break
        x = float(rng.uniform(field.minefield_x[0] + 0.30, field.minefield_x[1] - 0.30))
        y = float(rng.uniform(y_margin, field.width - y_margin))
        ok = True
        for c in cones:
            d = math.hypot(c.x - x, c.y - y)
            need = (red_min_clear_m if c.color == "red" else c.radius_base + 0.11)
            if d < need:
                ok = False
                break
        if ok:
            cones.append(SimCone(x, y, "red"))
            placed += 1
    return cones


def _ensure_usable_gap(ys: List[float], width: float, margin: float,
                       min_spacing: float, min_usable_gap: float,
                       cone_radius: float = 0.12) -> List[float]:
    """必要时拿掉最挡路的锥桶，直到线内出现一条足够宽的缝（至少保留 3 个）。"""
    ys = sorted(ys)
    for _ in range(len(ys)):
        if len(ys) <= 3:
            break
        edges = [margin] + ys + [width - margin]
        gaps = [(edges[i + 1] - edges[i]) - 2 * cone_radius for i in range(len(edges) - 1)]
        if max(gaps) >= min_usable_gap:
            break
        # 去掉"删掉后能腾出最宽缝"的那个锥桶
        best_i, best_gain = None, -1.0
        for i in range(len(ys)):
            trial = ys[:i] + ys[i + 1:]
            te = [margin] + trial + [width - margin]
            tg = max((te[k + 1] - te[k]) - 2 * cone_radius for k in range(len(te) - 1))
            if tg > best_gain:
                best_i, best_gain = i, tg
        if best_i is None:
            break
        ys.pop(best_i)
    # 保持"相邻锥桶间距 >= 40cm"的规则约束
    out: List[float] = []
    for y in ys:
        if not out or (y - out[-1]) >= min_spacing:
            out.append(y)
    return out


@dataclass
class SimRobotConfig:
    drive_model: str = "differential"
    wheelbase: float = .30
    max_steer: float = .52
    length: float = 0.42
    width: float = 0.30
    max_speed: float = 1.20
    max_yaw_rate: float = 2.60
    accel_tau: float = 0.12              # 一阶速度响应时间常数
    odom_scale_error: float = 0.015      # 编码器 1.5% 标定误差（产生里程漂移）
    odom_noise: float = 0.006
    odom_w_scale_error: float = 0.010    # 轮速差推算角速度的标定误差
    odom_w_noise: float = 0.010          # rad/s
    imu_gyro_bias: float = 0.004         # rad/s
    imu_gyro_noise: float = 0.01
    imu_yaw_drift_deg_min: float = 1.2
    imu_accel_noise: float = 0.05
    lidar_noise: float = 0.010
    lidar_angle_noise: float = 0.0015     # rad；编码器/装配角度误差


class SimWorld:
    """一个可重复（给定随机种子）的赛场真值仿真。

    ``cones``/``pose``/``contacts`` are truth.  The mission must consume only
    the noisy samples returned by ``lidar_scan``/``imu_sample``/``odom_sample``
    (and the rendered camera image) through SimIO.
    """

    def __init__(self, field: Field, cones: Sequence[SimCone],
                 rng: np.random.Generator,
                 robot: Optional[SimRobotConfig] = None,
                 lidar_beams: int = 720, lidar_range: Tuple[float, float] = (0.12, 12.0)):
        self.field = field
        self.cones = list(cones)
        self.rng = rng
        self.robot = robot or SimRobotConfig()
        self.lidar_beams = int(lidar_beams)
        self.lidar_range = lidar_range
        st = field.start
        self.x = st.cx
        self.y = st.cy
        self.yaw = field.start_heading + float(rng.normal(0.0, math.radians(1.5)))
        self.v = 0.0
        self.w = 0.0
        self.cmd_v = 0.0
        self.cmd_w = 0.0
        self.t = 0.0
        self.contacts: List[Dict[str, Any]] = []
        self.wall_contacts = 0
        self.bump_until = -1.0
        self._accel_bias = np.array([float(rng.normal(0, 0.02)) for _ in range(3)])
        self._yaw_drift = 0.0
        self._last_imu_yaw = self.yaw

    # ---------------------------------------------------------------- #
    @property
    def pose(self) -> Tuple[float, float, float]:
        return self.x, self.y, self.yaw

    def set_pose(self, x: float, y: float, yaw: float) -> None:
        self.x, self.y, self.yaw = float(x), float(y), float(yaw)

    # ---------------------------------------------------------------- #
    def set_cmd(self, v: float, w: float) -> None:
        self.cmd_v = float(v)
        self.cmd_w = float(w)

    def step(self, dt: float) -> None:
        """积分一步车辆运动与碰撞。"""
        r = self.robot
        tau = max(r.accel_tau, 1e-3)
        self.v += (G.clamp(self.cmd_v, -r.max_speed, r.max_speed) - self.v) * dt / tau
        target_w = G.clamp(self.cmd_w, -r.max_yaw_rate, r.max_yaw_rate)
        if r.drive_model == "ackermann":
            # cmd_w is yaw rate; actual car cannot turn with zero longitudinal speed.
            steer = math.atan(r.wheelbase * self.cmd_w / self.cmd_v) if abs(self.cmd_v) > .01 else 0.
            steer = G.clamp(steer, -r.max_steer, r.max_steer)
            target_w = self.v * math.tan(steer) / r.wheelbase
        self.w += (target_w - self.w) * dt / tau
        self.yaw = G.wrap_pi(self.yaw + self.w * dt)
        yaw_mid = self.yaw - 0.5 * self.w * dt
        previous=(self.x,self.y,self.yaw)
        self.x += self.v * math.cos(yaw_mid) * dt
        self.y += self.v * math.sin(yaw_mid) * dt
        self.t += dt
        self._yaw_drift += math.radians(r.imu_yaw_drift_deg_min) / 60.0 * dt
        self._check_collisions()
        self._keep_inside()
        if self.field.boundary and not all(self.field.inside_arena(p) for p in G.rect_corners(self.x,self.y,r.length,r.width,self.yaw)):
            self.wall_contacts+=1
            self.x,self.y,self.yaw=previous
            self.v=self.w=0.

    # ---------------------------------------------------------------- #
    def _check_collisions(self) -> None:
        r = self.robot
        for c in self.cones:
            if c.hit:
                continue
            if G.rect_circle_overlap(self.x, self.y, r.length, r.width, self.yaw,
                                     c.xy, c.radius_base):
                c.hit = True
                c.hit_t = self.t
                self.contacts.append({"t": self.t, "color": c.color, "x": c.x, "y": c.y,
                                      "line": c.line})
                self.bump_until = self.t + 0.35
                # 撞上之后速度略降（轮胎摩擦）
                self.v *= 0.75

    def _keep_inside(self) -> None:
        r = self.robot
        corners = G.rect_corners(self.x, self.y, r.length, r.width, self.yaw)
        lo = corners.min(axis=0)
        hi = corners.max(axis=0)
        dx = dy = 0.0
        if lo[0] < 0.0:
            dx = -lo[0]
        elif hi[0] > self.field.length:
            dx = self.field.length - hi[0]
        if lo[1] < 0.0:
            dy = -lo[1]
        elif hi[1] > self.field.width:
            dy = self.field.width - hi[1]
        if dx or dy:
            self.x += dx
            self.y += dy
            self.wall_contacts += 1
            self.v *= 0.3

    # ---------------------------------------------------------------- #
    def lidar_scan(self, t: float) -> Scan:
        """2D 光线投射：锥桶（圆）+ 四面墙，带测距噪声与少量丢点。

        注意：波束角在车体坐标系下均匀分布（Scan.angles 返回车体系角度），
        但求交必须在世界坐标系做，因此每条射线使用 (车体角 + 当前航向)。
        """
        n = self.lidar_beams
        ang = np.linspace(-math.pi, math.pi, n, endpoint=False)      # 车体系
        # Real encoders never report a mathematically perfect angular grid.
        # Apply a small per-beam angle error before ray casting and report the
        # same measured angles to the mission.
        if self.robot.lidar_angle_noise > 0:
            ang = ang + self.rng.normal(0.0, self.robot.lidar_angle_noise, size=n)
        ang_w = ang + self.yaw                                       # 世界系
        d = np.stack([np.cos(ang_w), np.sin(ang_w)], axis=-1)
        o = np.array([self.x, self.y], dtype=float)
        rng = self.lidar_range[1]
        ranges = np.full(n, rng, dtype=float)

        if self.field.boundary:
            poly=self.field.boundary
            for a,b in zip(poly,poly[1:]+poly[:1]):
                edge=np.asarray(b)-a;offset=np.asarray(a)-o
                denominator=d[:,0]*edge[1]-d[:,1]*edge[0]
                with np.errstate(divide='ignore',invalid='ignore'):
                    distance=(offset[0]*edge[1]-offset[1]*edge[0])/denominator
                    u=(offset[0]*d[:,1]-offset[1]*d[:,0])/denominator
                ranges=np.minimum(ranges,np.where((distance>1e-3)&(u>=0)&(u<=1),distance,np.inf))
        else:
            # 墙面
            for axis, lo, hi in ((0, 0.0, self.field.length), (1, 0.0, self.field.width)):
                with np.errstate(divide="ignore", invalid="ignore"):
                    for bound in (lo, hi):
                        denom = d[:, axis]
                        t_b = np.where(np.abs(denom) > 1e-9, (bound - o[axis]) / denom, np.inf)
                        t_b = np.where(t_b > 1e-3, t_b, np.inf)
                        ranges = np.minimum(ranges, t_b)
    
        # 锥桶
        for c in self.cones:
            if c.hit:
                continue
            m = o - np.array([c.x, c.y])
            b = d @ m
            cc = float(m @ m) - c.radius_lidar ** 2
            disc = b * b - cc
            ok = disc > 0.0
            if not ok.any():
                continue
            sq = np.sqrt(np.where(ok, disc, 0.0))
            t1 = -b - sq
            t1 = np.where(ok & (t1 > 0.05), t1, np.inf)
            ranges = np.minimum(ranges, t1)

        ranges = ranges + self.rng.normal(0.0, self.robot.lidar_noise, size=n)
        # 少量丢点（强反光/被遮挡）
        drop = self.rng.random(n) < 0.004
        ranges = np.where(drop, np.nan, ranges)
        ranges = np.clip(ranges, self.lidar_range[0], self.lidar_range[1])
        return Scan(angles=ang, ranges=ranges, t=t)

    # ---------------------------------------------------------------- #
    def imu_sample(self, t: float) -> ImuSample:
        r = self.robot
        w_meas = self.w + r.imu_gyro_bias + float(self.rng.normal(0.0, r.imu_gyro_noise))
        yaw_meas = G.wrap_pi(self.yaw + self._yaw_drift
                             + float(self.rng.normal(0.0, math.radians(0.4))))
        acc = np.array([self.v * self.w, 0.0, 9.81])    # 向心加速度 + 纵向 + 重力比力
        if self.t < self.bump_until:
            acc = acc + np.array([0.0, 0.0, 7.0])       # 碰撞冲击
        acc = acc + self._accel_bias + self.rng.normal(0.0, r.imu_accel_noise, size=3)
        return ImuSample(t=t, yaw=yaw_meas, yaw_rate=w_meas, accel=acc)

    def odom_sample(self, t: float) -> OdomSample:
        r = self.robot
        v_meas = self.v * (1.0 + r.odom_scale_error) + float(self.rng.normal(0.0, r.odom_noise))
        # Wheel-differential yaw rate is noisy and slightly mis-scaled; the
        # mission should not receive the simulator's exact angular velocity.
        w_meas = self.w * (1.0 + r.odom_w_scale_error) + float(
            self.rng.normal(0.0, r.odom_w_noise))
        return OdomSample(t=t, v=v_meas, w=w_meas)

    # ---------------------------------------------------------------- #
    def metrics(self) -> Dict[str, Any]:
        blue = [c for c in self.contacts if c["color"] == "blue"]
        red = [c for c in self.contacts if c["color"] == "red"]
        return {
            "blue_cone_hits": len(blue),
            "red_cone_hits": len(red),
            "wall_contacts": self.wall_contacts,
            "penalty_s": 5.0 * len(blue) + 20.0 * len(red),
            "contact_events": self.contacts,
        }
