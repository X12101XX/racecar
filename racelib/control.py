"""运动控制：纯跟踪路径跟踪 + 速度规划 + 安全监控。"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

import numpy as np

from . import geometry as G
from .config import Config


@dataclass
class ControlParams:
    lookahead_min_m: float = 0.35
    lookahead_gain: float = 0.6
    max_speed_mps: float = 0.90
    accel_mps2: float = 1.20
    decel_mps2: float = 1.80
    yaw_kp: float = 2.20
    yaw_kd: float = 0.10
    yaw_rate_limit: float = 2.20
    heading_gate_rad: float = 1.10
    clearance_stop_m: float = 0.22
    clearance_slow_m: float = 0.85
    lat_accel_limit_mps2: float = 0.90
    stop_distance_m: float = 0.10
    # ---- 实车适配（仿真里保持默认 = 不改变行为）----
    cmd_gain_linear: float = 1.0       # 需要发多大指令才能达到目标速度（标定得到）
    cmd_gain_angular: float = 1.0
    min_linear_cmd_mps: float = 0.0    # 底盘死区补偿：要动就至少给这么多
    min_angular_cmd_rps: float = 0.0

    @classmethod
    def from_config(cls, cfg: Config) -> "ControlParams":
        c = cls()
        for k in c.__dataclass_fields__:  # type: ignore[attr-defined]
            v = cfg.get(f"control.{k}")
            if v is not None:
                setattr(c, k, float(v))
        return c


class PurePursuit:
    """纯跟踪控制器：输出 (v, w) 供底盘差速驱动。"""

    def __init__(self, params: ControlParams):
        self.p = params
        self.v_cmd = 0.0
        self._prev_alpha: Optional[float] = None
        self.last_lookahead = 0.0
        self.last_target = np.zeros(2)

    def reset(self) -> None:
        self.v_cmd = 0.0
        self._prev_alpha = None

    # ---------------------------------------------------------------- #
    def compute(self, pose: Sequence[float], path: Optional[np.ndarray],
                speed_cap: float, dt: float,
                distance_to_goal: Optional[float] = None,
                clearance_ahead: float = math.inf,
                stop_at_end: bool = True) -> Tuple[float, float]:
        p = self.p
        if path is None or len(path) < 2:
            self.v_cmd = max(0.0, self.v_cmd - p.decel_mps2 * dt)
            return self._shape(self.v_cmd, 0.0)

        dist, s_now, _ = G.closest_point_on_polyline(pose[:2], path)
        total = G.polyline_length(path)
        remaining = max(0.0, total - s_now)
        if distance_to_goal is not None:
            remaining = min(remaining, float(distance_to_goal))

        # 到终点后减速停车
        v_target = min(speed_cap, p.max_speed_mps)
        if stop_at_end:
            v_stop = math.sqrt(max(0.0, 2.0 * p.decel_mps2 *
                                   max(0.0, remaining - p.stop_distance_m)))
            v_target = min(v_target, v_stop)
        # 前方障碍减速
        if math.isfinite(clearance_ahead):
            if clearance_ahead <= p.clearance_stop_m:
                v_target = 0.0
            elif clearance_ahead < p.clearance_slow_m:
                v_target *= max(0.15, (clearance_ahead - p.clearance_stop_m) /
                                max(1e-3, p.clearance_slow_m - p.clearance_stop_m))

        # 曲率限速
        look = max(p.lookahead_min_m, p.lookahead_gain * max(self.v_cmd, 0.15))
        self.last_lookahead = look
        target = G.point_at_arc_length(path, s_now + look)
        self.last_target = target
        alpha = G.angle_diff(math.atan2(target[1] - pose[1], target[0] - pose[0]), pose[2])
        kappa = abs(G.curvature_at(path, min(s_now + look, total - 1e-3)))
        if kappa > 1e-3:
            v_target = min(v_target, math.sqrt(p.lat_accel_limit_mps2 / kappa))

        # 大角度偏差：先慢速摆正
        v_target = G.clamp(v_target, 0.0, p.max_speed_mps)
        v_near = p.clearance_stop_m * 0.6
        if abs(alpha) > p.heading_gate_rad:
            v_target = min(v_target, v_near)

        # 加速度限制
        if v_target > self.v_cmd:
            self.v_cmd = min(v_target, self.v_cmd + p.accel_mps2 * dt)
        else:
            self.v_cmd = max(v_target, self.v_cmd - p.decel_mps2 * dt)

        # 角速度：纯跟踪 + PD 项（前视点角度误差）
        v_ref = max(self.v_cmd, 0.12)
        w_pp = 2.0 * v_ref * math.sin(alpha) / max(look, 1e-3)
        w_pd = p.yaw_kp * alpha
        if self._prev_alpha is not None and dt > 1e-4:
            w_pd += p.yaw_kd * (alpha - self._prev_alpha) / dt
        w = 0.5 * w_pp + 0.5 * w_pd
        self._prev_alpha = alpha
        w = G.clamp(w, -p.yaw_rate_limit, p.yaw_rate_limit)
        if self.v_cmd < 1e-3 and abs(alpha) < 0.05:
            w = 0.0
        return self._shape(self.v_cmd, w)

    # ---------------------------------------------------------------- #
    def _shape(self, v: float, w: float) -> Tuple[float, float]:
        """把"期望速度"换算成"实际下发的底盘指令"。

        - 标定系数：实车往往需要更大指令才能达到目标速度（或相反），
          用 `calibrate_drive` 测出来后填进 plan.json；
        - 死区补偿：很多底盘 PWM 有死区，小指令等于没给，这里保证
          "只要想动就至少给到最小有效指令"，避免车在原地抖。
        仿真里这几个参数保持默认（1.0 / 0.0），因此仿真行为完全不变。
        """
        p = self.p
        v_out = v * p.cmd_gain_linear
        if p.min_linear_cmd_mps > 0.0 and 0.0 < abs(v_out) < p.min_linear_cmd_mps:
            v_out = math.copysign(p.min_linear_cmd_mps, v_out)
        w_out = w * p.cmd_gain_angular
        if p.min_angular_cmd_rps > 0.0 and 0.0 < abs(w_out) < p.min_angular_cmd_rps:
            w_out = math.copysign(p.min_angular_cmd_rps, w_out)
        return float(v_out), float(w_out)


def min_clearance_ahead(scan, pose, half_angle_deg: float = 30.0,
                        max_range: float = 3.0) -> float:
    """车体前向扇区内的最近障碍距离（用于减速）。"""
    if scan is None:
        return math.inf
    a = scan.angles
    r = scan.ranges
    m = np.isfinite(r) & (np.abs(a) < math.radians(half_angle_deg)) & (r > 0.05)
    if not m.any():
        return math.inf
    return float(min(np.min(r[m]), max_range))


def clearance_toward(scan, angle_body: float, half_angle_deg: float = 18.0,
                     max_range: float = 4.0) -> float:
    """指定车体方向（弧度）上的最近障碍距离，用于"对准缝隙直行"策略。"""
    if scan is None:
        return math.inf
    a = scan.angles
    r = scan.ranges
    d = np.abs(np.arctan2(np.sin(a - angle_body), np.cos(a - angle_body)))
    m = np.isfinite(r) & (d < math.radians(half_angle_deg)) & (r > 0.03)
    if not m.any():
        return math.inf
    return float(min(float(np.min(r[m])), max_range))


@dataclass
class SafetyEvent:
    kind: str            # estop_distance / cmd_timeout / bump
    detail: str = ""
    t: float = 0.0


class SafetyMonitor:
    """安全监控：贴障碍急停、控制指令超时、撞击（加速度突变）检测。"""

    def __init__(self, cfg: Config):
        self.estop_distance = float(cfg.get("safety.estop_distance_m", 0.14))
        self.estop_accel_g = float(cfg.get("safety.estop_accel_g", 1.30))
        self.cmd_timeout = float(cfg.get("safety.cmd_timeout_s", 0.40))
        self._last_cmd_t: Optional[float] = None
        self._last_bump_t = -1e9
        self.events: list = []
        self.gravity = 9.81

    # ---------------------------------------------------------------- #
    def note_cmd(self, t: float) -> None:
        self._last_cmd_t = t

    def check_timeout(self, t: float) -> bool:
        return (self._last_cmd_t is not None) and (t - self._last_cmd_t) > self.cmd_timeout

    def check_distance(self, scan, pose, margin_deg: float = 40.0) -> bool:
        if scan is None:
            return False
        a = scan.angles
        r = scan.ranges
        m = np.isfinite(r) & (np.abs(a) < math.radians(margin_deg)) & (r > 0.02)
        if not m.any():
            return False
        return bool(np.min(r[m]) < self.estop_distance)

    def check_bump(self, accel: Sequence[float], t: float,
                   debounce_s: float = 0.4) -> Optional[SafetyEvent]:
        """撞击检测：比力模长偏离重力超过阈值（IMU 输出含重力的比力）。"""
        a = np.asarray(accel, dtype=float).ravel()
        if a.shape[0] < 3:
            a = np.concatenate([a, np.zeros(3 - a.shape[0])])
        dev = abs(float(np.linalg.norm(a)) - self.gravity)
        if dev > self.estop_accel_g * self.gravity / 4.0 and \
                (t - self._last_bump_t) > debounce_s:
            self._last_bump_t = t
            ev = SafetyEvent("bump", f"|a| deviation {dev:.2f} m/s^2", t)
            self.events.append(ev)
            return ev
        return None
