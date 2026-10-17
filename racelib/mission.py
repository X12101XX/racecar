"""任务状态机：识别色卡 -> 播报 -> 到据点 -> 停留 3s -> 穿越雷区 -> 到营地 -> 停车。

两种路线（线上赛不抽色卡，直接指定）：
    路线 A（蓝色）：起点 -> 据点A（左上）-> 斜穿雷区 -> 营地A（右下）
    路线 B（红色）：起点 -> 据点B（左下）-> 斜穿雷区 -> 营地B（右上）
线下/总决赛抽色卡时可开启 use_card，用摄像头识别蓝/红色卡自动选路线。
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

from . import geometry as G
from .config import Config
from .control import (ControlParams, PurePursuit, SafetyEvent, SafetyMonitor,
                      clearance_toward, min_clearance_ahead)
from .field import Cone, Field, Zone
from .fusion import ConeMap, ConeMapConfig
from .gridmap import GridMap
from .io import ImuSample, OdomSample, RobotIO
from .lidarproc import Scan, extract_cones
from .localization import PoseEstimator, PoseEstimatorConfig
from .planner import plan_gap_waypoints, plan_path
from .vision import Vision


class State:
    INIT = "INIT"
    CARD = "CARD"
    ANNOUNCE = "ANNOUNCE"
    GO_STRONGHOLD = "GO_STRONGHOLD"
    HOLD = "HOLD"
    CROSS = "CROSS"
    FINISH = "FINISH"
    DONE = "DONE"
    FAILED = "FAILED"


PENALTY_BLUE_CONE = 5.0
PENALTY_NO_ANNOUNCE = 10.0
PENALTY_HUMAN = 100.0
PENALTY_NO_STOP = 20.0


@dataclass
class MissionConfig:
    use_card: bool = False
    monochrome: bool = False
    route: str = "A"
    hold_seconds: float = 3.0
    zone_tolerance_m: float = 0.32
    center_tolerance_m: float = 0.25
    stage_timeout_s: Dict[str, float] = field(default_factory=lambda: {
        "go_stronghold": 60.0, "cross_mine": 150.0, "go_camp": 60.0, "finish": 20.0})
    card_timeout_s: float = 12.0
    cross_speed_mps: float = 0.55
    creep_speed_mps: float = 0.22

    @classmethod
    def from_config(cls, cfg: Config, route: Optional[str] = None,
                    use_card: bool = False) -> "MissionConfig":
        mc = cls()
        mc.use_card = bool(use_card)
        if mc.use_card:
            mc.monochrome = False
        else:
            mc.monochrome = bool(cfg.get("mission.monochrome", False))
        if route:
            mc.route = str(route).upper()
        for k in ("hold_seconds", "zone_tolerance_m", "center_tolerance_m"):
            v = cfg.get(f"mission.{k}")
            if v is not None:
                setattr(mc, k, float(v))
        st = cfg.get("mission.stage_timeout_s")
        if isinstance(st, dict):
            mc.stage_timeout_s = {k: float(v) for k, v in st.items()}
        for k in ("cross_speed_mps", "creep_speed_mps"):
            v = cfg.get(f"control.{k}")
            if v is not None:
                setattr(mc, k, float(v))
        return mc


class Mission:
    """完整任务逻辑（平台无关，仿真与实车 1:1 复用）。"""

    def __init__(self, io: RobotIO, cfgs: Dict[str, Config],
                 mission_cfg: Optional[MissionConfig] = None):
        self.io = io
        self.cfgs = cfgs
        self.cfg = mission_cfg or MissionConfig.from_config(cfgs["plan"])
        self.field = Field.from_config(cfgs["field"])
        vision_cfg=cfgs["vision"]
        if self.cfg.monochrome:
            import copy
            vision_cfg=Config(copy.deepcopy(vision_cfg.data))
            vision_cfg.data['ai']={'backend':'fast_color'}
        self.vision = Vision(vision_cfg)
        self.cones = ConeMap(self.field, ConeMapConfig())
        self.estimator = PoseEstimator(self.field, PoseEstimatorConfig())
        # step() polls sensors before its INIT handler. Initialize the map frame now,
        # otherwise the first scan seeds ghost obstacles around the (0,0,0) origin.
        self.estimator.set_pose(self.field.start.cx,self.field.start.cy,self.field.start_heading)
        self.grid = GridMap(
            self.field,
            res=float(cfgs["plan"].get("grid.resolution_m", 0.05)),
            hard_clearance=float(cfgs["plan"].get("grid.hard_inflate_m", 0.22)),
            wall_clearance=float(cfgs["plan"].get("grid.wall_inflate_m", 0.20)),
            soft_inflate=float(cfgs["plan"].get("grid.soft_inflate_m", 0.45)),
            soft_weight=float(cfgs["plan"].get("grid.soft_cost_weight", 2.5)),
        )
        self.control = PurePursuit(ControlParams.from_config(cfgs["plan"]))
        self.safety = SafetyMonitor(cfgs["plan"])

        self.car_len = float(cfgs["robot"].get("geometry.length_m", 0.42))
        self.car_wid = float(cfgs["robot"].get("geometry.width_m", 0.30))

        if "route" not in cfgs or cfgs.get("route") is None:
            cfg_dir = os.path.dirname(cfgs["field"].path)
            cfgs["route"] = Config.load(os.path.join(cfg_dir, "route_a.json"))
        self.route = cfgs["route"]
        if self.cfg.route.lower() != str(self.route.get("id", "")).lower():
            self.cfg.route = str(self.route.get("id", self.cfg.route)).upper()

        self.state = State.INIT
        self.t0 = io.now()
        self.state_t0 = self.t0
        self.events: List[Dict[str, Any]] = []
        self.metrics: Dict[str, Any] = {
            "route": self.cfg.route,
            "t_start_line": None,
            "t_finish": None,
            "run_time_s": None,
            "blue_contacts": 0,
            "red_contacts": 0,
            "penalties_s": 0.0,
            "announced": False,
            "stopped_at_finish": False,
            "hold_actual_s": None,
            "gaps_used": [],
            "min_cone_clearance_m": None,
        }
        self._last_scan_t = -1.0
        self._last_img_t = -1.0
        self._last_odom: Optional[OdomSample] = None
        self._last_imu: Optional[ImuSample] = None
        self._last_scan: Optional[Scan] = None
        self._last_card: Optional[str] = None
        self._card_seen_t = 0.0
        self._last_cones: List[Any] = []          # 最近一帧相机的锥桶观测（判撞锥颜色用）
        self._floor = {"blue": 0.0, "red": 0.0}
        self._floor_streak = {"blue": 0, "red": 0}
        self._path: Optional[np.ndarray] = None
        self._path_goal: Optional[np.ndarray] = None
        self._last_plan_t = -1e9
        self._waypoints: List[np.ndarray] = []
        self._wp_index = 0
        self._hold_started: Optional[float] = None
        self._started_moving = False
        self._min_clearance = math.inf
        self._last_dt = 0.05
        self._planning_radius_bonus = float(getattr(self.cones.cfg,
                                                    "planning_radius_bonus", 0.04))
        self._backoff_until = -1e9
        self._backoff_deadline = -1e9
        self._good_clearance = self.grid.hard_clearance
        self._guard_since_plan = False
        self._breadcrumbs: List[Tuple[float, float, float, float]] = []
        self._last_breadcrumb_t = -1e9
        self._last_progress_t = self.t0
        self._best_goal_dist = math.inf
        self._recover_target: Optional[np.ndarray] = None
        self._recover_count = 0
        self._wp_fail_since: Optional[float] = None
        self._guard_trigger_clearance = 0.0
        self._avoid_zones: List[Tuple[float, float, float, float]] = []
        self._last_fail_log_t = -1e9
        self._last_guard_log_t = -1e9
        self.log("mission_init", route=self.cfg.route, field=self.field.summary())

    # ================================================================= #
    def log(self, event: str, **fields: Any) -> None:
        rec = {"t": float(self.io.now()), "event": event}
        rec.update(fields)
        self.events.append(rec)
        try:
            self.io.log(event, **fields)
        except Exception:  # noqa: BLE001 - 记录失败不影响主流程
            pass

    def pose(self) -> Tuple[float, float, float]:
        return self.estimator.pose

    def status(self) -> Dict[str, Any]:
        return {
            "state": self.state,
            "t": round(float(self.io.now()), 3),
            "pose": [round(v, 3) for v in self.pose()],
            "route": self.cfg.route,
            "cones": self.cones.stats(),
            "wp_index": self._wp_index,
            "waypoints": len(self._waypoints),
            "metrics": {k: v for k, v in self.metrics.items() if k != "gaps_used"},
        }

    # ================================================================= #
    def step(self, dt: float) -> Tuple[float, float]:
        """执行一步（建议 20~50Hz 调用）。返回本周期指令 (v, w)。"""
        self._last_dt = max(1e-4, float(dt))
        self._poll_sensors()
        self._predict(self._last_dt)
        handler = {
            State.INIT: self._st_init,
            State.CARD: self._st_card,
            State.ANNOUNCE: self._st_announce,
            State.GO_STRONGHOLD: self._st_go_stronghold,
            State.HOLD: self._st_hold,
            State.CROSS: self._st_cross,
            State.FINISH: self._st_finish,
        }.get(self.state)
        if handler is None:
            self.io.cmd(0.0, 0.0)
            return 0.0, 0.0
        v, w = handler(self._last_dt)
        self._update_progress()
        v, w = self._apply_close_guard(v, w)
        v, w = self._apply_safety(v, w)
        self.io.cmd(v, w)
        self.safety.note_cmd(self.io.now())
        self._track_metrics(self._last_dt)
        return v, w

    # ----------------------------------------------------------------- #
    def _poll_sensors(self) -> None:
        odom = self.io.odom()
        if odom is not None:
            self._last_odom = odom
        imu = self.io.imu()
        if imu is not None:
            self._last_imu = imu
            ev = self.safety.check_bump(imu.accel, imu.t)
            if ev is not None:
                self._on_bump(ev)

        scan = self.io.scan()
        if scan is not None and scan.t > self._last_scan_t:
            self._last_scan_t = scan.t
            self._last_scan = scan
            dets = extract_cones(scan.points_body())
            pose = self.pose()
            self.cones.update(dets, pose, scan.t)
            near = self.cones.nearest_distance(pose)
            self.estimator.update_walls(scan, near)

        img = self.io.image()
        if img is not None and not self.cfg.monochrome:
            image, t_img = img
            if t_img > self._last_img_t:
                self._last_img_t = t_img
                want_card = self.state in (State.CARD, State.INIT)
                res = self.vision.process(image, want_card=want_card)
                self._floor["blue"] = res.floor_blue_ratio
                self._floor["red"] = res.floor_red_ratio
                for k in ("blue", "red"):
                    self._floor_streak[k] = (self._floor_streak[k] + 1
                                             if self._floor[k] >= self.vision.patch_min_ratio
                                             else 0)
                if res.cones:
                    self.cones.annotate_colors(res.cones, self.pose())
                self._last_cones = list(res.cones)
                if res.card_color:
                    self._last_card = res.card_color
                    self._card_seen_t = t_img

    def _predict(self, dt: float) -> None:
        v = self._last_odom.v if self._last_odom else 0.0
        w = (self._last_imu.yaw_rate if self._last_imu
             else (self._last_odom.w if self._last_odom else 0.0))
        self.estimator.predict(dt, v, w)
        if self._last_imu is not None and self._last_imu.yaw is not None:
            self.estimator.update_imu_yaw(self._last_imu.yaw)
        if self._started_moving:
            self._min_clearance = min(self._min_clearance,
                                      self.cones.nearest_distance(self.pose()))

    # ----------------------------------------------------------------- #
    def _apply_safety(self, v: float, w: float) -> Tuple[float, float]:
        if v > 0.0 and self._last_scan is not None and \
                self.safety.check_distance(self._last_scan, self.pose()):
            self.log("estop_distance")
            return 0.0, w * 0.3
        if self.safety.check_timeout(self.io.now()):
            self.log("estop_cmd_timeout")
            return 0.0, 0.0
        return v, w

    # ----------------------------------------------------------------- #
    def footprint_clearance(self) -> float:
        """当前位姿下，车体矩形到"已建图锥桶"的最小间隙（米，含地图误差补偿）。"""
        tracks = self.cones.blockers()
        if not tracks:
            return math.inf
        pose = self.pose()
        C = np.array([[t.x, t.y] for t in tracks], dtype=float)
        R = np.array([t.radius + self._planning_radius_bonus for t in tracks], dtype=float)
        rel = C - np.array([pose[0], pose[1]], dtype=float)[None, :]
        c, s = math.cos(pose[2]), math.sin(pose[2])
        vx = c * rel[:, 0] + s * rel[:, 1]
        vy = -s * rel[:, 0] + c * rel[:, 1]
        qx = np.clip(vx, -self.car_len / 2.0, self.car_len / 2.0)
        qy = np.clip(vy, -self.car_wid / 2.0, self.car_wid / 2.0)
        return float(np.min(np.hypot(vx - qx, vy - qy) - R))

    def _rear_clearance(self) -> float:
        if self._last_scan is None:
            return math.inf
        a = self._last_scan.angles
        r = self._last_scan.ranges
        m = np.isfinite(r) & (np.abs(np.abs(a) - math.pi) < math.radians(35.0)) & (r > 0.03)
        if not m.any():
            return math.inf
        return float(np.min(r[m]))

    def _freest_direction(self) -> Tuple[float, float]:
        """返回 (车体系下最空旷的方向角, 该方向的最近距离)。"""
        if self._last_scan is None:
            return 0.0, math.inf
        a = self._last_scan.angles
        r = np.where(np.isfinite(self._last_scan.ranges), self._last_scan.ranges, 0.0)
        best_ang, best_r = 0.0, -1.0
        for ang in np.radians(np.arange(-150, 151, 10.0)):
            m = np.abs(np.arctan2(np.sin(a - ang), np.cos(a - ang))) < math.radians(20.0)
            if not m.any():
                continue
            rr = float(np.min(r[m]))
            if rr > best_r:
                best_r, best_ang = rr, float(ang)
        return best_ang, best_r

    # ----------------------------------------------------------------- #
    # 进度监控与"死胡同"恢复
    # ----------------------------------------------------------------- #
    def _update_progress(self) -> None:
        """记录足迹面包屑；长时间不靠近目标就判定为卡在死胡同，准备原路退出。"""
        now = self.io.now()
        if self.state not in (State.GO_STRONGHOLD, State.CROSS):
            return
        goal = (self._target_zone("camp").center if self.state == State.CROSS
                else self._target_zone("stronghold").center)
        pose = self.pose()
        d = float(np.linalg.norm(goal - pose[:2]))
        if d < self._best_goal_dist - 0.25:
            self._best_goal_dist = d
            self._last_progress_t = now
        if (now - self._last_breadcrumb_t) > 0.2:
            self._last_breadcrumb_t = now
            self._breadcrumbs.append((pose[0], pose[1], now,
                                      self.footprint_clearance()))
            self._breadcrumbs = [b for b in self._breadcrumbs if (now - b[2]) < 14.0]
        if self._recover_target is None and self.state == State.CROSS and \
                (now - self._last_progress_t) > 8.0:
            self._start_recover()

    def _start_recover(self) -> None:
        """从最近的历史足迹里挑一个"安全且更靠近目标"的点作为临时目标。

        关键点：不能按"间隙最大"挑，否则会把车一路送回起点开阔区，形成来回打转；
        这里改为在最近 8 秒内、间隙达标（>=0.28m）的足迹里选"离终点最近"的那个，
        也就是退回死胡同入口前的位置。
        """
        now = self.io.now()
        pose = self.pose()
        goal = self._target_zone("camp").center
        recent = [b for b in self._breadcrumbs if (now - b[2]) <= 8.0]
        good = [b for b in recent if b[3] >= 0.28] or recent
        if not good:
            self._last_progress_t = now
            return

        def score(b) -> float:
            d_goal = float(np.linalg.norm(np.array(b[:2]) - goal))
            d_now = math.hypot(b[0] - pose[0], b[1] - pose[1])
            return d_goal + 0.5 * d_now

        best = min(good, key=score)
        if math.hypot(best[0] - pose[0], best[1] - pose[1]) < 0.5:
            self._last_progress_t = now
            return
        self._recover_target = np.array([best[0], best[1]], dtype=float)
        self._recover_count += 1
        # 把死胡同记成软禁区（20 秒），避免绕一圈又钻回来
        self._avoid_zones.append((pose[0], pose[1], 0.7, now + 20.0))
        self._path = None
        self._good_clearance = self.grid.hard_clearance
        self._last_progress_t = now
        self._best_goal_dist = math.inf
        self.log("recover_start", count=self._recover_count,
                 target=[round(float(best[0]), 2), round(float(best[1]), 2)],
                 clearance=round(float(best[3]), 3))

    def _apply_close_guard(self, v: float, w: float) -> Tuple[float, float]:
        """贴身保护（反应层）：与锥桶过近时"避让式转向 + 限速"，而不是一味倒车。

        窄缝（0.5~0.7m）里倒车会让车永远卡在缝口；正确的做法是：离锥桶越近、
        方位越靠近车头，就越慢，同时朝远离锥桶的一侧轻微打舵，让车贴着锥桶滑过去。
        只有在几乎贴死（地图上已经重叠）且锥桶正对着车头时才倒车脱离。
        """
        now = self.io.now()
        if now < self._backoff_deadline:
            cl = self.footprint_clearance()
            if cl > self._guard_trigger_clearance + 0.12:
                self._backoff_deadline = now        # 脱离成功
                self._path = None                   # 强制重规划
                return 0.0, 0.0
            rear = self._rear_clearance()
            if rear > 0.18:
                # 后向净空越紧张，倒车越慢
                v_back = -min(0.15, max(0.05, (rear - 0.14) * 0.7))
                return v_back, 0.0
            # 车尾被堵：原地转向最空旷的方向脱困
            free_ang, free_r = self._freest_direction()
            w = G.clamp(1.6 * free_ang, -1.5, 1.5)
            v = 0.08 if (abs(free_ang) < 0.5 and free_r > 0.5) else 0.0
            return v, w
        if now < self._backoff_until:
            return 0.0, 0.0
        cl = self.footprint_clearance()
        if cl > 0.30 or not self.cones.snapshot():
            return v, w

        # 找最近的已建图锥桶，按"方位 + 间隙"做反应式避让
        pose = self.pose()
        near = None
        near_d = 1e9
        for tr in self.cones.blockers():
            d = float(np.hypot(tr.x - pose[0], tr.y - pose[1]))
            if d < near_d:
                near, near_d = tr, d
        if near is None:
            return v, w
        bearing = G.angle_diff(math.atan2(near.y - pose[1], near.x - pose[0]), pose[2])
        # 只关心车头前方 ±100° 的目标；后方目标交给安全监控
        if abs(bearing) > math.radians(100.0):
            return v, w
        # 距离越近、越正对车头 -> 越慢；疑似地雷（红/未标色）留更大余量
        is_mine = near.color in ("red", "unknown")
        slow_zone = 0.42 if is_mine else 0.30
        if cl < slow_zone:
            ratio = max(0.0, min(1.0, (cl - 0.02) / (slow_zone - 0.02)))
            v = min(v, self.cfg.creep_speed_mps * max(0.25, ratio))
        if cl < (0.10 if is_mine else 0.02) and abs(bearing) < math.radians(35.0):
            # 已经贴上且正对车头：短促倒车脱离
            self._backoff_until = now + 0.30
            self._backoff_deadline = now + 1.5
            self._guard_trigger_clearance = cl
            self._guard_since_plan = True
            self._avoid_zones.append((near.x, near.y, 0.40, now + 4.0))
            if (now - self._last_guard_log_t) > 1.0:
                self._last_guard_log_t = now
                self.log("close_guard_backoff", clearance=round(cl, 3))
            return 0.0, w
        # 避让转向：远离最近锥桶（疑似地雷更早开始让开）
        steer_zone = 0.24 if is_mine else 0.16
        if cl < steer_zone:
            steer = -1.0 if bearing > 0 else 1.0
            w = G.clamp(w + steer * 0.6 * (steer_zone - cl) / steer_zone,
                        -self.control.p.yaw_rate_limit, self.control.p.yaw_rate_limit)
            if (now - self._last_guard_log_t) > 1.5:
                self._last_guard_log_t = now
                self.log("close_guard_slow", clearance=round(cl, 3),
                         bearing_deg=round(math.degrees(bearing), 1))
        return v, w

    def _active_avoid_zones(self) -> List[np.ndarray]:
        now = self.io.now()
        self._avoid_zones = [z for z in self._avoid_zones if z[3] > now]
        return [np.array([z[0], z[1], z[2]]) for z in self._avoid_zones]

    def avoid_penalties(self) -> List[Tuple[float, float, float, float]]:
        """把"刚才蹭到的位置"变成软代价项 (x, y, 半径, 权重)。"""
        return [(float(z[0]), float(z[1]), 0.45, 4.0) for z in self._active_avoid_zones()]

    def _path_hit_by_new_cones(self, threshold_m: float = 0.60) -> bool:
        """新出现的锥桶是否落在当前路径附近（决定是否立刻重规划）。"""
        if self._path is None or not len(self.cones.last_added):
            return False
        pts = G.resample_polyline(self._path, 0.10)
        for tr in self.cones.last_added:
            d = float(np.min(np.linalg.norm(pts - tr.xy[None, :], axis=1)))
            if d < threshold_m + tr.radius:
                return True
        return False

    def plan_ladder(self) -> List[float]:
        """规划安全余量尝试序列：先用上次成功的余量，再退到保守值。"""
        base = float(self.grid.hard_clearance)
        cand = [float(self._good_clearance), base, base - 0.03, base - 0.06, base - 0.08,
                base - 0.10]
        out: List[float] = []
        for c in cand:
            if c >= 0.15 and all(abs(c - o) > 1e-6 for o in out):
                out.append(c)
        return out

    def planning_cones(self) -> List[Cone]:
        """规划用锥桶：观测半径 + 地图误差补偿；疑似地雷再加大一圈。

        赛题罚则不对称：蹭到蓝锥只罚 5 秒，碰到红色"地雷"直接判失败，
        所以红锥/未标色目标额外放大一圈（"宁蹭蓝锥，不碰地雷"）。
        """
        extra = float(getattr(self.cones.cfg, "mine_extra_margin_m", 0.07))
        out = []
        for t in self.cones.blockers():
            bonus = self._planning_radius_bonus
            if t.color in ("red", "unknown"):
                bonus += extra
            out.append(Cone(t.x, t.y, t.color, t.radius + bonus, "fused", 0.9))
        return out

    def _on_bump(self, ev: SafetyEvent) -> None:
        """撞到东西时判断"撞的是什么颜色"。

        判据要区分两类东西：**锥桶**（竖条状目标）和**地面色块**（据点/营地地贴）。
        早期版本直接看"画面下方蓝/红占比"，结果在红色据点附近撞到蓝锥时，
        地贴的红色被当成锥桶颜色 -> 把"罚 5 秒"误判成"撞雷失败"，直接丢分。
        现在按可靠性排序：
          1. 雷达地图里正前方 0.45m 内、方位 ±75° 的锥桶（位置证据）
          2. 相机"锥桶检测"结果里，画面下方中部面积最大的蓝/红竖条（形状证据）
          3. 都不明确 -> unknown（只记录，不判失败：宁可少罚，不可错杀）
        """
        pose = self.pose()
        front = G.body_to_world(pose, [self.car_len / 2.0, 0.0])
        near_color, near_d = None, math.inf
        for tr in self.cones.snapshot():
            d = float(np.linalg.norm(tr.xy - front))
            bearing = abs(G.angle_diff(math.atan2(tr.y - pose[1], tr.x - pose[0]), pose[2]))
            if d < near_d and bearing < math.radians(75.0):
                near_d, near_color = d, tr.color

        hit = "unknown"
        evidence = "none"
        if near_d < 0.45 and near_color in ("blue", "red"):
            hit, evidence = near_color, "map"
        else:
            # 形状证据：只用"锥桶检测"（竖条状、面积大），不用地面色块
            best_area, best_color = 0.0, None
            for obs in self._last_cones:
                if obs.bbox[1] < self.vision.cam.height * 0.35:      # 太靠上，不是车头前的
                    continue
                area = obs.area_px
                if area > best_area:
                    best_area, best_color = area, obs.color
            if best_color and best_area > self.vision.cone_min_area * 4:
                hit, evidence = best_color, "camera"
        self.log("bump", detail=ev.detail, hit_color=hit, evidence=evidence,
                 distance=round(near_d, 3))
        if hit == "red":
            self.metrics["red_contacts"] += 1
            self._fail("撞到地雷（红色锥桶）")
        elif hit == "blue":
            self.metrics["blue_contacts"] += 1
            self.metrics["penalties_s"] += PENALTY_BLUE_CONE

    def _fail(self, reason: str) -> None:
        if self.state != State.FAILED:
            self.log("mission_failed", reason=reason)
            self.state = State.FAILED
            self.io.cmd(0.0, 0.0)

    # ----------------------------------------------------------------- #
    def _st_init(self, dt: float) -> Tuple[float, float]:
        st = self.field.start
        self.estimator.set_pose(st.cx, st.cy, self.field.start_heading)
        self.state = State.CARD if self.cfg.use_card else State.ANNOUNCE
        self.state_t0 = self.io.now()
        self.log("init_done", state=self.state, use_card=self.cfg.use_card)
        return 0.0, 0.0

    def _st_card(self, dt: float) -> Tuple[float, float]:
        now = self.io.now()
        if self._last_card and (now - self._card_seen_t) < 1.5:
            route = self.vision.card_to_route(self._last_card)
            self.log("card_recognized", color=self._last_card, route=route)
            if route:
                self._set_route(route)
            self.state = State.ANNOUNCE
            self.state_t0 = now
        elif (now - self.state_t0) > self.cfg.card_timeout_s:
            self.log("card_timeout")
            self._fail("未识别到连续两帧色卡；请检查相机或显式使用手动路线模式")
        return 0.0, 0.0

    def _set_route(self, route: str) -> None:
        path = os.path.join(os.path.dirname(self.cfgs["field"].path),
                            f"route_{route.lower()}.json")
        if os.path.exists(path):
            self.cfgs["route"] = Config.load(path)
            self.route = self.cfgs["route"]
            self.cfg.route = route.upper()
            self.metrics["route"] = self.cfg.route

    def _st_announce(self, dt: float) -> Tuple[float, float]:
        text = str(self.route.get("announce", f"我将前往路线{self.cfg.route}"))
        self.io.speak(text)
        self.io.display([text,
                         f"路线 {self.cfg.route}",
                         f"据点 {self.route.get('stronghold')} -> 营地 {self.route.get('camp')}"])
        self.metrics["announced"] = True
        self.log("announce", text=text)
        self.state = State.GO_STRONGHOLD
        self.state_t0 = self.io.now()
        return 0.0, 0.0

    # ----------------------------------------------------------------- #
    def _target_zone(self, key: str) -> Zone:
        return self.field.zone(str(self.route.get(key)))

    def _st_go_stronghold(self, dt: float) -> Tuple[float, float]:
        zone = self._target_zone("stronghold")
        if self._zone_reached(zone):
            self._enter_hold()
            return 0.0, 0.0
        if self.io.now() - self.state_t0 > self.cfg.stage_timeout_s.get("go_stronghold", 60.0):
            self._fail("前往据点超时")
            return 0.0, 0.0
        return self._drive_to(zone.center, speed_cap=self.cfg.cross_speed_mps,
                              goal_tolerance=self.cfg.zone_tolerance_m)

    def _enter_hold(self) -> None:
        self.state = State.HOLD
        self.state_t0 = self.io.now()
        self._hold_started = self.io.now()
        self.log("stronghold_arrived", zone=str(self.route.get("stronghold")),
                 pose=[round(v, 3) for v in self.pose()])

    def _st_hold(self, dt: float) -> Tuple[float, float]:
        now = self.io.now()
        held = now - (self._hold_started or now)
        if held >= self.cfg.hold_seconds:
            self.metrics["hold_actual_s"] = round(held, 3)
            self.log("hold_done", seconds=round(held, 3))
            self._start_cross()
        return 0.0, 0.0

    # ----------------------------------------------------------------- #
    def _start_cross(self) -> None:
        self.state = State.CROSS
        self.state_t0 = self.io.now()
        self._build_waypoints()
        self._path = None
        self._last_plan_t = -1e9
        self.log("cross_start",
                 waypoints=[[round(float(p[0]), 2), round(float(p[1]), 2)]
                            for p in self._waypoints])

    def _build_waypoints(self) -> None:
        f = self.field
        entry_cfg = self.route.get("cross_entry") or {}
        exit_cfg = self.route.get("cross_exit") or {}
        y_lo, y_hi = self.route.get("corridor_y", [0.35, f.width - 0.35])
        y_lo = max(0.35, float(y_lo))
        y_hi = min(f.width - 0.35, float(y_hi))
        entry = np.array([float(entry_cfg.get("x", f.minefield_x[0] + 0.45)),
                          G.clamp(float(entry_cfg.get("y_hint", 0.5 * (y_lo + y_hi))),
                                  y_lo, y_hi)])
        exit_ = np.array([float(exit_cfg.get("x", f.minefield_x[1] + 0.15)),
                          G.clamp(float(exit_cfg.get("y_hint", 0.5 * (y_lo + y_hi))),
                                  y_lo, y_hi)])
        min_gap = max(self.car_wid + 0.16, f.min_cone_gap)
        min_gap = max(min_gap, 2.0 * self.grid.hard_clearance + 0.06)
        gap_wps = plan_gap_waypoints(f, self.cones.snapshot(), entry, exit_,
                                     y_lo, y_hi, min_gap=min_gap)
        camp = self._target_zone("camp")
        self._waypoints = [entry] + list(gap_wps) + [exit_, camp.center]
        self.metrics["gaps_used"] = [[round(float(p[0]), 2), round(float(p[1]), 2)]
                                     for p in gap_wps]
        self._wp_index = 0

    def _st_cross(self, dt: float) -> Tuple[float, float]:
        if self.io.now() - self.state_t0 > self.cfg.stage_timeout_s.get("cross_mine", 150.0):
            self._fail("穿越雷区超时")
            return 0.0, 0.0
        camp = self._target_zone("camp")
        if self._zone_reached(camp):
            self._enter_finish()
            return 0.0, 0.0
        pose = self.pose()
        if self._recover_target is not None:
            # 死胡同恢复：先退回到历史足迹中的开阔点，再重新规划
            if float(np.linalg.norm(self._recover_target - pose[:2])) < 0.45:
                self.log("recover_done", target=[round(float(self._recover_target[0]), 2),
                                                 round(float(self._recover_target[1]), 2)])
                self._recover_target = None
                self._path = None
                self._best_goal_dist = math.inf
                self._last_progress_t = self.io.now()
            else:
                return self._drive_to_multi([self._recover_target],
                                            speed_cap=self.cfg.creep_speed_mps * 1.2)
        # 到达当前路段点就推进到下一个（否则会在"已到达的点"上反复规划,
        # 路径退化成 0 长度、纯跟踪的目标速度恰好为 0，车就永久停在原地）
        while self._wp_index < len(self._waypoints) and \
                float(np.linalg.norm(np.asarray(self._waypoints[self._wp_index]) - pose[:2])) \
                < max(0.35, self.cfg.zone_tolerance_m):
            self._wp_index += 1
        wp = self._current_waypoint()
        goals = [camp.center]
        if wp is not None:
            goals.append(np.asarray(wp, dtype=float))
        if len(self._waypoints) >= 2:
            goals.append(np.asarray(self._waypoints[-2], dtype=float))
        v, w = self._drive_to_multi(goals, speed_cap=self.cfg.cross_speed_mps)
        return v, w

    def _current_waypoint(self) -> Optional[np.ndarray]:
        if self._wp_index < len(self._waypoints):
            return self._waypoints[self._wp_index]
        return None

    def _drive_to_multi(self, goals: Sequence[np.ndarray], speed_cap: float
                        ) -> Tuple[float, float]:
        """依次尝试多个目标点，直到规划成功（营地 -> 当前缝隙点 -> 出口点）。

        关键稳定性设计：
        - 只在"新锥桶出现在当前路径附近 / 贴身保护触发 / 超过 3 秒 / 目标改变"时重规划，
          避免路径每 0.5 秒翻一次导致车原地来回摆；
        - 规划本身要几十毫秒，失败时也限速重试，保证控制周期不被拖垮。
        """
        now = self.io.now()
        replan_period = float(self.cfgs["plan"].get("planner.replan_period_s", 0.5))
        min_period = min(replan_period, 0.25)
        hit = self._path_hit_by_new_cones()
        long_period = max(3.0, replan_period * 4.0)
        need_plan = ((self._path is None
                      or hit
                      or self._guard_since_plan
                      or (now - self._last_plan_t) > long_period)
                     and (now - self._last_plan_t) > min_period)
        if need_plan:
            self._guard_since_plan = False
            self.cones.new_since_last_query = 0
            cones = self.planning_cones()
            skip = 0.25 if self.footprint_clearance() < 0.12 else 0.0
            pose0 = np.asarray(self.pose()[:2], dtype=float)
            # 过滤掉"已经踩在脚下"的目标点：规划一条 0.03m 的路径毫无意义，
            # 还会让纯跟踪目标速度变成 0（这正是"原地卡死"的一个来源）
            cand = [np.asarray(g, dtype=float) for g in goals]
            far = [g for g in cand if float(np.linalg.norm(g - pose0)) > 0.45]
            plan_goals = far if far else cand[-1:]
            for g in plan_goals:
                g = np.asarray(g, dtype=float)
                path, clearance = plan_path(self.grid, self.pose()[:2], g, cones,
                                            self.car_len, self.car_wid, margin=0.03,
                                            skip_start_m=skip,
                                            clearances=self.plan_ladder(),
                                            penalties=self.avoid_penalties())
                if path is not None:
                    self._path = path
                    self._path_goal = g
                    self._good_clearance = float(clearance)
                    if clearance < self.grid.hard_clearance - 1e-6:
                        self.log("plan_relaxed", clearance=round(clearance, 3),
                                 goal=[round(float(g[0]), 2), round(float(g[1]), 2)])
                    break
            else:
                self._path = None
                if (now - self._last_fail_log_t) > 1.0:
                    self._last_fail_log_t = now
                    self.log("plan_failed",
                             goal=[round(float(goals[0][0]), 2), round(float(goals[0][1]), 2)],
                             blocked_ratio=round(float(self.grid.blocked.mean()), 3),
                             footprint_clearance=round(self.footprint_clearance(), 3))
            self._last_plan_t = now
        goal = np.asarray(self._path_goal if self._path_goal is not None else goals[0],
                          dtype=float)
        pose = self.pose()
        dist_goal = float(np.linalg.norm(goal - pose[:2]))
        clearance = min_clearance_ahead(self._last_scan, pose, half_angle_deg=25.0)
        if self._path is not None:
            local_clear = self.grid.clearance(pose[:2])
            # 通过"压缩过安全余量"的窄缝时进一步降速（余量越小越慢）
            if self._good_clearance < 0.20:
                speed_cap = min(speed_cap, 0.20)
            if local_clear < 0.30:
                speed_cap = min(speed_cap, self.cfg.creep_speed_mps)
            elif local_clear < 0.52:
                speed_cap = min(speed_cap, 0.38)
            return self.control.compute(pose, self._path, speed_cap=speed_cap,
                                        dt=self._last_dt, distance_to_goal=dist_goal,
                                        clearance_ahead=clearance)
        alpha = G.angle_diff(math.atan2(goal[1] - pose[1], goal[0] - pose[0]), pose[2])
        w = G.clamp(2.0 * alpha, -self.control.p.yaw_rate_limit,
                    self.control.p.yaw_rate_limit)
        v = 0.0
        if abs(alpha) < 0.25 and (not math.isfinite(clearance) or clearance > 0.80):
            v = self.cfg.creep_speed_mps
        return v, w

    # ----------------------------------------------------------------- #
    def _enter_finish(self) -> None:
        self.state = State.FINISH
        self.state_t0 = self.io.now()
        self.log("camp_arrived", zone=str(self.route.get("camp")),
                 pose=[round(v, 3) for v in self.pose()])

    def _st_finish(self, dt: float) -> Tuple[float, float]:
        now = self.io.now()
        self.control.reset()
        if (now - self.state_t0) > 1.5:
            self.metrics["stopped_at_finish"] = True
            self.metrics["t_finish"] = round(now, 3)
            if self.metrics["t_start_line"] is not None:
                self.metrics["run_time_s"] = round(
                    now - float(self.metrics["t_start_line"]), 3)
            self.metrics["min_cone_clearance_m"] = (
                None if not math.isfinite(self._min_clearance)
                else round(self._min_clearance, 3))
            self.log("mission_done", **self.metrics)
            self.state = State.DONE
        return 0.0, 0.0

    # ----------------------------------------------------------------- #
    def _zone_reached(self, zone: Zone) -> bool:
        pose = self.pose()
        d = zone.distance_to_center(pose)
        inside = zone.contains(pose, tol=0.10) or d < self.cfg.center_tolerance_m
        patch_ok = ((zone.color == "blue" and self._floor_streak["blue"] >= 2)
                    or (zone.color == "red" and self._floor_streak["red"] >= 2))
        return bool(inside and (patch_ok or d < self.cfg.center_tolerance_m * 0.8))

    def _drive_to(self, goal: Sequence[float], speed_cap: float,
                  goal_tolerance: float = 0.20,
                  replan: bool = True) -> Tuple[float, float]:
        now = self.io.now()
        pose = self.pose()
        goal = np.asarray(goal, dtype=float)
        replan_period = float(self.cfgs["plan"].get("planner.replan_period_s", 0.4))
        new_goal = (self._path_goal is None
                    or float(np.linalg.norm(goal - self._path_goal)) > 0.15)
        hit = self._path_hit_by_new_cones()
        long_period = max(3.0, replan_period * 4.0)
        need_plan = (self._path is None or new_goal or hit
                     or self._guard_since_plan
                     or (replan and (now - self._last_plan_t) > long_period))
        if (now - self._last_plan_t) <= min(replan_period, 0.25):
            need_plan = False
        if need_plan:
            self._guard_since_plan = False
            self.cones.new_since_last_query = 0
            cones = self.planning_cones()
            skip = 0.25 if self.footprint_clearance() < 0.12 else 0.0
            path, scale = plan_path(
                self.grid, pose[:2], goal, cones, self.car_len, self.car_wid,
                margin=0.03, skip_start_m=skip, clearances=self.plan_ladder(),
                penalties=self.avoid_penalties())
            self._last_plan_t = now
            self._path_goal = goal
            self._path = path
            if path is not None:
                self._good_clearance = float(scale)
            if path is not None and scale < self.grid.hard_clearance - 1e-6:
                self.log("plan_relaxed", clearance=round(scale, 3))
            if path is None:
                self.log("plan_failed", goal=[round(float(goal[0]), 2),
                                              round(float(goal[1]), 2)])
        dist_goal = float(np.linalg.norm(goal - pose[:2]))
        clearance = min_clearance_ahead(self._last_scan, pose, half_angle_deg=25.0)
        # 贴障碍时自动降速（缝隙内慢行，保证不蹭锥桶）
        if self._path is not None:
            local_clear = self.grid.clearance(pose[:2])
            if local_clear < 0.32:
                speed_cap = min(speed_cap, self.cfg.creep_speed_mps)
            elif local_clear < 0.55:
                speed_cap = min(speed_cap, 0.35)
        if self._path is not None:
            v, w = self.control.compute(pose, self._path, speed_cap=speed_cap,
                                        dt=self._last_dt,
                                        distance_to_goal=dist_goal,
                                        clearance_ahead=clearance)
        else:
            alpha = G.angle_diff(math.atan2(goal[1] - pose[1],
                                            goal[0] - pose[0]), pose[2])
            w = G.clamp(2.0 * alpha, -self.control.p.yaw_rate_limit,
                        self.control.p.yaw_rate_limit)
            # 规划失败时绝不盲目前进：先原地对准目标，只有前方足够空旷才慢速直行
            v = 0.0
            if abs(alpha) < 0.25 and (not math.isfinite(clearance) or clearance > 0.80):
                v = self.cfg.creep_speed_mps
        return v, w

    # ----------------------------------------------------------------- #
    def _track_metrics(self, dt: float) -> None:
        pose = self.pose()
        v = self._last_odom.v if self._last_odom else 0.0
        if not self._started_moving and abs(v) > 0.05:
            self._started_moving = True
            self.log("robot_moving")
        if self.metrics["t_start_line"] is None and self._started_moving:
            st = self.field.start
            front_x = pose[0] + (self.car_len / 2.0) * math.cos(pose[2])
            line_x = st.cx + st.length / 2.0
            if front_x > line_x:
                self.metrics["t_start_line"] = round(self.io.now(), 3)
                self.log("start_line_crossed", front_x=round(front_x, 3))

    def summary(self) -> Dict[str, Any]:
        out = dict(self.metrics)
        out["state"] = self.state
        out["events"] = len(self.events)
        total = out.get("run_time_s")
        if total is not None:
            out["effective_time_s"] = round(float(total) + float(out["penalties_s"]), 3)
        return out
