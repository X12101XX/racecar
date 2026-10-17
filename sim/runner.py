"""完整任务仿真：世界真值 -> 模拟传感器 -> 任务状态机 -> 底盘。

The mission never receives ``world.cones`` or ``world.pose``.  Truth is used
only to synthesize sensor samples and, after the run, to score the result.
"""

from __future__ import annotations

import json
import math
import os
import subprocess
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np
from PIL import Image, ImageDraw

from racelib import geometry as G
from racelib.config import Config, load_all
from racelib.field import Cone, Field
from racelib.mission import Mission, MissionConfig, State
from racelib.ackermann import AckermannMission
from racelib.vision import CameraModel, Vision

from .camera import CameraRenderer
from .io_sim import SimIO
from .lidar_view import draw_lidar_view
from .world import SimCone, SimRobotConfig, SimWorld, generate_cones


@dataclass
class RunConfig:
    route: str = "A"
    seed: int = 0
    use_card: bool = False
    prior: str = "none"            # none / blue / all（赛前公布位置时用）
    max_time_s: float = 180.0
    physics_dt: float = 0.01
    control_dt: float = 0.02
    video: bool = True
    video_fps: float = 6.0
    outdir: Optional[str] = None
    save_frames: bool = True
    verbose: bool = True
    camera_quality: float = 1.0     # 1.0 正常；0.7 更暗更糊（鲁棒性测试）
    card_color: Optional[str] = None


@dataclass
class RunResult:
    summary: Dict[str, Any]
    events: List[Dict[str, Any]]
    trajectory: np.ndarray
    contacts: List[Dict[str, Any]]
    perception: Dict[str, Any]
    wall_time_s: float
    outdir: Optional[str] = None
    video_path: Optional[str] = None
    cone_map_dump: Dict[str, Any] = field(default_factory=dict)
    # Estimated pose/map produced by the mission, not simulator truth.
    estimated_trajectory: Optional[np.ndarray] = None
    tracks: List[Any] = field(default_factory=list)

    @property
    def success(self) -> bool:
        s = self.summary
        return (s.get("final_state") == State.DONE
                and s.get("reached_camp")
                and s.get("red_cone_hits", 0) == 0
                and s.get("cone_contacts", 0) == 0
                and s.get("wall_contacts",0) == 0)


def evaluate_perception(mission: Mission, cones: Sequence[SimCone],
                        match_dist: float = 0.28) -> Dict[str, Any]:
    """把最终锥桶地图与真值比对，给出检出率/颜色准确率。"""
    tracks = mission.cones.snapshot()
    used = set()
    tp = 0
    color_ok = 0
    color_total = 0
    for c in cones:
        best, best_d = None, match_dist
        for i, tr in enumerate(tracks):
            if i in used:
                continue
            d = math.hypot(tr.x - c.x, tr.y - c.y)
            if d < best_d:
                best, best_d = i, d
        if best is not None:
            used.add(best)
            tp += 1
            tr = tracks[best]
            if tr.color in ("blue", "red"):
                color_total += 1
                if tr.color == c.color:
                    color_ok += 1
    fp = len(tracks) - len(used)
    return {
        "true_cones": len(cones),
        "tracks": len(tracks),
        "true_positive": tp,
        "false_positive": fp,
        "recall": round(tp / max(1, len(cones)), 3),
        "precision": round(tp / max(1, len(tracks)), 3),
        "color_labelled": color_total,
        "color_correct": color_ok,
        "color_accuracy": round(color_ok / max(1, color_total), 3),
    }


def run_mission(cfgs: Optional[Dict[str, Config]] = None, cfg: Optional[RunConfig] = None,
                cones: Optional[Sequence[SimCone]] = None) -> RunResult:
    cfg = cfg or RunConfig()
    cfgs = cfgs or load_all(route=cfg.route)
    field = Field.from_config(cfgs["field"])
    rng = np.random.default_rng(cfg.seed)
    if cones is None:
        cones = generate_cones(field, rng)
    cones = list(cones)

    robot_cfg = cfgs["robot"]
    sim_robot = SimRobotConfig(
        drive_model=robot_cfg.get("drive.type", "differential"),
        wheelbase=float(robot_cfg.get("geometry.wheel_base_m", .30)),
        max_steer=float(robot_cfg.get("drive.max_steer_rad", .52)),
        length=float(robot_cfg.get("geometry.length_m", 0.42)),
        width=float(robot_cfg.get("geometry.width_m", 0.30)),
        max_speed=float(robot_cfg.get("drive.max_linear_mps", 1.2)),
        max_yaw_rate=float(robot_cfg.get("drive.max_angular_rps", 2.6)),
    )
    world = SimWorld(field, cones, rng, robot=sim_robot,
                     lidar_beams=int(robot_cfg.get("sensors.lidar.n_beams", 720)))

    vision_cfg = cfgs["vision"]
    cam = CameraModel(width=int(vision_cfg.get("camera.width", 640)),
                      height=int(vision_cfg.get("camera.height", 480)),
                      hfov_deg=float(vision_cfg.get("camera.hfov_deg", 90.0)),
                      pitch_deg=float(vision_cfg.get("camera.pitch_deg", -12.0)),
                      height_m=float(vision_cfg.get("camera.height_m", 0.18)))
    mount = robot_cfg.get("sensors.camera.mount", [0.16, 0.0, 0.18])
    quality = float(cfg.camera_quality)
    is_mono = False if cfg.use_card else bool(cfgs["plan"].get("mission.monochrome", False))
    if cfg.use_card:
        cfgs["plan"].data.setdefault("mission", {})["monochrome"] = False
    renderer = CameraRenderer(field, cam, mount_xy=(float(mount[0]), float(mount[1])),
                              height_m=float(mount[2]),
                              brightness=0.85 + 0.3 * quality,
                              noise_sigma=3.0 / max(quality, 0.2),
                              blur=quality < 0.95,
                              monochrome=is_mono,
                              colored_materials=not cfg.use_card)
    # The monochrome mission does not read camera frames. Only synthesize them
    # when vision needs them or when a video is requested, instead of burning
    # CPU on unused 15 Hz images.
    uses_vision = not is_mono
    needs_camera = uses_vision or cfg.video

    if cfg.prior in ("blue", "all"):
        # 把"赛前公布的锥桶位置"写进场地配置，再创建任务（任务内部会重新读配置）
        prior_data = cfgs["field"].data.setdefault("prior", {})
        lines: List[List[float]] = [[] for _ in field.blockade_lines_x]
        for c in cones:
            if c.color != "blue":
                continue
            li = min(range(len(field.blockade_lines_x)),
                     key=lambda i: abs(field.blockade_lines_x[i] - c.x))
            lines[li].append(round(float(c.y), 3))
        prior_data["blue_lines_y"] = [sorted(ys) for ys in lines]
    if cfg.prior == "all":
        cfgs["field"].data.setdefault("prior", {})["red_mines"] = [
            [round(float(c.x), 3), round(float(c.y), 3)] for c in cones if c.color == "red"]

    io = SimIO(t0=0.0)
    mission_cfg = MissionConfig.from_config(cfgs["plan"], route=cfg.route,
                                            use_card=cfg.use_card)
    mission_class = AckermannMission if robot_cfg.get("drive.type") == "ackermann" else Mission
    mission = mission_class(io, cfgs, mission_cfg)
    if cfg.prior != "none" and not mission.cones.tracks:
        seeded = mission.cones.seed_prior(0.0)
        if cfg.verbose:
            print(f"[sim] 先验锥桶已注入地图：{seeded} 个（来自赛前公布的位置）")

    traj: List[Tuple[float, float, float, float]] = []
    estimated_traj: List[Tuple[float, float, float, float]] = []
    frames: List[Image.Image] = []
    t_next_scan = 0.0
    t_next_image = 0.0
    t_next_imu = 0.0
    t_next_odom = 0.0
    t_next_control = 0.0
    t_next_frame = 0.0
    scan_period = 1.0 / float(robot_cfg.get("sensors.lidar.rate_hz", 10.0))
    cam_period = 1.0 / float(robot_cfg.get("sensors.camera.rate_hz", 15.0))
    frame_period = 1.0 / max(cfg.video_fps, 0.1)
    wall0 = time.time()
    t = 0.0
    reached_camp = False
    last_img: Optional[np.ndarray] = None

    progress_t = 0.
    while t < cfg.max_time_s:
        if cfg.verbose and t >= progress_t:
            print(f"[progress] t={t:.1f} state={mission.state} pose={world.pose}", flush=True)
            progress_t += 10.
        # ---- 传感器（按各自频率产生）----
        if t >= t_next_scan:
            io.put_scan(world.lidar_scan(t))
            t_next_scan += scan_period
        if t >= t_next_image:
            # Always draw the camera noise pair at the camera cadence, even
            # when frames are not synthesized. This keeps the physics/sensor
            # RNG stream identical for a given seed whether video/vision is on.
            yaw_noise = float(rng.normal(0, math.radians(0.3)))
            pitch_noise = float(rng.normal(0, math.radians(0.2)))
            if needs_camera:
                card_now = (cfg.card_color or ("blue" if cfg.route == "A" else "red")) if (cfg.use_card and t < 2.5) else None
                # Truth -> physical camera sensor synthesis.  The image is the
                # only thing handed to the mission via SimIO.
                img = renderer.render(world.cones, world.pose,
                                      yaw_noise=yaw_noise, pitch_noise=pitch_noise,
                                      card_color=card_now)
                io.put_image(img, t)
                last_img = img
            t_next_image += cam_period
        if t >= t_next_imu:
            io.put_imu(world.imu_sample(t))
            t_next_imu += 0.01
        if t >= t_next_odom:
            io.put_odom(world.odom_sample(t))
            t_next_odom += 0.02

        # ---- 任务控制（50Hz）----
        if t >= t_next_control:
            v, w = mission.step(cfg.control_dt)
            world.set_cmd(v, w)
            t_next_control += cfg.control_dt
            state_now = mission.state
            if state_now in (State.FINISH, State.DONE):
                reached_camp = True
            if state_now in (State.DONE, State.FAILED):
                break
            # Keep simulator truth and the mission's sensor-derived estimate
            # separate.  trajectory.csv is truth for evaluation; the normal
            # map/video uses only estimated_trajectory.
            traj.append((t, world.x, world.y, world.yaw))
            px, py, pyaw = mission.pose()
            estimated_traj.append((t, px, py, pyaw))

        # ---- 录制画面 ----
        if cfg.video and t >= t_next_frame:
            frames.append(_compose_frame(renderer, cam, world, mission, io, t, cfgs,
                                         last_img))
            t_next_frame += frame_period

        # ---- 物理推进 ----
        world.set_cmd(*io.last_cmd)
        world.step(cfg.physics_dt)
        io.advance(cfg.physics_dt)
        t += cfg.physics_dt

    wall_time = time.time() - wall0
    expected_route = ("A" if cfg.card_color == "blue" else "B") if cfg.use_card and cfg.card_color else cfg.route
    camp = field.zone('camp_a' if expected_route == 'A' else 'camp_b')
    body_in_camp = all(camp.contains(p) for p in G.rect_corners(
        world.x, world.y, world.robot.length, world.robot.width, world.yaw))
    stopped_in_expected_camp = (body_in_camp and abs(world.v) < .025
                                and mission.cfg.route == expected_route)
    summary = mission.summary()
    summary.update({
        "seed": cfg.seed,
        "prior": cfg.prior,
        "use_card": cfg.use_card,
        "final_state": mission.state,
        "reached_camp": reached_camp and stopped_in_expected_camp,
        "expected_route": expected_route,
        "finish_body_in_camp_truth": body_in_camp,
        "finish_speed_mps": world.v,
        "sim_time_s": round(t, 3),
        "wall_time_s": round(wall_time, 3),
        "real_time_factor": round(t / max(wall_time, 1e-3), 2),
        "true_cones": len(cones),
        "true_blue": sum(1 for c in cones if c.color == "blue"),
        "true_red": sum(1 for c in cones if c.color == "red"),
    })
    sim_metrics = world.metrics()
    summary.update({
        "cone_contacts": len(world.contacts),
        "blue_cone_hits": sim_metrics["blue_cone_hits"],
        "red_cone_hits": sim_metrics["red_cone_hits"],
        "wall_contacts": sim_metrics["wall_contacts"],
        "penalty_true_s": sim_metrics["penalty_s"],
    })
    perception = evaluate_perception(mission, cones)
    summary["perception"] = perception
    summary["map_source"] = "mission_sensor_fusion"
    summary["evaluation_truth"] = {
        "map": "evaluation_truth.png",
        "cones": "cones.json",
        "trajectory": "trajectory.csv",
    }
    cone_map_dump = {
        "tracks": [{"x": round(t.x, 3), "y": round(t.y, 3), "r": round(t.radius, 3),
                    "color": t.color, "hits": t.hits, "spread": round(t.spread, 3),
                    "prior": t.from_prior} for t in mission.cones.snapshot()],
        "truth": [{"x": round(c.x, 3), "y": round(c.y, 3), "color": c.color,
                   "line": c.line, "hit": bool(c.hit)} for c in cones],
    }
    summary["cone_map_tracks"] = len(cone_map_dump["tracks"])

    result = RunResult(summary=summary, events=mission.events,
                       trajectory=np.asarray(traj, dtype=float),
                       contacts=sim_metrics["contact_events"], perception=perception,
                       wall_time_s=wall_time,
                       estimated_trajectory=np.asarray(estimated_traj, dtype=float),
                       tracks=mission.cones.snapshot())
    result.cone_map_dump = cone_map_dump

    if cfg.outdir:
        os.makedirs(cfg.outdir, exist_ok=True)
        _save_artifacts(result, cfgs, cones, frames, cfg)
        result.outdir = cfg.outdir
    return result


# --------------------------------------------------------------------------- #
# 可视化与产物保存
# --------------------------------------------------------------------------- #
def _world_to_px(px_per_m: float, origin_px: Tuple[float, float], p) -> Tuple[float, float]:
    """世界坐标 -> 图像像素（y 轴翻转，使场地"上"为 y 大）。"""
    return (origin_px[0] + float(p[0]) * px_per_m,
            origin_px[1] - float(p[1]) * px_per_m)


def draw_map(field: Field, cones: Sequence[SimCone], mission: Optional[Mission] = None,
             trajectory: Optional[np.ndarray] = None, size: Tuple[int, int] = (660, 420),
             path: Optional[np.ndarray] = None, waypoints: Optional[Sequence] = None,
             tracks: Optional[Sequence] = None, pose: Optional[Tuple[float, float, float]] = None,
             title: str = "") -> Image.Image:
    """Draw a field view.

    ``cones`` is simulator truth and is intended only for the separate
    evaluation image.  Normal maps pass an empty sequence and draw ``tracks``
    (the mission's sensor-fused estimate) instead.
    """
    img = Image.new("RGB", size, (28, 30, 34))
    d = ImageDraw.Draw(img)
    margin = 28
    px_per_m = min((size[0] - 2 * margin) / field.length, (size[1] - 2 * margin) / field.width)
    ox = margin + (size[0] - 2 * margin - field.length * px_per_m) / 2.0
    oy = size[1] - margin - (size[1] - 2 * margin - field.width * px_per_m) / 2.0
    origin = (ox, oy)

    def W2P(p):
        return _world_to_px(px_per_m, origin, p)

    # 场地
    corners = [W2P([0, 0]), W2P([field.length, 0]), W2P([field.length, field.width]),
               W2P([0, field.width])]
    d.polygon(corners, fill=(46, 49, 54), outline=(160, 165, 175))
    # 雷区
    mf = [W2P([field.minefield_x[0], 0]), W2P([field.minefield_x[1], 0]),
          W2P([field.minefield_x[1], field.width]), W2P([field.minefield_x[0], field.width])]
    d.polygon(mf, fill=(64, 64, 68))
    for lx in field.blockade_lines_x:
        d.line([W2P([lx, 0]), W2P([lx, field.width])], fill=(110, 110, 118), width=1)
    # 区域
    zone_colors = {"blue": (40, 90, 200), "red": (205, 55, 45)}
    for name, z in list(field.zones.items()) + [("start", field.start)]:
        col = zone_colors.get(z.color, (170, 170, 170))
        c = [W2P(z.corners()[0]), W2P(z.corners()[1]), W2P(z.corners()[2]), W2P(z.corners()[3])]
        d.polygon(c, fill=tuple(int(v * 0.8) for v in col), outline=col)
        tx, ty = W2P(z.center)
        d.text((tx - 16, ty - 5), name.replace("stronghold", "据点").replace("camp", "营地")
               .replace("start", "起点"), fill=(240, 240, 240))
    # 仅评估用真值锥桶（普通 map.png 传入空列表）
    for c in cones:
        x, y = W2P([c.x, c.y])
        r = 0.115 * px_per_m
        color = getattr(c, "color", "unknown")
        col = ((70, 130, 255) if color == "blue" else
               (240, 70, 60) if color == "red" else (150, 158, 165))
        if getattr(c, "hit", False):
            d.ellipse([x - r, y - r, x + r, y + r], outline=(255, 220, 0), width=2)
        else:
            d.ellipse([x - r, y - r, x + r, y + r], fill=col)
    # 检测到的锥桶
    if tracks:
        for tr in tracks:
            x, y = W2P([tr.x, tr.y])
            r = max(3.0, 0.24 * px_per_m)
            col = {"blue": (120, 180, 255), "red": (255, 120, 110)}.get(tr.color, (200, 200, 200))
            d.ellipse([x - r, y - r, x + r, y + r], outline=col)
    # 规划路径
    if path is not None and len(path) > 1:
        d.line([W2P(p) for p in path], fill=(255, 190, 60), width=2)
    if waypoints:
        for wp in waypoints:
            x, y = W2P(wp)
            d.rectangle([x - 3, y - 3, x + 3, y + 3], outline=(120, 255, 140))
    # 轨迹
    if trajectory is not None and len(trajectory) > 1:
        d.line([W2P(p) for p in trajectory[:, 1:3]], fill=(0, 240, 200), width=2)
    # 车辆
    if pose is not None:
        c = [W2P(p) for p in G.rect_corners(pose[0], pose[1], 0.42, 0.30, pose[2])]
        d.polygon(c, outline=(255, 255, 255))
        hx, hy = W2P([pose[0] + 0.3 * math.cos(pose[2]), pose[1] + 0.3 * math.sin(pose[2])])
        d.line([W2P(pose[:2]), (hx, hy)], fill=(255, 255, 0), width=2)
    if title:
        d.text((8, 6), title, fill=(230, 230, 230))
    return img


def _mission_panel(mission: Mission, io: SimIO, t: float,
                   size: Tuple[int, int]) -> Image.Image:
    """Text panel built only from mission state and sensor-derived estimates."""
    width, height = size
    img = Image.new("RGB", size, (12, 20, 28))
    draw = ImageDraw.Draw(img)
    scan = mission.last_scan
    points = 0
    if scan is not None:
        points = int(np.isfinite(scan.ranges).sum())
    stats = mission.cones.stats()
    pose = mission.pose()
    path_len = len(mission._path) if mission._path is not None else 0
    lines = [
        "MISSION / SENSOR ONLY",
        f"t={t:5.2f}s   state={mission.state}",
        f"route={mission.cfg.route}   planner={mission.metrics.get('planner_engine', '?')}",
        f"pose est: x={pose[0]:.2f} y={pose[1]:.2f} yaw={pose[2]:.2f}",
        f"cmd: v={io.last_cmd[0]:.2f} m/s  w={io.last_cmd[1]:.2f} rad/s",
        f"scan: {points} pts",
        f"tracks: blue={stats['blue']} red={stats['red']} unknown={stats['unknown']}",
        f"path: {path_len} pts   waypoints={len(mission._waypoints)}",
    ]
    y = max(18, int(height * 0.09))
    for i, line in enumerate(lines):
        draw.text((12, y + i * max(18, height // 12)), line,
                  fill=(190, 215, 225) if i else (120, 235, 205))
    draw.text((12, height - 18), "truth only in evaluation artifacts",
              fill=(120, 145, 158))
    return img


def _compose_frame(renderer: CameraRenderer, cam: CameraModel, world: SimWorld,
                   mission: Mission, io: SimIO, t: float,
                   cfgs: Dict[str, Config],
                   image: Optional[np.ndarray] = None) -> Image.Image:
    """Compose camera + sensor map + LiDAR + mission status for the video."""
    if image is None:
        image = renderer.render(world.cones, world.pose)
    cam_img = Image.fromarray(image)
    d = ImageDraw.Draw(cam_img)
    # 视觉检测可视化：优先复用任务本帧已算出的结果。重复执行有状态的
    # 视觉管线既浪费 CPU，也会让色卡连续帧确认被视频帧“加速”。
    if not mission.cfg.monochrome:
        res = mission.perception_for(image)
        if res is None:
            try:
                res = mission.vision.process(image, want_card=False)
            except Exception:  # noqa: BLE001 - 可视化失败不影响仿真
                res = None
        if res is not None:
            for obs in res.cones:
                x0, y0, x1, y1 = obs.bbox
                col = (80, 160, 255) if obs.color == "blue" else (255, 90, 80)
                d.rectangle([x0, y0, x1, y1], outline=col, width=2)
                d.text((x0, max(0, y0 - 12)), f"{obs.color} {obs.dist_est:.2f}m", fill=col)
            if res.card_color and res.card_bbox is not None:
                d.rectangle(res.card_bbox, outline=(255, 255, 0), width=3)
                d.text((res.card_bbox[0], res.card_bbox[1] - 14),
                       f"CARD {res.card_color}", fill=(255, 255, 0))
    tracks = mission.cones.snapshot()
    stats = mission.cones.stats()
    pose_est = mission.pose()
    text = (f"t={t:5.2f}s  state={mission.state:<15s} route={mission.cfg.route}  "
            f"cones B{stats['blue']}/R{stats['red']}/?{stats['unknown']}  "
            f"cmd=({io.last_cmd[0]:.2f}, {io.last_cmd[1]:.2f})")
    d.rectangle([0, cam.height - 20, cam.width, cam.height], fill=(0, 0, 0))
    d.text((6, cam.height - 16), text, fill=(0, 255, 120))

    # The top-down panel is the robot's own estimate. Truth is deliberately
    # not drawn here; it belongs to the separate evaluation artifacts.
    mp = draw_map(renderer.field, [], mission,
                  trajectory=np.array([[t, pose_est[0], pose_est[1], pose_est[2]]]),
                  size=(cam.width, cam.height), path=mission._path,
                  waypoints=mission._waypoints, tracks=tracks,
                  pose=pose_est,
                  title=f"route {mission.cfg.route}  state {mission.state}  "
                        f"sensor map {len(tracks)} cones")
    lidar_img = draw_lidar_view(mission.last_scan, pose_est, tracks,
                                size=(cam.width, cam.height),
                                car_length=mission.car_len,
                                car_width=mission.car_wid)
    status = _mission_panel(mission, io, t, (cam.width, cam.height))
    out = Image.new("RGB", (cam.width * 2, cam.height * 2), (20, 20, 20))
    out.paste(cam_img, (0, 0))
    out.paste(mp, (cam.width, 0))
    out.paste(lidar_img, (0, cam.height))
    out.paste(status, (cam.width, cam.height))
    return out


def _save_artifacts(result: RunResult, cfgs: Dict[str, Config],
                    cones: Sequence[SimCone], frames: Sequence[Image.Image],
                    cfg: RunConfig) -> None:
    outdir = cfg.outdir or "."
    os.makedirs(outdir, exist_ok=True)
    field = Field.from_config(cfgs["field"])
    with open(os.path.join(outdir, "summary.json"), "w", encoding="utf-8") as f:
        json.dump(result.summary, f, ensure_ascii=False, indent=2)
    with open(os.path.join(outdir, "events.json"), "w", encoding="utf-8") as f:
        json.dump(result.events, f, ensure_ascii=False, indent=2)
    with open(os.path.join(outdir, "perception.json"), "w", encoding="utf-8") as f:
        json.dump(result.perception, f, ensure_ascii=False, indent=2)
    if result.cone_map_dump:
        with open(os.path.join(outdir, "cones.json"), "w", encoding="utf-8") as f:
            json.dump(result.cone_map_dump, f, ensure_ascii=False, indent=2)
    if len(result.trajectory):
        np.savetxt(os.path.join(outdir, "trajectory.csv"), result.trajectory,
                   delimiter=",", header="t,x,y,yaw", comments="")
    estimated = (result.estimated_trajectory
                 if result.estimated_trajectory is not None
                 and len(result.estimated_trajectory) else result.trajectory)
    if len(estimated):
        np.savetxt(os.path.join(outdir, "trajectory_estimated.csv"), estimated,
                   delimiter=",", header="t,x,y,yaw", comments="")
    # Normal map: sensor-derived tracks/pose/plan only.
    mp = draw_map(field, [],
                  trajectory=estimated,
                  tracks=result.tracks,
                  size=(900, 560),
                  title=(f"route {cfg.route} seed {cfg.seed} sensor map "
                         f"state={result.summary.get('final_state')} "
                         f"time={result.summary.get('run_time_s')}s "
                         f"tracks={len(result.tracks)}"))
    mp.save(os.path.join(outdir, "map.png"))
    # Evaluation-only truth overlay, never used by the controller.
    truth_mp = draw_map(field, cones,
                        trajectory=result.trajectory,
                        tracks=result.tracks,
                        size=(900, 560),
                        title=(f"evaluation truth route {cfg.route} "
                               f"seed {cfg.seed} state={result.summary.get('final_state')} "
                               f"blue_hits={result.summary.get('blue_cone_hits')} "
                               f"red_hits={result.summary.get('red_cone_hits')}"))
    truth_mp.save(os.path.join(outdir, "evaluation_truth.png"))
    if frames and cfg.save_frames:
        fdir = os.path.join(outdir, "frames")
        os.makedirs(fdir, exist_ok=True)
        for i, fr in enumerate(frames):
            fr.save(os.path.join(fdir, f"frame_{i:05d}.png"))
        video = make_video(fdir, os.path.join(outdir, "video.mp4"), cfg.video_fps)
        if video:
            result.video_path = video


def make_video(frames_dir: str, out_path: str, fps: float) -> Optional[str]:
    """用 ffmpeg 把帧序列合成 H.264 视频（失败返回 None）。"""
    try:
        cmd = ["ffmpeg", "-y", "-framerate", str(fps), "-i",
               os.path.join(frames_dir, "frame_%05d.png"),
               "-c:v", "libx264", "-pix_fmt", "yuv420p", "-movflags", "+faststart",
               "-loglevel", "error", out_path]
        temporary = os.path.splitext(out_path)[0] + '.partial.mp4'
        cmd[-1] = temporary
        subprocess.run(cmd, check=True)
        os.replace(temporary, out_path)
        return out_path
    except Exception:  # noqa: BLE001
        return None
