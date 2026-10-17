"""穿越火线任务主节点（在 Atlas 200I DK A2 / ROS2 Humble 小车上运行）。

启动示例：
    # 1) 先按你原来的方式启动底盘、雷达、IMU、相机
    ros2 launch racecar Run_car_one.launch.py

    # 2) 跑路线 A（蓝卡路线，线上赛不抽色卡时直接指定）
    ros2 launch racecar_mission mission.launch.py route:=A

    # 3) 线下抽色卡时开启识别
    ros2 launch racecar_mission mission.launch.py use_card:=true

    # 4) 架空空转测试（不发布速度指令）
    ros2 launch racecar_mission mission.launch.py publish_cmd:=false
"""

from __future__ import annotations

import json
import math
import os
import sys
import time
from typing import Any, Dict, Optional

import numpy as np

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from racelib.config import Config, deep_merge, load_json
from racelib.mission import Mission, MissionConfig, State
from racelib.ackermann import AckermannMission

from .ros_io import RosRobotIO
from .topics import detect_topics


class MissionNode(Node):
    def __init__(self) -> None:
        super().__init__("racecar_mission")
        self.declare_parameter("route", "A")
        self.declare_parameter("use_card", False)
        self.declare_parameter("config_dir", "")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("publish_cmd", False)
        self.declare_parameter("tts", "auto")
        self.declare_parameter("control_rate_hz", 50.0)
        self.declare_parameter("diagnostics_rate_hz", 2.0)
        self.declare_parameter("publish_markers", True)
        self.declare_parameter("monochrome", True)
        self.declare_parameter("ai_backend", "")
        for k in ("scan", "image", "image_compressed", "imu", "odom", "cmd_vel"):
            self.declare_parameter(f"topics.{k}", "")
        self.declare_parameter("overrides_json", "")   # 直接塞 JSON 覆盖任意配置

        self.publish_cmd = bool(self.get_parameter("publish_cmd").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.route_id = str(self.get_parameter("route").value).upper()
        self.use_card = bool(self.get_parameter("use_card").value)
        self.monochrome = bool(self.get_parameter("monochrome").value)
        self.ai_backend = str(self.get_parameter("ai_backend").value)

        cfg_dir = str(self.get_parameter("config_dir").value) or os.path.join(
            get_package_share_directory("racecar_mission"), "config")
        cfgs = self._load_configs(cfg_dir)
        self.config_dir = cfg_dir
        if self.ai_backend:
            if "ai" not in cfgs["vision"].data:
                cfgs["vision"].data["ai"] = {}
            cfgs["vision"].data["ai"]["backend"] = self.ai_backend

        prefer = {k: str(self.get_parameter(f"topics.{k}").value)
                  for k in ("scan", "image", "image_compressed", "imu", "odom", "cmd_vel")}
        topics = detect_topics(self, prefer)
        self.topics = topics
        self.io = RosRobotIO(self, topics, {"tts": self.get_parameter("tts").value, "publish_cmd": self.publish_cmd, "async_planning": True})
        mission_class = AckermannMission if cfgs["robot"].get("drive.type") == "ackermann" else Mission
        mission_cfg = MissionConfig.from_config(cfgs["plan"], route=self.route_id,
                                                use_card=self.use_card)
        if self.use_card:
            mission_cfg.monochrome = False
        else:
            mission_cfg.monochrome = self.monochrome
        self.mission = mission_class(self.io, cfgs, mission_cfg)
        self.cfgs = cfgs
        self._last_step_t = time.monotonic()
        rate = max(5.0, float(self.get_parameter("control_rate_hz").value))
        self.timer = self.create_timer(1.0 / rate, self._on_control)
        diag_rate = max(0.5, float(self.get_parameter("diagnostics_rate_hz").value))
        self.diag_timer = self.create_timer(1.0 / diag_rate, self._on_diagnostics)
        self.marker_pub = None
        if bool(self.get_parameter("publish_markers").value):
            from visualization_msgs.msg import MarkerArray
            self.marker_pub = self.create_publisher(MarkerArray, "/crossfire/mission/markers", 1)
        self._warned: Dict[str, float] = {}
        self.get_logger().info(
            f"任务节点就绪：路线 {self.route_id}，使用色卡={self.use_card}，"
            f"发布速度指令={self.publish_cmd}，配置目录={self.config_dir}")

    # ------------------------------------------------------------------ #
    def _load_configs(self, cfg_dir: str) -> Dict[str, Config]:
        """从 config_dir 读 JSON 配置，并应用 overrides_json（调参用）。"""
        cfgs = {
            "field": Config.load(os.path.join(cfg_dir, "field.json")),
            "robot": Config.load(os.path.join(cfg_dir, "robot.json")),
            "vision": Config.load(os.path.join(cfg_dir, "vision.json")),
            "plan": Config.load(os.path.join(cfg_dir, "plan.json")),
            "route": Config.load(os.path.join(cfg_dir, f"route_{self.route_id.lower()}.json")),
        }
        over = str(self.get_parameter("overrides_json").value or "")
        if over:
            try:
                patch = json.loads(over)
                for key, sub in patch.items():
                    if key in cfgs and isinstance(sub, dict):
                        cfgs[key].data = deep_merge(cfgs[key].data, sub)
                self.get_logger().info("已应用 overrides_json 覆盖配置")
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"overrides_json 解析失败：{exc}")
        return cfgs

    # ------------------------------------------------------------------ #
    def _on_control(self) -> None:
        now = time.monotonic()
        dt = max(1e-3, min(0.2, now - self._last_step_t))
        self._last_step_t = now
        if not self.io.sensors_fresh():
            self.io.cmd(0., 0.)
            return
        try:
            v, w = self.mission.step(dt)
        except Exception as exc:  # noqa: BLE001 - 任何异常都必须让车停下来
            self.get_logger().error(f"任务循环异常：{exc}")
            self.mission._fail("任务循环异常")
            self.io.cmd(0., 0.)
    # ------------------------------------------------------------------ #
    def _on_diagnostics(self) -> None:
        status = self.mission.status()
        status["health"] = self.io.health()
        self.io.state_pub.publish(String(data=json.dumps(status, ensure_ascii=False,
                                                         default=str)))
        # 传感器看门狗：缺数据时给出明确告警（避免"车不动又不知道为什么"）
        need = {"scan": "雷达", "imu": "IMU"}
        for key, label in need.items():
            if self.io.received.get(key, 0) == 0 and \
                    self._warned.get(key, 0.0) < self.io.now() - 5.0:
                self._warned[key] = self.io.now()
                self.get_logger().warn(
                    f"还没有收到{label}数据（当前映射：{self.topics.get(key)}）")
        if self.io.received.get("image", 0) == 0 and \
                not self.topics.get("image") and not self.topics.get("image_compressed"):
            if self._warned.get("image", 0.0) < self.io.now() - 20.0:
                self._warned["image"] = self.io.now()
                self.get_logger().warn(
                    "未发现相机话题：地面色块确认将退化为纯里程计判定（仍可完赛）")
        if self.marker_pub is not None:
            self.marker_pub.publish(self._make_markers())

    # ------------------------------------------------------------------ #
    def _make_markers(self) -> MarkerArray:
        from geometry_msgs.msg import Point
        from visualization_msgs.msg import Marker, MarkerArray
        arr = MarkerArray()
        now = self.get_clock().now().to_msg()
        mid = 0

        def base_marker(kind: int) -> Marker:
            nonlocal mid
            m = Marker()
            m.header.frame_id = self.frame_id
            m.header.stamp = now
            m.id = mid
            mid += 1
            m.type = kind
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            return m

        colors = {"blue": (0.15, 0.35, 0.95), "red": (0.95, 0.2, 0.15),
                  "unknown": (0.8, 0.8, 0.8)}
        for tr in self.mission.cones.snapshot():
            m = base_marker(Marker.CYLINDER)
            m.pose.position.x = float(tr.x)
            m.pose.position.y = float(tr.y)
            m.pose.position.z = 0.2
            m.scale.x = m.scale.y = 0.22
            m.scale.z = 0.4
            c = colors.get(tr.color, colors["unknown"])
            m.color.r, m.color.g, m.color.b, m.color.a = c[0], c[1], c[2], 0.9
            arr.markers.append(m)
        path = self.mission._path
        if path is not None and len(path) > 1:
            m = base_marker(Marker.LINE_STRIP)
            m.scale.x = 0.03
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.75, 0.1, 1.0
            for p in path:
                pt = Point()
                pt.x, pt.y, pt.z = float(p[0]), float(p[1]), 0.05
                m.points.append(pt)
            arr.markers.append(m)
        x, y, yaw = self.mission.pose()
        m = base_marker(Marker.ARROW)
        m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(y), 0.05
        m.pose.orientation.z = math.sin(yaw / 2.0)
        m.pose.orientation.w = math.cos(yaw / 2.0)
        m.scale.x, m.scale.y, m.scale.z = 0.5, 0.12, 0.12
        m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 1.0, 0.5, 1.0
        arr.markers.append(m)
        m = base_marker(Marker.TEXT_VIEW_FACING)
        m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(y), 0.8
        m.scale.z = 0.3
        m.color.r = m.color.g = m.color.b = m.color.a = 1.0
        m.text = f"{self.mission.state} route{self.mission.cfg.route}"
        arr.markers.append(m)
        return arr


def main(argv=None) -> int:
    from rclpy.signals import SignalHandlerOptions
    import signal
    def interrupted(signum, frame):raise KeyboardInterrupt
    rclpy.init(args=argv, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT,interrupted)
    signal.signal(signal.SIGTERM,interrupted)
    node: Optional[MissionNode] = None
    try:
        node = MissionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:      # 无论什么情况都要让车停下来
                for _ in range(5):
                    node.io.cmd(0.0, 0.0)
                    time.sleep(0.02)
                summary = node.mission.summary()
                node.get_logger().info(
                    "任务结果：" + json.dumps(summary, ensure_ascii=False, default=str))
                out_dir = os.environ.get("CROSSFIRE_RUN_DIR", os.path.expanduser("~/.local/state/crossfire"))
                os.makedirs(out_dir, exist_ok=True)
                out = os.path.join(out_dir, "mission_result.json")
                with open(out, "w", encoding="utf-8") as f:
                    json.dump({"summary": summary, "events": node.mission.events},
                              f, ensure_ascii=False, indent=2)
                node.get_logger().info(f"结果与事件已写入 {out}")
            finally:
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
