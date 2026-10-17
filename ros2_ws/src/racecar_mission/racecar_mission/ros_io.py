"""RobotIO 的 ROS2 实现：把小车话题接到任务状态机上。

设计原则：
1. 话题名优先用参数指定；未指定时按消息类型自动探测（车上各驱动的话题名
   可能随发行版变化，自动探测让程序"插上就能跑"）。
2. 传感器订阅统一使用 SensorDataQoS（best effort），兼容可靠/尽力两种发布端。
3. 所有图像编码（bgr8/rgb8/mono8/yuv422/compressed）都在这里归一化成 RGB。
"""

from __future__ import annotations

import json
import math
import subprocess
import time
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

from racelib.io import ImuSample, OdomSample, RobotIO
from racelib.lidarproc import Scan

try:  # pragma: no cover - 只在车上导入
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import CompressedImage, Image, Imu, LaserScan
    from std_msgs.msg import String
except Exception:  # noqa: BLE001 - 本机没有 rclpy 时不影响 racelib 仿真
    rclpy = None  # type: ignore


def _qos_sensor():
    if rclpy is None:
        return None
    return QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST)


def image_to_rgb(msg) -> Optional[np.ndarray]:
    """sensor_msgs/Image 或 CompressedImage -> RGB uint8 (H, W, 3)。"""
    try:
        if msg.__class__.__name__ == "CompressedImage":
            import cv2  # type: ignore
            buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
            bgr = cv2.imdecode(buf, cv2.IMREAD_COLOR)
            if bgr is None:
                return None
            return bgr[:, :, ::-1].copy()
        enc = (getattr(msg, "encoding", "") or "").lower()
        h, w = int(msg.height), int(msg.width)
        buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        channels = 3 if enc in ("bgr8","8uc3","rgb8") else 1 if enc in ("mono8","8uc1") else 2
        stride = int(getattr(msg,'step',0)) or w*channels
        if h<=0 or w<=0 or stride<w*channels or len(buf)<h*stride:return None
        buf = buf[:h*stride].reshape(h,stride)[:,:w*channels].copy()
        if enc in ("bgr8", "8uc3"):
            img=buf.reshape(h,w,3)[:,:,::-1]
        elif enc=="rgb8":
            img=buf.reshape(h,w,3)
        elif enc in ("mono8","8uc1"):
            img=np.repeat(buf.reshape(h,w,1),3,axis=2)
        elif enc in ("yuv422","uyvy","yuyv","yuv422_yuy2"):
            import cv2
            if w%2:return None
            code=cv2.COLOR_YUV2RGB_UYVY if enc in ("yuv422","uyvy") else cv2.COLOR_YUV2RGB_YUY2
            img=cv2.cvtColor(buf.reshape(h,w,2),code)
        else:
            return None
        return np.ascontiguousarray(img)
    except Exception:  # noqa: BLE001
        return None


class RosRobotIO(RobotIO):
    """把 ROS2 话题包装成任务状态机需要的接口。"""

    def __init__(self, node: "Node", topics: Dict[str, str], params: Dict[str, Any]):
        self.node = node
        self.topics = topics
        self.params = params
        self.t0 = time.monotonic()
        self._scan: Optional[Scan] = None
        self._scan_t = 0.0
        self._scan_served = -1.0
        self._image: Optional[np.ndarray] = None
        self._image_t = 0.0
        self._image_served = -1.0
        self._imu = ImuSample(t=0.0)
        self._imu_t = 0.0
        self._imu_served = -1.0
        self._odom = OdomSample(t=0.0)
        self._odom_t = 0.0
        self._odom_served = -1.0
        self.last_cmd: Tuple[float, float] = (0.0, 0.0)
        self.received = {"scan": 0, "image": 0, "imu": 0, "odom": 0}
        self.spoken: List[str] = []
        self.displays: List[str] = []
        self.events: List[Dict[str, Any]] = []

        self.publish_cmd = bool(params.get("publish_cmd", False))
        if self.publish_cmd and topics["cmd_vel"] != "/crossfire/desired_cmd":
            raise ValueError("任务节点只允许向 /crossfire/desired_cmd 输出，由独立网关接入底盘")
        self.cmd_pub = node.create_publisher(Twist, topics["cmd_vel"], 1) if self.publish_cmd else None
        self.state_pub = node.create_publisher(String, "/crossfire/mission/state", 10)
        self.announce_pub = node.create_publisher(String, "/crossfire/mission/announce", 10)
        self.event_pub = node.create_publisher(String, "/crossfire/mission/events", 10)
        qos = _qos_sensor()
        node.create_subscription(LaserScan, topics["scan"], self._on_scan, qos)
        if topics.get("image"):
            node.create_subscription(Image, topics["image"], self._on_image, qos)
        if topics.get("image_compressed"):
            node.create_subscription(CompressedImage, topics["image_compressed"],
                                     self._on_image, qos)
        if topics.get("imu"):
            node.create_subscription(Imu, topics["imu"], self._on_imu, qos)
        if topics.get("odom"):
            node.create_subscription(Odometry, topics["odom"], self._on_odom, qos)
        node.get_logger().info(
            "话题映射: " + json.dumps(topics, ensure_ascii=False))

    def sensors_fresh(self, max_age=.5):
        now = self.now()
        return all(self.received[k] > 0 and now-t < max_age for k,t in
                   (("scan",self._scan_t),("imu",self._imu_t),("odom",self._odom_t)))

    # ---- 时间 ----
    def now(self) -> float:
        return time.monotonic() - self.t0

    # ---- 回调 ----
    def _on_scan(self, msg) -> None:
        n = len(msg.ranges)
        if n == 0:
            return
        angles = (np.arange(n, dtype=np.float64) * msg.angle_increment
                  + msg.angle_min)
        ranges = np.asarray(msg.ranges, dtype=np.float64)
        ranges = np.where(np.isfinite(ranges), ranges,
                          np.where(np.isnan(msg.range_max), 0.0, msg.range_max))
        self._scan = Scan(angles=angles, ranges=ranges, t=self.now())
        self._scan_t = self._scan.t
        self.received["scan"] += 1

    def _on_image(self, msg) -> None:
        img = image_to_rgb(msg)
        if img is None:
            return
        self._image = img
        self._image_t = self.now()
        self.received["image"] += 1

    def _on_imu(self, msg) -> None:
        q = msg.orientation
        yaw = None
        if abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.) < .05 and getattr(msg, "orientation_covariance", [0])[0] != -1:
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny, cosy)
        acc = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y,
                        msg.linear_acceleration.z])
        self._imu = ImuSample(t=self.now(), yaw=yaw, yaw_rate=float(msg.angular_velocity.z),
                              accel=acc)
        self._imu_t = self._imu.t
        self.received["imu"] += 1

    def _on_odom(self, msg) -> None:
        v = float(msg.twist.twist.linear.x)
        w = float(msg.twist.twist.angular.z)
        self._odom = OdomSample(t=self.now(), v=v, w=w)
        self._odom_t = self._odom.t
        self.received["odom"] += 1

    # ---- RobotIO ----
    def scan(self) -> Optional[Scan]:
        if self._scan is not None and self._scan_t > self._scan_served:
            self._scan_served = self._scan_t
            return self._scan
        return None

    def image(self) -> Optional[Tuple[np.ndarray, float]]:
        if self._image is not None and self._image_t > self._image_served:
            self._image_served = self._image_t
            return self._image, self._image_t
        return None

    def imu(self) -> Optional[ImuSample]:
        if self._imu_t > self._imu_served:
            self._imu_served = self._imu_t
            return self._imu
        return None

    def odom(self) -> Optional[OdomSample]:
        if self._odom_t > self._odom_served:
            self._odom_served = self._odom_t
            return self._odom
        return None

    def cmd(self, v: float, w: float) -> None:
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        if not all(math.isfinite(q) for q in (v,w)):
            msg.linear.x = msg.angular.z = 0.0
        if self.cmd_pub is not None:
            self.cmd_pub.publish(msg)
        self.last_cmd = (float(v), float(w))

    def speak(self, text: str) -> None:
        self.spoken.append(text)
        self.announce_pub.publish(String(data=text))
        self.node.get_logger().info(f"【播报】{text}")
        mode = str(self.params.get("tts", "auto"))
        if mode == "none":
            return
        try:
            from . import tts as tts_mod
            how = tts_mod.speak(text)
            if how != "recording":
                self.node.get_logger().info(f"（播报方式：{how}）")
        except Exception as exc:  # noqa: BLE001 - 没有语音模块时只显示
            self.node.get_logger().warn(f"语音播报不可用，已用显示代替：{exc}")

    def display(self, lines: Sequence[str]) -> None:
        text = " | ".join(lines)
        self.displays.append(text)
        self.state_pub.publish(String(data=json.dumps({"display": text},
                                                      ensure_ascii=False)))
        self.node.get_logger().info(f"【显示】{text}")

    def log(self, event: str, **fields: Any) -> None:
        rec = {"t": round(self.now(), 3), "event": event}
        rec.update(fields)
        self.events.append(rec)
        import os
        if os.environ.get("CROSSFIRE_RUN_DIR"):
            try:
                with open(os.path.join(os.environ["CROSSFIRE_RUN_DIR"],"events_live.jsonl"),"a") as f:
                    f.write(json.dumps(rec,ensure_ascii=False,default=str)+"\n")
            except OSError:pass
        try:
            self.event_pub.publish(String(data=json.dumps(rec, ensure_ascii=False,
                                                          default=str)))
        except Exception:  # noqa: BLE001
            pass

    # ---- 诊断 ----
    def health(self) -> Dict[str, Any]:
        return {
            "received": dict(self.received),
            "cmd": list(self.last_cmd),
            "scan_age": round(self.now() - self._scan_t, 2) if self._scan_t else None,
            "image_age": round(self.now() - self._image_t, 2) if self._image_t else None,
        }
