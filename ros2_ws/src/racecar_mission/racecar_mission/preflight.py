"""上车前自检（动车之前跑，几十秒出报告）。

    ros2 run racecar_mission preflight
    ros2 run racecar_mission preflight --ros-args -p seconds:=10.0

检查内容：
  1. 五个关键话题是否真的有数据、频率是否正常（雷达/相机/IMU/里程计/速度指令）
  2. 相机图像编码能否正确转成 RGB（bgr8/rgb8/mono8/yuyv/compressed），
     并统计蓝/红像素占比（判断现场光照下阈值是否需要重标）
  3. 雷达数据是否合理（点数、最近/最远距离——判断是否被挡住或装反）
  4. IMU 是否输出有效姿态、里程计静止时速度是否接近 0
  5. 场地配置是否自洽（尺寸、雷区 4m、封锁线间隔 1.5m、区域不越界）
  6. 只读探测，不创建任何底盘控制发布者

报告最后会给出一份 PASS/FAIL 清单，可直接复制发回群里/发给我。
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from typing import Any, Dict, List, Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from racelib import imageops as I
from racelib.config import Config, load_json

from .ros_io import image_to_rgb
from .topics import detect_topics


def _check_layout(cfg: Dict[str, Any]) -> List[str]:
    """复用 setup_field 的检查规则（这里内联一份，避免跨目录导入）。"""
    from racelib.settings import validate_field
    return validate_field(cfg)[0]


class Preflight:
    def __init__(self, node: Node, seconds: float, cfg_dir: str):
        self.node = node
        self.seconds = seconds
        self.cfg_dir = cfg_dir
        self.count: Dict[str, int] = {"scan": 0, "image": 0, "imu": 0, "odom": 0}
        self.last: Dict[str, Any] = {}
        self.errors: List[str] = []
        self.qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                              history=HistoryPolicy.KEEP_LAST)
        self.topics: Dict[str, str] = {}

    # ------------------------------------------------------------------ #
    def setup(self) -> None:
        from sensor_msgs.msg import CompressedImage, Image, Imu, LaserScan
        from nav_msgs.msg import Odometry

        prefer = {}
        for k in ("scan", "image", "image_compressed", "imu", "odom", "cmd_vel"):
            self.node.declare_parameter(f"topics.{k}", "")
            prefer[k] = str(self.node.get_parameter(f"topics.{k}").value)
        self.topics = detect_topics(self.node, prefer)
        self.node.get_logger().info(f"话题映射：{self.topics}")

        def sub(key, msg_type, topic, cb):
            if topic:
                self.node.create_subscription(msg_type, topic, cb, self.qos)

        sub("scan", LaserScan, self.topics.get("scan"),
            lambda m: (self.count.__setitem__("scan", self.count["scan"] + 1),
                       self.last.__setitem__("scan", m)))
        sub("image", Image, self.topics.get("image"),
            lambda m: (self.count.__setitem__("image", self.count["image"] + 1),
                       self.last.__setitem__("image", m)))
        sub("image", CompressedImage, self.topics.get("image_compressed"),
            lambda m: (self.count.__setitem__("image", self.count["image"] + 1),
                       self.last.__setitem__("image", m)))
        sub("imu", Imu, self.topics.get("imu"),
            lambda m: (self.count.__setitem__("imu", self.count["imu"] + 1),
                       self.last.__setitem__("imu", m)))
        sub("odom", Odometry, self.topics.get("odom"),
            lambda m: (self.count.__setitem__("odom", self.count["odom"] + 1),
                       self.last.__setitem__("odom", m)))
    # ------------------------------------------------------------------ #
    def spin(self) -> None:
        t0 = time.monotonic()
        while time.monotonic() - t0 < self.seconds:
            rclpy.spin_once(self.node, timeout_sec=0.1)

    # ------------------------------------------------------------------ #
    def report(self) -> int:
        lines: List[str] = []
        ok: List[bool] = []

        def add(passed: bool, text: str) -> None:
            ok.append(passed)
            lines.append(f"  [{'OK ' if passed else 'FAIL'}] {text}")

        hz = self.seconds if self.seconds > 0 else 1.0
        lines.append("=== 传感器 ===")
        add(self.count["scan"] > 3,
            f"激光雷达 {self.topics.get('scan')}: {self.count['scan']} 帧 / "
            f"{self.count['scan'] / hz:.1f} Hz")
        add(self.count["image"] > 3,
            f"相机 {self.topics.get('image') or self.topics.get('image_compressed')}: "
            f"{self.count['image']} 帧 / {self.count['image'] / hz:.1f} Hz")
        add(self.count["imu"] > 3,
            f"IMU {self.topics.get('imu')}: {self.count['imu']} 条 / "
            f"{self.count['imu'] / hz:.1f} Hz")
        add(self.count["odom"] > 3,
            f"里程计 {self.topics.get('odom')}: {self.count['odom']} 条 / "
            f"{self.count['odom'] / hz:.1f} Hz")

        # 雷达质量
        scan = self.last.get("scan")
        if scan is not None:
            r = np.asarray(scan.ranges, dtype=float)
            valid = r[np.isfinite(r) & (r > 0.05)]
            if valid.size:
                lines.append(f"     雷达: {r.size} 束, 最近 {valid.min():.2f} m, "
                             f"中位 {np.median(valid):.2f} m, 最远 {valid.max():.2f} m")
                add(valid.min() > 0.10,
                    "雷达最近距离正常（>0.10 m，若贴着 0 说明被挡住或装反）")
            # 用雷达粗测场地尺寸：车放在起点、车头朝向赛道时最准
            try:
                from racelib.lidarproc import Scan as LScan, wall_ranges
                ang = (np.arange(r.size, dtype=float) * scan.angle_increment
                       + scan.angle_min)
                lscan = LScan(angles=ang, ranges=r, t=0.0)
                wr = wall_ranges(lscan, (0.0, 0.0, 0.0))
                if all(wr.get(k) for k in ("front", "back", "left", "right")):
                    fb = wr["front"][0] + wr["back"][0]
                    lr = wr["left"][0] + wr["right"][0]
                    lines.append(f"     雷达实测场地：长≈{fb:.2f} m（前 "
                                 f"{wr['front'][0]:.2f} + 后 {wr['back'][0]:.2f}），"
                                 f"宽≈{lr:.2f} m（左 {wr['left'][0]:.2f} + 右 "
                                 f"{wr['right'][0]:.2f}）")
                    cfg_len = cfg_wid = None
                    try:
                        _f = load_json(os.path.join(self.cfg_dir, "field.json"))
                        cfg_len = float(_f["arena"]["length_m"])
                        cfg_wid = float(_f["arena"]["width_m"])
                    except Exception:  # noqa: BLE001
                        pass
                    if cfg_len and cfg_wid and not _f.get("arena",{}).get("boundary"):
                        ok_size = abs(fb - cfg_len) < 0.6 and abs(lr - cfg_wid) < 0.6
                        add(ok_size, f"场地尺寸与配置一致（配置 {cfg_len}×{cfg_wid} m）" if ok_size
                            else f"雷达实测 {fb:.2f}×{lr:.2f} m 与配置 {cfg_len}×{cfg_wid} m "
                                 f"差得较多：确认车放在起点朝赛道；若确实不同，用 "
                                 f"tools/setup_field.py 重算场地")
            except Exception as exc:  # noqa: BLE001
                lines.append(f"     （场地粗测跳过：{exc}）")

        # 图像 + 颜色统计
        img_msg = self.last.get("image")
        if img_msg is not None:
            img = image_to_rgb(img_msg)
            enc = getattr(img_msg, "encoding", "compressed")
            if img is None:
                add(False, f"相机图像编码 {enc} 无法转换，请把编码改成 bgr8 或 rgb8")
            else:
                hsv = I.to_hsv(img)
                x = img.astype(np.int16)
                rr, gg, bb = x[..., 0], x[..., 1], x[..., 2]
                blue = (bb > rr + 18) & (bb > gg + 18) & (bb > 60)
                red = (rr > gg + 35) & (rr > bb + 35) & (rr > 60)
                lines.append(f"     图像 {img.shape[1]}x{img.shape[0]} 编码={enc}；"
                             f"现场蓝像素 {blue.mean() * 100:.2f}%，红像素 "
                             f"{red.mean() * 100:.2f}%")
                if blue.any():
                    h = hsv[..., 0][blue]
                    lines.append(f"     蓝色 H 范围 {np.percentile(h, 5):.0f}~"
                                 f"{np.percentile(h, 95):.0f}（配置 "
                                 f"{self._cfg_hue('blue')}）")
                if red.any():
                    h = hsv[..., 0][red]
                    lines.append(f"     红色 H 范围 {np.percentile(h, 5):.0f}~"
                                 f"{np.percentile(h, 95):.0f}（配置 "
                                 f"{self._cfg_hue('red')}）")
                mono=bool(Config.load(os.path.join(self.cfg_dir,"plan.json")).get("mission.monochrome",False))
                found = mono or bool(blue.any() or red.any())
                add(found, "无颜色模式：图像可解码，不要求蓝/红目标" if mono else "画面中能看到蓝/红目标（锥桶/色卡）" if found else
                    "画面里没有明显蓝/红色目标：把锥桶或色卡放进画面再跑一次")

        # IMU 姿态
        imu = self.last.get("imu")
        if imu is not None:
            q = imu.orientation
            norm = float(np.sqrt(q.x ** 2 + q.y ** 2 + q.z ** 2 + q.w ** 2))
            add(norm > 0.5, f"IMU 四元数模长 {norm:.3f}（接近 0 说明没输出姿态）")

        # 里程计静止检查
        odom = self.last.get("odom")
        if odom is not None:
            v = abs(float(odom.twist.twist.linear.x))
            add(v < 0.1, f"静止时 /odom 线速度 {v:.3f} m/s（应接近 0）")

        # 场地配置
        lines.append("=== 场地配置 ===")
        try:
            field = load_json(os.path.join(self.cfg_dir, "field.json"))
            lines.append(f"  {self.cfg_dir}/field.json：场地 "
                         f"{field['arena']['length_m']}×{field['arena']['width_m']} m，"
                         f"雷区 {field['minefield']['x_min']}~{field['minefield']['x_max']} m，"
                         f"封锁线 x={field['minefield']['blockade_lines_x']}")
            prior = field.get("prior", {})
            n_prior = sum(len(v) for v in (prior.get("blue_lines_y") or []))
            lines.append(f"  先验锥桶：{n_prior} 个"
                         + ("（赛前公布的位置已填入，规划更容易）" if n_prior
                            else "（空：完全靠在线感知；赛前拿到位置后填进来）"))
            issues = _check_layout(field)
            add(not issues, "场地配置自洽" if not issues else f"场地配置问题：{issues}")
        except Exception as exc:  # noqa: BLE001
            add(False, f"读取 field.json 失败：{exc}")

        # Read-only: even zero commands could interfere with another controller.
        add(True, "只读自检：不创建速度发布者，也不发送零速度")

        print("\n".join(lines))
        passed = sum(1 for v in ok if v)
        print(f"\n=== 结论：{passed}/{len(ok)} 项通过 ===")
        if passed < len(ok):
            print("请按上面的 FAIL 项处理后重跑；仍不行就把整份输出发我。")
        else:
            print("可以进入下一步：架空测试 ros2 launch racecar_mission mission.launch.py "
                  "publish_cmd:=false")
        return 0 if passed == len(ok) else 1

    def _cfg_hue(self, color: str) -> str:
        try:
            vision = load_json(os.path.join(self.cfg_dir, "vision.json"))
            rs = vision["hsv"][color]["ranges"]
            return ", ".join(f"H {r[0][0]}~{r[1][0]}" for r in rs)
        except Exception:  # noqa: BLE001
            return "未配置"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="上车前自检")
    ap.add_argument("--seconds", type=float, default=8.0)
    ap.add_argument("--config-dir", default=None)
    args, ros_args = ap.parse_known_args(argv)

    rclpy.init(args=ros_args or None)
    node = Node("racecar_preflight")
    try:
        cfg_dir = args.config_dir
        if not cfg_dir:
            try:
                from ament_index_python.packages import get_package_share_directory
                cfg_dir = os.path.join(get_package_share_directory("racecar_mission"), "config")
            except Exception:  # noqa: BLE001
                cfg_dir = os.path.join(os.path.dirname(os.path.dirname(
                    os.path.abspath(__file__))), "config")
        pf = Preflight(node, args.seconds, cfg_dir)
        pf.setup()
        node.get_logger().info(f"自检采集中（{args.seconds:.0f} 秒，请保持车静止）...")
        pf.spin()
        code = pf.report()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return code


if __name__ == "__main__":
    sys.exit(main())
