"""任务录像节点：把相机画面 + 识别结果 + 播报/状态文字录成 mp4（线上赛提交用）。

赛题要求提交"一镜到底"的视频，清晰展示机器人自主运行状态和各项识别结果的播报。
本节点直接在小车上完成这件事：

    ros2 run racecar_mission record_node --ros-args \
        -p out_path:=$HOME/mission_video.mp4 -p draw_detections:=true

或随任务一起启动：

    ros2 launch racecar_mission mission.launch.py route:=A record:=true

- 画面：相机话题（自动探测，也可用 `-p image_topic:=...` 指定）
- 叠加：任务状态（来自 /mission/state）、播报文本（来自 /mission/announce）、
  时间戳；`draw_detections:=true` 时再用同一套视觉算法画出蓝/红锥桶与色卡框
- 输出：cv2.VideoWriter（mp4v）；若环境不支持编码则退化为按秒存 JPEG 帧
"""

from __future__ import annotations

import json
import os
import time
from typing import Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from racelib.config import Config, load_all
from racelib.vision import Vision

from .ros_io import image_to_rgb
from .topics import detect_topics


class RecordNode(Node):
    def __init__(self) -> None:
        super().__init__("mission_recorder")
        self.declare_parameter("image_topic", "")          # 留空则自动探测
        self.declare_parameter("out_path", os.path.join(os.path.expanduser("~"),
                                                        "mission_video.mp4"))
        self.declare_parameter("fps", 15.0)
        self.declare_parameter("draw_detections", True)
        self.declare_parameter("config_dir", "")
        self.out_path = str(self.get_parameter("out_path").value)
        self.fps = float(self.get_parameter("fps").value)
        self.draw = bool(self.get_parameter("draw_detections").value)
        self.writer = None
        self.frames = 0
        self._jpeg_fallback = True
        self._logged_write_error = False
        self.t0 = time.monotonic()
        self.state_text = ""
        self.announce_text = ""
        self._vision: Optional[Vision] = None
        if self.draw:
            try:
                d = str(self.get_parameter("config_dir").value)
                cfg_dir = d or os.path.join(
                    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config")
                cfgs = load_all(config_dir=cfg_dir)
                self._vision = Vision(cfgs["vision"])
            except Exception as exc:  # noqa: BLE001 - 画框失败不影响录像
                self.get_logger().warn(f"视觉叠加不可用（只录原图）：{exc}")
                self.draw = False

        from sensor_msgs.msg import CompressedImage, Image
        from std_msgs.msg import String

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        prefer = {"image": str(self.get_parameter("image_topic").value or "")}
        topics = detect_topics(self, prefer)
        if topics.get("image"):
            self.create_subscription(Image, topics["image"], self._on_image, qos)
        if topics.get("image_compressed"):
            self.create_subscription(CompressedImage, topics["image_compressed"],
                                     self._on_image, qos)
        self.create_subscription(String, "/crossfire/mission/state", self._on_state, 10)
        self.create_subscription(String, "/crossfire/mission/announce", self._on_announce, 10)
        self.get_logger().info(f"录像已就绪：{topics.get('image') or topics.get('image_compressed')}"
                               f" -> {self.out_path}（{self.fps:.0f} fps）")

    # ------------------------------------------------------------------ #
    def _on_state(self, msg) -> None:
        try:
            data = json.loads(msg.data)
            st = data.get("state", "")
            pose = data.get("pose", [0, 0, 0])
            self.state_text = (f"state={st} pos=({pose[0]:.2f},{pose[1]:.2f}) "
                               f"route={data.get('route','?')}")
        except Exception:  # noqa: BLE001
            self.state_text = str(msg.data)[:80]

    def _on_announce(self, msg) -> None:
        self.announce_text = str(msg.data)

    # ------------------------------------------------------------------ #
    def _on_image(self, msg) -> None:
        img = image_to_rgb(msg)
        if img is None:
            return
        self._ensure_writer(img)
        frame = self._decorate(img)
        try:
            if self.writer is not None:
                self.writer.write(frame[:, :, ::-1])     # RGB -> BGR
            elif self._jpeg_fallback:                    # 退化：每秒存一张 JPEG
                try:
                    import cv2  # type: ignore
                except Exception:  # noqa: BLE001
                    self._jpeg_fallback = False
                else:
                    if self.frames % max(1, int(self.fps)) == 0:
                        cv2.imwrite(
                            os.path.splitext(self.out_path)[0] + f"_{self.frames:06d}.jpg",
                            frame[:, :, ::-1])
            self.frames += 1
        except Exception as exc:  # noqa: BLE001
            if not self._logged_write_error:
                self._logged_write_error = True
                self.get_logger().error(f"写视频失败（只提示一次）：{exc}")

    def _ensure_writer(self, img: np.ndarray) -> None:
        if self.writer is not None:
            return
        h, w = img.shape[:2]
        try:
            import cv2  # type: ignore
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            self.writer = cv2.VideoWriter(self.out_path, fourcc, self.fps, (w, h))
            if not self.writer.isOpened():
                self.writer = None
        except Exception:  # noqa: BLE001
            self.writer = None
        if self.writer is None:
            self.get_logger().warn("cv2.VideoWriter 不可用，改为按秒保存 JPEG 帧")

    def _decorate(self, img: np.ndarray) -> np.ndarray:
        out = img.copy()
        if self.draw and self._vision is not None:
            try:
                res = self._vision.process(out, want_card=False)
                for obs in res.cones:
                    x0, y0, x1, y1 = [int(v) for v in obs.bbox]
                    col = (255, 160, 60) if obs.color == "blue" else (60, 60, 255)
                    out[max(0, y0 - 1):y1 + 2, max(0, x0 - 1):max(0, x0) + 2] = col
                    out[max(0, y0 - 1):y1 + 2, x1:x1 + 2] = col
                    out[max(0, y0 - 1):y0 + 1, x0:x1 + 2] = col
                    out[y1:y1 + 2, x0:x1 + 2] = col
            except Exception:  # noqa: BLE001
                pass
        # 文字条（状态 + 播报）——写在画面底部，便于裁判看清
        bar = out.shape[0] - 46
        out[bar:, :] = (out[bar:, :] * 0.25).astype(np.uint8)
        self._put_text(out, self.state_text, 8, bar + 4)
        if self.announce_text:
            self._put_text(out, self.announce_text, 8, bar + 24)
        return out

    @staticmethod
    def _put_text(img: np.ndarray, text: str, x: int, y: int) -> None:
        """不依赖 cv2.putText（它不支持中文），用 PIL 画字，失败则跳过。"""
        if not text:
            return
        try:
            from PIL import Image, ImageDraw, ImageFont
            pil = Image.fromarray(img)
            d = ImageDraw.Draw(pil)
            font = None
            for path in ("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
                         "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
                         "/usr/share/fonts/noto-cjk/NotoSansCJK-Regular.ttc"):
                if os.path.exists(path):
                    font = ImageFont.truetype(path, 18)
                    break
            d.text((x, y), text, fill=(255, 255, 255), font=font)
            img[:, :] = np.asarray(pil)
        except Exception:  # noqa: BLE001
            pass

    def destroy_node(self):
        if self.writer is not None:
            self.writer.release()
            self.get_logger().info(f"视频已保存：{self.out_path}（{self.frames} 帧）")
        super().destroy_node()


def main(argv=None) -> int:
    rclpy.init(args=argv)
    node = RecordNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
