"""现场标定颜色阈值：给一张（或一组）照片，自动给出蓝/红锥桶与色卡的 HSV 区间。

用法（照片可以是手机拍的、也可以是小车相机截的）：

    # 1) 只看建议（不改配置），并把掩码图存下来肉眼确认
    python3 tools/tune_hsv.py --image 现场照片.jpg --debug output/hsv_debug.png

    # 2) 满意后写回 config/vision.json
    python3 tools/tune_hsv.py --image 现场照片.jpg --apply

    # 3) 直接在小车上抓一帧（车上装了 cv2 时可用）
    python3 tools/tune_hsv.py --ros-topic /usb_cam/image_raw --apply

判据：先用"通道占优"粗挑蓝色/红色像素（B 明显大于 R/G、R 明显大于 G/B），
再统计这些像素的 H/S/V 分位数，最后给出带余量的阈值区间并写进配置。
HSV 约定与 OpenCV 一致：H 0~179，S/V 0~255。
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from typing import Dict, List, Optional, Tuple

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from racelib import imageops as I  # noqa: E402
from racelib.config import default_config_dir  # noqa: E402


def load_image(path: str) -> np.ndarray:
    from PIL import Image
    img = Image.open(path).convert("RGB")
    return np.asarray(img)


def grab_from_topic(topic: str, timeout: float = 10.0) -> np.ndarray:
    """从小车相机话题抓一帧（需要 rclpy；在车上运行）。"""
    import time

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CompressedImage, Image

    pkg_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                           "ros2_ws", "src", "racecar_mission")
    if pkg_dir not in sys.path:
        sys.path.insert(0, pkg_dir)
    from racecar_mission.ros_io import image_to_rgb

    rclpy.init()
    node = Node("hsv_tuner")
    box: Dict[str, np.ndarray] = {}
    qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                     history=HistoryPolicy.KEEP_LAST)
    msg_type = CompressedImage if "compressed" in topic else Image
    node.create_subscription(msg_type, topic,
                             lambda m: box.setdefault("img", image_to_rgb(m)), qos)
    deadline = time.time() + timeout
    while time.time() < deadline and "img" not in box:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()
    if "img" not in box:
        raise RuntimeError(f"{topic} 在 {timeout:.0f}s 内没有收到图像")
    return box["img"]


def candidate_masks(img: np.ndarray) -> Dict[str, np.ndarray]:
    """通道占优粗挑：只用来"找样本像素"，不当作最终阈值。"""
    x = img.astype(np.int16)
    r, g, b = x[..., 0], x[..., 1], x[..., 2]
    blue = (b > r + 18) & (b > g + 18) & (b > 60)
    red = (r > g + 35) & (r > b + 35) & (r > 60)
    return {"blue": blue, "red": red}


def suggest_ranges(img: np.ndarray, min_pixels: int = 200) -> Dict[str, object]:
    hsv = I.to_hsv(img)
    out: Dict[str, object] = {}
    for color, mask in candidate_masks(img).items():
        n = int(mask.sum())
        if n < min_pixels:
            out[color] = {"pixels": n, "ranges": None,
                          "note": f"样本太少（{n} 像素），把目标放近一点或换张更清楚的照片"}
            continue
        h = hsv[..., 0][mask]
        s = hsv[..., 1][mask]
        v = hsv[..., 2][mask]
        s_min = int(max(0, np.percentile(s, 2) - 25))
        v_min = int(max(0, np.percentile(v, 2) - 35))
        # 红色会横跨 0/180 边界：按环形处理
        if color == "red":
            hh = np.where(h > 90, h - 180.0, h)      # 把 170~180 映射到 -10~0
            lo, hi = np.percentile(hh, [2, 98])
            lo, hi = lo - 4.0, hi + 4.0
            ranges = []
            if hi > -2.0:                            # 低端 0 一侧
                ranges.append([[0, s_min, v_min], [int(min(179, hi)), 255, 255]])
            if lo < -2.0:                            # 高端 180 一侧
                ranges.append([[int(max(0, 180 + lo)), s_min, v_min], [179, 255, 255]])
            if not ranges:
                ranges = [[[0, 80, 50], [10, 255, 255]], [[170, 80, 50], [179, 255, 255]]]
        else:
            lo, hi = np.percentile(h, [2, 98])
            ranges = [[[int(max(0, lo - 8)), s_min, v_min],
                       [int(min(179, hi + 8)), 255, 255]]]
        out[color] = {
            "pixels": n,
            "ranges": ranges,
            "h_percentiles": [round(float(np.percentile(h, p)), 1) for p in (2, 50, 98)],
            "s_percentiles": [round(float(np.percentile(s, p)), 1) for p in (2, 50, 98)],
            "v_percentiles": [round(float(np.percentile(v, p)), 1) for p in (2, 50, 98)],
        }
    return out


def save_debug(img: np.ndarray, ranges: Dict[str, object], path: str) -> None:
    from PIL import Image
    hsv = I.to_hsv(img)
    vis = img.copy()
    overlay = {"blue": (0, 120, 255), "red": (255, 60, 40)}
    for color, info in ranges.items():
        rs = info.get("ranges")  # type: ignore[union-attr]
        if not rs:
            continue
        m = I.in_ranges(hsv, rs)
        vis[m] = overlay[color]
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    Image.fromarray(vis).save(path)


def apply_to_config(ranges: Dict[str, object], cfg_dir: str) -> str:
    path = os.path.join(cfg_dir, "vision.json")
    with open(path, "r", encoding="utf-8") as f:
        cfg = json.load(f)
    changed = []
    for color, info in ranges.items():
        rs = info.get("ranges")  # type: ignore[union-attr]
        if not rs:
            continue
        cfg.setdefault("hsv", {})[color] = {"ranges": rs}
        changed.append(color)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(cfg, f, ensure_ascii=False, indent=2)
    return f"已写回 {path}（更新：{', '.join(changed) or '无'}）"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="颜色阈值标定")
    ap.add_argument("--image", action="append", default=[],
                    help="现场照片（可重复传多张，统计会合并）")
    ap.add_argument("--ros-topic", default=None,
                    help="在车上直接抓一帧（例如 /usb_cam/image_raw）")
    ap.add_argument("--config-dir", default=None)
    ap.add_argument("--apply", action="store_true", help="把建议阈值写回 vision.json")
    ap.add_argument("--debug", default="output/hsv_debug.png", help="掩码可视化输出路径")
    args = ap.parse_args(argv)

    cfg_dir = args.config_dir or default_config_dir()
    if args.ros_topic and not args.image:
        img = grab_from_topic(args.ros_topic)
        args.image.append("__topic__")
        merged = img
        print(f"已从 {args.ros_topic} 抓取一帧：{img.shape[1]}x{img.shape[0]}")
    elif not args.image:
        print("请用 --image 指定现场照片（可传多张），或在车上用 --ros-topic 直接抓帧。例如：")
        print("  python3 tools/tune_hsv.py --image ~/Pictures/cones.jpg --debug output/hsv_debug.png")
        print("  python3 tools/tune_hsv.py --ros-topic /usb_cam/image_raw --apply")
        return 2
    merged: Optional[np.ndarray] = None
    for path in args.image:
        if path == "__topic__":
            continue
        img = load_image(path)
        merged = img if merged is None else np.concatenate([merged, img], axis=0)
        print(f"读取 {path}：{img.shape[1]}x{img.shape[0]}")
    assert merged is not None
    ranges = suggest_ranges(merged)
    print("\n=== 建议阈值（OpenCV HSV：H 0~179, S/V 0~255）===")
    for color, info in ranges.items():
        if info["ranges"] is None:  # type: ignore[index]
            print(f"{color}: {info['note']}")  # type: ignore[index]
            continue
        print(f"{color}: {info['pixels']} 像素")  # type: ignore[index]
        print(f"   H 分位 {info['h_percentiles']}  S 分位 {info['s_percentiles']}  "
              f"V 分位 {info['v_percentiles']}")
        print(f"   建议区间 {json.dumps(info['ranges'], ensure_ascii=False)}")
    try:
        save_debug(merged, ranges, args.debug)
        print(f"\n掩码可视化已保存：{args.debug}（打开确认蓝色/红色区域是否被准确覆盖）")
    except Exception as exc:  # noqa: BLE001
        print(f"（掩码图生成失败：{exc}）")
    if args.apply:
        print(apply_to_config(ranges, cfg_dir))
    else:
        print("确认无误后加 --apply 写回配置。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
