"""话题自动探测：按消息类型找传感器，减少换车/换驱动时的适配成本。"""

from __future__ import annotations

from typing import Dict, List, Optional, Sequence, Tuple


def list_topics(node) -> Dict[str, List[str]]:
    out: Dict[str, List[str]] = {}
    for name, types in node.get_topic_names_and_types():
        out[name] = list(types)
    return out


def _match(topics: Dict[str, List[str]], type_name: str) -> List[str]:
    return sorted([n for n, ts in topics.items() if type_name in ts])


def _pick(candidates: Sequence[str], prefer_names: Sequence[str],
          keywords: Sequence[str]) -> Optional[str]:
    for want in prefer_names:
        if not want:
            continue
        if want in candidates:
            return want
    for kw in keywords:
        for c in candidates:
            if kw in c.lower():
                return c
    return candidates[0] if candidates else None


def detect_topics(node, prefer: Dict[str, str]) -> Dict[str, str]:
    """返回 {scan,image,image_compressed,imu,odom,cmd_vel} 的实际话题名。"""
    topics = list_topics(node)
    out: Dict[str, str] = {}
    for key in ("image", "image_compressed", "scan", "imu", "odom"):
        p = prefer.get(key, None)
        if p:
            out[key] = p
    if "scan" not in out:
        c = _match(topics, "sensor_msgs/msg/LaserScan")
        out["scan"] = _pick(c, [prefer.get("scan", "")], ["scan", "laser", "lidar"]) or "/scan"
    if "image" not in out and "image_compressed" not in out:
        c_img = _match(topics, "sensor_msgs/msg/Image")
        pick = _pick(c_img, [prefer.get("image", "")], ["image_raw", "image", "color"])
        if pick:
            out["image"] = pick
        else:
            c_cmp = _match(topics, "sensor_msgs/msg/CompressedImage")
            pick = _pick(c_cmp, [prefer.get("image_compressed", "")],
                         ["compressed", "image"])
            if pick:
                out["image_compressed"] = pick
    if "imu" not in out:
        c = _match(topics, "sensor_msgs/msg/Imu")
        pick = _pick(c, [prefer.get("imu", "")], ["imu", "hipnuc"])
        if pick:
            out["imu"] = pick
    if "odom" not in out:
        c = _match(topics, "nav_msgs/msg/Odometry")
        pick = _pick(c, [prefer.get("odom", "")], ["odom", "ekf", "wheel"])
        if pick:
            out["odom"] = pick
    out["cmd_vel"] = prefer.get("cmd_vel", "/crossfire/desired_cmd") or "/crossfire/desired_cmd"
    return out
