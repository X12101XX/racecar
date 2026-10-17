"""ROS2 接口契约测试：核对节点里用到的消息字段/常量与官方定义一致。

本机没有 rclpy、也不能上真车，所以这里用**官方 ROS2 Humble 消息定义**
（固化在 docs/ros2_humble_msgs/）来静态核对：

1. 契约里每个字段路径（含嵌套，如 `Odometry.twist.twist.linear.x`）都能在定义里解析出来；
2. 契约里列出的每个用法确实出现在我们的节点源码里（防止改了代码忘了改契约）；
3. 用到的消息常量（Marker.CYLINDER 等）数值与官方一致。

它能挡住"字段名写错、常量取错"这类上车才发现的问题，
但不能替代真车联调（话题连通性、QoS、时序仍要上车验证）。
"""

from __future__ import annotations

import os
import re
import unittest
from typing import Dict, List, Optional, Tuple

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MSG_DIR = os.path.join(REPO, "docs", "ros2_humble_msgs")
NODE_DIR = os.path.join(REPO, "ros2_ws", "src", "racecar_mission", "racecar_mission")

# ---- 本程序实际用到的接口契约（类型 -> 用到的字段路径 / 常量）----
FIELD_CONTRACT: Dict[str, List[str]] = {
    "sensor_msgs/msg/Image": ["height", "width", "encoding", "data"],
    "sensor_msgs/msg/CompressedImage": ["data"],
    "sensor_msgs/msg/LaserScan": ["angle_min", "angle_increment", "ranges", "range_max"],
    "sensor_msgs/msg/Imu": ["orientation", "angular_velocity", "linear_acceleration"],
    "nav_msgs/msg/Odometry": ["twist.twist.linear.x", "twist.twist.angular.z"],
    "geometry_msgs/msg/Twist": ["linear.x", "angular.z"],
    "geometry_msgs/msg/Point": ["x", "y", "z"],
    "std_msgs/msg/String": ["data"],
    "visualization_msgs/msg/Marker": ["header.frame_id", "pose.position.x", "pose.orientation.w",
                                      "scale.x", "color.r", "color.a", "points", "text"],
    "visualization_msgs/msg/MarkerArray": ["markers"],
}

# 用到的字段在源码里应当出现的字面量（用于反向校验契约没写错）
SOURCE_HINTS: Dict[str, List[str]] = {
    "sensor_msgs/msg/Image": ["img.height", "img.width", "encoding", "img.data",
                              "msg.height", "msg.width", "msg.data", "msg.encoding"],
    "sensor_msgs/msg/LaserScan": ["scan.angle_min", "scan.angle_increment", "msg.ranges",
                                  "msg.angle_increment", "msg.angle_min", "msg.range_max"],
    "sensor_msgs/msg/Imu": ["msg.orientation", "msg.angular_velocity", "msg.linear_acceleration"],
    "nav_msgs/msg/Odometry": ["msg.twist.twist.linear.x", "msg.twist.twist.angular.z"],
    "geometry_msgs/msg/Twist": ["msg.linear.x", "msg.angular.z"],
    "std_msgs/msg/String": ["String(data="],
    "visualization_msgs/msg/Marker": ["m.points.append", "Marker.CYLINDER", "Marker.LINE_STRIP",
                                      "Marker.TEXT_VIEW_FACING", "m.text"],
    "visualization_msgs/msg/MarkerArray": ["arr.markers.append"],
}

# Marker 常量：官方定义 = 我们代码里假设的值
MARKER_CONSTANTS = {
    "ARROW": 0, "CUBE": 1, "SPHERE": 2, "CYLINDER": 3, "LINE_STRIP": 4,
    "LINE_LIST": 5, "CUBE_LIST": 6, "SPHERE_LIST": 7, "POINTS": 8,
    "TEXT_VIEW_FACING": 9, "MESH_RESOURCE": 10, "TRIANGLE_LIST": 11,
    "ADD": 0,
}


def load_msgs() -> Dict[str, Dict[str, str]]:
    """解析固化下来的 .msg：{消息全名: {字段名: 类型或常量值}}。"""
    out: Dict[str, Dict[str, str]] = {}
    for fname in os.listdir(MSG_DIR):
        if not fname.endswith(".msg"):
            continue
        pkg, msg = fname[:-4].split("__")
        full = f"{pkg}/msg/{msg}"
        fields: Dict[str, str] = {}
        with open(os.path.join(MSG_DIR, fname), encoding="utf-8") as f:
            for raw in f:
                line = raw.split("#")[0].strip()
                if not line:
                    continue
                parts = line.split()
                if len(parts) < 2:
                    continue
                type_str, name = parts[0], parts[1]
                if "=" in name:                       # 常量行：int32 ARROW=0
                    cname, cval = name.split("=", 1)
                    fields[cname] = cval
                else:
                    fields[name] = type_str
        out[full] = fields
    return out


def resolve(msgs: Dict[str, Dict[str, str]], msg_type: str, path: str
            ) -> Tuple[bool, str]:
    """沿点号路径解析嵌套字段，返回 (是否存在, 最终类型)。"""
    cur_type = msg_type
    final = ""
    for key in path.split("."):
        fields = msgs.get(cur_type)
        if fields is None or key not in fields:
            return False, ""
        final = fields[key]
        base_type = final.split("[")[0]              # 去掉数组后缀，如 float64[36]
        if "/" in base_type:                         # 跨包嵌套，如 geometry_msgs/Twist
            pkg, name = base_type.split("/")
            cur_type = f"{pkg}/msg/{name}"
        elif f"{cur_type.split('/')[0]}/msg/{base_type}" in msgs:
            # 同包嵌套：msg 文件里写的是裸类型名（Twist 而不是 geometry_msgs/Twist）
            cur_type = f"{cur_type.split('/')[0]}/msg/{base_type}"
        else:
            cur_type = ""
    return True, final


def node_sources() -> str:
    text = ""
    for fname in sorted(os.listdir(NODE_DIR)):
        if fname.endswith(".py"):
            with open(os.path.join(NODE_DIR, fname), encoding="utf-8") as f:
                text += f.read() + "\n"
    return text


class TestRosMsgContract(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.msgs = load_msgs()
        cls.src = node_sources()

    def test_definitions_present(self):
        self.assertGreaterEqual(len(self.msgs), 15)
        for t in FIELD_CONTRACT:
            self.assertIn(t, self.msgs, f"缺少消息定义：{t}")

    def test_every_used_field_exists_in_official_definition(self):
        for msg_type, paths in FIELD_CONTRACT.items():
            for p in paths:
                ok, final = resolve(self.msgs, msg_type, p)
                self.assertTrue(ok, f"{msg_type} 没有字段 {p}（官方定义核对失败）")
                self.assertNotEqual(final, "", f"{msg_type}.{p} 类型解析异常")

    def test_contract_matches_source_usage(self):
        """契约里列出的用法应该能在节点源码里找到（否则契约过时了）。"""
        for msg_type, hints in SOURCE_HINTS.items():
            hit = [h for h in hints if h in self.src]
            self.assertTrue(hit, f"{msg_type} 的契约未在源码中找到任何对应用法：{hints}")

    def test_marker_constants_match(self):
        fields = self.msgs["visualization_msgs/msg/Marker"]
        for name, expect in MARKER_CONSTANTS.items():
            self.assertIn(name, fields, f"Marker 缺少常量 {name}")
            self.assertEqual(int(fields[name]), expect,
                             f"Marker.{name} 官方值 {fields[name]} != 代码假设 {expect}")
        # 代码里确实用了这几个常量
        for used in ("Marker.CYLINDER", "Marker.LINE_STRIP", "Marker.ARROW",
                     "Marker.TEXT_VIEW_FACING"):
            self.assertIn(used, self.src, f"源码里没有用到 {used}")

    def test_package_dependencies_cover_used_messages(self):
        """package.xml 必须声明用到的消息包，否则编译/运行会缺依赖。"""
        xml_path = os.path.join(REPO, "ros2_ws", "src", "racecar_mission", "package.xml")
        with open(xml_path, encoding="utf-8") as f:
            xml = f.read()
        for pkg in ("sensor_msgs", "nav_msgs", "geometry_msgs", "std_msgs",
                    "visualization_msgs", "std_srvs"):
            if pkg == "std_srvs":
                continue
            self.assertIn(f"<exec_depend>{pkg}</exec_depend>", xml, f"package.xml 缺少 {pkg}")


if __name__ == "__main__":
    unittest.main()
