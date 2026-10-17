"""ROS2 节点冒烟测试（本机没有 rclpy，用桩件模拟 ROS2 接口）。

目的：在没有小车的环境下，尽量提前暴露"话题探测 / 参数读取 / 发布-订阅 /
标记发布 / 诊断输出"这条链路上的接口错误（属性名写错、调用方式不对等）。

注意：桩件只覆盖本节点实际用到的那部分 API，**不能替代真车联调**。
"""

from __future__ import annotations

import os
import sys
import types
import unittest

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, "ros2_ws", "src", "racecar_mission"))

# --------------------------------------------------------------------------- #
# 桩件：rclpy / 消息类型 / ament_index_python
# --------------------------------------------------------------------------- #


class _Param:
    def __init__(self, value):
        self.value = value


class _Publisher:
    def __init__(self, msg_type, topic):
        self.msg_type = msg_type
        self.topic = topic
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class _Subscription:
    def __init__(self, msg_type, topic, callback, qos=None):
        self.msg_type = msg_type
        self.topic = topic
        self.callback = callback
        self.qos = qos


class _Timer:
    def __init__(self, period, callback):
        self.period = period
        self.callback = callback
        self.cancelled = False

    def cancel(self):
        self.cancelled = True


class _Clock:
    class _Now:
        def to_msg(self):
            return object()

    def now(self):
        return _Clock._Now()


class _Logger:
    def __init__(self):
        self.records = []

    def _log(self, level, msg):
        self.records.append((level, str(msg)))

    def info(self, msg):
        self._log("info", msg)

    def warn(self, msg):
        self._log("warn", msg)

    def error(self, msg):
        self._log("error", msg)


class Node:
    # 类属性：桩件里由测试统一设置"车上可见的话题表"
    topics_table = []

    def __init__(self, name="node"):
        self._name = name
        # 保留测试预先声明的参数（真实节点构造时会再 super().__init__ 一次）
        self._params = getattr(self, "_params", {})
        self._publishers = {}
        self._subscriptions = {}
        self._timers = []
        self._logger = _Logger()

    # 参数
    def declare_parameter(self, name, default=None):
        self._params.setdefault(name, default)

    def get_parameter(self, name):
        if name not in self._params:
            raise KeyError(f"未声明的参数：{name}")
        return _Param(self._params[name])

    def set_parameters(self, values):
        self._params.update(values)

    # 话题
    def create_publisher(self, msg_type, topic, qos=None):
        pub = _Publisher(msg_type, topic)
        self._publishers[topic] = pub
        return pub

    def create_subscription(self, msg_type, topic, callback, qos=None):
        sub = _Subscription(msg_type, topic, callback, qos)
        self._subscriptions[topic] = sub
        return sub

    def create_timer(self, period, callback):
        timer = _Timer(period, callback)
        self._timers.append(timer)
        return timer

    def get_topic_names_and_types(self):
        return list(self.topics_table)

    def get_logger(self):
        return self._logger

    def get_clock(self):
        return _Clock()

    def destroy_node(self):
        pass


def _ns(**kw):
    return types.SimpleNamespace(**kw)


def _vec3(x=0.0, y=0.0, z=0.0):
    return _ns(x=x, y=y, z=z)


def _make_twist():
    return _ns(linear=_vec3(), angular=_vec3())


def _make_odometry():
    return _ns(twist=_ns(twist=_ns(linear=_vec3(), angular=_vec3())), header=_ns())


def _make_imu():
    return _ns(orientation=_ns(x=0.0, y=0.0, z=0.0, w=1.0),
               angular_velocity=_vec3(), linear_acceleration=_vec3(), header=_ns())


def _make_image(**kw):
    return _ns(height=kw.get("height", 480), width=kw.get("width", 640),
               encoding=kw.get("encoding", "rgb8"), data=kw.get("data", b""),
               header=_ns())


def _make_scan(arena=(7.0, 4.0), pose=(0.45, 2.0, 0.0)):
    """生成一帧"真实矩形场地"的雷达数据（传感器在起点朝 +x）。

    之前用"四向都是 5m"的假数据，自检里新增的场地尺寸校验会正确判 FAIL——
    说明那条校验有效；这里改成按几何算出来的射线距离，才是真实场景。
    """
    import math as _m
    n = 720
    ang = [_m.pi * (2.0 * i / n - 1.0) for i in range(n)]
    x, y, yaw = pose
    ranges = []
    for a in ang:
        aw = a + yaw
        dx, dy = _m.cos(aw), _m.sin(aw)
        ts = []
        for axis, lo, hi in ((0, 0.0, arena[0]), (1, 0.0, arena[1])):
            d = dx if axis == 0 else dy
            o = x if axis == 0 else y
            if abs(d) > 1e-9:
                for bound in (lo, hi):
                    t = (bound - o) / d
                    if t > 0:
                        ts.append(t)
        ranges.append(min(ts) if ts else 12.0)
    return _ns(angle_min=-3.141592653589793, angle_increment=2 * 3.141592653589793 / n,
               ranges=ranges, range_max=12.0, header=_ns())


def _make_marker():
    return _ns(header=_ns(stamp=None, frame_id=""), id=0, type=0, action=0,
               pose=_ns(position=_vec3(), orientation=_ns(x=0.0, y=0.0, z=0.0, w=1.0)),
               scale=_vec3(1.0, 1.0, 1.0), color=_ns(r=1.0, g=1.0, b=1.0, a=1.0),
               points=[], text="")


def _make_marker_array():
    return _ns(markers=[])


def _make_string(**kw):
    return _ns(data=kw.get("data", ""))


def install_stubs():
    """把 rclpy / 消息类型 / ament_index_python 装进 sys.modules。"""

    class _QoS:
        def __init__(self, depth=10, reliability=None, history=None):
            self.depth = depth
            self.reliability = reliability
            self.history = history

    qos_mod = types.ModuleType("rclpy.qos")
    qos_mod.QoSProfile = _QoS
    qos_mod.ReliabilityPolicy = types.SimpleNamespace(BEST_EFFORT="best_effort",
                                                      RELIABLE="reliable")
    qos_mod.HistoryPolicy = types.SimpleNamespace(KEEP_LAST="keep_last")
    qos_mod.qos_profile_sensor_data = _QoS()

    node_mod = types.ModuleType("rclpy.node")
    node_mod.Node = Node

    rclpy = types.ModuleType("rclpy")
    rclpy.node = node_mod
    rclpy.qos = qos_mod
    rclpy.init = lambda *a, **k: None
    rclpy.spin = lambda *a, **k: None
    rclpy.shutdown = lambda *a, **k: None

    def _msg_module(pkg, factories):
        mod = types.ModuleType(pkg)
        for name, factory in factories.items():
            cls = type(name, (), {"__init__": lambda self, _f=factory, **kw: None})
            setattr(mod, name, factory)
        return mod

    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msgs.msg = _msg_module("geometry_msgs.msg",
                                    {"Twist": _make_twist, "Point": _vec3})
    nav_msgs = types.ModuleType("nav_msgs")
    nav_msgs.msg = _msg_module("nav_msgs.msg", {"Odometry": _make_odometry})
    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs.msg = _msg_module("sensor_msgs.msg", {
        "LaserScan": _make_scan, "Image": _make_image,
        "CompressedImage": lambda **kw: _ns(data=kw.get("data", b""), header=_ns()),
        "Imu": _make_imu,
    })
    std_msgs = types.ModuleType("std_msgs")
    std_msgs.msg = _msg_module("std_msgs.msg", {"String": _make_string})
    visualization_msgs = types.ModuleType("visualization_msgs")
    def _init_marker(self):
        self.__dict__.update(_make_marker().__dict__)

    marker_cls = type("Marker", (), {
        "CYLINDER": 3, "LINE_STRIP": 4, "ARROW": 0, "TEXT_VIEW_FACING": 9, "ADD": 0,
        "__init__": _init_marker,
    })
    marker_array_cls = type("MarkerArray", (), {
        "__init__": lambda self: self.__dict__.update(_make_marker_array().__dict__)})
    visualization_msgs.msg = types.ModuleType("visualization_msgs.msg")
    visualization_msgs.msg.Marker = marker_cls
    visualization_msgs.msg.MarkerArray = marker_array_cls

    ament = types.ModuleType("ament_index_python")
    ament_pkgs = types.ModuleType("ament_index_python.packages")
    ament_pkgs.get_package_share_directory = lambda name: os.path.join(REPO, "ros2_ws",
                                                                       "src", name)
    ament.packages = ament_pkgs

    for name, mod in (("rclpy", rclpy), ("rclpy.node", node_mod), ("rclpy.qos", qos_mod),
                      ("geometry_msgs", geometry_msgs), ("geometry_msgs.msg", geometry_msgs.msg),
                      ("nav_msgs", nav_msgs), ("nav_msgs.msg", nav_msgs.msg),
                      ("sensor_msgs", sensor_msgs), ("sensor_msgs.msg", sensor_msgs.msg),
                      ("std_msgs", std_msgs), ("std_msgs.msg", std_msgs.msg),
                      ("visualization_msgs", visualization_msgs),
                      ("visualization_msgs.msg", visualization_msgs.msg),
                      ("ament_index_python", ament),
                      ("ament_index_python.packages", ament_pkgs)):
        sys.modules.setdefault(name, mod)


class FakeTime:
    """可控时钟：替换 mission_node / ros_io 里的 time.monotonic。"""

    def __init__(self):
        self.t = 1000.0

    def advance(self, dt: float = 0.0):
        """既当作 time.monotonic() 用（dt 缺省），也用来推进时钟。"""
        self.t += dt
        return self.t


class TestRosNodeSmoke(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        install_stubs()
        from racecar_mission import mission_node as mn
        from racecar_mission import ros_io as rio
        cls.mn = mn
        cls.rio = rio
        cls.clock = FakeTime()
        cls.mn.time.monotonic = cls.clock.advance  # type: ignore[attr-defined]
        from racecar_mission import ros_io
        ros_io.time.monotonic = cls.clock.advance  # type: ignore[attr-defined]

    def _make_node(self, route="A", publish_cmd=True):
        node = self.mn.MissionNode.__new__(self.mn.MissionNode)
        self.mn.Node.__init__(node, "racecar_mission")
        node.declare_parameter("route", route)
        node.declare_parameter("use_card", False)
        node.declare_parameter("config_dir", os.path.join(REPO, "config"))
        node.declare_parameter("frame_id", "map")
        node.declare_parameter("publish_cmd", publish_cmd)
        node.declare_parameter("tts", "none")
        node.declare_parameter("control_rate_hz", 50.0)
        node.declare_parameter("diagnostics_rate_hz", 2.0)
        node.declare_parameter("publish_markers", True)
        for k in ("scan", "image", "image_compressed", "imu", "odom", "cmd_vel"):
            node.declare_parameter(f"topics.{k}", "")
        node.declare_parameter("overrides_json", "")
        # 模拟车上真实的话题表
        self.mn.Node.topics_table = [
            ("/scan", ["sensor_msgs/msg/LaserScan"]),
            ("/usb_cam/image_raw", ["sensor_msgs/msg/Image"]),
            ("/imu/data", ["sensor_msgs/msg/Imu"]),
            ("/odom", ["nav_msgs/msg/Odometry"]),
            ("/tf", ["tf2_msgs/msg/TFMessage"]),
        ]
        self.mn.MissionNode.__init__(node)
        return node

    def _feed_sensors(self, node, duration_s=1.0, step=0.05):
        io = node.io
        n = int(duration_s / step)
        for k in range(n):
            t = self.clock.t
            scan = _make_scan()
            scan.angle_min = -3.141592653589793
            scan.angle_increment = 2 * 3.141592653589793 / 720
            scan.range_max = 12.0
            scan.ranges = [5.0] * 720
            io._on_scan(scan)
            odom = _make_odometry()
            odom.twist = types.SimpleNamespace(
                twist=types.SimpleNamespace(
                    linear=types.SimpleNamespace(x=0.3),
                    angular=types.SimpleNamespace(z=0.0)))
            io._on_odom(odom)
            imu = _make_imu()
            imu.orientation = types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)
            imu.angular_velocity = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
            imu.linear_acceleration = types.SimpleNamespace(x=0.0, y=0.0, z=9.81)
            io._on_imu(imu)
            img = _make_image()
            img.height, img.width, img.encoding = 480, 640, "rgb8"
            img.data = (np.full((480, 640, 3), 140, dtype=np.uint8)).tobytes()
            io._on_image(img)
            self.clock.advance(step)

    # ------------------------------------------------------------------ #
    def test_topic_autodetect(self):
        node = self._make_node()
        self.assertEqual(node.topics["scan"], "/scan")
        self.assertEqual(node.topics["image"], "/usb_cam/image_raw")
        self.assertEqual(node.topics["imu"], "/imu/data")
        self.assertEqual(node.topics["odom"], "/odom")
        self.assertEqual(node.topics["cmd_vel"], "/crossfire/desired_cmd")
        self.assertIn("/scan", node._subscriptions)
        self.assertIn("/crossfire/desired_cmd", node._publishers)
        self.assertIn("/crossfire/mission/state", node._publishers)

    def test_state_machine_advances_and_announces(self):
        node = self._make_node()
        self._feed_sensors(node, duration_s=1.0)
        for _ in range(30):
            node._on_control()
            self.clock.advance(0.02)
        self.assertIn(node.mission.state,
                      ("ANNOUNCE", "MAPPING", "GO_STRONGHOLD", "HOLD", "CROSS"))
        announce = node._publishers["/crossfire/mission/announce"].messages
        self.assertTrue(announce, "应发布 /mission/announce 播报文本")
        self.assertIn("我将前往路线A", announce[0].data)
        # 速度指令已发布且为有限值
        cmds = node._publishers["/crossfire/desired_cmd"].messages
        self.assertTrue(cmds)
        self.assertTrue(np.isfinite(cmds[-1].linear.x))
        self.assertTrue(np.isfinite(cmds[-1].angular.z))

    def test_publish_cmd_false_has_no_control_publisher(self):
        node=self._make_node(publish_cmd=False)
        self._feed_sensors(node,duration_s=1.)
        node._on_control()
        self.assertIsNone(node.io.cmd_pub)
        self.assertNotIn('/cmd_vel',node._publishers)
        self.assertNotIn('/car_cmd_vel',node._publishers)
        self.assertNotIn('/crossfire/desired_cmd',node._publishers)

    def test_diagnostics_and_markers(self):
        node = self._make_node()
        self._feed_sensors(node, duration_s=0.5)
        node._on_control()
        node._on_diagnostics()
        states = node._publishers["/crossfire/mission/state"].messages
        self.assertTrue(states)
        self.assertIn("state", states[-1].data)
        markers = node._publishers["/crossfire/mission/markers"].messages
        self.assertTrue(markers)
        self.assertGreaterEqual(len(markers[-1].markers), 1)

    def test_route_b_announcement(self):
        node = self._make_node(route="B")
        self._feed_sensors(node, duration_s=1.0)
        for _ in range(30):
            node._on_control()
            self.clock.advance(0.02)
        announce = node._publishers["/crossfire/mission/announce"].messages
        self.assertTrue(announce)
        self.assertIn("我将前往路线B", announce[0].data)

    def test_preflight_report(self):
        """上车自检节点：注入合成传感器数据后应逐项通过。"""
        from racecar_mission import preflight as pf_mod
        node = self.mn.Node("racecar_preflight")
        for k in ("scan", "image", "image_compressed", "imu", "odom", "cmd_vel"):
            node.declare_parameter(f"topics.{k}", "")
        self.mn.Node.topics_table = [
            ("/scan", ["sensor_msgs/msg/LaserScan"]),
            ("/usb_cam/image_raw", ["sensor_msgs/msg/Image"]),
            ("/imu/data", ["sensor_msgs/msg/Imu"]),
            ("/odom", ["nav_msgs/msg/Odometry"]),
        ]
        pf = pf_mod.Preflight(node, 1.0, os.path.join(REPO, "config"))
        pf.setup()
        # 造一帧带蓝锥/红锥的图，外加雷达/IMU/里程计
        img = np.full((240, 320, 3), 150, dtype=np.uint8)
        img[60:200, 120:160] = (30, 80, 200)
        img[80:200, 200:230] = (200, 45, 40)
        pf.count.update({"scan": 10, "image": 10, "imu": 100, "odom": 50})
        pf.last["scan"] = _make_scan()
        pf.last["imu"] = _make_imu()
        pf.last["odom"] = _make_odometry()
        pf.last["image"] = _make_image(data=img.tobytes(), height=240, width=320,
                                       encoding="rgb8")
        pf.seconds = 5.0
        code = pf.report()
        self.assertEqual(code, 0, "自检应全部通过")
        self.assertNotIn("/cmd_vel", node._publishers)
        self.assertNotIn("/car_cmd_vel", node._publishers)


if __name__ == "__main__":
    unittest.main()
