"""工具脚本的纯逻辑测试（不依赖 ROS2 / 不写真实配置）。"""

from __future__ import annotations

import importlib.util
import json
import os
import subprocess
import sys
import tempfile
import unittest

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)


def load_module(path: str, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(mod)
    return mod


class TestSetupField(unittest.TestCase):
    def setUp(self):
        self.mod = load_module(os.path.join(REPO, "tools", "setup_field.py"), "setup_field")

    def test_generated_layout_is_self_consistent(self):
        for L, W in ((7.0, 4.0), (6.5, 3.5), (8.0, 5.0)):
            cfg = self.mod.build_layout(L, W)
            issues = self.mod.check_layout(cfg)
            self.assertEqual(issues, [], f"{L}x{W} 布局应自洽，实际：{issues}")
            self.assertAlmostEqual(float(cfg["minefield"]["x_max"])
                                   - float(cfg["minefield"]["x_min"]), 4.0, delta=0.01)

    def test_check_catches_bad_layout(self):
        cfg = self.mod.build_layout(7.0, 4.0)
        cfg["arena"]["length_m"] = 9.5                     # 超规格
        cfg["minefield"]["blockade_lines_x"] = [2.0, 4.2, 5.0]   # 间隔不是 1.5
        issues = self.mod.check_layout(cfg)
        self.assertTrue(any("长度" in s for s in issues))
        self.assertTrue(any("间隔" in s for s in issues))

    def test_cli_cones_grouping_writes_prior(self):
        """--cones 应按最近封锁线分组写入 prior（在临时目录里跑，不动真实配置）。"""
        with tempfile.TemporaryDirectory() as d:
            for name in ("field.json", "route_a.json", "route_b.json"):
                src = os.path.join(REPO, "config", name)
                with open(src, "r", encoding="utf-8") as f_src, \
                        open(os.path.join(d, name), "w", encoding="utf-8") as f_dst:
                    f_dst.write(f_src.read())
            out = subprocess.run(
                [sys.executable, os.path.join(REPO, "tools", "setup_field.py"),
                 "--config-dir", d, "--length", "7.0", "--width", "4.0", "--apply",
                 "--figure", os.path.join(d, "fig.png"),
                 "--cones", "2.05,1.2 1.95,2.4 3.5,0.8 5.0,2.2",
                 "--mines", "2.6,2.3"],
                capture_output=True, text=True)
            self.assertEqual(out.returncode, 0, out.stderr)
            with open(os.path.join(d, "field.json"), "r", encoding="utf-8") as f:
                cfg = json.load(f)
            self.assertEqual(cfg["prior"]["blue_lines_y"], [[1.2, 2.4], [0.8], [2.2]])
            self.assertEqual(cfg["prior"]["red_mines"], [[2.6, 2.3]])


class TestControlShaping(unittest.TestCase):
    def test_defaults_are_identity(self):
        from racelib.control import ControlParams, PurePursuit
        pp = PurePursuit(ControlParams())
        self.assertEqual(pp._shape(0.5, 0.8), (0.5, 0.8))
        self.assertEqual(pp._shape(0.0, 0.0), (0.0, 0.0))

    def test_gain_and_deadband(self):
        from racelib.control import ControlParams, PurePursuit
        pp = PurePursuit(ControlParams(cmd_gain_linear=1.2, cmd_gain_angular=0.8,
                                       min_linear_cmd_mps=0.15,
                                       min_angular_cmd_rps=0.25))
        v, w = pp._shape(0.5, 1.0)
        self.assertAlmostEqual(v, 0.6)
        self.assertAlmostEqual(w, 0.8)
        v, w = pp._shape(0.05, 0.1)          # 小指令被抬到死区下限
        self.assertAlmostEqual(v, 0.15)
        self.assertAlmostEqual(w, 0.25)
        v, w = pp._shape(0.0, 0.0)           # 停止指令必须保持 0
        self.assertEqual((v, w), (0.0, 0.0))


class TestCalibrationMath(unittest.TestCase):
    def setUp(self):
        path = os.path.join(REPO, "ros2_ws", "src", "racecar_mission",
                            "racecar_mission", "calibrate_drive.py")
        with open(path, encoding="utf-8") as f:
            src = f.read()
        ns = {}
        exec("import numpy as np\nfrom typing import Dict, List, Optional, Tuple\n"
             + src[src.index("def fit_cmd_to_actual"):src.index("class CalibrateNode")], ns)
        self.summarize = ns["summarize_test"]

    def test_recovers_deadband_and_gain(self):
        cmds = [round(0.05 + 0.05 * i, 2) for i in range(9)]
        for dead, slope in ((0.10, 1.0), (0.10, 0.5), (0.15, 1.2), (0.0, 1.0)):
            actual = [round(max(0.0, (c - dead)) * slope, 4) for c in cmds]
            r = self.summarize(cmds, actual, "m/s")
            self.assertAlmostEqual(r["suggest_gain"], round(1.0 / slope, 2), delta=0.06)
            self.assertAlmostEqual(r["suggest_min_cmd"], dead, delta=0.06)


class TestTuneHsv(unittest.TestCase):
    def setUp(self):
        self.mod = load_module(os.path.join(REPO, "tools", "tune_hsv.py"), "tune_hsv")

    def test_suggestions_cover_synthetic_targets(self):
        from racelib import imageops as I
        img = np.full((240, 320, 3), 150, dtype=np.uint8)
        img[60:200, 120:160] = (30, 80, 200)      # 蓝
        img[80:200, 200:230] = (200, 45, 40)      # 红
        res = self.mod.suggest_ranges(img)
        self.assertIsNotNone(res["blue"]["ranges"])
        self.assertIsNotNone(res["red"]["ranges"])
        hsv = I.to_hsv(img)
        # 建议阈值必须能覆盖住目标像素
        for color, gt in (("blue", (30, 80, 200)), ("red", (200, 45, 40))):
            mask = I.in_ranges(hsv, res[color]["ranges"])
            target = np.all(np.abs(img.astype(int) - np.array(gt)) <= 2, axis=-1)
            self.assertGreater(int((mask & target).sum()), 0.9 * int(target.sum()),
                               f"{color} 建议阈值没覆盖住目标像素")

    def test_too_few_pixels(self):
        img = np.full((120, 160, 3), 150, dtype=np.uint8)
        res = self.mod.suggest_ranges(img)
        self.assertIsNone(res["blue"]["ranges"])


if __name__ == "__main__":
    unittest.main()
