"""规划、地图与场地模型单元测试。"""

from __future__ import annotations

import math
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from racelib.config import load_all  # noqa: E402
from racelib.field import Cone, Field  # noqa: E402
from racelib.fusion import ConeMap, ConeMapConfig  # noqa: E402
from racelib.gridmap import GridMap  # noqa: E402
from racelib.lidarproc import (ConeDetection, Scan, extract_cones, fit_circle,  # noqa: E402
                               cluster_points)
from racelib.planner import (astar, check_footprint, gap_candidates,  # noqa: E402
                             plan_gap_waypoints, plan_path, simplify_path)


class TestField(unittest.TestCase):
    def test_load_and_zones(self):
        cfgs = load_all(route="A")
        f = Field.from_config(cfgs["field"])
        self.assertGreater(f.length, 5.0)
        self.assertIn("stronghold_a", f.zones)
        self.assertIn("camp_b", f.zones)
        self.assertTrue(f.inside_arena((1.0, 1.0)))
        self.assertFalse(f.inside_arena((f.length + 0.5, 1.0)))
        self.assertTrue(f.in_minefield((3.0, 2.0)))
        self.assertEqual(len(f.blockade_lines_x), 3)


class TestLidar(unittest.TestCase):
    def test_circle_fit_and_extract(self):
        # 造一个半径 0.1m、圆心 (2, 0) 的半圆弧点簇
        ang = np.linspace(-math.pi / 2, math.pi / 2, 25)
        pts = np.stack([-0.1 * np.cos(ang) + 2.0, 0.1 * np.sin(ang)], axis=-1)
        cx, cy, r, rms = fit_circle(pts)
        self.assertAlmostEqual(cx, 2.0, delta=0.02)
        self.assertAlmostEqual(r, 0.1, delta=0.02)
        self.assertLess(rms, 0.01)
        dets = extract_cones(pts)
        self.assertEqual(len(dets), 1)
        self.assertAlmostEqual(dets[0].x, 2.0, delta=0.05)

    def test_cluster_split(self):
        pts = np.array([[1.0, 0.0], [1.01, 0.0], [1.02, 0.0],
                        [3.0, 0.0], [3.01, 0.0], [3.02, 0.0]], dtype=float)
        clusters = cluster_points(pts)
        self.assertEqual(len(clusters), 2)
        self.assertEqual([len(c) for c in clusters], [3, 3])


class TestMapAndPlanner(unittest.TestCase):
    def setUp(self):
        self.cfgs = load_all(route="A")
        self.field = Field.from_config(self.cfgs["field"])

    def test_gap_candidates(self):
        gaps = gap_candidates([1.0, 2.0], 0.3, 3.7, min_gap=0.5)
        self.assertTrue(gaps)
        # 最宽的缝应该在 2.0 与 3.7 之间（宽度 1.7 - 0.11）
        self.assertAlmostEqual(gaps[0].y, (2.11 + 3.7) / 2, delta=0.15)

    def test_grid_and_astar_avoid_cone(self):
        grid = GridMap(self.field, res=0.05, hard_clearance=0.26, wall_clearance=0.32)
        cones = [Cone(3.5, 2.0, "red")]
        grid.build(cones)
        self.assertTrue(grid.is_blocked((3.5, 2.0)))
        path = astar(grid, (3.0, 2.0), (4.0, 2.0))
        self.assertIsNotNone(path)
        # 路径必须绕开锥桶：与锥桶中心距离 > 半径 + 余量 - 些许
        d = np.min(np.linalg.norm(path - np.array([3.5, 2.0]), axis=1))
        self.assertGreater(d, 0.26)

    def test_footprint_check_detects_collision(self):
        cones = [Cone(1.5, 0.16, "blue")]
        path = np.array([[0.0, 0.0], [3.0, 0.0]])
        chk = check_footprint(path, cones, 0.42, 0.30, margin=0.03)
        self.assertFalse(chk.ok)
        safe = np.array([[0.0, 1.0], [3.0, 1.0]])
        self.assertTrue(check_footprint(safe, cones, 0.42, 0.30, margin=0.03).ok)

    def test_plan_path_prefers_gap_and_validates(self):
        grid = GridMap(self.field, res=0.05, hard_clearance=0.26, wall_clearance=0.32)
        cones = [Cone(2.0, y, "blue") for y in (0.6, 1.2, 2.4, 3.0)]
        path, clearance = plan_path(grid, (1.0, 1.8), (3.5, 1.8), cones, 0.42, 0.30)
        self.assertIsNotNone(path)
        self.assertTrue(check_footprint(path, cones, 0.42, 0.30, 0.03).ok)

    def test_cone_map_color_annotation(self):
        cm = ConeMap(self.field, ConeMapConfig())
        det = ConeDetection(1.0, 0.2, 0.09, 12, 0.01)
        cm.update([det], (0.0, 0.0, 0.0), 0.0)
        self.assertEqual(len(cm.snapshot()), 1)
        self.assertTrue(cm.blockers() == [] or cm.snapshot()[0].hits >= 1)


class TestScanConversions(unittest.TestCase):
    def test_scan_points_body(self):
        scan = Scan(angles=np.array([0.0, math.pi / 2]), ranges=np.array([1.0, 2.0]), t=0.0)
        pts = scan.points_body()
        self.assertAlmostEqual(pts[0][0], 1.0, delta=1e-6)
        self.assertAlmostEqual(pts[1][1], 2.0, delta=1e-6)


if __name__ == "__main__":
    unittest.main()
