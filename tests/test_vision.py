"""视觉与图像基础操作单元测试（不依赖 cv2）。"""

from __future__ import annotations

import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from racelib import imageops as I  # noqa: E402
from racelib.config import load_all  # noqa: E402
from racelib.vision import CameraModel, Vision  # noqa: E402


def make_image(h=480, w=640, fill=(140, 140, 145)):
    img = np.zeros((h, w, 3), dtype=np.uint8)
    img[:, :] = fill
    return img


class TestImageOps(unittest.TestCase):
    def test_hsv_known_colors(self):
        px = np.array([[[255, 0, 0], [0, 255, 0], [0, 0, 255],
                        [255, 255, 255], [0, 0, 0]]], dtype=np.uint8)
        hsv = I.to_hsv(px)
        self.assertLess(hsv[0, 0, 0], 1.0)              # 红 H≈0
        self.assertAlmostEqual(hsv[0, 1, 0], 60.0, delta=1.0)   # 绿 H≈60
        self.assertAlmostEqual(hsv[0, 2, 0], 120.0, delta=1.0)  # 蓝 H≈120
        self.assertAlmostEqual(hsv[0, 4, 2], 0.0, delta=1.0)    # 黑 V=0

    def test_in_ranges_and_components(self):
        cfg = load_all()["vision"]
        vis = Vision(cfg)
        img = make_image()
        img[200:260, 100:130] = (30, 80, 200)      # 蓝块
        img[200:260, 300:330] = (210, 40, 40)      # 红块
        hsv = I.to_hsv(img)
        blue = I.in_ranges(hsv, vis.ranges["blue"])
        red = I.in_ranges(hsv, vis.ranges["red"])
        self.assertGreater(int(blue.sum()), 1000)
        self.assertGreater(int(red.sum()), 1000)
        labels, stats = I.connected_components(blue)
        self.assertEqual(len(stats), 1)
        self.assertAlmostEqual(stats[0]["area"], float(blue.sum()), delta=2.0)


class TestVision(unittest.TestCase):
    def setUp(self):
        self.cfg = load_all()["vision"]
        self.vis = Vision(self.cfg)

    def test_card_blue_and_red(self):
        for color, rgb in (("blue", (20, 60, 200)), ("red", (210, 40, 35))):
            img = make_image()
            img[120:290, 200:320] = rgb
            res = self.vis.process(img, want_card=True)
            self.assertEqual(res.card_color, color)

    def test_cone_detection_and_bearing(self):
        img = make_image()
        img[200:300, 300:340] = (230, 40, 30)      # 正前方红锥
        res = self.vis.process(img, want_card=False)
        self.assertTrue(any(c.color == "red" for c in res.cones))
        cone = [c for c in res.cones if c.color == "red"][0]
        self.assertLess(abs(cone.bearing), 0.05)   # 正前方方位角≈0
        self.assertGreater(cone.dist_est, 0.1)

    def test_card_to_route(self):
        self.assertEqual(self.vis.card_to_route("blue"), "A")
        self.assertEqual(self.vis.card_to_route("red"), "B")
        self.assertIsNone(self.vis.card_to_route(None))

    def test_ground_distance_monotonic(self):
        cam = self.vis.cam
        rows = np.arange(200, 480, 20)
        dists = [cam.ground_distance(float(r)) for r in rows]
        for a, b in zip(dists, dists[1:]):
            self.assertGreater(a, b)               # 行号越大 -> 距离越近
        self.assertAlmostEqual(cam.cx, cam.width / 2.0)


if __name__ == "__main__":
    unittest.main()
