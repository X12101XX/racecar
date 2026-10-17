"""The video LiDAR panel must render sensor samples and fused tracks."""
from types import SimpleNamespace
import unittest

import numpy as np

from racelib.lidarproc import Scan
from sim.lidar_view import draw_lidar_view


class TestLidarView(unittest.TestCase):
    def test_renders_scan_points_and_track(self):
        scan = Scan(angles=np.array([0.0, np.pi / 2, -np.pi / 2]),
                    ranges=np.array([1.0, 2.0, 3.0]), t=0.0)
        track = SimpleNamespace(x=1.0, y=0.0, radius=.11, color="blue")
        img = draw_lidar_view(scan, (0.0, 0.0, 0.0), [track], size=(320, 240))
        self.assertEqual(img.size, (320, 240))
        colors = img.getcolors(maxcolors=320 * 240)
        self.assertIsNotNone(colors)
        self.assertGreater(len(colors), 8)

    def test_missing_scan_is_safe(self):
        img = draw_lidar_view(None, (1.0, 2.0, 0.3), [], size=(160, 120))
        self.assertEqual(img.size, (160, 120))


if __name__ == "__main__":
    unittest.main()
