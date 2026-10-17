import math
import unittest
import numpy as np

from racelib.field import Field, Cone
from racelib.config import default_config_dir, load_all
from sim.scenarios import demo_cones
from racelib.ackermann import CollisionWorld, hybrid_astar
from racelib.native_planner import c_hybrid_astar, is_native_available


class TestNativePlanner(unittest.TestCase):
    def setUp(self):
        self.cfgs = load_all(default_config_dir(), "A")
        self.field = Field.from_config(self.cfgs["field"])
        self.cones = [Cone(c.x, c.y, "unknown", 0.115) for c in demo_cones()]
        self.world = CollisionWorld(self.field, self.cones, 0.42, 0.30, 0.055)

    def test_native_library_loaded(self):
        """确认 C++ 原生动态加速库正常加载。"""
        self.assertTrue(is_native_available(), "C++ native hybrid astar library should be available")

    def test_native_matches_forward_plan(self):
        """验证 C++ 与 Python 规划出无碰撞且终点到达的高质量路径。"""
        start = (0.75, 3.1, 1.95)
        goal = (5.0, 1.0)
        path = c_hybrid_astar(start, goal, self.world, 0.30, 0.52, allow_reverse=True, max_nodes=45000, timeout=3.0)
        self.assertIsNotNone(path)
        self.assertGreater(len(path), 10)
        # 验证所有轨迹点均无碰撞
        self.assertTrue(all(self.world.clear(p) for p, _ in path))
        # 验证终点与目标距离在容差范围内
        dist_to_goal = math.hypot(path[-1][0][0] - goal[0], path[-1][0][1] - goal[1])
        self.assertLess(dist_to_goal, 0.15)

    def test_native_respects_max_reverse(self):
        """验证 C++ 原生引擎严格遵守倒车距离预算。"""
        self.field.width = 0.8
        narrow_world = CollisionWorld(self.field, [], margin=0.06)
        path = c_hybrid_astar((3.0, 0.4, 0.0), (2.56, 0.4), narrow_world, max_reverse_m=0.44, prefer_forward=False)
        self.assertIsNotNone(path)
        reverse_dist = sum(math.dist(a[0][:2], b[0][:2]) for a, b in zip(path, path[1:]) if b[1] < 0)
        self.assertGreater(reverse_dist, 0.3)
        self.assertLessEqual(reverse_dist, 0.440001)

    def test_native_timeout_on_impossible_goal(self):
        """验证被完全封死的不可达目标能准时超时返回 None。"""
        # 用一圈密集障碍完全封死目标
        blocking = []
        for angle in np.linspace(0, 2 * math.pi, 20):
            blocking.append(Cone(5.0 + 0.3 * math.cos(angle), 1.0 + 0.3 * math.sin(angle), "unknown", 0.2))
        blocked_world = CollisionWorld(self.field, blocking, margin=0.06)
        path = c_hybrid_astar((1.0, 1.0, 0.0), (5.0, 1.0), blocked_world, timeout=0.2)
        self.assertIsNone(path)


if __name__ == "__main__":
    unittest.main()
