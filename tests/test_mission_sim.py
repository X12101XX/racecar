"""端到端：在仿真里跑完整任务，检查赛题流程各项都有落实。"""

from __future__ import annotations

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from racelib.config import load_all  # noqa: E402
from racelib.mission import State  # noqa: E402
from sim.runner import RunConfig, run_mission  # noqa: E402


class TestMissionEndToEnd(unittest.TestCase):
    def _run(self, route: str, seed: int, prior: str = "none", max_time: float = 150.0):
        cfgs = load_all(route=route)
        cfg = RunConfig(route=route, seed=seed, prior=prior, video=False,
                        verbose=False, max_time_s=max_time)
        from sim.scenarios import demo_cones
        return run_mission(cfgs, cfg, cones=demo_cones())

    def test_route_a_seed3_completes(self):
        res = self._run("A", 3)
        s = res.summary
        self.assertEqual(s["final_state"], State.DONE, msg=str(s))
        self.assertTrue(s["reached_camp"])
        self.assertEqual(s["red_cone_hits"], 0)
        self.assertIsNotNone(s["run_time_s"])
        self.assertGreaterEqual(s["hold_actual_s"], 3.0)
        # 赛题要求的播报文本
        announce = [e for e in res.events if e["event"] == "announce"]
        self.assertTrue(announce)
        self.assertIn("我将前往路线A", announce[0]["text"])
        self.assertIn("到达据点A并穿越雷区返回营地A", announce[0]["text"])

    def test_route_b_seed3_completes(self):
        res = self._run("B", 3)
        s = res.summary
        self.assertEqual(s["final_state"], State.DONE, msg=str(s))
        announce = [e for e in res.events if e["event"] == "announce"]
        self.assertIn("我将前往路线B", announce[0]["text"])

    def test_event_sequence(self):
        res = self._run("A", 3)
        names = [e["event"] for e in res.events]
        for must in ("init_done", "announce", "stronghold_arrived", "hold_done",
                     "cross_start", "camp_arrived", "mission_done"):
            self.assertIn(must, names)
        # 顺序检查
        self.assertLess(names.index("announce"), names.index("stronghold_arrived"))
        self.assertLess(names.index("hold_done"), names.index("cross_start"))
        self.assertLess(names.index("cross_start"), names.index("camp_arrived"))

    def test_perception_quality(self):
        res = self._run("A", 3)
        p = res.perception
        self.assertGreaterEqual(p["recall"], 0.8)
        self.assertGreaterEqual(p["precision"], 0.7)


if __name__ == "__main__":
    unittest.main()
