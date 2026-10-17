"""Guards the simulator/mission boundary: no world truth in the controller."""
import ast
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from racelib.config import load_all  # noqa: E402
from racelib.ackermann import AckermannMission  # noqa: E402
from sim.io_sim import SimIO  # noqa: E402


class TestSensorBoundary(unittest.TestCase):
    def test_controller_does_not_import_simulator_truth(self):
        for name in ("racelib/mission.py", "racelib/ackermann.py",
                     "racelib/fusion.py", "racelib/localization.py"):
            tree = ast.parse((ROOT / name).read_text(encoding="utf-8"))
            imports = []
            for node in ast.walk(tree):
                if isinstance(node, ast.Import):
                    imports.extend(alias.name for alias in node.names)
                elif isinstance(node, ast.ImportFrom) and node.module:
                    imports.append(node.module)
            self.assertFalse(
                any(module == "sim" or module.startswith("sim.")
                    for module in imports),
                f"{name} must not import simulator/world truth",
            )

    def test_default_mission_starts_with_empty_obstacle_map(self):
        mission = AckermannMission(SimIO(), load_all(route="A"))
        self.assertEqual(mission.cones.snapshot(), [])

    def test_run_mission_does_not_pass_world_to_mission(self):
        tree = ast.parse((ROOT / "sim/runner.py").read_text(encoding="utf-8"))
        calls = [node for node in ast.walk(tree)
                 if isinstance(node, ast.Call)
                 and isinstance(node.func, ast.Name)
                 and node.func.id == "mission_class"]
        self.assertTrue(calls)
        for call in calls:
            names = [arg.id for arg in call.args if isinstance(arg, ast.Name)]
            self.assertNotIn("world", names)


if __name__ == "__main__":
    unittest.main()
