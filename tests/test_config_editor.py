"""Pure checks for the Web editor's local-simulation bridge."""
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
import config_editor as ce  # noqa: E402


class TestConfigEditorSimulationBridge(unittest.TestCase):
    def test_command_is_local_and_whitelisted(self):
        out = ROOT / "output" / "ui_test"
        cmd = ce.build_sim_command("/tmp/scene.json", out, route="A",
                                   planner="native", vision="card",
                                   video=False, seed=7)
        self.assertIn("run_local_sim.py", cmd[2])
        self.assertEqual(cmd[cmd.index("--planner-engine") + 1], "native")
        self.assertIn("--no-video", cmd)
        self.assertEqual(cmd[cmd.index("--card-color") + 1], "blue")
        self.assertNotIn("--ros", cmd)
        self.assertNotIn("ros-sim", cmd)
        self.assertNotIn("run_car", cmd)

    def test_invalid_options_are_rejected(self):
        for kwargs in (dict(route="C"), dict(planner="shell"),
                       dict(vision="yolo"), dict(seed=-1)):
            with self.assertRaises(ValueError):
                ce.build_sim_command("/tmp/scene.json", ROOT / "output" / "ui_test",
                                     **kwargs)

    def test_only_known_artifacts_are_served(self):
        self.assertTrue(ce.artifact_allowed("map.png"))
        self.assertTrue(ce.artifact_allowed("summary.json"))
        self.assertFalse(ce.artifact_allowed("../config/field.json"))
        self.assertFalse(ce.artifact_allowed("run_car.sh"))

    def test_vehicle_package_without_sim_hides_runner(self):
        with tempfile.TemporaryDirectory() as tmp:
            with mock.patch.object(ce, "ROOT", Path(tmp)):
                manager = ce.SimulationManager()
                self.assertFalse(manager.available)
                with self.assertRaises(ValueError):
                    manager.start({}, name="ui")


if __name__ == "__main__":
    unittest.main()
