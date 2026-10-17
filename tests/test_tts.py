"""播报链路测试：录音选择、播放器优先级、兜底到显示。"""

from __future__ import annotations

import os
import sys
import types
import unittest
from unittest import mock

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, "ros2_ws", "src", "racecar_mission"))

from racecar_mission import tts  # noqa: E402


class TestTts(unittest.TestCase):
    def test_pick_wav_by_route(self):
        d = os.path.join(REPO, "ros2_ws", "src", "racecar_mission", "assets")
        text_a = "我将前往路线A，到达据点A并穿越雷区返回营地A。"
        text_b = "我将前往路线B，到达据点B并穿越雷区返回营地B。"
        self.assertTrue(tts.pick_wav(text_a, d).endswith("announce_a.wav"))
        self.assertTrue(tts.pick_wav(text_b, d).endswith("announce_b.wav"))
        self.assertIsNone(tts.pick_wav("与路线无关的文本", d))

    def test_assets_exist_and_nonempty(self):
        d = tts.assets_dir()
        for name in ("announce_a.wav", "announce_b.wav"):
            path = os.path.join(d, name)
            self.assertTrue(os.path.exists(path), f"缺少录音文件 {path}")
            self.assertGreater(os.path.getsize(path), 50_000)

    def test_assets_dir_prefers_ament_share_when_installed(self):
        """装到车上后录音在 <prefix>/share/racecar_mission/assets，
        assets_dir() 必须优先用 ament 的包共享目录，否则播报会退化成 TTS 合成。"""
        real = tts.assets_dir()
        self.assertTrue(os.path.isdir(real))
        fake_share = os.path.join(REPO, "ros2_ws", "src", "racecar_mission")
        fake_mod = types.ModuleType("ament_index_python.packages")
        fake_mod.get_package_share_directory = lambda name: fake_share  # type: ignore[attr-defined]
        with mock.patch.dict(sys.modules, {"ament_index_python": types.ModuleType("ament_index_python"),
                                           "ament_index_python.packages": fake_mod}):
            picked = tts.assets_dir()
        self.assertEqual(os.path.normpath(picked),
                         os.path.normpath(os.path.join(fake_share, "assets")))

    def test_missing_assets_falls_back_to_source_tree(self):
        """没有 ament（本地跑仿真）时，仍要能找到源码树里的录音。"""
        with mock.patch.dict(sys.modules, {"ament_index_python": None}):
            d = tts.assets_dir()
        self.assertTrue(os.path.isdir(d), d)

    def test_speak_prefers_recording(self):
        called = {}

        def fake_popen(argv, **kw):
            called["argv"] = argv
            return mock.Mock()

        with mock.patch("shutil.which", lambda name: "/usr/bin/" + name), \
                mock.patch("subprocess.Popen", fake_popen):
            how = tts.speak("我将前往路线A，到达据点A并穿越雷区返回营地A。")
        self.assertEqual(how, "recording")
        self.assertIn("paplay", called["argv"][0])
        self.assertTrue(called["argv"][-1].endswith("announce_a.wav"))

    def test_speak_falls_back_to_tts_then_display(self):
        with mock.patch("shutil.which", lambda name: None):
            self.assertEqual(tts.speak("我将前往路线A"), "display")
        # 录音文件不可用时，用 espeak-ng
        with mock.patch("racecar_mission.tts.pick_wav", lambda *a, **k: None), \
                mock.patch("shutil.which",
                           lambda name: "/usr/bin/espeak-ng" if name == "espeak-ng" else None), \
                mock.patch("subprocess.Popen", lambda *a, **k: mock.Mock()):
            self.assertEqual(tts.speak("我将前往路线B"), "tts")


if __name__ == "__main__":
    unittest.main()
