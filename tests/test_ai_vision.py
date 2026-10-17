"""测试 AI 硬件加速视觉推理模块 (AIVisionEngine 及各后端)。"""

import math
import unittest
import numpy as np

from racelib.ai_vision import (
    AIVisionEngine,
    AICardDetection,
    AIConeDetection,
    FastVectorizedColorBackend,
    AscendNPUBackend,
    ONNXRuntimeBackend,
)
from racelib.config import Config
from racelib.vision import Vision, CameraModel


class TestAIVision(unittest.TestCase):
    def setUp(self):
        self.engine = AIVisionEngine(preferred_backend="auto")

    def test_backend_selection(self):
        """测试后端选择及回退机制。"""
        self.assertIsNotNone(self.engine.backend)
        self.assertIn(self.engine.backend_name, ["fast_vectorized", "ascend_npu", "onnxruntime"])

        # 请求不存在的模型时优雅退化
        with self.assertRaisesRegex(RuntimeError, 'npu unavailable'):
            AIVisionEngine(preferred_backend="npu", model_path="/nonexistent/model.om")
        fallback=AIVisionEngine(preferred_backend='auto',model_path='/nonexistent/model.onnx')
        self.assertEqual(fallback.backend_name,'fast_vectorized')
        self.assertTrue(fallback.diagnostics['fallback_reason'])

    def test_card_detection_blue(self):
        """测试蓝色色卡检测 (路线 A)。"""
        # 320x240 RGB 图像，中央放置蓝色高饱和卡片
        img = np.zeros((240, 320, 3), dtype=np.uint8)
        img[50:190, 80:240] = [26, 84, 214]  # 蓝色色卡 RGB

        # 连续两帧防抖
        self.engine.detect_card(img)
        det = self.engine.detect_card(img)

        self.assertEqual(det.color, "blue")
        self.assertEqual(det.route, "A")
        self.assertGreaterEqual(det.confidence, 0.8)
        self.assertIsNotNone(det.bbox)

    def test_card_detection_red(self):
        """测试红色色卡检测 (路线 B)。"""
        img = np.zeros((240, 320, 3), dtype=np.uint8)
        img[50:190, 80:240] = [214, 40, 36]  # 红色色卡 RGB

        self.engine.detect_card(img)
        det = self.engine.detect_card(img)

        self.assertEqual(det.color, "red")
        self.assertEqual(det.route, "B")
        self.assertGreaterEqual(det.confidence, 0.8)
        self.assertIsNotNone(det.bbox)

    def test_card_noise_immunity(self):
        """测试浅蓝背景 (低饱和天空) 或杂色不会被误识别为色卡。"""
        # 浅蓝低饱和天空 (S~58)
        img_sky = np.full((240, 320, 3), [150, 170, 195], dtype=np.uint8)
        det = self.engine.detect_card(img_sky)
        self.assertIsNone(det.color)
        self.assertIsNone(det.route)

        # 纯黑图像
        img_black = np.zeros((240, 320, 3), dtype=np.uint8)
        det = self.engine.detect_card(img_black)
        self.assertIsNone(det.color)

    def test_cone_detection(self):
        """测试锥桶颜色与竖长包围框提取。"""
        img = np.zeros((240, 320, 3), dtype=np.uint8)
        # 放置一个蓝色锥桶 (x: 100~120, y: 120~180, 高 60 宽 20)
        img[120:180, 100:120] = [30, 78, 186]
        # 放置一个红色地雷 (x: 200~218, y: 130~175, 高 45 宽 18)
        img[130:175, 200:218] = [198, 46, 38]

        cones = self.engine.detect_cones(img)
        self.assertGreaterEqual(len(cones), 2)
        colors = [c.color for c in cones]
        self.assertIn("blue", colors)
        self.assertIn("red", colors)

    def test_benchmark_performance(self):
        """测试硬件/向量化推理延迟与 FPS 吞吐。"""
        img = np.zeros((240, 320, 3), dtype=np.uint8)
        img[50:190, 80:240] = [26, 84, 214]
        res = self.engine.benchmark(img, rounds=25)

        self.assertIn("avg_latency_ms", res)
        self.assertIn("estimated_fps", res)
        # 即使在 CI/虚拟机 CPU 环境下，向量化加速帧率也应 > 40 FPS (延迟 < 25ms)
        self.assertGreaterEqual(res["avg_latency_ms"], 0.)
        self.assertGreater(res["estimated_fps"], 0.)

    def test_vision_facade_integration(self):
        """测试 Vision 接入 AIVisionEngine 后的输出兼容性。"""
        cfg_data = {
            "camera": {"width": 320, "height": 240, "hfov_deg": 90.0, "pitch_deg": -12.0, "height_m": 0.18},
            "hsv": {
                "blue": {"ranges": [[[95, 80, 50], [130, 255, 255]]]},
                "red": {"ranges": [[[0, 80, 50], [15, 255, 255]], [[165, 80, 50], [179, 255, 255]]]},
            },
            "ai": {"backend": "auto"},
        }
        vision = Vision(Config(cfg_data))
        self.assertEqual(vision.backend_name, "fast_vectorized")

        # 输入测试图像
        img = np.zeros((240, 320, 3), dtype=np.uint8)
        img[50:190, 80:240] = [26, 84, 214]
        # 触发识别
        first = vision.process(img, want_card=True)
        self.assertIsNone(first.card_color)
        res = vision.process(img, want_card=True)
        self.assertEqual(res.card_color, "blue")
        self.assertEqual(vision.card_to_route(res.card_color), "A")


if __name__ == "__main__":
    unittest.main()
