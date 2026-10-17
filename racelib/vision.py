"""视觉感知：色卡识别、锥桶颜色分类、地面色块（据点/营地）识别。

设计要点：
1. 相机只负责"颜色 + 方位"，锥桶精确位置由激光雷达给出（fusion.py 融合）。
2. 地面距离用相机-地面几何模型估计（用于无雷达时的兜底与远距离预判）。
3. 同一份代码在仿真与本机 numpy/scipy 后端上可运行，车上自动切到 OpenCV。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from . import imageops as I
from .config import Config
from .ai_vision import AIVisionEngine


@dataclass
class CameraModel:
    """针孔相机模型（含俯仰角），行 -> 地面距离，列 -> 方位角。"""

    width: int = 640
    height: int = 480
    hfov_deg: float = 90.0
    pitch_deg: float = -12.0
    height_m: float = 0.18

    @property
    def fx(self) -> float:
        return (self.width / 2.0) / math.tan(math.radians(self.hfov_deg) / 2.0)

    @property
    def fy(self) -> float:
        return self.fx

    @property
    def cx(self) -> float:
        return self.width / 2.0

    @property
    def cy(self) -> float:
        return self.height / 2.0

    def bearing(self, col: float) -> float:
        """像素列 -> 车体方位角（右为负、左为正，与坐标 y 向左一致）。"""
        return math.atan2(self.cx - col, self.fx)

    def horizon_row(self) -> float:
        pitch_down = -math.radians(self.pitch_deg)
        return self.cy - self.fy * math.tan(pitch_down)

    def ground_distance(self, row: float) -> float:
        """像素行 -> 该行落在水平地面上的水平距离（米）。"""
        pitch_down = -math.radians(self.pitch_deg)
        phi = math.atan2(row - self.cy, self.fy)
        below = pitch_down + phi
        if below <= 0.03:
            return math.inf
        return self.height_m / math.tan(below)

    def pixel_height_of_object(self, dist_m: float, obj_height_m: float) -> float:
        """给定距离与物体高度，估算像素高度（用于阈值筛选）。"""
        if dist_m <= 0.05:
            return self.height * 2
        return obj_height_m * self.fy / dist_m


@dataclass
class ConeObs:
    """一帧图像里的锥桶观测（颜色 + 方位 + 粗距离）。"""

    color: str
    bearing: float
    dist_est: float
    area_px: float
    bbox: Tuple[float, float, float, float]
    confidence: float = 1.0

    @property
    def x_body(self) -> float:
        return self.dist_est * math.cos(self.bearing)

    @property
    def y_body(self) -> float:
        return self.dist_est * math.sin(self.bearing)


@dataclass
class VisionResult:
    card_color: Optional[str]
    card_ratio: float
    card_bbox: Optional[Tuple[float, float, float, float]]
    cones: List[ConeObs]
    floor_blue_ratio: float
    floor_red_ratio: float


class Vision:
    """视觉感知器：同一实例连续调用，内部无状态（便于分布式部署）。"""

    def __init__(self, cfg: Config):
        self.cfg = cfg
        cam = cfg.section("camera")
        self.cam = CameraModel(
            width=int(cam.get("width", 640)),
            height=int(cam.get("height", 480)),
            hfov_deg=float(cam.get("hfov_deg", 90.0)),
            pitch_deg=float(cam.get("pitch_deg", -12.0)),
            height_m=float(cam.get("height_m", 0.18)),
        )
        hsv = cfg.section("hsv")
        self.ranges: Dict[str, list] = {
            "blue": hsv.get("blue.ranges", []),
            "red": hsv.get("red.ranges", []),
        }
        self.morph_open_px = int(cfg.get("morph.open_px", 3))
        self.morph_close_px = int(cfg.get("morph.close_px", 3))
        self.cone_min_area = float(cfg.get("cone.min_area_px", 60))
        self.cone_height_m = float(cfg.get("cone.height_m", 0.40))
        self.cone_max_dist = float(cfg.get("cone.max_distance_m", 4.0))
        self.cone_downsample = max(1, int(cfg.get("cone.downsample", 2)))
        self.crop_lower_ratio = float(cfg.get("cone.crop_lower_ratio", 0.28))
        self.card_min_ratio = float(cfg.get("card.min_area_ratio", 0.015))
        self.patch_min_ratio = float(cfg.get("floor_patch.min_area_ratio", 0.08))
        self.patch_bottom_band = float(cfg.get("floor_patch.bottom_band_ratio", 0.30))
        ai_cfg = cfg.section("ai")
        preferred_backend = str(ai_cfg.get("backend", "auto"))
        model_path = str(ai_cfg.get("model_path", ""))
        self.ai_engine = AIVisionEngine(preferred_backend=preferred_backend, model_path=model_path)

    @property
    def backend_name(self) -> str:
        return self.ai_engine.backend_name

    # ---------------------------------------------------------------- #
    def _blobs(self, hsv: np.ndarray, color: str, min_area: float) -> List[Dict[str, float]]:
        return I.blobs_for_ranges(None, hsv, self.ranges.get(color, []), min_area,
                                  self.morph_open_px, self.morph_close_px)

    # ---------------------------------------------------------------- #
    def detect_card(self, hsv: np.ndarray, img_rgb: Optional[np.ndarray] = None) -> Tuple[Optional[str], float,
                                                    Optional[Tuple[float, float, float, float]]]:
        """识别贴在镜头前的色卡：优先使用硬件加速引擎，未检出时由 HSV 形状筛选兜底。"""
        if img_rgb is not None and getattr(self, "ai_engine", None) is not None:
            ai_card = self.ai_engine.detect_card(img_rgb)
            if ai_card.color is not None and ai_card.confidence > 0.4:
                return ai_card.color, ai_card.confidence, ai_card.bbox

        total = float(hsv.shape[0] * hsv.shape[1])
        best = (None, 0.0, None)
        for color in ("blue", "red"):
            min_area = self.card_min_ratio * total
            for s in self._blobs(hsv, color, min_area):
                w = s["x1"] - s["x0"] + 1.0
                h = s["y1"] - s["y0"] + 1.0
                if w < 8 or h < 8:
                    continue
                ratio = float(s["area"]) / total
                aspect = max(w, h) / max(1.0, min(w, h))
                fill = float(s["area"]) / max(1.0, w * h)
                if ratio < self.card_min_ratio or aspect > 3.0 or fill < 0.5:
                    continue
                if ratio > best[1]:
                    best = (color, ratio, (s["x0"], s["y0"], s["x1"], s["y1"]))
        return best

    def detect_floor_patch(self, hsv: np.ndarray) -> Tuple[float, float]:
        """画面下方区域的地面蓝/红色占比（用于确认车体已进入据点/营地色块）。"""
        h = hsv.shape[0]
        y0 = int(h * (1.0 - self.patch_bottom_band))
        band = hsv[y0:, :, :]
        total = float(band.shape[0] * band.shape[1])
        ratios = {}
        for color in ("blue", "red"):
            m = I.in_ranges(band, self.ranges.get(color, []))
            ratios[color] = float(m.sum()) / total
        return ratios["blue"], ratios["red"]

    def detect_cones(self, hsv: np.ndarray, row_offset: int = 0) -> List[ConeObs]:
        """检测视野内的蓝/红锥桶：面积 + 竖长形状过滤，输出方位与粗距离。"""
        img_h, img_w = hsv.shape[:2]
        horizon = self.cam.horizon_row()
        out: List[ConeObs] = []
        for color in ("blue", "red"):
            for s in I.blobs_scaled(hsv, self.ranges.get(color, []),
                                    self.cone_min_area, self.morph_open_px,
                                    self.morph_close_px, self.cone_downsample):
                x0, y0, x1, y1 = s["x0"], s["y0"], s["x1"], s["y1"]
                w = x1 - x0 + 1.0
                h = y1 - y0 + 1.0
                if h < w * 1.05:          # 锥桶应比宽高（近处），太扁的多半是地面色块
                    continue
                if h < 6:
                    continue
                base_row = y1 + row_offset
                dist = self.cam.ground_distance(base_row)
                if not math.isfinite(dist):
                    continue
                # 用已知锥桶高度反推距离做交叉校验（两者取小，偏保守）
                dist_h = self.cam.fy * self.cone_height_m / max(h, 1.0)
                dist = min(dist, dist_h * 1.15)
                dist = float(np.clip(dist, 0.15, self.cone_max_dist))
                bearing = self.cam.bearing(s["cx"])
                # 置信度：形状越接近竖长、距离越近越可信
                conf = float(np.clip(0.4 + 0.6 * min(1.0, h / 40.0), 0.0, 1.0))
                out.append(ConeObs(color=color, bearing=bearing, dist_est=dist,
                                   area_px=float(s["area"]),
                                   bbox=(x0, y0 + row_offset, x1, y1 + row_offset),
                                   confidence=conf))
        out.sort(key=lambda c: c.dist_est)
        return out

    # ---------------------------------------------------------------- #
    def process(self, img_rgb: np.ndarray, want_card: bool = True) -> VisionResult:
        h, w = img_rgb.shape[:2]
        y0 = 0
        if not want_card and self.crop_lower_ratio > 0.0:
            # 锥桶/地面色块只可能出现在地平线以下，裁掉上半部分省算力
            y0 = int(h * self.crop_lower_ratio)
        sub = img_rgb[y0:, :, :] if y0 > 0 else img_rgb
        hsv = I.to_hsv(sub)
        if want_card:
            card_color, card_ratio, card_bbox = self.detect_card(hsv, img_rgb=img_rgb)
            if card_bbox is not None and card_bbox[1] < y0:
                card_bbox = (card_bbox[0], card_bbox[1], card_bbox[2], card_bbox[3])
            elif card_bbox is not None and y0 > 0:
                card_bbox = (card_bbox[0], card_bbox[1] + y0, card_bbox[2], card_bbox[3] + y0)
        else:
            card_color, card_ratio, card_bbox = None, 0.0, None
        cones = self.detect_cones(hsv, row_offset=y0)
        blue_ratio, red_ratio = self.detect_floor_patch(hsv)
        return VisionResult(card_color, card_ratio, card_bbox, cones, blue_ratio, red_ratio)

    # ---------------------------------------------------------------- #
    def card_to_route(self, color: Optional[str]) -> Optional[str]:
        """色卡颜色 -> 路线：蓝色=A（据点A/营地A），红色=B。"""
        if color == "blue":
            return "A"
        if color == "red":
            return "B"
        return None
