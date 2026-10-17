"""Color-card inference with explicit backends and CPU geometry/color helpers.

Only the card classifier is accelerated; cone positions still come from lidar.
"""
from __future__ import annotations

import math
import os
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple, Sequence

import numpy as np

from .config import Config


@dataclass
class AICardDetection:
    """色卡检测结果。"""
    color: Optional[str]  # "blue", "red", 或 None
    confidence: float
    bbox: Optional[Tuple[float, float, float, float]] = None  # (x0, y0, x1, y1)
    route: Optional[str] = None  # "A", "B", 或 None


@dataclass
class AIConeDetection:
    """锥桶检测与颜色分类结果。"""
    color: str  # "blue", "red", "unknown"
    confidence: float
    bbox: Tuple[float, float, float, float]
    cx: float
    cy: float


class BaseAIBackend:
    """硬件推理后端抽象基类。"""

    name: str = "base"

    def is_available(self) -> bool:
        return True

    def detect_card(self, img_rgb: np.ndarray) -> AICardDetection:
        raise NotImplementedError

    def detect_cones(self, img_rgb: np.ndarray, row_offset: int = 0) -> List[AIConeDetection]:
        raise NotImplementedError

    def detect_floor_patch(self, img_rgb: np.ndarray) -> Tuple[float, float]:
        raise NotImplementedError


# =========================================================================== #
# 后端 1: 华为昇腾 NPU / CANN 后端 (Atlas 200I DK A2)
# =========================================================================== #
def card_tensor(img_rgb):
    """Contract: centre 70% ROI, RGB float32 [0,1], NCHW 1x3x224x224."""
    h,w=img_rgb.shape[:2]
    y0,y1=int(h*.15),int(h*.85);x0,x1=int(w*.15),int(w*.85)
    roi=img_rgb[y0:y1,x0:x1]
    if not roi.size: raise ValueError('empty image')
    # Fixed nearest sampling also works without OpenCV on the target.
    yy=np.minimum((np.arange(224)*roi.shape[0]/224).astype(int),roi.shape[0]-1)
    xx=np.minimum((np.arange(224)*roi.shape[1]/224).astype(int),roi.shape[1]-1)
    tensor=np.ascontiguousarray(roi[yy[:,None],xx].transpose(2,0,1)[None],dtype=np.float32)/255.
    return tensor,(float(x0),float(y0),float(x1),float(y1))


def decode_card(outputs,bbox):
    scores=np.asarray(outputs[0],dtype=float)
    if scores.shape not in ((1,3),(3,)) or not np.isfinite(scores).all():
        raise ValueError('card model must output finite scores [1,3]: background, blue, red; YOLO is not supported')
    scores=scores.reshape(3)
    idx=int(np.argmax(scores))
    if idx==0 or scores[idx]<.8: return AICardDetection(None,0.)
    color='blue' if idx==1 else 'red'
    return AICardDetection(color,float(min(1.,scores[idx])),bbox,'A' if idx==1 else 'B')


class AscendNPUBackend(BaseAIBackend):
    """ACLLite card classifier adapter; actual board/CANN compatibility is unverified."""
    name='ascend_npu'
    def __init__(self,model_path=''):
        self.model=None;self.error='';self.providers=['Ascend ACLLite']
        try:
            if not model_path or not os.path.isfile(model_path): raise ValueError('missing .om model_path')
            import acl
            from acllite_resource import AclLiteResource
            from acllite_model import AclLiteModel
            self.resource=AclLiteResource();self.resource.init()
            self.model=AclLiteModel(model_path)
            desc=self.model._model_desc
            dims,ret=acl.mdl.get_input_dims(desc,0)
            if (ret or acl.mdl.get_num_inputs(desc)!=1 or acl.mdl.get_num_outputs(desc)!=1
                or tuple(dims.get('dims',[]))!=(1,3,224,224) or acl.mdl.get_input_data_type(desc,0)!=0
                or acl.mdl.get_input_size_by_index(desc,0)!=1*3*224*224*4
                or acl.mdl.get_output_size_by_index(desc,0)!=3*4
                or acl.mdl.get_output_data_type(desc,0)!=0):
                raise ValueError('NPU model must be float32 RGB NCHW [1,3,224,224] -> scores [1,3]')
            decode_card(self.model.execute([np.zeros((1,3,224,224),np.float32)]),(0,0,224,224))
        except Exception as exc:
            self.error=str(exc);self.model=None
    def is_available(self): return self.model is not None
    def detect_card(self,img_rgb):
        tensor,bbox=card_tensor(img_rgb)
        return decode_card(self.model.execute([tensor]),bbox)


class ONNXRuntimeBackend(BaseAIBackend):
    """Executes the fixed card-classifier contract; reports installed session providers."""
    name='onnxruntime'
    def __init__(self,model_path=''):
        self._session=None;self.providers=[];self.error=''
        try:
            if not model_path or not os.path.isfile(model_path): raise ValueError('missing .onnx model_path')
            import onnxruntime as ort
            providers=[p for p in ('CUDAExecutionProvider','CPUExecutionProvider') if p in ort.get_available_providers()]
            options=ort.SessionOptions();options.intra_op_num_threads=1;options.inter_op_num_threads=1
            session=ort.InferenceSession(model_path,sess_options=options,providers=providers)
            ins=session.get_inputs()
            if len(ins)!=1 or ins[0].shape != [1,3,224,224] or ins[0].type!='tensor(float)':
                raise ValueError('card input must be float32 [1,3,224,224]')
            self.input_name=ins[0].name
            decode_card(session.run(None,{self.input_name:np.zeros((1,3,224,224),np.float32)}),(0,0,224,224))
            self._session=session;self.providers=session.get_providers()
        except Exception as exc:
            self.error=str(exc)
    def is_available(self): return self._session is not None
    def detect_card(self,img_rgb):
        tensor,bbox=card_tensor(img_rgb)
        return decode_card(self._session.run(None,{self.input_name:tensor}),bbox)


# =========================================================================== #
# 后端 3: 快速向量化色彩判断加速引擎 (FastVectorizedColorBackend)
# =========================================================================== #
class FastVectorizedColorBackend(BaseAIBackend):
    """高效的 SIMD / 向量化并行色彩提取后端。

    使用 NumPy 的 ROI 降采样和颜色阈值；实际延迟需在运行平台测量。
    """

    name = "fast_vectorized"

    def __init__(self, cfg: Optional[Config] = None):
        self.cfg = cfg

    def is_available(self) -> bool:
        return True

    @staticmethod
    def _rgb_to_fast_hsv(img: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """高速提取 H, S, V 单通道（纯向量化并行）。"""
        x = img.astype(np.float32)
        r, g, b = x[..., 0], x[..., 1], x[..., 2]
        maxc = np.maximum(np.maximum(r, g), b)
        minc = np.minimum(np.minimum(r, g), b)
        delta = maxc - minc
        v = maxc
        s = np.zeros_like(maxc)
        mask = maxc > 1e-4
        s[mask] = (delta[mask] / maxc[mask]) * 255.0

        h = np.zeros_like(maxc)
        delta_safe = np.maximum(delta, 1e-4)
        m_r = (maxc == r) & (delta > 1e-4)
        m_g = (maxc == g) & (delta > 1e-4)
        m_b = (maxc == b) & (delta > 1e-4)

        h[m_r] = ((g[m_r] - b[m_r]) / delta_safe[m_r]) % 6.0
        h[m_g] = ((b[m_g] - r[m_g]) / delta_safe[m_g]) + 2.0
        h[m_b] = ((r[m_b] - g[m_b]) / delta_safe[m_b]) + 4.0
        h = (h * 30.0) % 180.0  # 映射到 OpenCV 约定 0~179
        return h, s, v

    def detect_card(self, img_rgb: np.ndarray) -> AICardDetection:
        """色卡快速识别：判断镜头中央区域是否存在大面积蓝/红色块。"""
        h_img, w_img = img_rgb.shape[:2]
        # 裁剪中心 60% 区域（色卡通常出示在正前方）
        cy0, cy1 = int(h_img * 0.15), int(h_img * 0.85)
        cx0, cx1 = int(w_img * 0.15), int(w_img * 0.85)
        crop = img_rgb[cy0:cy1, cx0:cx1:2]  # 水平方向 2 倍降采样加速
        h, s, v = self._rgb_to_fast_hsv(crop)

        # 蓝卡阈值：H: 95~130, S >= 120, V >= 60 (高饱和纯色，排除淡蓝天空 S~58)
        blue_mask = (h >= 95.0) & (h <= 130.0) & (s >= 120.0) & (v >= 60.0)
        # 红卡阈值：H <= 15 或 H >= 165, S >= 120, V >= 60
        red_mask = ((h <= 15.0) | (h >= 165.0)) & (s >= 120.0) & (v >= 60.0)

        total_px = float(blue_mask.size)
        blue_ratio = float(np.count_nonzero(blue_mask)) / total_px
        red_ratio = float(np.count_nonzero(red_mask)) / total_px

        # 阈值判定
        min_card_ratio = 0.03  # 3% 覆盖度即锁定
        if blue_ratio > red_ratio and blue_ratio >= min_card_ratio:
            return AICardDetection("blue", float(min(1.0, blue_ratio / 0.12)),
                                   (float(cx0), float(cy0), float(cx1), float(cy1)), "A")
        elif red_ratio > blue_ratio and red_ratio >= min_card_ratio:
            return AICardDetection("red", float(min(1.0, red_ratio / 0.12)),
                                   (float(cx0), float(cy0), float(cx1), float(cy1)), "B")
        return AICardDetection(None, 0.0, None, None)

    def detect_cones(self, img_rgb: np.ndarray, row_offset: int = 0) -> List[AIConeDetection]:
        """快速提取图像中的红/蓝锥桶。"""
        h_img, w_img = img_rgb.shape[:2]
        # 降采样 2 倍
        sub = img_rgb[::2, ::2]
        h, s, v = self._rgb_to_fast_hsv(sub)

        blue_mask = (h >= 95.0) & (h <= 130.0) & (s >= 100.0) & (v >= 60.0)
        red_mask = ((h <= 15.0) | (h >= 165.0)) & (s >= 100.0) & (v >= 60.0)

        out: List[AIConeDetection] = []
        for color, mask in [("blue", blue_mask), ("red", red_mask)]:
            # 垂直投影柱状图（找出锥桶的列聚集）
            col_sums = np.sum(mask, axis=0)
            cols = np.where(col_sums >= 3)[0]
            if len(cols) == 0:
                continue

            # 按连续段聚类列
            clusters: List[List[int]] = []
            curr: List[int] = [cols[0]]
            for c in cols[1:]:
                if c - curr[-1] <= 2:
                    curr.append(c)
                else:
                    if len(curr) >= 2:
                        clusters.append(curr)
                    curr = [c]
            if len(curr) >= 2:
                clusters.append(curr)

            for cl in clusters:
                c_min, c_max = cl[0], cl[-1]
                strip = mask[:, c_min:c_max + 1]
                rows = np.where(np.any(strip, axis=1))[0]
                if len(rows) < 3:
                    continue
                r_min, r_max = rows[0], rows[-1]
                # 恢复到原图坐标
                x0, x1 = float(c_min * 2), float((c_max + 1) * 2)
                y0, y1 = float(r_min * 2 + row_offset), float((r_max + 1) * 2 + row_offset)
                bw = x1 - x0
                bh = y1 - y0
                # 竖直锥桶纵横比筛选：高度通常大于或接近宽度
                if bh < 6.0 or bh < bw * 0.8:
                    continue
                conf = float(np.clip(0.5 + 0.5 * min(1.0, bh / 30.0), 0.0, 1.0))
                cx = (x0 + x1) / 2.0
                cy = (y0 + y1) / 2.0
                out.append(AIConeDetection(color=color, confidence=conf, bbox=(x0, y0, x1, y1), cx=cx, cy=cy))

        return out

    def detect_floor_patch(self, img_rgb: np.ndarray) -> Tuple[float, float]:
        """快速提取画面底部地贴的蓝/红色比例。"""
        h_img = img_rgb.shape[0]
        y0 = int(h_img * 0.70)
        bot = img_rgb[y0::2, ::2]
        h, s, v = self._rgb_to_fast_hsv(bot)

        b_mask = (h >= 95.0) & (h <= 130.0) & (s >= 70.0) & (v >= 50.0)
        r_mask = ((h <= 10.0) | (h >= 170.0)) & (s >= 70.0) & (v >= 50.0)
        total = float(b_mask.size)
        return float(np.count_nonzero(b_mask)) / total, float(np.count_nonzero(r_mask)) / total


# =========================================================================== #
# AI 视觉加速总引擎 (Facade)
# =========================================================================== #
class AIVisionEngine:
    """Explicit card inference selection; only auto permits CPU fallback."""

    def __init__(self, preferred_backend: str = "auto", model_path: str = ""):
        self.preferred_backend = preferred_backend.lower()
        self.backend: BaseAIBackend = self._select_backend(preferred_backend, model_path)
        self._card_history: List[AICardDetection] = []
        self._last_process_time_ms: float = 0.0

    def _select_backend(self,pref,model_path):
        pref=pref.lower()
        self.fallback_reason=''
        if pref in ('fast_color','cpu'): return FastVectorizedColorBackend()
        if pref not in ('auto','npu','onnx'): raise ValueError('unknown vision backend: '+pref)
        if pref=='auto' and not model_path:
            self.fallback_reason='no model configured; using CPU color thresholds'
            return FastVectorizedColorBackend()
        cls=AscendNPUBackend if pref=='npu' or (pref=='auto' and model_path.endswith('.om')) else ONNXRuntimeBackend
        backend=cls(model_path)
        if backend.is_available(): return backend
        if pref!='auto': raise RuntimeError(f'{pref} unavailable: {backend.error}')
        self.fallback_reason=backend.error
        return FastVectorizedColorBackend()

    @property
    def diagnostics(self):
        return dict(requested=self.preferred_backend,actual=self.backend_name,
                    providers=getattr(self.backend,'providers',['CPU/NumPy']),
                    fallback_reason=self.fallback_reason)

    @property
    def backend_name(self) -> str:
        return self.backend.name

    def detect_card(self, img_rgb: np.ndarray) -> AICardDetection:
        """色卡识别（含时序防抖滤波）。"""
        t0 = time.monotonic()
        det = self.backend.detect_card(img_rgb)
        if not det.color or det.confidence < .8:
            self._card_history=[]
            return AICardDetection(None,0.)
        if self._card_history and self._card_history[-1].color != det.color:
            self._card_history=[]
        self._card_history.append(det)
        self._card_history=self._card_history[-2:]
        if len(self._card_history)<2:
            return AICardDetection(None,0.)

        self._last_process_time_ms = (time.monotonic() - t0) * 1000.0
        return det

    def detect_cones(self, img_rgb: np.ndarray, row_offset: int = 0) -> List[AIConeDetection]:
        t0 = time.monotonic()
        cones = FastVectorizedColorBackend().detect_cones(img_rgb, row_offset)
        self._last_process_time_ms = (time.monotonic() - t0) * 1000.0
        return cones

    def detect_floor_patch(self, img_rgb: np.ndarray) -> Tuple[float, float]:
        return FastVectorizedColorBackend().detect_floor_patch(img_rgb)

    def benchmark(self, img_rgb: np.ndarray, rounds: int = 20) -> Dict[str, Any]:
        """基准测速：评估当前后端处理延迟与 FPS。"""
        # 预热
        self.detect_card(img_rgb)
        self.detect_cones(img_rgb)

        t0 = time.monotonic()
        for _ in range(rounds):
            self.backend.detect_card(img_rgb)
            self.detect_cones(img_rgb)
        total = time.monotonic() - t0
        avg_ms = (total / rounds) * 1000.0
        fps = rounds / max(total, 1e-6)
        return {
            "backend": self.backend_name,
            "rounds": rounds,
            "avg_latency_ms": round(avg_ms, 2),
            "estimated_fps": round(fps, 1),
        }
