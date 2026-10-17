"""图像基础操作，三套后端自动选择：cv2 -> scipy -> 纯 numpy。

比赛要求使用 OpenCV，车辆上走 cv2 后端；本机没有 python cv2 时自动退化为
scipy/numpy 实现，保证同一套感知代码在仿真与实车之间完全一致。
HSV 统一采用 OpenCV 约定：H 0~179，S 0~255，V 0~255。
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

import numpy as np

try:  # pragma: no cover - 环境相关
    import cv2  # type: ignore

    BACKEND = "cv2"
except Exception:  # noqa: BLE001
    cv2 = None  # type: ignore
    try:
        from scipy import ndimage as _ndi  # type: ignore

        BACKEND = "scipy"
    except Exception:  # noqa: BLE001
        _ndi = None  # type: ignore
        BACKEND = "numpy"


# --------------------------------------------------------------------------- #
# 颜色空间
# --------------------------------------------------------------------------- #
def to_hsv(img_rgb: np.ndarray) -> np.ndarray:
    """RGB(uint8, HxWx3) -> OpenCV 约定 HSV(float32, HxWx3)。"""
    x = np.asarray(img_rgb)
    if x.dtype != np.uint8:
        x = np.clip(x, 0, 255).astype(np.uint8)
    if BACKEND == "cv2":
        return cv2.cvtColor(x, cv2.COLOR_RGB2HSV).astype(np.float32)

    rgb = x.astype(np.float32) / 255.0
    r, g, b = rgb[..., 0], rgb[..., 1], rgb[..., 2]
    maxc = np.max(rgb, axis=-1)
    minc = np.min(rgb, axis=-1)
    v = maxc
    delta = maxc - minc
    s = np.where(maxc > 1e-6, delta / np.maximum(maxc, 1e-6), 0.0)
    h = np.zeros_like(maxc)
    nz = delta > 1e-6
    rc = np.where(nz, (maxc - r) / np.maximum(delta, 1e-6), 0.0)
    gc = np.where(nz, (maxc - g) / np.maximum(delta, 1e-6), 0.0)
    bc = np.where(nz, (maxc - b) / np.maximum(delta, 1e-6), 0.0)
    h = np.where(maxc == r, bc - gc, h)
    h = np.where(maxc == g, 2.0 + rc - bc, h)
    h = np.where(maxc == b, 4.0 + gc - rc, h)
    h = (h / 6.0) % 1.0
    out = np.stack([h * 180.0, s * 255.0, v * 255.0], axis=-1)
    return out.astype(np.float32)


def in_ranges(hsv: np.ndarray, ranges) -> np.ndarray:
    """按若干 [[lo_h,lo_s,lo_v],[hi_h,hi_s,hi_v]] 阈值取并集掩码。"""
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    mask = np.zeros(h.shape, dtype=bool)
    for rng in ranges:
        lo, hi = rng
        m = ((h >= lo[0]) & (h <= hi[0]) & (s >= lo[1]) & (s <= hi[1])
             & (v >= lo[2]) & (v <= hi[2]))
        mask |= m
    return mask


# --------------------------------------------------------------------------- #
# 形态学 / 滤波
# --------------------------------------------------------------------------- #
def _shift_max(mask: np.ndarray, dy: int, dx: int) -> np.ndarray:
    out = np.zeros_like(mask)
    h, w = mask.shape
    ys_src = slice(max(0, -dy), h - max(0, dy))
    ys_dst = slice(max(0, dy), h - max(0, -dy))
    xs_src = slice(max(0, -dx), w - max(0, dx))
    xs_dst = slice(max(0, dx), w - max(0, -dx))
    out[ys_dst, xs_dst] = mask[ys_src, xs_src]
    return out


def dilate(mask: np.ndarray, k: int = 3) -> np.ndarray:
    if k <= 1:
        return mask
    if BACKEND == "cv2":
        kernel = np.ones((k, k), np.uint8)
        return cv2.dilate(mask.astype(np.uint8), kernel).astype(bool)
    if BACKEND == "scipy":
        return _ndi.grey_dilation(mask, size=(k, k))
    r = k // 2
    out = mask.copy()
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            out |= _shift_max(mask, dy, dx)
    return out


def erode(mask: np.ndarray, k: int = 3) -> np.ndarray:
    if k <= 1:
        return mask
    if BACKEND == "cv2":
        kernel = np.ones((k, k), np.uint8)
        return cv2.erode(mask.astype(np.uint8), kernel).astype(bool)
    if BACKEND == "scipy":
        return _ndi.grey_erosion(mask, size=(k, k))
    return ~dilate(~mask, k)


def morph_open(mask: np.ndarray, k: int = 3) -> np.ndarray:
    """先腐蚀后膨胀，去孤立噪点。"""
    return dilate(erode(mask, k), k)


def morph_close(mask: np.ndarray, k: int = 3) -> np.ndarray:
    """先膨胀后腐蚀，补小孔。"""
    return erode(dilate(mask, k), k)


def gaussian_blur(img: np.ndarray, k: int = 3, sigma: float = 1.0) -> np.ndarray:
    if BACKEND == "cv2":
        return cv2.GaussianBlur(img, (k, k), sigma)
    r = k // 2
    xs = np.arange(-r, r + 1, dtype=np.float32)
    g = np.exp(-(xs ** 2) / (2 * sigma * sigma))
    g /= g.sum()
    x = np.asarray(img, dtype=np.float32)
    if x.ndim == 3:
        g_row = g[None, :, None]
        g_col = g[:, None, None]
    else:
        g_row = g[None, :]
        g_col = g[:, None]
    pad_h = np.pad(x, ((r, r), (0, 0)) + ((0, 0),) * (x.ndim - 2), mode="edge")
    tmp = sum(g_col[i] * pad_h[i:i + x.shape[0]] for i in range(k))
    pad_w = np.pad(tmp, ((0, 0), (r, r)) + ((0, 0),) * (x.ndim - 2), mode="edge")
    out = sum(g_row[0, i] * pad_w[:, i:i + x.shape[1]] if x.ndim == 3
              else g_row[i] * pad_w[:, i:i + x.shape[1]] for i in range(k))
    return out


# --------------------------------------------------------------------------- #
# 连通域
# --------------------------------------------------------------------------- #
def _cc_numpy(mask: np.ndarray) -> Tuple[np.ndarray, List[Dict[str, float]]]:
    """纯 numpy 的行程编码 + 并查集连通域标记。"""
    h, w = mask.shape
    labels = np.zeros((h, w), dtype=np.int32)
    parent: List[int] = [0]

    def find(x: int) -> int:
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    def union(a: int, b: int) -> None:
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[max(ra, rb)] = min(ra, rb)

    prev_runs: List[Tuple[int, int, int]] = []
    for y in range(h):
        idx = np.flatnonzero(mask[y])
        runs: List[Tuple[int, int, int]] = []
        if idx.size:
            splits = np.flatnonzero(np.diff(idx) > 1)
            starts = np.concatenate([[0], splits + 1])
            ends = np.concatenate([splits + 1, [idx.size]])
            for s, e in zip(starts, ends):
                x0, x1 = int(idx[s]), int(idx[e - 1])
                lab = 0
                for (px0, px1, plab) in prev_runs:
                    if px1 >= x0 and px0 <= x1:
                        if lab == 0:
                            lab = plab
                        else:
                            union(lab, plab)
                if lab == 0:
                    parent.append(len(parent))
                    lab = len(parent) - 1
                labels[y, x0:x1 + 1] = lab
                runs.append((x0, x1, lab))
        prev_runs = runs

    # 打平标签并统计
    stats: Dict[int, Dict[str, float]] = {}
    ys, xs = np.nonzero(mask)
    if ys.size:
        labs = labels[ys, xs]
        for y, x, l in zip(ys.tolist(), xs.tolist(), labs.tolist()):
            r = find(int(l))
            st = stats.get(r)
            if st is None:
                stats[r] = {"area": 1.0, "x0": x, "x1": x, "y0": y, "y1": y,
                            "sum_x": float(x), "sum_y": float(y)}
            else:
                st["area"] += 1
                st["x0"] = min(st["x0"], x)
                st["x1"] = max(st["x1"], x)
                st["y0"] = min(st["y0"], y)
                st["y1"] = max(st["y1"], y)
                st["sum_x"] += x
                st["sum_y"] += y
    out_stats: List[Dict[str, float]] = []
    for i, st in enumerate(stats.values()):
        st["cx"] = st["sum_x"] / st["area"]
        st["cy"] = st["sum_y"] / st["area"]
        out_stats.append(st)
    return labels, out_stats


def connected_components(mask: np.ndarray) -> Tuple[np.ndarray, List[Dict[str, float]]]:
    """连通域标记。返回 (labels, stats)，stats 含 area / bbox / 质心。"""
    m = np.asarray(mask, dtype=bool)
    if BACKEND == "cv2":
        n, labels, st, cent = cv2.connectedComponentsWithStats(m.astype(np.uint8), 8)
        out: List[Dict[str, float]] = []
        for i in range(1, n):
            x0, y0, ww, hh, area = st[i]
            out.append({"area": float(area), "x0": float(x0), "y0": float(y0),
                        "x1": float(x0 + ww - 1), "y1": float(y0 + hh - 1),
                        "cx": float(cent[i][0]), "cy": float(cent[i][1])})
        return labels.astype(np.int32), out
    if BACKEND == "scipy":
        labels, n = _ndi.label(m)
        out = []
        for i, sl in enumerate(_ndi.find_objects(labels), start=1):
            if sl is None:
                continue
            ys, xs = sl
            area = float((labels[sl] == i).sum())
            if area <= 0:
                continue
            yy, xx = np.nonzero(labels[sl] == i)
            out.append({"area": area, "x0": float(xs.start), "y0": float(ys.start),
                        "x1": float(xs.stop - 1), "y1": float(ys.stop - 1),
                        "cx": float(xs.start + xx.mean()), "cy": float(ys.start + yy.mean())})
        return labels.astype(np.int32), out
    return _cc_numpy(m)


# --------------------------------------------------------------------------- #
# 其他
# --------------------------------------------------------------------------- #
def resize_nearest(img: np.ndarray, scale: int) -> np.ndarray:
    """整倍数最近邻下采样（加速用）。"""
    if scale <= 1:
        return img
    return img[::scale, ::scale]


def blobs_for_ranges(mask_fn, hsv: np.ndarray, ranges, min_area: float,
                     morph_open_px: int = 3, morph_close_px: int = 3
                     ) -> List[Dict[str, float]]:
    """通用：颜色阈值 -> 形态学 -> 连通域 -> 过滤面积。"""
    m = in_ranges(hsv, ranges)
    if morph_open_px > 1:
        m = morph_open(m, morph_open_px)
    if morph_close_px > 1:
        m = morph_close(m, morph_close_px)
    _, stats = connected_components(m)
    return [s for s in stats if s["area"] >= min_area]


def blobs_scaled(hsv: np.ndarray, ranges, min_area: float,
                 morph_open_px: int = 3, morph_close_px: int = 3,
                 scale: int = 1) -> List[Dict[str, float]]:
    """先降采样再形态学/连通域（车轮级算力优化），返回值换算回原图坐标。

    对锥桶这种小目标，降采样 2 倍仍能稳定检出（1m 处锥桶高度仍有约 20 像素），
    但 HSV 阈值化与连通域的开销显著下降。
    """
    m = in_ranges(hsv, ranges)
    if scale > 1:
        m = m[::scale, ::scale]
    k_open = max(1, int(round(morph_open_px / max(1, scale))) | 1)
    k_close = max(1, int(round(morph_close_px / max(1, scale))) | 1)
    if k_open > 1:
        m = morph_open(m, k_open)
    if k_close > 1:
        m = morph_close(m, k_close)
    _, stats = connected_components(m)
    a2 = float(scale * scale)
    out: List[Dict[str, float]] = []
    for s in stats:
        area = s["area"] * a2
        if area < min_area:
            continue
        out.append({
            "area": area,
            "x0": s["x0"] * scale, "y0": s["y0"] * scale,
            "x1": s["x1"] * scale + (scale - 1), "y1": s["y1"] * scale + (scale - 1),
            "cx": s["cx"] * scale, "cy": s["cy"] * scale,
        })
    return out
