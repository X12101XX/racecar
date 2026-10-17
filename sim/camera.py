"""相机成像模拟：把场地/锥桶/色块投影成真实像素图，供视觉算法处理。

采用针孔模型 + 近平面裁剪 + 画家算法（分组排序）光栅化，带亮度扰动、
高斯噪声与轻微模糊，保证视觉算法在仿真里面对的是"难看的图"而不是标签。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np

from racelib.field import Field
from racelib.vision import CameraModel


COLOR_FLOOR = (206, 206, 202)
COLOR_WALL = (92, 92, 98)
COLOR_START = (238, 238, 234)
COLOR_BLUE = (30, 78, 186)
COLOR_RED = (198, 46, 38)
COLOR_BLUE_CARD = (26, 84, 214)
COLOR_RED_CARD = (214, 40, 36)
COLOR_SKY = (150, 170, 195)


@dataclass
class Poly:
    """待光栅化的 3D 多边形（世界坐标，右手系 z 向上）。"""

    verts: np.ndarray
    color: Tuple[int, int, int]
    group: int          # 0 地面 / 1 墙 / 2 地面色块 / 3 锥桶


class CameraRenderer:
    def __init__(self, field: Field, cam: CameraModel,
                 mount_xy: Tuple[float, float] = (0.16, 0.0),
                 height_m: float = 0.18,
                 brightness: float = 1.0, noise_sigma: float = 3.0,
                 blur: bool = True, monochrome: bool = False):
        self.monochrome = monochrome
        self.field = field
        self.cam = cam
        self.mount = mount_xy
        self.height = height_m
        self.brightness = brightness
        self.noise_sigma = noise_sigma
        self.blur = blur
        xs = np.arange(cam.width)
        ys = np.arange(cam.height)
        self._grid_x, self._grid_y = np.meshgrid(xs, ys)

    # ---------------------------------------------------------------- #
    def scene_polygons(self, cones: Sequence, t: float = 0.0,
                       robot_pose: Optional[Sequence[float]] = None,
                       card_color: Optional[str] = None) -> List[Poly]:
        f = self.field
        polys: List[Poly] = []
        L, W = f.length, f.width
        wh = f.wall_height
        # 四面墙
        walls = [
            ([[L, 0, 0], [L, W, 0], [L, W, wh], [L, 0, wh]], COLOR_WALL),
            ([[0, 0, 0], [0, W, 0], [0, W, wh], [0, 0, wh]], COLOR_WALL),
            ([[0, W, 0], [L, W, 0], [L, W, wh], [0, W, wh]], COLOR_WALL),
            ([[0, 0, 0], [L, 0, 0], [L, 0, wh], [0, 0, wh]], COLOR_WALL),
        ]
        if f.boundary:
            walls=[([[a[0],a[1],0],[b[0],b[1],0],[b[0],b[1],wh],[a[0],a[1],wh]],COLOR_WALL)
                   for a,b in zip(f.boundary,f.boundary[1:]+f.boundary[:1])]
        for verts, col in walls:
            polys.append(Poly(np.array(verts, float), col, 1))
        # 地面色块（起点 / 据点 / 营地）
        z = 0.003
        polys.append(Poly(self._pad(f.start.cx, f.start.cy, f.start.length, f.start.width, z),
                          COLOR_START, 2))
        for name, zone in f.zones.items():
            col = (120, 120, 120) if self.monochrome else (COLOR_BLUE if zone.color == "blue" else COLOR_RED)
            polys.append(Poly(np.column_stack([zone.corners(),np.full(4,z)]),
                              tuple(int(c * 0.92) for c in col), 2))
            # 营地/据点边上的 A4 卡片（0.21m x 0.297m，平放）
            card = self._pad(zone.cx + 0.62, zone.cy, 0.30, 0.21, z + 0.001)
            card_col = (120,120,120) if self.monochrome else (COLOR_BLUE_CARD if zone.color == "blue" else COLOR_RED_CARD)
            polys.append(Poly(card, card_col, 2))
        # 锥桶（四棱锥近似圆锥，侧面按光照做明暗）
        for c in cones:
            if getattr(c, "hit", False):
                continue
            r = c.radius_base
            h = getattr(c, "height", 0.40)
            base = np.array([[c.x + r, c.y + r, 0], [c.x - r, c.y + r, 0],
                             [c.x - r, c.y - r, 0], [c.x + r, c.y - r, 0]], float)
            apex = np.array([[c.x, c.y, h]])
            col = (75,75,75) if self.monochrome else (COLOR_BLUE if c.color == "blue" else COLOR_RED)
            for k in range(4):
                p0, p1 = base[k], base[(k + 1) % 4]
                shade = 1.0 + 0.12 * math.cos(k * math.pi / 2.0 + 0.4)
                polys.append(Poly(np.array([p0, p1, apex[0]], float),
                                  tuple(int(min(255, v * shade)) for v in col), 3))
        if card_color and not self.monochrome and robot_pose is not None:
            col = COLOR_BLUE_CARD if card_color == "blue" else COLOR_RED_CARD
            px, py, pyaw = robot_pose[0], robot_pose[1], robot_pose[2]
            c, s = math.cos(pyaw), math.sin(pyaw)
            # 车头前方 0.32m 竖置 A4 色卡（确保绕向在投影后顺时针，_fill_polygon 可正确着色）
            d_card = 0.32
            w_half = 0.12
            pts_body = [
                [d_card, -w_half, 0.22],
                [d_card,  w_half, 0.22],
                [d_card,  w_half, 0.02],
                [d_card, -w_half, 0.02],
            ]
            card_pts = [[px + pt[0]*c - pt[1]*s, py + pt[0]*s + pt[1]*c, pt[2]] for pt in pts_body]
            polys.append(Poly(np.array(card_pts, float), col, 4))
        return polys

    @staticmethod
    def _pad(cx: float, cy: float, length: float, width: float, z: float) -> np.ndarray:
        hl, hw = length / 2.0, width / 2.0
        return np.array([[cx + hl, cy - hw, z], [cx + hl, cy + hw, z],
                         [cx - hl, cy + hw, z], [cx - hl, cy - hw, z]], float)

    # ---------------------------------------------------------------- #
    def _to_camera(self, pts: np.ndarray, cam_pos: np.ndarray,
                   yaw: float, pitch: float) -> np.ndarray:
        v = pts - cam_pos[None, :]
        c, s = math.cos(yaw), math.sin(yaw)
        x1 = c * v[:, 0] + s * v[:, 1]
        y1 = -s * v[:, 0] + c * v[:, 1]
        z1 = v[:, 2]
        cp, sp = math.cos(pitch), math.sin(pitch)
        # Camera pitch is negative downward; transform by the inverse rotation.
        xc = cp * x1 + sp * z1
        zc = -sp * x1 + cp * z1
        return np.stack([xc, y1, zc], axis=-1)

    def _project(self, cam_pts: np.ndarray) -> np.ndarray:
        cam = self.cam
        x = np.maximum(cam_pts[:, 0], 1e-3)
        u = cam.cx - cam.fx * (cam_pts[:, 1] / x)
        v = cam.cy - cam.fy * (cam_pts[:, 2] / x)
        return np.stack([u, v], axis=-1)

    @staticmethod
    def _clip_near(pts: np.ndarray, near: float = 0.06) -> Optional[np.ndarray]:
        """Sutherland-Hodgman：裁剪到 x_c >= near。"""
        out = []
        n = len(pts)
        for i in range(n):
            a = pts[i]
            b = pts[(i + 1) % n]
            ain = a[0] >= near
            bin_ = b[0] >= near
            if ain:
                out.append(a)
            if ain != bin_:
                t = (near - a[0]) / (b[0] - a[0])
                out.append(a + t * (b - a))
        if len(out) < 3:
            return None
        return np.asarray(out, dtype=float)

    def _fill_polygon(self, img: np.ndarray, poly_px: np.ndarray,
                      color: Sequence[int]) -> None:
        h, w = img.shape[:2]
        x0 = max(0, int(np.floor(poly_px[:, 0].min())))
        x1 = min(w - 1, int(np.ceil(poly_px[:, 0].max())))
        y0 = max(0, int(np.floor(poly_px[:, 1].min())))
        y1 = min(h - 1, int(np.ceil(poly_px[:, 1].max())))
        if x1 < x0 or y1 < y0:
            return
        xs = np.arange(x0, x1 + 1, dtype=np.float32)
        ys = np.arange(y0, y1 + 1, dtype=np.float32)
        gx, gy = np.meshgrid(xs, ys)
        inside = np.ones(gx.shape, dtype=bool)
        n = len(poly_px)
        for i in range(n):
            ax, ay = poly_px[i]
            bx, by = poly_px[(i + 1) % n]
            cross = (bx - ax) * (gy - ay) - (by - ay) * (gx - ax)
            inside &= (cross <= 0.0)
        if not inside.any():
            return
        img[y0:y1 + 1, x0:x1 + 1][inside] = np.asarray(color, dtype=np.uint8)

    # ---------------------------------------------------------------- #
    def render(self, cones: Sequence, pose: Sequence[float], yaw_noise: float = 0.0,
               pitch_noise: float = 0.0, card_color: Optional[str] = None) -> np.ndarray:
        cam = self.cam
        yaw = pose[2] + yaw_noise
        pitch = math.radians(cam.pitch_deg) + pitch_noise
        cam_pos = np.array([pose[0] + self.mount[0] * math.cos(yaw),
                            pose[1] + self.mount[0] * math.sin(yaw),
                            self.height], dtype=float)
        img = np.zeros((cam.height, cam.width, 3), dtype=np.uint8)
        img[:, :] = np.asarray(COLOR_SKY, dtype=np.uint8)
        # 地面用一个"地平线以下整行填充"的近似（比多边形光栅化快得多且视觉等价）
        horizon = int(round(cam.cy - cam.fy * math.tan(-pitch)))
        h0 = max(0, min(cam.height, horizon))
        if h0 < cam.height:
            img[h0:, :] = np.asarray(COLOR_FLOOR, dtype=np.uint8)

        polys = self.scene_polygons(cones, robot_pose=pose, card_color=card_color)
        render_list = []
        for p in polys:
            cam_pts = self._to_camera(p.verts, cam_pos, yaw, pitch)
            clipped = self._clip_near(cam_pts)
            if clipped is None:
                continue
            depth = float(np.mean(clipped[:, 0]))
            render_list.append((p.group, -depth, self._project(clipped), p.color))
        render_list.sort(key=lambda r: (r[0], r[1]))
        for _, _, px, col in render_list:
            self._fill_polygon(img, px, col)

        out = img.astype(np.float32) * self.brightness
        if self.noise_sigma > 0:
            out += np.random.normal(0.0, self.noise_sigma, size=out.shape)
        if self.blur:
            k = np.array([1.0, 2.0, 1.0], dtype=np.float32) / 4.0
            pad = np.pad(out, ((0, 0), (1, 1), (0, 0)), mode="edge")
            out = (pad[:, :-2] * k[0] + pad[:, 1:-1] * k[1] + pad[:, 2:] * k[2])
            pad = np.pad(out, ((1, 1), (0, 0), (0, 0)), mode="edge")
            out = (pad[:-2] * k[0] + pad[1:-1] * k[1] + pad[2:] * k[2])
        return np.clip(out, 0, 255).astype(np.uint8)
