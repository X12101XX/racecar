"""画出两条路线的示意图（供技术文档 / 答辩 PPT 使用）。

    python3 tools/make_route_figure.py            # 输出 output/route_ab_overview.png
"""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from PIL import Image, ImageDraw, ImageFont  # noqa: E402

from racelib.config import load_all  # noqa: E402
from racelib.field import Field  # noqa: E402


def draw_route(d: ImageDraw.ImageDraw, field: Field, route_cfg, ppm: float,
               origin, color) -> None:
    def W2P(p):
        return (origin[0] + p[0] * ppm, origin[1] - p[1] * ppm)

    entry = route_cfg.get("cross_entry")
    exit_ = route_cfg.get("cross_exit")
    sh = field.zone(route_cfg.get("stronghold")).center
    camp = field.zone(route_cfg.get("camp")).center
    pts = [field.start.center, sh, (entry["x"], entry["y_hint"]),
           (exit_["x"], exit_["y_hint"]), camp]
    for a, b in zip(pts, pts[1:]):
        d.line([W2P(a), W2P(b)], fill=color, width=3)
    for label, p in (("起点", field.start.center), ("据点", sh), ("营地", camp)):
        x, y = W2P(p)
        d.ellipse([x - 5, y - 5, x + 5, y + 5], fill=color)
        d.text((x + 8, y - 6), label, fill=color)


def main() -> int:
    cfgs = load_all()
    field = Field.from_config(cfgs["field"])
    cfgs_a = load_all(route="A")["route"]
    cfgs_b = load_all(route="B")["route"]
    ppm = 110
    W, H = int(field.length * ppm) + 80, int(field.width * ppm) + 80
    img = Image.new("RGB", (W, H), (26, 28, 32))
    d = ImageDraw.Draw(img)
    try:
        d.font = ImageFont.truetype(  # type: ignore[attr-defined]
            "/usr/share/fonts/noto-cjk/NotoSansCJK-Regular.ttc", 16)
    except Exception:  # noqa: BLE001
        try:
            d.font = ImageFont.truetype(  # type: ignore[attr-defined]
                "/usr/share/fonts/noto-cjk/NotoSansCJK-DemiLight.ttc", 16)
        except Exception:  # noqa: BLE001
            pass
    ox, oy = 40, H - 40

    def W2P(p):
        return (ox + p[0] * ppm, oy - p[1] * ppm)

    def box(x0: float, y0: float, x1: float, y1: float):
        a, b = W2P((x0, y1)), W2P((x1, y0))     # 注意 y 轴翻转，左上->右下
        return [a, b]

    # 场地与雷区
    d.rectangle(box(0, 0, field.length, field.width), outline=(200, 205, 215), width=2)
    d.rectangle(box(field.minefield_x[0], 0, field.minefield_x[1], field.width),
                fill=(58, 60, 66))
    for lx in field.blockade_lines_x:
        d.line([W2P((lx, 0)), W2P((lx, field.width))], fill=(120, 124, 132), width=1)
    # 区域
    for name, z in field.zones.items():
        col = (45, 95, 210) if z.color == "blue" else (210, 60, 50)
        d.rectangle(box(z.cx - 0.5, z.cy - 0.5, z.cx + 0.5, z.cy + 0.5),
                    fill=tuple(int(c * 0.55) for c in col), outline=col, width=2)
    d.rectangle(box(field.start.cx - 0.45, field.start.cy - 0.4,
                    field.start.cx + 0.45, field.start.cy + 0.4),
                outline=(230, 230, 230), width=2)
    draw_route(d, field, cfgs_a, ppm, (ox, oy), (70, 150, 255))
    draw_route(d, field, cfgs_b, ppm, (ox, oy), (255, 110, 90))
    d.text((12, 12), "穿越火线：路线A(蓝卡, 据点A->营地A) / 路线B(红卡, 据点B->营地B)",
           fill=(230, 230, 235))
    d.text((12, 30), "灰带=雷区(3条蓝色封锁线+红锥地雷)，虚线=封锁线位置", fill=(180, 184, 190))
    out = "output/route_ab_overview.png"
    os.makedirs("output", exist_ok=True)
    img.save(out)
    print("saved", out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
