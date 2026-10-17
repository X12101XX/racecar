"""赛前改场地：按现场尺寸自动生成 field.json 与两条路线的进出点（并做一致性检查）。

    # 只看当前配置是否自洽（不改任何文件）
    python3 tools/setup_field.py

    # 现场量到长 7.5m、宽 4.2m，重新生成布局并写回配置
    python3 tools/setup_field.py --length 7.5 --width 4.2 --apply

规则依据（赛题 PDF）：场地长 6~8m、宽 3~5m；左/右两侧各 1~2m 放据点/营地（各 1m×1m），
中间 4m 为雷区；雷区内 3 条蓝色封锁线、相邻线间隔 1.5m；起点在左侧墙中间。
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from typing import Dict, List, Tuple

from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from racelib.config import default_config_dir  # noqa: E402


def build_layout(length: float, width: float, minefield_w: float = 4.0,
                 line_spacing: float = 1.5) -> Dict[str, object]:
    side = (length - minefield_w) / 2.0
    mf0, mf1 = side, side + minefield_w
    sx = side / 2.0
    cx = length - side / 2.0
    top_y = width - 0.75
    bot_y = 0.75
    lines = [round(mf0 + 0.5 + i * line_spacing, 2) for i in range(3)]
    return {
        "arena": {"length_m": round(length, 2), "width_m": round(width, 2),
                  "wall_height_m": 0.30},
        # 起点贴着左侧墙（与赛题示意图一致：0.9m 宽的起点区从墙边开始）
        "start": {"center": [0.45, round(width / 2, 2)],
                  "size": [0.9, 0.8], "heading_deg": 0.0},
        "zones": {
            "stronghold_a": {"center": [round(sx, 2), round(top_y, 2)], "size": [1.0, 1.0],
                             "color": "blue"},
            "stronghold_b": {"center": [round(sx, 2), round(bot_y, 2)], "size": [1.0, 1.0],
                             "color": "red"},
            "camp_a": {"center": [round(cx, 2), round(bot_y, 2)], "size": [1.0, 1.0],
                       "color": "blue"},
            "camp_b": {"center": [round(cx, 2), round(top_y, 2)], "size": [1.0, 1.0],
                       "color": "red"},
        },
        "minefield": {"x_min": round(mf0, 2), "x_max": round(mf1, 2),
                      "blockade_lines_x": lines, "line_spacing_m": line_spacing,
                      "min_cone_gap_m": 0.40},
        "prior": {"blue_lines_y": [[], [], []], "red_mines": []},
    }


def check_layout(cfg: Dict[str, object]) -> List[str]:
    """返回问题列表（空表示自洽）。"""
    issues: List[str] = []
    L = float(cfg["arena"]["length_m"])          # type: ignore[index]
    W = float(cfg["arena"]["width_m"])           # type: ignore[index]
    if not (6.0 <= L <= 8.0):
        issues.append(f"场地长度 {L} m 超出赛题 6~8m")
    if not (3.0 <= W <= 5.0):
        issues.append(f"场地宽度 {W} m 超出赛题 3~5m")
    mf0 = float(cfg["minefield"]["x_min"])       # type: ignore[index]
    mf1 = float(cfg["minefield"]["x_max"])       # type: ignore[index]
    if abs((mf1 - mf0) - 4.0) > 0.05:
        issues.append(f"雷区宽度 {mf1 - mf0:.2f} m（赛题给的是 4m）")
    lines = cfg["minefield"]["blockade_lines_x"]  # type: ignore[index]
    for i in range(len(lines) - 1):
        gap = float(lines[i + 1]) - float(lines[i])
        if abs(gap - 1.5) > 0.01:
            issues.append(f"封锁线 {i}->{i + 1} 间隔 {gap:.2f} m（应为 1.5m）")
    if lines and (float(lines[0]) - mf0 < 0.25 or mf1 - float(lines[-1]) < 0.25):
        issues.append("封锁线离雷区边界太近（<0.25m），建议至少 0.4m")
    for name, z in cfg["zones"].items():          # type: ignore[union-attr]
        x, y = z["center"]
        half = 0.5
        # 区域是地面标线，只要在场地内即可（贴着墙是允许的，车照样能停进去）
        if x - half < 0.05 or x + half > L - 0.05 or y - half < 0.05 or y + half > W - 0.05:
            issues.append(f"{name} 的 1m×1m 区域超出场地边界")
        if name.startswith("stronghold") and x + half > mf0 - 0.10:
            issues.append(f"{name} 离雷区太近（右边缘 {x + half:.2f}，雷区起点 {mf0:.2f}；"
                          f"建议至少留 0.1m 余量，避免车在据点里就压到封锁线锥桶的安全区）")
        if name.startswith("camp") and x - half < mf1 + 0.10:
            issues.append(f"{name} 离雷区太近（左边缘 {x - half:.2f}，雷区终点 {mf1:.2f}；"
                          f"建议至少留 0.1m 余量）")
    st = cfg["start"]                              # type: ignore[index]
    sx, sy = st["center"]
    if sx - st["size"][0] / 2 < -0.01 or sy - st["size"][1] / 2 < -0.01 \
            or sx + st["size"][0] / 2 > L + 0.01 or sy + st["size"][1] / 2 > W + 0.01:
        issues.append("起点区域超出场地边界")
    return issues


def render(cfg: Dict[str, object], path: str, ppm: float = 100.0) -> None:
    L = float(cfg["arena"]["length_m"])           # type: ignore[index]
    W = float(cfg["arena"]["width_m"])            # type: ignore[index]
    img = Image.new("RGB", (int(L * ppm) + 80, int(W * ppm) + 80), (26, 28, 32))
    d = ImageDraw.Draw(img)
    try:
        font = ImageFont.truetype("/usr/share/fonts/noto-cjk/NotoSansCJK-Regular.ttc", 16)
        d.font = font  # type: ignore[attr-defined]
    except Exception:  # noqa: BLE001
        pass
    ox, oy = 40, img.height - 40

    def W2P(p):
        return (ox + p[0] * ppm, oy - p[1] * ppm)

    def box(x0, y0, x1, y1):
        return [W2P((x0, y1)), W2P((x1, y0))]

    d.rectangle(box(0, 0, L, W), outline=(200, 205, 215), width=2)
    mf = cfg["minefield"]                          # type: ignore[index]
    d.rectangle(box(float(mf["x_min"]), 0, float(mf["x_max"]), W), fill=(58, 60, 66))
    for lx in mf["blockade_lines_x"]:              # type: ignore[union-attr]
        d.line([W2P((float(lx), 0)), W2P((float(lx), W))], fill=(120, 124, 132), width=1)
    for name, z in cfg["zones"].items():           # type: ignore[union-attr]
        col = (45, 95, 210) if z["color"] == "blue" else (210, 60, 50)
        x, y = z["center"]
        d.rectangle(box(x - .5, y - .5, x + .5, y + .5),
                    fill=tuple(int(c * 0.55) for c in col), outline=col, width=2)
        d.text(W2P((x, y)), name, fill=(240, 240, 240), anchor="mm")
    st = cfg["start"]                              # type: ignore[index]
    sx, sy = st["center"]
    d.rectangle(box(sx - st["size"][0] / 2, sy - st["size"][1] / 2,
                    sx + st["size"][0] / 2, sy + st["size"][1] / 2),
                outline=(235, 235, 235), width=2)
    d.text(W2P((sx, sy)), "起点", fill=(235, 235, 235), anchor="mm")
    d.text((12, 10), f"场地 {L}×{W} m，雷区 {mf['x_min']}~{mf['x_max']} m，"
                     f"封锁线 x={mf['blockade_lines_x']}", fill=(225, 228, 235))
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    img.save(path)


def update_routes(cfg: Dict[str, object], cfg_dir: str) -> List[str]:
    mf = cfg["minefield"]                          # type: ignore[index]
    W = float(cfg["arena"]["width_m"])             # type: ignore[index]
    y_lo, y_hi = 0.35, W - 0.35
    notes = []
    for rid, s_key, c_key in (("a", "stronghold_a", "camp_a"), ("b", "stronghold_b", "camp_b")):
        path = os.path.join(cfg_dir, f"route_{rid}.json")
        if not os.path.exists(path):
            continue
        with open(path, "r", encoding="utf-8") as f:
            route = json.load(f)
        sh_y = float(cfg["zones"][s_key]["center"][1])   # type: ignore[index]
        camp_y = float(cfg["zones"][c_key]["center"][1])  # type: ignore[index]
        route["cross_entry"] = {"x": round(float(mf["x_min"]) + 0.05, 2),
                                "y_hint": round(min(max(sh_y, y_lo), y_hi), 2)}
        route["cross_exit"] = {"x": round(float(mf["x_max"]) - 0.05, 2),
                               "y_hint": round(min(max(camp_y, y_lo), y_hi), 2)}
        route["corridor_y"] = [round(y_lo, 2), round(y_hi, 2)]
        with open(path, "w", encoding="utf-8") as f:
            json.dump(route, f, ensure_ascii=False, indent=2)
        notes.append(f"route_{rid}.json: 入口 ({route['cross_entry']['x']}, "
                     f"{route['cross_entry']['y_hint']}) -> 出口 "
                     f"({route['cross_exit']['x']}, {route['cross_exit']['y_hint']})")
    return notes


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="按现场尺寸生成/检查场地配置")
    ap.add_argument("--length", type=float, default=None, help="场地长（6~8m）")
    ap.add_argument("--width", type=float, default=None, help="场地宽（3~5m）")
    ap.add_argument("--config-dir", default=None)
    ap.add_argument("--apply", action="store_true", help="写回 field.json 与两条路线")
    ap.add_argument("--cones", default=None,
                    help="赛前公布的蓝色封锁线坐标，形如 \"2.05,1.2 2.0,2.4 3.5,0.8\""
                         "（x,y 用逗号，多个用空格分隔）；会自动归到最近的一条封锁线")
    ap.add_argument("--mines", default=None,
                    help="抽签得到的地雷坐标，形如 \"2.6,2.3 4.1,0.9\"")
    ap.add_argument("--figure", default="output/field_layout.png")
    args = ap.parse_args(argv)

    cfg_dir = args.config_dir or default_config_dir()
    field_path = os.path.join(cfg_dir, "field.json")
    if args.length is None and args.width is None:
        with open(field_path, "r", encoding="utf-8") as f:
            cfg = json.load(f)
        print(f"当前配置：{field_path}")
    else:
        with open(field_path, "r", encoding="utf-8") as f:
            cur = json.load(f)
        L = args.length or float(cur["arena"]["length_m"])
        W = args.width or float(cur["arena"]["width_m"])
        cfg = build_layout(L, W)
        print(f"按 {L}×{W} m 重新计算布局（雷区固定 4m，左右各 {round((L - 4) / 2, 2)}m）")

    issues = check_layout(cfg)
    z = cfg["zones"]                                # type: ignore[index]
    print("\n=== 关键位置 ===")
    print(f"  起点      {cfg['start']['center']}")   # type: ignore[index]
    print(f"  据点A(蓝) {z['stronghold_a']['center']}    营地A(蓝) {z['camp_a']['center']}")
    print(f"  据点B(红) {z['stronghold_b']['center']}    营地B(红) {z['camp_b']['center']}")
    print(f"  雷区 x    {cfg['minefield']['x_min']} ~ {cfg['minefield']['x_max']}")  # type: ignore[index]
    print(f"  封锁线 x  {cfg['minefield']['blockade_lines_x']}")  # type: ignore[index]

    if issues:
        print("\n⚠ 发现 %d 个问题：" % len(issues))
        for s in issues:
            print("   -", s)
    else:
        print("\n✅ 布局自洽（尺寸、雷区宽度、封锁线间隔、区域不越界/不重叠）")
    try:
        render(cfg, args.figure)
        print(f"布局图已生成：{args.figure}")
    except Exception as exc:  # noqa: BLE001
        print(f"（布局图生成失败：{exc}）")

    if args.apply:
        if issues:
            print("\n有上述问题，仍要写入请加 --force（先人工确认）")
            if "--force" not in (argv or sys.argv):
                return 2
        if args.cones or args.mines:
            lines_x = [float(v) for v in cfg["minefield"]["blockade_lines_x"]]  # type: ignore[index]
            if args.cones:
                groups: List[List[float]] = [[] for _ in lines_x]
                for rec in args.cones.replace(";", " ").split():
                    try:
                        x_str, y_str = rec.split(",")
                        x, y = float(x_str), float(y_str)
                    except ValueError:
                        print(f"  跳过无法解析的坐标：{rec}")
                        continue
                    k = min(range(len(lines_x)), key=lambda i: abs(lines_x[i] - x))
                    if abs(lines_x[k] - x) > 0.6:
                        print(f"  ⚠ 坐标 ({x},{y}) 离最近封锁线 x={lines_x[k]} 超过 0.6m，"
                              f"仍按最近线归入（请确认抄写正确）")
                    groups[k].append(round(y, 3))
                cfg["prior"]["blue_lines_y"] = [sorted(g) for g in groups]  # type: ignore[index]
                print(f"  已填入先验蓝锥：{ [len(g) for g in groups] } 个（每条线）")
            if args.mines:
                mines: List[List[float]] = []
                for rec in args.mines.replace(";", " ").split():
                    try:
                        xs, ys = rec.split(",")
                        mines.append([round(float(xs), 3), round(float(ys), 3)])
                    except ValueError:
                        print(f"  跳过无法解析的地雷坐标：{rec}")
                cfg["prior"]["red_mines"] = mines  # type: ignore[index]
                print(f"  已填入地雷点位：{len(mines)} 个")
        with open(field_path, "w", encoding="utf-8") as f:
            json.dump(cfg, f, ensure_ascii=False, indent=2)
        print(f"\n已写入 {field_path}")
        for note in update_routes(cfg, cfg_dir):
            print("  " + note)
        print("\n下一步：把赛前公布的锥桶 y 坐标填进 field.json 的 prior.blue_lines_y，"
              "并重跑 bash tools/selfcheck.sh 确认")
    else:
        print("\n（未改动任何文件；确认后加 --apply 写回）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
