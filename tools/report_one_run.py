"""把一个运行目录的关键结果打印成一行（自检/演示脚本用）。"""

from __future__ import annotations

import json
import os
import sys


def main(argv=None) -> int:
    args = list(sys.argv[1:] if argv is None else argv)
    path = args[0] if args else "output"
    summary = os.path.join(path, "summary.json")
    if not os.path.exists(summary):
        print(f"没有找到 {summary}（仿真可能失败了，看同目录的 .log）")
        return 1
    with open(summary, "r", encoding="utf-8") as f:
        s = json.load(f)
    p = s.get("perception") or {}
    print(f"  结果: {s.get('final_state')}  完赛 {s.get('run_time_s')}s  "
          f"有效 {s.get('effective_time_s')}s  蓝撞 {s.get('blue_cone_hits')}  "
          f"红撞 {s.get('red_cone_hits')}  碰墙 {s.get('wall_contacts')}")
    print(f"  感知: 召回 {p.get('recall')}  精度 {p.get('precision')}  "
          f"（真值锥桶 {p.get('true_cones')} 个，识别 {p.get('tracks')} 个）")
    print(f"  仿真 {s.get('sim_time_s')}s / 实际 {s.get('wall_time_s')}s "
          f"（实时率 {s.get('real_time_factor')}）")
    return 0 if s.get("final_state") == "DONE" else 1


if __name__ == "__main__":
    raise SystemExit(main())
