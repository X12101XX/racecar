"""命令行入口：

    python3 -m sim.cli --route A --seed 1 --outdir output/routeA_seed1
    python3 -m sim.cli --route B --prior blue --video
"""

from __future__ import annotations

import argparse
import json
import os
import sys

from racelib.config import load_all

from .runner import RunConfig, run_mission


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="穿越火线任务仿真")
    ap.add_argument("--scenario",choices=["demo","random","configured"],default="demo")
    ap.add_argument("--route", default="A", choices=["A", "B"])
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--prior", default="none", choices=["none", "blue", "all"],
                    help="是否注入赛前公布的锥桶先验位置")
    ap.add_argument("--use-card", action="store_true", help="启用色卡识别流程")
    ap.add_argument("--outdir", default=None)
    ap.add_argument("--no-video", action="store_true")
    ap.add_argument("--fps", type=float, default=6.0)
    ap.add_argument("--max-time", type=float, default=180.0)
    ap.add_argument("--camera-quality", type=float, default=1.0)
    ap.add_argument("--config-dir", default=None)
    args = ap.parse_args(argv)

    cfgs = load_all(config_dir=args.config_dir, route=args.route)
    cfg = RunConfig(route=args.route, seed=args.seed, prior=args.prior,
                    use_card=args.use_card, outdir=args.outdir,
                    video=not args.no_video, video_fps=args.fps,
                    max_time_s=args.max_time, camera_quality=args.camera_quality)
    from .scenarios import demo_cones
    if args.scenario=="configured":
        from pathlib import Path
        from .world import SimCone
        source=Path(args.config_dir or "config")/"sim_cones.json"
        configured=[SimCone(float(c["x"]),float(c["y"]),"unknown",radius_base=float(c.get("radius",.115)),radius_lidar=min(.085,float(c.get("radius",.115))*.74)) for c in json.loads(source.read_text())]
    res = run_mission(cfgs, cfg, cones=configured if args.scenario=="configured" else demo_cones() if args.scenario=="demo" else None)
    print(json.dumps(res.summary, ensure_ascii=False, indent=2))
    print(f"[sim] 仿真耗时 {res.wall_time_s:.1f}s；产物目录：{res.outdir}")
    return 0 if res.success else 1


if __name__ == "__main__":
    sys.exit(main())
