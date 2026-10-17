"""批量随机场地测试：给定若干随机种子跑完整任务，统计成功率与用时分布。

    python3 -m sim.stress --route A --seeds 20 --jobs 6
    python3 -m sim.stress --route both --seeds 20 --prior blue
"""

from __future__ import annotations

import argparse
import json
import multiprocessing as mp
import os

# 必须在 import numpy 之前限制 BLAS 线程：多线程 BLAS + fork 会死锁
for _k in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS",
           "NUMEXPR_NUM_THREADS", "VECLIB_MAXIMUM_THREADS"):
    os.environ.setdefault(_k, "1")

import statistics as stats
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from typing import Any, Dict, List

from racelib.config import load_all

from .runner import RunConfig, run_mission


def _one(args) -> Dict[str, Any]:
    route, seed, prior, max_time, quality = args
    cfgs = load_all(route=route)
    cfg = RunConfig(route=route, seed=seed, prior=prior, video=False,
                    outdir=None, verbose=False, max_time_s=max_time,
                    camera_quality=quality)
    res = run_mission(cfgs, cfg)
    s = res.summary
    return {
        "route": route,
        "seed": seed,
        "prior": prior,
        "final_state": s.get("final_state"),
        "success": bool(res.success),
        "reached_camp": bool(s.get("reached_camp")),
        "run_time_s": s.get("run_time_s"),
        "effective_time_s": s.get("effective_time_s"),
        "sim_time_s": s.get("sim_time_s"),
        "blue_contacts": s.get("blue_contacts"),
        "red_contacts": s.get("red_contacts"),
        "blue_cone_hits": s.get("blue_cone_hits"),
        "red_cone_hits": s.get("red_cone_hits"),
        "wall_contacts": s.get("wall_contacts"),
        "min_clearance": s.get("min_cone_clearance_m"),
        "recall": (s.get("perception") or {}).get("recall"),
        "precision": (s.get("perception") or {}).get("precision"),
        "color_acc": (s.get("perception") or {}).get("color_accuracy"),
        "rtf": s.get("real_time_factor"),
    }


def summarize(rows: List[Dict[str, Any]]) -> Dict[str, Any]:
    ok = [r for r in rows if r["success"]]
    times = [r["effective_time_s"] for r in ok if r["effective_time_s"] is not None]
    return {
        "runs": len(rows),
        "success": len(ok),
        "success_rate": round(len(ok) / max(1, len(rows)), 3),
        "effective_time_mean_s": round(stats.mean(times), 2) if times else None,
        "effective_time_min_s": round(min(times), 2) if times else None,
        "effective_time_max_s": round(max(times), 2) if times else None,
        "red_hit_runs": sum(1 for r in rows if (r["red_cone_hits"] or 0) > 0),
        "blue_hit_runs": sum(1 for r in rows if (r["blue_cone_hits"] or 0) > 0),
        "wall_contact_runs": sum(1 for r in rows if (r["wall_contacts"] or 0) > 0),
        "timeout_runs": sum(1 for r in rows if r["final_state"] not in ("DONE",)),
        "recall_mean": round(stats.mean([r["recall"] for r in rows if r["recall"]]), 3)
        if any(r["recall"] for r in rows) else None,
        "precision_mean": round(stats.mean([r["precision"] for r in rows if r["precision"]]), 3)
        if any(r["precision"] for r in rows) else None,
        "rtf_mean": round(stats.mean([r["rtf"] for r in rows if r["rtf"]]), 2)
        if any(r["rtf"] for r in rows) else None,
    }


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="批量随机场地测试")
    ap.add_argument("--route", default="A", choices=["A", "B", "both"])
    ap.add_argument("--seeds", type=int, default=20)
    ap.add_argument("--seed-start", type=int, default=0)
    ap.add_argument("--prior", default="none", choices=["none", "blue", "all"])
    ap.add_argument("--jobs", type=int, default=6)
    ap.add_argument("--max-time", type=float, default=90.0)
    ap.add_argument("--camera-quality", type=float, default=1.0)
    ap.add_argument("--outdir", default="output/stress")
    args = ap.parse_args(argv)

    routes = ["A", "B"] if args.route == "both" else [args.route]
    jobs = []
    for route in routes:
        for k in range(args.seeds):
            jobs.append((route, args.seed_start + k, args.prior, args.max_time,
                         args.camera_quality))

    try:                      # 沙箱里 forkserver 可能被限制，优先用 fork
        mp.set_start_method("fork", force=True)
    except Exception:  # noqa: BLE001
        pass
    t0 = time.time()
    rows: List[Dict[str, Any]] = []
    with ProcessPoolExecutor(max_workers=args.jobs) as ex:
        for r in ex.map(_one, jobs):
            rows.append(r)
            flag = "OK " if r["success"] else "失败"
            print(f"[{flag}] route={r['route']} seed={r['seed']:>3} "
                  f"state={r['final_state']:<6} 有效用时={r['effective_time_s']} "
                  f"蓝撞={r['blue_cone_hits']} 红撞={r['red_cone_hits']} "
                  f"碰墙={r['wall_contacts']} 最小间隙={r['min_clearance']} "
                  f"召回={r['recall']}", flush=True)

    summary = summarize(rows)
    os.makedirs(args.outdir, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    path = os.path.join(args.outdir, f"stress_{args.route}_{args.prior}_{stamp}.json")
    with open(path, "w", encoding="utf-8") as f:
        json.dump({"config": vars(args), "summary": summary, "runs": rows},
                  f, ensure_ascii=False, indent=2)
    print("\n=== 汇总 ===")
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    print(f"总耗时 {time.time() - t0:.1f}s，明细已保存：{path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
