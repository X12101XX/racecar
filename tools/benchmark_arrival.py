"""Compare arrival time at identical limits, without ROS or simulator-truth input.

Build a private library first, then run this against a checkout or frozen source:
  python3 tools/benchmark_arrival.py --native-library .runtime/bench/lib.so \
    --outdir output/arrival-current --profile measured_example
The output directory must be new. --compare takes another matrix.json.
"""
import argparse
import concurrent.futures
import hashlib
import json
import os
from pathlib import Path
import statistics
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


def digest_source(root):
    return {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
            for folder in ('racelib', 'sim') for p in sorted((root/folder).rglob('*'))
            if p.is_file() and p.suffix in ('.py', '.cpp')}


def digest_scene(root, case):
    """Include unknown configured obstacles in evaluation input identity."""
    if case['scene'] in ('demo', 'random'):
        files = [root/'sim/world.py', root/'sim/scenarios.py']
    else:
        files = [root/'profiles'/(case['scene']+'.json'), root/'tools/profile_config.py']
    values = {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    return hashlib.sha256(json.dumps(values, sort_keys=True).encode()).hexdigest()


def worker(root, library, case, target, max_time):
    # Import only the selected source tree, and select its library before any
    # planner imports. Neither the source tree nor saved profiles are written.
    sys.path.insert(0, str(root))
    os.environ['CROSSFIRE_NATIVE_LIBRARY'] = str(library)
    os.environ['CROSSFIRE_PLANNER_ENGINE'] = 'native'
    for name in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
        os.environ[name] = '1'
    from collections import Counter
    import math
    import racelib
    from racelib.config import load_all
    from sim import runner
    from sim.io_sim import SimIO
    from sim.scenarios import demo_cones
    assert Path(racelib.__file__).resolve().is_relative_to(root)
    captures, phases, stops = [], Counter(), Counter()
    previous = ['', False]

    class ObservedIO(SimIO):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            captures.append(self)

    original = runner.AckermannMission

    class ObservedMission(original):
        def step(self, dt):
            phase = self.state
            result = super().step(dt)
            phases[phase] += dt
            moving = abs(result[0]) >= .025
            if previous[0] == phase and previous[1] and not moving:
                stops[phase] += 1
            previous[:] = [phase, moving]
            return result

    # Observers read time/state/commands only; never provide truth to a mission.
    runner.SimIO, runner.AckermannMission = ObservedIO, ObservedMission
    cfgs = load_all(root/'config', case['route'])
    scene = case['scene']
    if scene not in ('demo', 'random'):
        from tools.profile_config import compile_profile
        from sim.world import SimCone
        destination = target.parent/('config_'+case['name'])
        compile_profile(json.loads((root/'profiles'/(scene+'.json')).read_text()), destination)
        cfgs = load_all(destination, case['route'])
        cones = [SimCone(float(c['x']), float(c['y']), 'unknown',
            radius_base=float(c.get('radius', .115)),
            radius_lidar=min(.085, float(c.get('radius', .115))*.74))
            for c in json.loads((destination/'sim_cones.json').read_text())]
    else:
        cones = demo_cones() if scene == 'demo' else None
    cfgs['vision'].data.setdefault('camera', {}).update(width=320, height=240)
    config_digest = hashlib.sha256(json.dumps({k: v.data for k, v in cfgs.items()},
        ensure_ascii=False, sort_keys=True).encode()).hexdigest()
    config = runner.RunConfig(route=case['route'], seed=case['seed'], video=False,
        verbose=False, max_time_s=max_time, use_card=bool(case.get('card')),
        card_color=case.get('card'))
    result = runner.run_mission(cfgs, config, cones=cones)
    history = captures[0].cmd_history
    forward = reverse = 0.
    for a, b in zip(history, history[1:]):
        distance = (b[0]-a[0])*a[1]
        if distance < 0: reverse -= distance
        else: forward += distance
    row = dict(case=case, success=result.success, summary=result.summary,
        input_sha256=config_digest, scene_input_sha256=digest_scene(root, case),
        phases=dict(phases), command_stop_episodes=dict(stops),
        path_m=sum(math.dist(a[1:3], b[1:3]) for a, b in zip(result.trajectory, result.trajectory[1:])),
        command_forward_m=forward, command_reverse_m=reverse, events=result.events)
    target.write_text(json.dumps(row, ensure_ascii=False, indent=2)+'\n')
    return row


def comparison(before, after):
    a = {r['case']['name']: r for r in before}
    b = {r['case']['name']: r for r in after}
    common = sorted(a.keys() & b.keys())
    paired = [k for k in common if a[k]['success'] and b[k]['success']]
    def mean(rows, field):
        return round(statistics.mean(rows[k]['summary'][field] for k in paired), 4) if paired else None
    regressions = [k for k in common if a[k]['success'] and not b[k]['success']]
    inputs_same = bool(common) and all(a[k]['case'] == b[k]['case'] and
        a[k].get('input_sha256') is not None and
        a[k].get('scene_input_sha256') is not None and
        a[k].get('input_sha256') == b[k].get('input_sha256') and
        a[k].get('scene_input_sha256') == b[k].get('scene_input_sha256') for k in common)
    limits_same = all(a[k]['summary']['motion_settings'] == b[k]['summary']['motion_settings'] for k in common)
    result = dict(matched_cases=len(common), same_case_set=a.keys() == b.keys(),
        identical_inputs=inputs_same, identical_motion_limits=limits_same,
        success_before=sum(r['success'] for r in before), success_after=sum(r['success'] for r in after),
        new_failures=regressions, new_successes=[k for k in common if not a[k]['success'] and b[k]['success']],
        paired_successes=len(paired), mean_full_time_before_s=mean(a, 'sim_time_s'),
        mean_full_time_after_s=mean(b, 'sim_time_s'),
        mean_competition_time_before_s=mean(a, 'run_time_s'), mean_competition_time_after_s=mean(b, 'run_time_s'),
        contact_runs_after=sum(bool(r['summary']['cone_contacts'] or r['summary']['wall_contacts']) for r in after),
        per_case=[dict(name=k, success_before=a[k]['success'], success_after=b[k]['success'],
            before_s=a[k]['summary']['sim_time_s'], after_s=b[k]['summary']['sim_time_s']) for k in common])
    result['passed'] = bool(paired and result['same_case_set'] and inputs_same and limits_same
        and not regressions and result['contact_runs_after'] == 0
        and result['mean_full_time_after_s'] < result['mean_full_time_before_s'])
    return result


def compare_records(before, after):
    result = comparison(before['runs'], after['runs'])
    result['identical_time_budget'] = before.get('max_time_s') == after.get('max_time_s')
    physics = {k: v for k, v in before.get('source_sha256', {}).items() if k.startswith('sim/')}
    result['identical_simulator'] = bool(physics) and physics == {
        k: v for k, v in after.get('source_sha256', {}).items() if k.startswith('sim/')}
    result['passed'] = result['passed'] and result['identical_time_budget'] and result['identical_simulator']
    return result


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--source-root', type=Path, default=ROOT)
    ap.add_argument('--native-library', type=Path, required=True)
    ap.add_argument('--outdir', type=Path, required=True)
    ap.add_argument('--seeds', type=int, default=40, help='random seeds per route; demo A/B always included')
    ap.add_argument('--seed-start', type=int, default=0)
    ap.add_argument('--jobs', type=int, default=4)
    ap.add_argument('--max-time', type=float, default=90.)
    ap.add_argument('--profile', action='append', default=[])
    ap.add_argument('--cards', action='store_true')
    ap.add_argument('--compare', type=Path)
    ap.add_argument('--worker-case', help=argparse.SUPPRESS)
    args = ap.parse_args()
    root, library, out = args.source_root.resolve(), args.native_library.resolve(), args.outdir.absolute()
    if args.worker_case:
        row = worker(root, library, json.loads(args.worker_case), out, args.max_time)
        print(row['case']['name'], row['success'], row['summary'].get('run_time_s'), flush=True)
        return 0
    if not library.is_file(): ap.error('compile the selected native library first')
    if args.seeds < 0 or args.jobs < 1: ap.error('seeds must be >=0; jobs must be >=1')
    if out.exists() or out.is_symlink(): ap.error('refusing to overwrite existing results: '+str(out))
    out.mkdir(parents=True, exist_ok=False)
    initial_digest = digest_source(root)
    library_digest = hashlib.sha256(library.read_bytes()).hexdigest()
    cases = []
    for route in ('A', 'B'):
        for scene in ['random', 'demo', *args.profile]:
            for seed in (range(args.seed_start, args.seed_start+args.seeds) if scene == 'random' else [3]):
                cases.append(dict(name=f'{scene}_{route}_{seed}', route=route, scene=scene, seed=seed))
        if args.cards:
            color = 'red' if route == 'A' else 'blue'
            cases.append(dict(name=f'card_{route}_{color}', route=route, scene='demo', seed=3, card=color))
    if len({c['name'] for c in cases}) != len(cases): ap.error('duplicate profiles/cases')
    initial_scenes = {c['name']: digest_scene(root, c) for c in cases}
    def run(case):
        target = out/(case['name']+'.json')
        command = [sys.executable, '-B', str(Path(__file__).resolve()), '--source-root', str(root),
            '--native-library', str(library), '--outdir', str(target), '--max-time', str(args.max_time),
            '--worker-case', json.dumps(case)]
        proc = subprocess.run(command, capture_output=True, text=True)
        if proc.returncode: raise RuntimeError(str(command)+'\n'+proc.stderr)
        print(proc.stdout.strip(), flush=True)
        return json.loads(target.read_text())
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.jobs) as pool:
        rows = list(pool.map(run, cases))
    if (initial_digest != digest_source(root) or
        initial_scenes != {c['name']: digest_scene(root, c) for c in cases} or
        library_digest != hashlib.sha256(library.read_bytes()).hexdigest()):
        raise RuntimeError('source or library changed during benchmark; discard mixed results')
    record = dict(source_root=str(root), source_sha256=initial_digest,
        native_library_sha256=library_digest, max_time_s=args.max_time, runs=rows,
        scope='in-process sensor simulation; not ROS DDS or vehicle validation')
    if args.compare:
        other = json.loads(args.compare.read_text())
        record['comparison'] = compare_records(other, record)
    (out/'matrix.json').write_text(json.dumps(record, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(record.get('comparison', dict(runs=len(rows), success=sum(r['success'] for r in rows))),
                     ensure_ascii=False, indent=2))
    return 0


if __name__ == '__main__':
    sys.exit(main())
