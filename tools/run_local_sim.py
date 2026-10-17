"""Local simulation entry. Algorithm loop by default; optional existing native ROS."""
import argparse
import datetime
import json
import os
import re
from pathlib import Path
import shutil
import subprocess
import sys
import uuid

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'tools')]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--ros', action='store_true')
    ap.add_argument('--profile', help='保存的场地名称或导出的 JSON')
    ap.add_argument('--config-dir', type=Path)
    ap.add_argument('--route', choices=['A', 'B'], default='A')
    ap.add_argument('--scenario', choices=['demo', 'random', 'configured'], default='demo')
    ap.add_argument('--seed', type=int, default=3)
    ap.add_argument('--use-card', action='store_true')
    ap.add_argument('--color', action='store_true')
    ap.add_argument('--card-color', choices=['blue', 'red'])
    ap.add_argument('--ai-backend', choices=['auto', 'cpu', 'fast_color', 'onnx', 'npu'], default='')
    ap.add_argument('--ai-model', type=Path)
    ap.add_argument('--planner-engine', choices=['auto', 'native', 'python'], default='auto')
    ap.add_argument('--motion-preset', choices=['conservative', 'standard', 'aggressive'])
    ap.add_argument('--outdir', type=Path)
    ap.add_argument('--max-time', type=float, default=180)
    ap.add_argument('--no-video', action='store_true')
    ap.add_argument('--fps', type=float, default=4.)
    ap.add_argument('--fault', choices=['none', 'mission_kill', 'scan_drop', 'estop', 'competing_publisher'], default='none')
    args = ap.parse_args(argv)
    if args.card_color and not args.use_card:
        ap.error('--card-color 需与 --use-card 一起使用')
    if args.profile and args.config_dir:
        ap.error('--profile 与 --config-dir 不要同时使用')
    if not args.ros and args.fault != 'none':
        ap.error('独立网关故障检查需要 ./crossfire ros-sim')
    if args.ros and args.no_video:
        ap.error('ROS 结果目前要求完整视频；无视频快速检查使用 ./crossfire sim --no-video')
    stamp = datetime.datetime.now().strftime('%Y%m%d-%H%M%S-%f')
    out = (args.outdir or ROOT / 'output' / f'{"native_ros" if args.ros else "local"}_{args.route}_{stamp}').absolute()
    if not out.resolve().is_relative_to((ROOT / 'output').resolve()):
        ap.error('--outdir 必须位于本项目 output/ 下，避免写入其他代码目录')
    if out.exists() or out.is_symlink():
        ap.error('拒绝覆盖已有结果目录: ' + str(out))
    try:
        if not args.ros: import PIL.Image
        from racelib.config import load_all
        from racelib.settings import MOTION_PRESETS, validate_field
        from profile_config import compile_profile
    except ImportError as exc:
        ap.exit(2, f'缺少本机仿真依赖: {exc}；需要已有 NumPy/Pillow，可使用私有 venv，不能全局升级车端环境。\n')
    staging = ROOT / '.runtime/local-sim' / stamp
    config = staging / 'config'
    staging.mkdir(parents=True, exist_ok=False)
    if args.profile:
        source = Path(args.profile).expanduser()
        if not source.is_file(): source = ROOT / 'profiles' / (args.profile + '.json')
        compile_profile(json.loads(source.read_text()), config)
        args.scenario = 'configured'
        print('源场地：' + str(source.resolve()) + '（只读，使用独立配置快照）', flush=True)
    else:
        shutil.copytree(args.config_dir or ROOT / 'config', config)
    cfgs = load_all(config, args.route)
    if args.motion_preset:
        cfgs['plan'].data['ackermann'] = dict(MOTION_PRESETS[args.motion_preset])
    cfgs['plan'].data.setdefault('mission', {})['monochrome'] = not (args.use_card or args.color)
    if args.ai_backend: cfgs['vision'].data.setdefault('ai', {})['backend'] = args.ai_backend
    model = args.ai_model or cfgs['vision'].get('ai.model_path', '')
    if model:
        path = Path(model).expanduser()
        cfgs['vision'].data.setdefault('ai', {})['model_path'] = str(path.absolute() if path.is_absolute() or args.ai_model else (ROOT / path).absolute())
    # Small synthetic images reduce simulation cost; real-car camera config stays untouched.
    if not args.ros: cfgs['vision'].data.setdefault('camera', {}).update(width=320, height=240)
    for name in ('plan', 'vision'):
        (config / (name + '.json')).write_text(json.dumps(cfgs[name].data, ensure_ascii=False, indent=2))
    errors, _ = validate_field(cfgs['field'].data, cfgs['robot'].get('geometry'))
    if errors: ap.error('; '.join(errors))
    os.environ['CROSSFIRE_PLANNER_ENGINE'] = args.planner_engine
    if args.ros:
        env = os.environ.copy()
        # Every simulated topic is remapped as well as using a separate DDS domain.
        env.update(ROS_DOMAIN_ID='87', ROS_LOCALHOST_ONLY='1',
                   CROSSFIRE_SIM_NAMESPACE='/crossfire_sim_' + uuid.uuid4().hex,
                   ROS_HOME=str(staging / 'ros'), ROS_LOG_DIR=str(staging / 'ros/log'))
        command = [sys.executable, '-B', '-m', 'sim.ros_demo', '--config-dir', str(config),
                   '--outdir', str(out), '--route', args.route, '--scenario', args.scenario,
                   '--seed', str(args.seed), '--max-time', str(args.max_time),
                   '--planner-engine', args.planner_engine, '--fault', args.fault]
        if args.use_card: command.append('--use-card')
        if args.color: command.append('--color')
        if args.card_color: command += ['--card-color', args.card_color]
        code = subprocess.run(command, env=env, cwd=ROOT).returncode
    else:
        env = os.environ.copy()
        library = ROOT / '.runtime/local/libhybrid_astar.so'
        env['CROSSFIRE_NATIVE_LIBRARY'] = str(library)
        # Compile before importing the planner, which loads the library only once.
        if args.planner_engine != 'python':
            subprocess.run(['bash', str(ROOT / 'tools/build_native.sh'), str(library)], check=True)
        env['OMP_NUM_THREADS'] = env['OPENBLAS_NUM_THREADS'] = '1'
        command = [sys.executable, '-B', '-m', 'sim.cli', '--config-dir', str(config),
                   '--scenario', args.scenario, '--route', args.route, '--seed', str(args.seed),
                   '--outdir', str(out), '--max-time', str(args.max_time), '--fps', str(args.fps),
                   '--planner-engine', args.planner_engine]
        if args.use_card: command.append('--use-card')
        if args.color: command.append('--color')
        if args.card_color: command += ['--card-color', args.card_color]
        if args.no_video: command.append('--no-video')
        print('本机算法仿真：无需 Docker/ROS；完整 DDS 和网关验证使用 ./crossfire ros-sim。', flush=True)
        code = subprocess.run(command, env=env, cwd=ROOT).returncode
        if (out / 'summary.json').is_file():
            shutil.copytree(config, out / 'config')
            data = json.loads((out / 'summary.json').read_text())
            data.update(transport='in-process sensors (no ROS DDS)', simulation_backend='local',
                        success=data['final_state'] == 'DONE' and data['reached_camp'] and
                        data['cone_contacts'] == data['wall_contacts'] == 0)
            (out / 'summary.json').write_text(json.dumps(data, ensure_ascii=False, indent=2))
            (out / 'completed.json').write_text(json.dumps(dict(video_finalized=(out / 'video.mp4').is_file())))
    if (out / 'completed.json').is_file():
        from sim_results import show_result
        show_result('/output/' + str(out.relative_to(ROOT / 'output')),
                    profile_name=args.profile if args.profile and re.fullmatch(r'[A-Za-z0-9_-]+', args.profile) else '')
    return code


if __name__ == '__main__':
    sys.exit(main())
