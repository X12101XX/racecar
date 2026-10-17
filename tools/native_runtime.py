"""Private native runtime and unified entry. Uses existing dependencies; never invokes pip/apt/colcon."""
import argparse
import fcntl
import hashlib
import importlib
import json
import os
from pathlib import Path, PurePosixPath
import platform
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
RUNTIME = ROOT / '.runtime/car'


def verify_payload(root=ROOT):
    manifest = root / 'MANIFEST.json'
    if not manifest.is_file():
        return  # A development checkout has no release manifest.
    for name, expected in json.loads(manifest.read_text()).items():
        relative = PurePosixPath(name)
        if relative.is_absolute() or '..' in relative.parts:
            raise ValueError('非法校验路径: ' + name)
        path = root / name
        if path.is_symlink() or not path.resolve().is_relative_to(root.resolve()):
            raise ValueError('拒绝包外链接: ' + name)
        if relative.parts[0] in ('config', 'profiles'):
            continue  # User-editable data; checked again by the supervisor.
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise ValueError('文件校验失败: ' + name)


def environment():
    env = os.environ.copy()
    for name in ('BASH_ENV', 'ENV', 'PYTHONHOME'):
        env.pop(name, None)
    env['PYTHONDONTWRITEBYTECODE'] = '1'
    env['PYTHONNOUSERSITE'] = '1'
    return env


def command(setup, python, *args):
    return ['bash', str(ROOT / 'tools/native_env.sh'), str(setup), str(python), *map(str, args)]


def probe(setup, python):
    if not setup.is_file():
        raise ValueError('未找到已有 ROS2 Humble: ' + str(setup) +
                         '\n未安装系统包。可用 --ros-setup 指定；本机仿真可直接 ./crossfire sim。')
    proc = subprocess.run(command(setup, python, __file__, '_probe'), env=environment(),
                          capture_output=True, text=True)
    if proc.returncode:
        raise ValueError('现有 ROS/Python 依赖检查失败；没有安装或升级任何包。\n' + proc.stderr)
    info = json.loads(proc.stdout)
    if info['ros_distro'] != 'humble':
        raise ValueError('当前仅验证 ROS2 Humble，检测到: ' + info['ros_distro'])
    return info


def internal_probe():
    modules = {name: importlib.import_module(name) for name in
               ('rclpy', 'numpy', 'cv2', 'ament_index_python', 'sensor_msgs.msg',
                'nav_msgs.msg', 'geometry_msgs.msg', 'std_msgs.msg')}
    # Import only: no rclpy.init, devices, model sessions, nodes, or publishers.
    print(json.dumps(dict(python=sys.executable, python_version=platform.python_version(),
        architecture=platform.machine(), ros_distro=os.environ.get('ROS_DISTRO', ''),
        numpy=modules['numpy'].__version__, opencv=modules['cv2'].__version__,
        rclpy_origin=modules['rclpy'].__file__)))


def prepare(setup, python):
    if os.geteuid() == 0:
        raise ValueError('请以普通用户安装，不要 sudo')
    verify_payload()
    setup = setup.expanduser().absolute()
    python = shutil.which(str(python)) or str(python)
    info = probe(setup, python)  # Missing dependencies leave no runtime files.
    RUNTIME.mkdir(parents=True, exist_ok=True)
    with (RUNTIME / 'prepare.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        for name in ('tmp', 'cache'):
            (RUNTIME / name).mkdir(exist_ok=True)
        env = environment()
        env.update(TMPDIR=str(RUNTIME / 'tmp'), XDG_CACHE_HOME=str(RUNTIME / 'cache'),
                   CCACHE_DIR=str(RUNTIME / 'cache/ccache'))
        source = ROOT / 'racelib/native/hybrid_astar_core.cpp'
        source_hash = hashlib.sha256(source.read_bytes()).hexdigest()
        library = RUNTIME / 'libhybrid_astar.so'
        old = json.loads((RUNTIME / 'runtime.json').read_text()) if (RUNTIME / 'runtime.json').is_file() else {}
        if shutil.which(os.environ.get('CXX', 'g++')):
            subprocess.run(['bash', str(ROOT / 'tools/build_native.sh'), str(library)], env=env, check=True)
        elif old.get('native_source_sha256') != source_hash or old.get('architecture') != info['architecture']:
            # Never load an old library for changed source.
            library.unlink(missing_ok=True)
        available = False
        if library.is_file():
            check = subprocess.run(command(setup, info['python'], '-c',
                'from racelib.native_planner import is_native_available; '
                'raise SystemExit(0 if is_native_available() else 1)'), env=env)
            available = check.returncode == 0
            if not available:
                raise ValueError('当前架构 C++ 库无法加载；未写入就绪标记')
        info.update(ros_setup=str(setup), native_available=available,
                    native_source_sha256=source_hash, mode='native_source',
                    native_library_sha256=hashlib.sha256(library.read_bytes()).hexdigest() if available else None)
        temporary = RUNTIME / 'runtime.json.tmp'
        temporary.write_text(json.dumps(info, ensure_ascii=False, indent=2) + '\n')
        temporary.replace(RUNTIME / 'runtime.json')
    print('安装完成：无需 Docker / colcon / pip；未启动节点或接管底盘。')
    print('C++ 规划：' + ('已启用' if available else '未找到编译器，auto 将使用 Python'))
    print('只读自检：' + str(ROOT / 'crossfire') + ' run A --preflight')
    return 0


def state():
    verify_payload()
    path = RUNTIME / 'runtime.json'
    if not path.is_file():
        raise ValueError('尚未准备原生 ROS 环境，请先 ./crossfire install')
    data = json.loads(path.read_text())
    if data['architecture'] != platform.machine():
        raise ValueError('机器架构已变化，请在当前机器重新 ./crossfire install')
    library=RUNTIME/'libhybrid_astar.so'
    if data['native_available'] and (not library.is_file() or hashlib.sha256(library.read_bytes()).hexdigest()!=data.get('native_library_sha256')):
        raise ValueError('私有 C++ 库缺失或校验失败，请重新 ./crossfire install')
    if data['native_source_sha256'] != hashlib.sha256((ROOT / 'racelib/native/hybrid_astar_core.cpp').read_bytes()).hexdigest():
        raise ValueError('规划源码已更新，请重新 ./crossfire install')
    return data


def main(argv=None):
    args = list(sys.argv[1:] if argv is None else argv)
    if args == ['_probe']:
        internal_probe()
        return 0
    ap = argparse.ArgumentParser(
        prog='crossfire',
        description='统一入口：按当前包内容提供安装、自检、配置界面、测试、'
                    '只读实车运行与本机/ROS 仿真；不自动安装或升级系统包。')
    actions = ['install', 'doctor', 'run']
    if (ROOT / 'sim').is_dir():
        actions += ['sim', 'ros-sim']
    if (ROOT / 'tools/selfcheck.sh').is_file():
        actions += ['test']
    if (ROOT / 'tools/config_editor.py').is_file():
        actions += ['ui']
    ap.add_argument('action', choices=actions)
    ap.add_argument('--ros-setup', type=Path, default=Path('/opt/ros/humble/setup.bash'))
    ap.add_argument('--python', default='/usr/bin/python3')
    child_mode = bool(args and args[0] in ('run', 'sim', 'ros-sim', 'test', 'ui'))
    opts, rest = ap.parse_known_args(args[:1] if child_mode else args)
    if child_mode: rest = args[1:]
    try:
        if opts.action == 'install':
            if rest:
                ap.error('未知安装参数: ' + ' '.join(rest))
            return prepare(opts.ros_setup, opts.python)
        if opts.action == 'test':
            if os.geteuid() == 0:
                raise ValueError('请以普通用户运行测试，不要 sudo')
            os.execvpe('bash', ['bash', str(ROOT / 'tools/selfcheck.sh'), *rest],
                       environment())
        if opts.action == 'ui':
            os.execvpe(sys.executable,
                       [sys.executable, '-B', str(ROOT / 'tools/config_editor.py'), *rest],
                       environment())
        if opts.action == 'sim':
            os.execve(sys.executable, [sys.executable, '-B', str(ROOT / 'tools/run_local_sim.py'), *rest], environment())
        if opts.action in ('run', 'ros-sim') and any(x in ('-h', '--help') for x in rest):
            entry = ROOT / ('tools/car_supervisor.py' if opts.action == 'run' else 'tools/run_local_sim.py')
            os.execve(sys.executable, [sys.executable, '-B', str(entry), *rest], environment())
        if os.geteuid() == 0:
            raise ValueError('请以普通用户运行，不要 sudo')
        data = state()
        if opts.action == 'doctor':
            if rest: ap.error('doctor 不接受额外参数')
            info = probe(Path(data['ros_setup']), data['python'])
            info.update(native_available=data['native_available'], private_directory=str(ROOT),
                        starts_nodes=False, installs_system_packages=False)
            print(json.dumps(info, ensure_ascii=False, indent=2))
            return 0
        entry = ROOT / ('tools/car_supervisor.py' if opts.action == 'run' else 'tools/run_local_sim.py')
        forwarded = rest if opts.action == 'run' else ['--ros', *rest]
        os.execvpe('bash', command(data['ros_setup'], data['python'], entry, *forwarded), environment())
    except (ValueError, OSError, subprocess.CalledProcessError) as exc:
        print(str(exc), file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
