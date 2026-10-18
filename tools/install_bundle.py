"""绿色解压器：默认只校验并解压到全新目录，不检查 ROS、不编译、不启动程序。"""
import argparse
import datetime
import hashlib
import io
import json
import os
from pathlib import Path, PurePosixPath
import shlex
import subprocess
import sys
import tarfile


def extract_payload(blob, target):
    if target.exists() or target.is_symlink():
        raise ValueError('拒绝覆盖已有目录: ' + str(target))
    with tarfile.open(fileobj=io.BytesIO(blob), mode='r:gz') as archive:
        members = archive.getmembers()
        files = {}
        for member in members:
            path = PurePosixPath(member.name)
            if (path.is_absolute() or '..' in path.parts or not path.parts or
                path.parts[0] != 'crossfire-car-v2' or
                not (member.isdir() or member.isfile())):
                raise ValueError('压缩包包含非法路径或链接: ' + member.name)
            if member.isfile():
                name = str(PurePosixPath(*path.parts[1:]))
                if name in files: raise ValueError('压缩包包含重复文件: ' + name)
                files[name] = member
        if 'MANIFEST.json' not in files: raise ValueError('缺少 MANIFEST.json')
        manifest = json.load(archive.extractfile(files['MANIFEST.json']))
        if set(files) != set(manifest) | {'MANIFEST.json'}:
            raise ValueError('压缩包文件与清单不一致')
        for name, expected in manifest.items():
            if hashlib.sha256(archive.extractfile(files[name]).read()).hexdigest() != expected:
                raise ValueError('文件校验失败: ' + name)
        # Exclusive mkdir: another installer or an existing program cannot be replaced.
        target.mkdir(parents=True, exist_ok=False, mode=0o700)
        for name, member in files.items():
            dest = target / name
            dest.parent.mkdir(parents=True, exist_ok=True)
            with dest.open('xb') as output:
                output.write(archive.extractfile(member).read())
            dest.chmod(member.mode & 0o755)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('bundle', type=Path)
    ap.add_argument('--target', type=Path, help='自选全新目录；已有文件、目录或链接一律拒绝覆盖')
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument('--extract-only', action='store_true', help='兼容旧命令；现在默认就是只解压')
    mode.add_argument('--prepare', action='store_true', help='解压后额外准备本目录内的 ROS/C++ 运行环境')
    ap.add_argument('--ros-setup', help='与 --prepare 同用，指定已有 ROS setup.bash')
    ap.add_argument('--python', help='与 --prepare 同用，指定已有兼容 Python')
    args = ap.parse_args(argv)
    if not args.prepare and (args.ros_setup or args.python):
        ap.error('--ros-setup / --python 需要同时指定 --prepare，或解压后执行 ./crossfire install')
    if os.geteuid() == 0: ap.exit(2, '请以普通用户安装，不要 sudo。\n')
    target = (args.target or Path.home() / 'crossfire-releases' /
              ('crossfire-' + datetime.datetime.now().strftime('%Y%m%d-%H%M%S-%f'))).expanduser().absolute()
    parts = args.bundle.read_bytes().split(b'\n__CROSSFIRE_PAYLOAD__\n', 1)
    if len(parts) != 2: ap.exit(2, '缺少压缩内容；没有创建安装目录。\n')
    blob = parts[1]
    if hashlib.sha256(blob).hexdigest() != 'PAYLOAD_HASH':
        ap.exit(2, '压缩内容校验失败；没有创建安装目录。\n')
    try: extract_payload(blob, target)
    except (ValueError, OSError, tarfile.TarError) as exc: ap.exit(2, str(exc) + '\n')
    print('已校验并解压到独立目录: ' + str(target), flush=True)
    if not args.prepare:
        print('绿色解压完成；未检查 ROS、未编译、未安装依赖、未启动节点。')
        print('可移动整个目录；准备运行：\ncd -- ' + shlex.quote(str(target)) + '\n./crossfire install')
        return 0
    code = subprocess.run(['bash', str(target / 'tools/build_car.sh'),
        '--ros-setup', args.ros_setup or '/opt/ros/humble/setup.bash',
        '--python', args.python or '/usr/bin/python3']).returncode
    if code:
        print('运行环境尚未就绪，文件仅保留在上述新目录；没有修改原系统。', file=sys.stderr)
    return code


if __name__ == '__main__':
    sys.exit(main())
