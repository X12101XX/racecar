"""Embedded installer: validate first, extract only into a brand-new private prefix."""
import argparse
import datetime
import hashlib
import io
import json
import os
from pathlib import Path, PurePosixPath
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
    ap.add_argument('--target', type=Path)
    ap.add_argument('--extract-only', action='store_true', help='只解压校验，不准备 ROS 运行环境')
    ap.add_argument('--ros-setup', default='/opt/ros/humble/setup.bash')
    ap.add_argument('--python', default='/usr/bin/python3')
    args = ap.parse_args(argv)
    if os.geteuid() == 0: ap.exit(2, '请以普通用户安装，不要 sudo。\n')
    target = (args.target or Path.home() / 'crossfire-releases' /
              ('crossfire-' + datetime.datetime.now().strftime('%Y%m%d-%H%M%S-%f'))).expanduser().absolute()
    blob = args.bundle.read_bytes().split(b'\n__CROSSFIRE_PAYLOAD__\n', 1)[1]
    if hashlib.sha256(blob).hexdigest() != 'PAYLOAD_HASH':
        ap.exit(2, '压缩内容校验失败；没有创建安装目录。\n')
    try: extract_payload(blob, target)
    except (ValueError, OSError, tarfile.TarError) as exc: ap.exit(2, str(exc) + '\n')
    print('已校验并解压到独立目录: ' + str(target), flush=True)
    if args.extract_only:
        print('只解压；未安装系统依赖、未启动节点。准备运行：' + str(target / 'crossfire') + ' install')
        return 0
    code = subprocess.run(['bash', str(target / 'tools/build_car.sh'),
        '--ros-setup', args.ros_setup, '--python', args.python]).returncode
    if code:
        print('运行环境尚未就绪，文件仅保留在上述新目录；没有修改原系统。', file=sys.stderr)
    return code


if __name__ == '__main__':
    sys.exit(main())
