"""Point to this completed simulator run; never silently select an older video."""
import argparse
import json
import os
from pathlib import Path
import posixpath
import re
import shlex
import sys
import uuid

ROOT = Path(__file__).resolve().parents[1]


def show_result(container_dir, since_ns=0, root=ROOT, profile_name=''):
    if profile_name and not re.fullmatch(r'[A-Za-z0-9_-]{1,48}',profile_name):
        raise ValueError('非法配置别名')
    output = (Path(root) / 'output').resolve()
    normalized = posixpath.normpath(container_dir)
    if not normalized.startswith('/output/'):
        print('结果未位于 /output/ 子目录；不更新最新结果入口。', file=sys.stderr)
        return False
    result = (output / normalized.removeprefix('/output/')).resolve()
    if not result.is_relative_to(output) or result == output:
        raise ValueError('输出目录越界')
    summary = result / 'summary.json'
    if not summary.exists() or summary.stat().st_mtime_ns < since_ns:
        print('本次未生成新的 summary.json；保留原最新结果入口，不能将旧视频视为本次结果。', file=sys.stderr)
        return False
    partial=result/'video.partial.mp4'
    if partial.exists():
        print('视频尚未完成编码；不更新最新结果入口。',file=sys.stderr)
        return False
    data = json.loads(summary.read_text())
    route = data.get('route')
    if route not in ('A', 'B'):
        raise ValueError('结果中的路线非法')
    aliases=['latest','latest_'+route]
    profile_alias='latest_'+profile_name+'_'+route if profile_name else ''
    if profile_alias:aliases.append(profile_alias)
    for name in aliases:
        alias = output / name
        if alias.exists() and not alias.is_symlink():
            print('保留已有文件/目录，未覆盖：' + str(alias), file=sys.stderr)
            continue
        temporary = output / ('.' + name + '-' + uuid.uuid4().hex)
        try:
            temporary.symlink_to(os.path.relpath(result, output), target_is_directory=True)
            temporary.replace(alias)
        finally:
            temporary.unlink(missing_ok=True)
    print('\n本次仿真已结束：' + ('通过' if data.get('success') else '未完赛/失败，请查看 summary.json'))
    print('本次结果目录：' + str(result))
    print('本次视频：' + str(result / 'video.mp4'))
    print('播放本次视频：mpv ' + shlex.quote(str(result / 'video.mp4')))
    if (output / 'latest').is_symlink() and (output / 'latest').resolve() == result:
        print('最近一次已结束的结果入口：' + str(output / 'latest'))
    if profile_alias and (output / profile_alias).is_symlink() and (output / profile_alias).resolve()==result:
        print('本配置/路线的结果入口：'+str(output/profile_alias))
        print('播放本配置/路线：mpv '+shlex.quote(str(output/profile_alias/'video.mp4')))
    print('请勿打开 output/demo_routeA 等历史目录来查看本次结果。')
    return True


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('container_dir')
    ap.add_argument('--since-ns', type=int, default=0)
    ap.add_argument('--profile-name', default='')
    args = ap.parse_args()
    return 0 if show_result(args.container_dir, args.since_ns, profile_name=args.profile_name) else 1


if __name__ == '__main__':
    sys.exit(main())
