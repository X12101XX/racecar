"""Deterministic source payloads. No SSH, system writes, or architecture binaries."""
from pathlib import Path
import hashlib
import io
import json
import os
import shutil
import tarfile
import time
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
subprocess.run([sys.executable,str(ROOT/'tools/sync_workspace.py')],check=True)
OUT=ROOT/'release';OUT.mkdir(exist_ok=True)
STAGE=ROOT/'tmp/release-stage'
if STAGE.exists():shutil.rmtree(STAGE)
STAGE.mkdir(parents=True)
car=STAGE/'crossfire-car-v2';car.mkdir()

def copytree(src,dst):
    shutil.copytree(src,dst,ignore=shutil.ignore_patterns('__pycache__','*.pyc','*.so','build','install','log'))
for name in ('config','racelib','ros2_ws','ui','profiles','models'):
    copytree(ROOT/name,car/name)
(car/'tools').mkdir()
for name in ('build_car.sh','run_car.sh','car_supervisor.py','sync_workspace.py','profile_config.py','config_editor.py','configure.sh','export_model.py','build_native.sh'):
    shutil.copy2(ROOT/'tools'/name,car/'tools'/name)
(car/'docs').mkdir();shutil.copy2(ROOT/'docs/05_现场配置与运动调参.md',car/'docs/现场配置与运动调参.md');shutil.copy2(ROOT/'docs/02_车辆部署指南.md',car/'docs/车辆部署指南.md')
shutil.copy2(ROOT/'docs/06_实车适配与标定清单.md',car/'docs/06_实车适配与标定清单.md')
if (ROOT/'docs/07_NPU与GPU视觉加速指南.md').exists():
    shutil.copy2(ROOT/'docs/07_NPU与GPU视觉加速指南.md',car/'docs/07_NPU与GPU视觉加速指南.md')
shutil.copy2(ROOT/'docs/08_规划与视觉修复验证.md',car/'docs/08_规划与视觉修复验证.md')
copytree(ROOT/'docs/evidence',car/'docs/evidence')
(car/'README.md').write_text('''# 独立实车包 v2

首先阅读 docs/车辆部署指南.md。本包不安装系统依赖或驱动。
首次上车必读 docs/06_实车适配与标定清单.md，其中包含尚需完成的接口适配和现场标定记录表。

1. bash tools/build_car.sh
2. bash tools/run_car.sh A --preflight
3. bash tools/run_car.sh A  # 默认只读，无速度发布者

现场配置界面：bash tools/configure.sh（打开打印的本机地址）
导出配置后：bash tools/run_car.sh A --config-dir profiles/generated/配置名
运动预设：--motion-preset conservative / standard / aggressive
详见 docs/现场配置与运动调参.md。

--arm 仅在完成 profile 全部硬件/场地确认后可用；不要跳过标定。
本包未在当前实车联调，不能声称实车已经完赛。
''')
manifest={str(p.relative_to(car)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(car.rglob('*')) if p.is_file()}
(car/'MANIFEST.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')

def archive(directory,out):
    with tarfile.open(out,'w:gz') as tf:
        tf.add(directory,arcname=directory.name,filter=lambda info: info)
archive(car,OUT/'crossfire-car-v2.tar.gz')
# The installer copies only into a brand-new directory and never starts any process on the car.
header='''#!/usr/bin/env bash
set -euo pipefail
exec python3 - "$0" "$@" <<'PY_INSTALL'
import hashlib,io,json,os,pathlib,sys,tarfile,datetime
if os.geteuid()==0:raise SystemExit('请以普通用户运行，不要 sudo')
args=sys.argv[2:]
if args and (len(args)!=2 or args[0]!='--target'):
 raise SystemExit('用法: bash crossfire-car-v2.run [--target 全新目录]')
base=pathlib.Path.home()/'crossfire-releases'
target=pathlib.Path(args[1]).expanduser().absolute() if args else base/('crossfire-'+datetime.datetime.now().strftime('%Y%m%d-%H%M%S-%f'))
if target.exists() or target.is_symlink():raise SystemExit('拒绝覆盖已有目录: '+str(target))
blob=pathlib.Path(sys.argv[1]).read_bytes().split(b'\\n__CROSSFIRE_PAYLOAD__\\n',1)[1]
if hashlib.sha256(blob).hexdigest()!='PAYLOAD_HASH':raise SystemExit('压缩内容校验失败')
target.mkdir(parents=True,exist_ok=False,mode=0o700)
with tarfile.open(fileobj=io.BytesIO(blob),mode='r:gz') as tf:
 for member in tf.getmembers():
  parts=pathlib.PurePosixPath(member.name).parts
  if not parts or parts[0]!='crossfire-car-v2' or '..' in parts or member.name.startswith('/') or member.issym() or member.islnk() or not(member.isfile() or member.isdir()):
   raise SystemExit('压缩包含非法路径')
  dest=target.joinpath(*parts[1:])
  if member.isdir():dest.mkdir(parents=True,exist_ok=True)
  else:
   dest.parent.mkdir(parents=True,exist_ok=True)
   with dest.open('xb') as out:out.write(tf.extractfile(member).read())
   dest.chmod(member.mode&0o755)
manifest=json.loads((target/'MANIFEST.json').read_text())
for name,digest in manifest.items():
 if hashlib.sha256((target/name).read_bytes()).hexdigest()!=digest:raise SystemExit('文件校验失败: '+name)
print('已解压并校验。未安装系统依赖，未启动任何节点。')
print('独立目录: '+str(target))
print('下一步: cd '+str(target)+' && bash tools/build_car.sh')
PY_INSTALL
__CROSSFIRE_PAYLOAD__
'''
blob=(OUT/'crossfire-car-v2.tar.gz').read_bytes()
header=header.replace('PAYLOAD_HASH',hashlib.sha256(blob).hexdigest())
installer=OUT/'crossfire-car-v2.run';installer.write_bytes(header.encode()+blob);installer.chmod(0o755)
# Full portable simulator source; the Docker image is exported separately for offline reuse.
sim=STAGE/'crossfire-sim-v2';sim.mkdir()
for name in ('racelib','sim','ros2_ws','config','tests','tools','containers','ui','profiles','models'):
    copytree(ROOT/name,sim/name)
for name in ('README.md','requirements.txt','.dockerignore'):
    shutil.copy2(ROOT/name,sim/name)
(sim/'docs').mkdir()
for name in ('01_系统架构与算法.md','02_车辆部署指南.md','03_测试报告.md','05_现场配置与运动调参.md','06_实车适配与标定清单.md','07_NPU与GPU视觉加速指南.md','08_规划与视觉修复验证.md'):
    if (ROOT/'docs'/name).exists():shutil.copy2(ROOT/'docs'/name,sim/'docs'/name)
copytree(ROOT/'docs/evidence',sim/'docs/evidence')
copytree(ROOT/'docs/ros2_humble_msgs',sim/'docs/ros2_humble_msgs')
(sim/'output').mkdir()
# Keep current revision evidence separate from historical validation output.
evidence=ROOT/'docs/evidence/revision_validation.json'
if evidence.exists():
    metadata=json.loads(evidence.read_text())
    validation=ROOT/metadata['validation_directory']
    if validation.is_dir(): copytree(validation,sim/'output'/validation.name)
for name in ('unit-vision-final.log','ros-contract-final.log','planner-benchmark.json','planner-benchmark.log','revision-faults.json','target-install-revision.json'):
    if (ROOT/'output'/name).exists(): shutil.copy2(ROOT/'output'/name,sim/'output'/name)
archive(sim,OUT/'crossfire-sim-v2.tar.gz')
def file_hash(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for chunk in iter(lambda:f.read(1024*1024),b''):h.update(chunk)
    return h.hexdigest()
# Refresh artifact digests on every packaging run. Test evidence remains dated:
# packaging itself does not rerun validation or certify later source changes.
evidence=ROOT/'docs/evidence/revision_validation.json'
if evidence.exists():
    record=json.loads(evidence.read_text())
    receipt=dict(validation_date=record.get('date'),base_commit=record.get('base_commit'),
        unit_tests_passed=record['unit_tests']['passed'],native_ros_contract_passed=record['native_ros_contract']['passed'],
        ros_scenarios_passed=sum(bool(r['passed']) for r in record['runs']),
        faults_passed=sum(bool(r['passed']) for r in record['faults']),
        validation_directory=record['validation_directory'],vehicle_tested=False,
        note='Checks refer to the dated validation report; packaging does not rerun tests.',
        artifacts={p.name:file_hash(p) for p in sorted(OUT.iterdir()) if p.is_file() and p.suffix in ('.gz','.run')})
    (OUT/'VALIDATION.json').write_text(json.dumps(receipt,ensure_ascii=False,indent=2)+'\n')
files=[p for p in sorted(OUT.iterdir()) if p.is_file() and p.name!='SHA256SUMS']
(OUT/'SHA256SUMS').write_text(''.join(file_hash(p)+'  '+p.name+'\n' for p in files))
for p in files:print(p.name,p.stat().st_size)
