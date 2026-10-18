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
shutil.copy2(ROOT/'crossfire',car/'crossfire')
for name in ('native_runtime.py','native_env.sh','node_entry.py','install_bundle.py','build_car.sh','run_car.sh','car_supervisor.py','sync_workspace.py','profile_config.py','config_editor.py','configure.sh','export_model.py','build_native.sh'):
    shutil.copy2(ROOT/'tools'/name,car/'tools'/name)
(car/'docs').mkdir();shutil.copy2(ROOT/'docs/05_现场配置与运动调参.md',car/'docs/现场配置与运动调参.md');shutil.copy2(ROOT/'docs/02_车辆部署指南.md',car/'docs/车辆部署指南.md')
for name in ('02_车辆部署指南.md','05_现场配置与运动调参.md'):
    shutil.copy2(ROOT/'docs'/name,car/'docs'/name)
shutil.copy2(ROOT/'docs/06_实车适配与标定清单.md',car/'docs/06_实车适配与标定清单.md')
if (ROOT/'docs/07_NPU与GPU视觉加速指南.md').exists():
    shutil.copy2(ROOT/'docs/07_NPU与GPU视觉加速指南.md',car/'docs/07_NPU与GPU视觉加速指南.md')
shutil.copy2(ROOT/'docs/08_规划与视觉修复验证.md',car/'docs/08_规划与视觉修复验证.md')
shutil.copy2(ROOT/'docs/09_原生安装与隔离运行.md',car/'docs/09_原生安装与隔离运行.md')
shutil.copy2(ROOT/'docs/10_到达时间优化与验证.md',car/'docs/10_到达时间优化与验证.md')
copytree(ROOT/'docs/evidence',car/'docs/evidence')
(car/'README.md').write_text('''# 独立实车包 v2

首先阅读 docs/车辆部署指南.md。本包不安装系统依赖或驱动。
首次上车必读 docs/06_实车适配与标定清单.md，其中包含尚需完成的接口适配和现场标定记录表。

推荐将 crossfire-car-v2.tar.gz 放到车上，以普通用户解压到自选的新目录：
    DEST="$HOME/my-crossfire-v2"
    mkdir -m 700 -- "$DEST" &&
    tar -xzf crossfire-car-v2.tar.gz --strip-components=1 --keep-old-files -C "$DEST" &&
    cd -- "$DEST"
Linux 推荐 tar.gz，可保留脚本执行权限。无需运行安装器，解压不会执行包内程序。
可换成任意有写权限的新路径（父目录需已存在），支持空格和中文。
请保留 &&：已有目录（即使为空）、文件或链接导致 mkdir 失败，此时不继续解压。
不检查 ROS、不编译、不修改原有安装路径。不要向旧工作空间直接解压或覆盖升级。
不需要 Docker、colcon、pip 或系统安装；复用已有 ROS2 Humble / NumPy / OpenCV。
缺依赖时停止并列出原因，不自动安装或升级。

进入解压目录后：
1. ./crossfire install  # 显式准备本包 .runtime/car/，检查已有依赖、按本机架构编译
2. ./crossfire doctor
3. ./crossfire run A --preflight
4. ./crossfire run A  # 默认只读，无速度发布者

指定已有 ROS：./crossfire install --ros-setup /路径/setup.bash --python /匹配的/python3
可选的解压前校验工具：bash crossfire-car-v2.run --target /新的目录
可选解压并准备：bash crossfire-car-v2.run --target /新的目录 --prepare
同机移动：停止本包任务，整体搬动目录（含 .runtime），在新目录 ./crossfire doctor，无需改路径。
换机器：用原始包在新机器解压，重新 ./crossfire install；不携带其他架构的编译产物。
卸载：结束本包任务后删除自己选定的目录，无需恢复系统设置。
实车不运行仿真；本机无 Docker 仿真使用完整仿真包中的 ./crossfire sim。
详见 docs/09_原生安装与隔离运行.md。

现场配置界面：./crossfire ui（或 bash tools/configure.sh；打开打印的本机地址）
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
header = ('#!/usr/bin/env bash\nset -euo pipefail\n'
          'export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1\n'
          'exec python3 -B - "$0" "$@" <<\'PY_INSTALL\'\n'
          + (ROOT/'tools/install_bundle.py').read_text()
          + '\nPY_INSTALL\n__CROSSFIRE_PAYLOAD__\n')
blob=(OUT/'crossfire-car-v2.tar.gz').read_bytes()
header=header.replace('PAYLOAD_HASH',hashlib.sha256(blob).hexdigest())
installer=OUT/'crossfire-car-v2.run';installer.write_bytes(header.encode()+blob);installer.chmod(0o755)
# Full portable simulator source; the Docker image is exported separately for offline reuse.
sim=STAGE/'crossfire-sim-v2';sim.mkdir()
for name in ('racelib','sim','ros2_ws','config','tests','tools','containers','ui','profiles','models'):
    copytree(ROOT/name,sim/name)
for name in ('README.md','requirements.txt','.dockerignore','crossfire'):
    shutil.copy2(ROOT/name,sim/name)
(sim/'docs').mkdir()
for name in ('01_系统架构与算法.md','02_车辆部署指南.md','03_测试报告.md','05_现场配置与运动调参.md','06_实车适配与标定清单.md','07_NPU与GPU视觉加速指南.md','08_规划与视觉修复验证.md','09_原生安装与隔离运行.md','10_到达时间优化与验证.md'):
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
native_evidence=ROOT/'docs/evidence/native_install_validation.json'
if native_evidence.exists():
    validation=ROOT/json.loads(native_evidence.read_text())['validation_directory']
    if validation.is_dir(): copytree(validation,sim/'output'/validation.name)
arrival_evidence=ROOT/'docs/evidence/arrival_optimization_20261018.json'
if arrival_evidence.exists():
    validation=ROOT/json.loads(arrival_evidence.read_text())['validation_directory']
    if validation.is_dir(): copytree(validation,sim/'output'/validation.name)
green_evidence=ROOT/'docs/evidence/green_install_20261018.json'
if green_evidence.exists():
    validation=ROOT/json.loads(green_evidence.read_text())['validation_directory']
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
    native_evidence=ROOT/'docs/evidence/native_install_validation.json'
    if native_evidence.exists():
        receipt['native_install_validation']=json.loads(native_evidence.read_text())
    arrival_evidence=ROOT/'docs/evidence/arrival_optimization_20261018.json'
    if arrival_evidence.exists():
        receipt['arrival_optimization']=json.loads(arrival_evidence.read_text())
    green_evidence=ROOT/'docs/evidence/green_install_20261018.json'
    if green_evidence.exists():
        receipt['green_install_validation']=json.loads(green_evidence.read_text())
    (OUT/'VALIDATION.json').write_text(json.dumps(receipt,ensure_ascii=False,indent=2)+'\n')
files=[p for p in sorted(OUT.iterdir()) if p.is_file() and p.name!='SHA256SUMS']
(OUT/'SHA256SUMS').write_text(''.join(file_hash(p)+'  '+p.name+'\n' for p in files))
for p in files:print(p.name,p.stat().st_size)
