"""Run inside an existing Humble environment as a regular user, without Docker/colcon.

Usage: python3 tools/validate_native_install.py --bundle path.run --work empty_parent
The caller may use an external CI sandbox; none of the target commands require it.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys


def sha(path): return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--bundle',type=Path,required=True)
    ap.add_argument('--work',type=Path,required=True)
    args=ap.parse_args();work=args.work.absolute();work.mkdir(parents=True,exist_ok=False)
    home=work/'home';home.mkdir()
    vendor=home/'racecar';vendor.mkdir();(vendor/'driver.py').write_text('original vendor code\n')
    (home/'.bashrc').write_text('original shell environment\n')
    original={str(p):sha(p) for p in (vendor/'driver.py',home/'.bashrc')}
    forbidden=work/'forbidden';forbidden.mkdir();attempts=work/'forbidden-attempts.txt'
    for name in ('docker','colcon','ros2','pip','pip3','apt','apt-get'):
        path=forbidden/name
        path.write_text('#!/usr/bin/env bash\nprintf "%s\\n" "$0" >> "$CROSSFIRE_FORBIDDEN_LOG"\nexit 97\n');path.chmod(0o755)
    env=os.environ.copy();env.update(HOME=str(home),PATH=str(forbidden)+':'+env['PATH'],
        CROSSFIRE_FORBIDDEN_LOG=str(attempts),PYTHONNOUSERSITE='1',PYTHONDONTWRITEBYTECODE='1')
    def execute(command,expected=0,extra_env=None):
        result=subprocess.run(list(map(str,command)),env={**env,**(extra_env or {})},capture_output=True,text=True)
        with (work/'commands.log').open('a') as f:f.write(str(command)+'\n'+result.stdout+result.stderr+'\n')
        if (result.returncode==0)!=(expected==0):
            raise RuntimeError('unexpected exit '+str(result.returncode)+'\n'+result.stdout+result.stderr)
        return result
    a=work/'release A';b=work/'release B'
    execute(['bash',args.bundle,'--target',a])
    execute(['bash',a/'crossfire','doctor'])
    execute(['bash',args.bundle,'--target',b])
    execute(['bash',b/'crossfire','doctor'])
    fallback=work/'release without compiler'
    fallback_install=execute(['bash',args.bundle,'--target',fallback],extra_env={'CXX':'crossfire_missing_compiler'})
    assert 'auto 将使用 Python' in fallback_install.stdout
    fallback_state=json.loads((fallback/'.runtime/car/runtime.json').read_text())
    assert not fallback_state['native_available']
    execute(['bash',fallback/'crossfire','doctor'])
    execute(['bash',args.bundle,'--target',a],expected=2)
    codefiles={str(p):sha(p) for p in a.rglob('*.py')}
    refuse=execute(['bash',a/'crossfire','run','A','--arm'],expected=2)
    assert '拒绝实车输出' in refuse.stderr
    read_only=execute(['bash',a/'crossfire','run','A','--preflight'],expected=2)
    assert '只读自检未通过' in read_only.stderr
    # Inspect actual native nodes, even though no colcon install or ament registration exists.
    check=work/'check_nodes.py'
    check.write_text('''import pathlib,rclpy
from racecar_mission.mission_node import MissionNode
from racecar_mission.gateway import Gateway
import racecar_mission
root=pathlib.Path(__file__).parent/'release A'
assert pathlib.Path(racecar_mission.__file__).resolve().is_relative_to(root)
rclpy.init(args=['--ros-args','-p','config_dir:='+str(root/'config'),'-p','publish_markers:=false','-p','tts:=none'])
m=MissionNode();g=Gateway()
assert m.io.cmd_pub is None and g.pub is None
assert m.marker_pub is None
for topic in ('/car_cmd_vel','/cmd_vel','/crossfire/desired_cmd'): assert m.count_publishers(topic)==0
m.destroy_node();g.destroy_node();rclpy.shutdown()
print('native source nodes: no velocity publishers; no colcon needed')
''')
    state=json.loads((a/'.runtime/car/runtime.json').read_text())
    execute(['bash',a/'tools/native_env.sh',state['ros_setup'],state['python'],check])
    assert codefiles=={str(p):sha(p) for p in a.rglob('*.py')}
    assert all(sha(Path(name))==value for name,value in original.items())
    assert not attempts.exists(),'forbidden dependency command executed'
    assert not (home/'.local').exists(),'installer or preflight wrote shared global state'
    assert not (a/'ros2_ws/build').exists() and not (a/'ros2_ws/install').exists()
    result=dict(side_by_side_install=True,existing_code_and_shell_unchanged=True,
        no_system_install_commands=True,no_docker_colcon_ros2run=True,
        native_compiled=state['native_available'],source_nodes_verified=True,
        unarmed_has_no_velocity_publishers=True,arm_refused_without_calibration=True,
        no_external_home_state=True,code_unchanged_after_prepare=True,
        no_compiler_fallback_reported=True,
        architecture=state['architecture'],python=state['python_version'],vehicle_connected=False)
    (work/'report.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
    return 0

if __name__=='__main__':sys.exit(main())
