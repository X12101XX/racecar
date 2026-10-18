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


def tree_state(root):
    """Compare contents, permissions and modification times; reads may change atime."""
    result={}
    for path in sorted(root.rglob('*')):
        stat=path.lstat()
        result[str(path.relative_to(root))]=(stat.st_mode,stat.st_size if path.is_file() else None,
            stat.st_mtime_ns,sha(path) if path.is_file() else None)
    return result


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
    def doctor(package):
        before=tree_state(package)
        execute(['bash',package/'crossfire','doctor'])
        assert tree_state(package)==before, 'doctor changed package files or directories'
    a=work/'unprepared release A';b=work/'release B'
    execute(['bash',args.bundle,'--target',a])
    assert not (a/'.runtime').exists(),'default extraction prepared runtime'
    # The folder may be placed anywhere before or after local preparation.
    moved=work/'prepared release A';a.rename(moved);a=moved
    before_install=tree_state(a)
    execute(['bash',a/'crossfire','install'])
    after_install=tree_state(a)
    assert all(after_install.get(name)==value for name,value in before_install.items()), 'install changed payload files'
    created=sorted(set(after_install)-set(before_install))
    assert created and all(name in ('.runtime','.runtime/car') or name.startswith('.runtime/car/') for name in created)
    doctor(a)
    moved=work/'moved release A';a.rename(moved);a=moved
    doctor(a)
    execute(['bash',a/'crossfire','install'])  # Owned runtime can be regenerated after moving.
    execute(['bash',args.bundle,'--target',b,'--prepare'])
    doctor(b)
    fallback=work/'release without compiler'
    fallback_install=execute(['bash',args.bundle,'--target',fallback,'--prepare'],extra_env={'CXX':'crossfire_missing_compiler'})
    assert 'auto 将使用 Python' in fallback_install.stdout
    fallback_state=json.loads((fallback/'.runtime/car/runtime.json').read_text())
    assert not fallback_state['native_available']
    doctor(fallback)
    assert {str(p):sha(p) for p in home.rglob('*') if p.is_file()} == original, 'installation wrote outside its prefix'
    execute(['bash',args.bundle,'--target',a],expected=2)
    codefiles={str(p):sha(p) for p in a.rglob('*.py')}
    refuse=execute(['bash',a/'crossfire','run','A','--arm'],expected=2)
    assert '拒绝实车输出' in refuse.stderr
    read_only=execute(['bash',a/'crossfire','run','A','--preflight'],expected=2)
    assert '只读自检未通过' in read_only.stderr
    # Inspect actual native nodes, even though no colcon install or ament registration exists.
    check=work/'check_nodes.py'
    check.write_text('''import pathlib,rclpy,sys
from racecar_mission.mission_node import MissionNode
from racecar_mission.gateway import Gateway
import racecar_mission
root=pathlib.Path(sys.argv[1])
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
    execute(['bash',a/'tools/native_env.sh',state['ros_setup'],state['python'],check,a],
            extra_env={'ROS_HOME':str(a/'.runtime/node-check/ros'),
                       'ROS_LOG_DIR':str(a/'.runtime/node-check/ros/log')})
    assert codefiles=={str(p):sha(p) for p in a.rglob('*.py')}
    assert all(sha(Path(name))==value for name,value in original.items())
    assert {str(p):sha(p) for p in home.rglob('*') if p.is_file()} == original, 'read-only checks wrote outside their prefix'
    assert not attempts.exists(),'forbidden dependency command executed'
    assert not (home/'.local').exists(),'installer or preflight wrote shared global state'
    assert not (a/'ros2_ws/build').exists() and not (a/'ros2_ws/install').exists()
    result=dict(side_by_side_install=True,existing_code_and_shell_unchanged=True,
        install_only_wrote_private_runtime=True,install_created_paths=created,
        doctor_package_tree_unchanged=True,
        default_extract_only=True,custom_prefix_with_spaces=True,
        relocated_before_and_after_prepare=True,repeat_prepare_in_owned_runtime=True,
        home_unchanged_after_install_and_doctor=True,
        no_system_install_commands=True,no_docker_colcon_ros2run=True,
        native_compiled=state['native_available'],source_nodes_verified=True,
        unarmed_has_no_velocity_publishers=True,arm_refused_without_calibration=True,
        no_external_home_state=True,code_unchanged_after_prepare=True,
        no_compiler_fallback_reported=True,
        architecture=state['architecture'],python=state['python_version'],vehicle_connected=False)
    (work/'report.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
    return 0

if __name__=='__main__':sys.exit(main())
