"""Private process lifetime, exclusive lock, no network or serial reconfiguration.
Never starts/stops a vendor driver; never kills another controller.
"""
import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def main():
    p=argparse.ArgumentParser(description='独立 ROS2 穿越火线任务；默认只读，无底盘发布者')
    p.add_argument('route',choices=['A','B'],nargs='?',default='A')
    p.add_argument('--arm',action='store_true',help='明确启用独立安全网关')
    p.add_argument('--use-card',action='store_true',help='启动色卡识别（根据抽取的色卡自动决定路线）')
    p.add_argument('--color',action='store_true',help='启用彩色模式与 AI 硬件加速视觉感知')
    p.add_argument('--ai-backend',choices=['auto','npu','onnx','fast_color','cpu'],default='',help='指定色卡推理后端；缺省使用 vision.json')
    p.add_argument('--ai-model',type=Path,help='符合色卡分类输入输出协议的模型；相对路径从当前目录解析')
    p.add_argument('--planner-engine',choices=['auto','native','python'],default='auto')
    p.add_argument('--profile',type=Path)
    p.add_argument('--config-dir',type=Path,help='配置界面导出的目录')
    p.add_argument('--motion-preset',choices=['conservative','standard','aggressive'])
    p.add_argument('--preflight',action='store_true',help='仅做只读检查并退出')
    a=p.parse_args();root=Path(__file__).resolve().parents[1]
    profile=json.loads((a.profile or root/'config/vehicle_profile.json').read_text())
    source=(a.config_dir or root/'config').resolve()
    from racelib.settings import validate_field,motion_settings,MOTION_PRESETS
    field=json.loads((source/'field.json').read_text());robot_data=json.loads((source/'robot.json').read_text())
    plan_data=json.loads((source/'plan.json').read_text())
    if a.motion_preset:
        plan_data['ackermann']=dict(MOTION_PRESETS[a.motion_preset]);plan_data['control']['cross_speed_mps']=MOTION_PRESETS[a.motion_preset]['forward_mps']
    is_mono = not (a.use_card or a.color)
    plan_data.setdefault('mission',{})['monochrome']=is_mono
    vision_data=json.loads((source/'vision.json').read_text())
    if a.ai_backend: vision_data.setdefault('ai',{})['backend']=a.ai_backend
    if a.ai_model: vision_data.setdefault('ai',{})['model_path']=str(a.ai_model.resolve())
    model=vision_data.get('ai',{}).get('model_path','')
    if model and not Path(model).is_absolute():
        vision_data['ai']['model_path']=str((root/model).resolve())
    if not is_mono:
        from racelib.ai_vision import AIVisionEngine
        AIVisionEngine(vision_data.get('ai',{}).get('backend','auto'),vision_data.get('ai',{}).get('model_path',''))
    motion=motion_settings(plan_data)
    errors,_=validate_field(field,robot_data['geometry'])
    if errors:raise SystemExit('; '.join(errors))
    if a.arm:
        for k,pk in [('length_m','body_length'),('width_m','body_width'),('wheel_base_m','wheelbase')]:
            if abs(robot_data['geometry'][k]-profile[pk])>1e-6:
                raise SystemExit('场景车辆几何与已标定 profile 不一致，先核对 '+k)
        needed=['hardware_watchdog_confirmed','steering_mapping_calibrated','sensor_frames_calibrated','field_and_start_measured']
        missing=[k for k in needed if profile.get(k) is not True]
        if missing:raise SystemExit('拒绝实车输出，尚未验证: '+', '.join(missing))
        if not profile.get('allow_reverse'):
            raise SystemExit('当前路线规划可能需要倒车。请完成后向测距和底盘倒车验证后显式设置 allow_reverse=true。')
    domain=profile.get('ros_domain_id',0)
    if not isinstance(domain,int) or not 0<=domain<=101:raise SystemExit('非法 ROS_DOMAIN_ID')
    env=os.environ.copy();env['CROSSFIRE_PLANNER_ENGINE']=a.planner_engine;env['ROS_DOMAIN_ID']=str(domain);env['ROS_LOCALHOST_ONLY']='1'
    run=root/'runs'/time.strftime('%Y%m%d-%H%M%S');run.mkdir(parents=True,exist_ok=False)
    env['ROS_HOME']=str(run/'ros');env['ROS_LOG_DIR']=str(run/'ros/log');env['CROSSFIRE_RUN_DIR']=str(run)
    lock_path=Path.home()/'.local/state/crossfire/mission.lock';lock_path.parent.mkdir(parents=True,exist_ok=True)
    lock=lock_path.open('a')
    try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:raise SystemExit('已有独立穿越火线任务在运行')
    params={'topics.scan':profile['scan_topic'],'topics.imu':profile['imu_topic'],'topics.odom':profile['odom_topic'],'topics.image':profile['image_topic'],'topics.cmd_vel':'/crossfire/desired_cmd'}
    def command(executable,values):
        cmd=['ros2','run','racecar_mission',executable,'--ros-args']
        for k,v in values.items():cmd+=['-p',f'{k}:={str(v).lower() if isinstance(v,bool) else v}']
        return cmd
    # preflight has no publishers. Avoid changing any active system behavior.
    pre=subprocess.run(command('preflight',params)+['--seconds','5','--config-dir',str(source)],env=env)
    if pre.returncode:raise SystemExit('只读自检未通过，未启动实车输出。日志目录: '+str(run))
    if a.preflight:return 0
    children=[];files=[];stopping=False
    def stop(sig=None,frame=None):
        nonlocal stopping
        stopping=True
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    try:
        if a.arm:
            keys=('output_topic','output_mode','scan_topic','imu_topic','odom_topic','wheelbase','max_steer','max_speed','reverse_limit','body_length','body_width','allow_reverse','hardware_watchdog_confirmed')
            g={k:profile[k] for k in keys};g['armed']=True;g['calibration_confirmed']=True
            g['max_speed']=min(profile['max_speed'],motion['forward_mps']);g['reverse_limit']=min(profile['reverse_limit'],motion['reverse_mps']);g['brake_decel']=motion['brake_mps2']
            specs=[('gateway',g)]
        else:specs=[]
        # Consistent geometry on planner and gateway, copied per run; original files untouched.
        import shutil
        cfg_dir=run/'config';shutil.copytree(source,cfg_dir)
        (cfg_dir/'plan.json').write_text(json.dumps(plan_data,ensure_ascii=False,indent=2))
        (cfg_dir/'vision.json').write_text(json.dumps(vision_data,ensure_ascii=False,indent=2))
        specs.append(('mission_node',{**params,'route':a.route,'use_card':a.use_card,'monochrome':is_mono,'ai_backend':a.ai_backend,'publish_cmd':a.arm,'tts':'auto' if a.arm else 'none','config_dir':str(cfg_dir)}))
        for name,values in specs:
            f=(run/f'{name}.log').open('w');files.append(f)
            children.append(subprocess.Popen(command(name,values),env=env,stdout=f,stderr=subprocess.STDOUT,start_new_session=True))
        print(f'运行目录: {run}\n模式: {"已启用底盘输出" if a.arm else "只读，无速度发布者"}；Ctrl+C 结束本次任务。',flush=True)
        started=time.monotonic()
        while not stopping:
            if any(c.poll() is not None for c in children):raise RuntimeError('子进程退出，请检查本次日志')
            if time.monotonic()-started>420:raise RuntimeError('总运行时间达到 420 秒，结束任务')
            time.sleep(.1)
    finally:
        # Mission first, gateway last: gateway gets a chance to timeout and send neutral.
        for c in reversed(children):
            if c.poll() is None:
                os.killpg(c.pid,signal.SIGINT)
                try:c.wait(timeout=4)
                except subprocess.TimeoutExpired:os.killpg(c.pid,signal.SIGKILL);c.wait()
        for f in files:f.close()
    return 0

if __name__=='__main__':sys.exit(main())
