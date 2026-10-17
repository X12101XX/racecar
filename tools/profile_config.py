"""Single editable scene -> isolated runtime config. Never changes base config or arms a car."""
import argparse
import copy
import datetime
import json
import math
from pathlib import Path
import re
import shutil
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from racelib.settings import MOTION_PRESETS,MOTION_LIMITS,motion_settings,validate_field,boundary,inside


def default_profile():
    field=json.loads((ROOT/'config/field.json').read_text())
    cones=[dict(x=x,y=y,radius=.115,known=False) for x in (2.,3.5,5.) for y in (.45,1.25,2.05,2.85)]
    cones += [dict(x=x,y=y,radius=.115,known=False) for x,y in ((2.7,.8),(2.7,1.7),(4.2,1.1),(4.2,2.4),(4.65,.5))]
    return dict(version=1,name='现场配置',field=field,geometry=dict(length_m=.42,width_m=.30,wheel_base_m=.30),
                motion=copy.deepcopy(MOTION_PRESETS['standard']),cones=cones,
                routes={r:dict(stronghold='stronghold_'+r.lower(),camp='camp_'+r.lower()) for r in ('A','B')})


def validate(p):
    errors=[];warnings=[]
    try:
        geometry=p['geometry']
        for k in ('length_m','width_m','wheel_base_m'):
            if isinstance(geometry[k],bool) or not isinstance(geometry[k],(int,float)) or not math.isfinite(geometry[k]) or not .1<=geometry[k]<=1.5:
                errors.append('车辆 '+k+' 应在 0.1～1.5m 内')
        if geometry['wheel_base_m']>geometry['length_m']:errors.append('轴距不能大于车长')
        e,w=validate_field(p['field'],geometry);errors+=e;warnings+=w
        motion_settings({'ackermann':p['motion']})
        if len(p['cones'])>300:errors.append('障碍数量不能超过300')
        for i,c in enumerate(p['cones']):
            vals=[c['x'],c['y'],c['radius']]
            if any(isinstance(v,bool) or not isinstance(v,(float,int)) or not math.isfinite(v) for v in vals):
                raise ValueError(f'障碍 {i+1} 坐标或半径非法')
            if not .03<=vals[2]<=.5 or not inside(vals[:2],boundary(p['field']),vals[2]):
                errors.append(f'障碍 {i+1} 超出围栏或半径不在0.03～0.5m')
            if not isinstance(c.get('known',False),bool):errors.append('known 标记必须是布尔值')
        for route in ('A','B'):
            for key in ('stronghold','camp'):
                if p['routes'][route][key] not in p['field']['zones']:errors.append(f'路线 {route} 的 {key} 区域不存在')
    except (KeyError,TypeError,ValueError,IndexError) as e:errors.append(str(e))
    return {'errors':errors,'warnings':warnings,'valid':not errors}


def compile_profile(p,destination):
    result=validate(p)
    if result['errors']:raise ValueError('; '.join(result['errors']))
    d=Path(destination);d.mkdir(parents=True,exist_ok=True)
    for src in (ROOT/'config').glob('*.json'):shutil.copy2(src,d/src.name)
    hardware=json.loads((d/'vehicle_profile.json').read_text())
    for key in ('hardware_watchdog_confirmed','steering_mapping_calibrated','sensor_frames_calibrated','field_and_start_measured','allow_reverse'):
        hardware[key]=False
    (d/'vehicle_profile.json').write_text(json.dumps(hardware,ensure_ascii=False,indent=2))
    f=copy.deepcopy(p['field']);f['arena']['enforce_official_rules']=False
    f['prior']={'obstacles':[{k:c.get(k,.115) for k in ('x','y','radius')} for c in p['cones'] if c.get('known',False)]}
    (d/'field.json').write_text(json.dumps(f,ensure_ascii=False,indent=2))
    robot=json.loads((d/'robot.json').read_text());robot['geometry']=copy.deepcopy(p['geometry'])
    (d/'robot.json').write_text(json.dumps(robot,ensure_ascii=False,indent=2))
    plan=json.loads((d/'plan.json').read_text());plan['ackermann']=copy.deepcopy(p['motion']);plan['control']['cross_speed_mps']=p['motion']['forward_mps']
    (d/'plan.json').write_text(json.dumps(plan,ensure_ascii=False,indent=2))
    for r in ('A','B'):
        route=json.loads((d/f'route_{r.lower()}.json').read_text());route.update(p['routes'][r]);route['corridor_y']=[0.,float(f['arena']['width_m'])]
        (d/f'route_{r.lower()}.json').write_text(json.dumps(route,ensure_ascii=False,indent=2))
    # Simulation truth is deliberately in a separate file, not in controller field.json.
    (d/'sim_cones.json').write_text(json.dumps(p['cones'],ensure_ascii=False,indent=2))
    (d/'scene.json').write_text(json.dumps(p,ensure_ascii=False,indent=2))
    (d/'README.txt').write_text('仿真（在仿真项目目录执行）: bash tools/run_ros_sim.sh --profile /本配置目录/scene.json --route A\n实车: bash tools/run_car.sh A --config-dir 本目录路径\n实车仍受已标定 vehicle_profile.json 的限速及动作确认限制。\n')
    return result


def safe_name(name):
    if not re.fullmatch(r'[A-Za-z0-9_-]{1,48}',name):raise ValueError('配置名只能用1～48位英文字母、数字、下划线、短横线')
    return name


def save_profile(name,p,base=None):
    name=safe_name(name);base=Path(base or ROOT/'profiles');base.mkdir(exist_ok=True,parents=True)
    result=validate(p)
    if not result['valid']:raise ValueError('; '.join(result['errors']))
    target=base/(name+'.json')
    if target.exists():
        history=base/'history';history.mkdir(exist_ok=True)
        shutil.copy2(target,history/(name+'-'+datetime.datetime.now().strftime('%Y%m%d-%H%M%S-%f')+'.json'))
    tmp=base/(name+'.tmp');tmp.write_text(json.dumps(p,ensure_ascii=False,indent=2)+'\n');tmp.replace(target)
    generated=base/'generated'/name
    compile_profile(p,generated)
    return {'profile':str(target),'config_dir':str(generated),**result}


def main():
    ap=argparse.ArgumentParser(description='现场配置导出；不启动 ROS 或底盘')
    ap.add_argument('file',type=Path);ap.add_argument('--out',type=Path);ap.add_argument('--check',action='store_true')
    a=ap.parse_args();p=json.loads(a.file.read_text());result=validate(p)
    if a.out and result['valid']:compile_profile(p,a.out)
    print(json.dumps(result,ensure_ascii=False,indent=2));return 0 if result['valid'] else 2

if __name__=='__main__':sys.exit(main())
