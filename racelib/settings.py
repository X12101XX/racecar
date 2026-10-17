"""Shared motion knobs and geometry-only validation. No ROS or filesystem writes."""
import copy
import math

MOTION_PRESETS = {
    'conservative': dict(forward_mps=.28, corner_mps=.18, reverse_mps=.12, accel_mps2=.35,
                         brake_mps2=.6, lookahead_m=.18, planning_margin_m=.13, arrival_m=.16, shift_pause_s=.5),
    'standard': dict(forward_mps=.40, corner_mps=.22, reverse_mps=.20, accel_mps2=.6,
                     brake_mps2=.6, lookahead_m=.16, planning_margin_m=.10, arrival_m=.18, shift_pause_s=.35),
    'aggressive': dict(forward_mps=.65, corner_mps=.40, reverse_mps=.25, accel_mps2=1.,
                       brake_mps2=.6, lookahead_m=.30, planning_margin_m=.12, arrival_m=.18, shift_pause_s=.25),
}
MOTION_LIMITS = {'forward_mps':(.05,1.5), 'corner_mps':(.05,1.), 'reverse_mps':(.03,.5),
                 'accel_mps2':(.1,2.), 'brake_mps2':(.1,.6), 'lookahead_m':(.12,.7),
                 'planning_margin_m':(.06,.35), 'arrival_m':(.04,.25), 'shift_pause_s':(.2,2.)}


def motion_settings(plan):
    data = plan.data if hasattr(plan,'data') else plan
    m=copy.deepcopy(MOTION_PRESETS['standard']);m.update(data.get('ackermann',{}))
    for key,(lo,hi) in MOTION_LIMITS.items():
        x=m[key]
        if isinstance(x,bool) or not isinstance(x,(int,float)) or not math.isfinite(x) or not lo<=x<=hi:
            raise ValueError(f'运动参数 {key} 必须在 {lo}～{hi} 之间')
    if m['corner_mps']>m['forward_mps'] or m['reverse_mps']>m['forward_mps']:
        raise ValueError('弯道/倒车速度不能超过直行速度')
    return m


def boundary(field):
    a=field['arena']
    return a.get('boundary') or [[0.,0.],[a['length_m'],0.],[a['length_m'],a['width_m']],[0.,a['width_m']]]


def cross(a,b,c):return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])


def inside(p,poly,margin=0.):
    signs=[cross(a,b,p)/max(1e-12,math.hypot(b[0]-a[0],b[1]-a[1])) for a,b in zip(poly,poly[1:]+poly[:1])]
    return min(signs)>=margin-1e-8 or max(signs)<=-margin+1e-8


def rect_corners(center,size,heading=0.):
    c,s=math.cos(math.radians(heading)),math.sin(math.radians(heading))
    return [[center[0]+c*x-s*y,center[1]+s*x+c*y] for x,y in
            [(-size[0]/2,-size[1]/2),(size[0]/2,-size[1]/2),(size[0]/2,size[1]/2),(-size[0]/2,size[1]/2)]]


def validate_field(f, geometry=None):
    """Return physical errors and independent official-rule warnings."""
    errors=[];warnings=[]
    try:
        def finite_tree(v):
            if isinstance(v,float) and not math.isfinite(v):raise ValueError('配置不能包含 NaN/Infinity')
            if isinstance(v,dict):
                for x in v.values():finite_tree(x)
            if isinstance(v,list):
                for x in v:finite_tree(x)
        finite_tree(f)
        a=f['arena'];L,W=float(a['length_m']),float(a['width_m'])
        if not(.8<=L<=30 and .8<=W<=30):raise ValueError('场地包围框长宽应在 0.8～30m 内')
        poly=boundary(f)
        if len(poly)!=4 or any(len(v)!=2 for v in poly):raise ValueError('围栏请填写按顺/逆时针排序的四个角点')
        turns=[cross(poly[i],poly[(i+1)%4],poly[(i+2)%4]) for i in range(4)]
        if not(min(turns)>1e-6 or max(turns)<-1e-6):raise ValueError('围栏四角必须按顺序组成凸四边形，不能交叉或重合')
        if any(not(0<=x<=L and 0<=y<=W) for x,y in poly):errors.append('围栏角点超出场地包围框')
        zones={'start':f['start'],**f['zones']}
        for key in ('stronghold_a','stronghold_b','camp_a','camp_b'):
            if key not in f['zones']:errors.append('缺少区域 '+key)
        for name,z in zones.items():
            size=z['size'];center=z['center']
            if len(size)!=2 or len(center)!=2 or min(size)<=0:raise ValueError(name+' 的尺寸/中心无效')
            heading=z.get('heading_deg',0.) if name!='start' else 0.
            if not all(inside(p,poly) for p in rect_corners(center,size,heading)):
                errors.append(name+' 超出围栏边界')
            if name!='start' and geometry:
                radius=math.hypot(geometry['length_m'],geometry['width_m'])/2
                if min(size)/2<radius+.055+.04:errors.append(name+' 太小，不能容纳车辆和到点误差')
            if name!='start' and any(abs(v-1.)>.01 for v in size):warnings.append(name+' 尺寸不是赛题的 1×1m')
        if geometry:
            if not all(inside(p,poly,.055) for p in rect_corners(f['start']['center'],[geometry['length_m'],geometry['width_m']],f['start'].get('heading_deg',0.))):
                errors.append('起点朝向下，车体和保护余量超出围栏')
        mf=f['minefield'];lo,hi=float(mf['x_min']),float(mf['x_max'])
        if not 0<=lo<hi<=L:errors.append('雷区 x 范围必须在场地内且左小右大')
        xs=mf['blockade_lines_x']
        if len(xs)!=3 or any(not lo<x<hi for x in xs) or sorted(set(xs))!=xs:
            errors.append('三条封锁线 x 坐标应递增且位于雷区内（不要求等间距）')
        if not(6<=L<=8 and 3<=W<=5):warnings.append('场地尺寸偏离赛题 6～8m × 3～5m，仅作提示')
        if abs(hi-lo-4)>.05:warnings.append('雷区宽度不是 4m，仅作提示')
        if len(xs)==3 and any(abs(b-a-1.5)>.01 for a,b in zip(xs,xs[1:])):warnings.append('封锁线间距不是 1.5m，仅作提示')
        if a.get('boundary'):warnings.append('已使用四角实测围栏；定位仍需正确的起点、IMU/里程计标定')
    except (KeyError,TypeError,ValueError,IndexError,OverflowError) as e:errors.append(str(e))
    return errors,warnings
