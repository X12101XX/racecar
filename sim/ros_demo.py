"""Real ROS2 end-to-end simulation: world -> sensors -> mission -> gateway -> bicycle.
Runs real rclpy/messages in isolated Docker. World has truth; controller gets sensor topics only.
"""
from __future__ import annotations
import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import shutil
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Imu, Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
from visualization_msgs.msg import Marker, MarkerArray
from racelib.config import load_all,default_config_dir
from racelib.settings import MOTION_PRESETS,motion_settings,validate_field
from racelib.field import Field
from racelib.vision import CameraModel
from .world import SimWorld, SimRobotConfig, generate_cones
from .scenarios import demo_cones
from .camera import CameraRenderer


class WorldNode(Node):
    def __init__(self,args):
        super().__init__('crossfire_sim_world')
        self.args=args;self.out=Path(args.outdir);self.out.mkdir(parents=True,exist_ok=False)
        self.cfg=load_all(config_dir=args.config_dir,route=args.route)
        if args.motion_preset:
            self.cfg['plan'].data['ackermann']=dict(MOTION_PRESETS[args.motion_preset])
            self.cfg['plan'].data['control']['cross_speed_mps']=MOTION_PRESETS[args.motion_preset]['forward_mps']
        motion=motion_settings(self.cfg['plan']);self.motion=motion
        errors,_=validate_field(self.cfg['field'].data,self.cfg['robot'].get('geometry'))
        if errors:raise ValueError('; '.join(errors))
        self.runtime_config=self.out/'config';shutil.copytree(args.config_dir or default_config_dir(),self.runtime_config,dirs_exist_ok=True)
        (self.runtime_config/'plan.json').write_text(json.dumps(self.cfg['plan'].data,ensure_ascii=False,indent=2))
        f=Field.from_config(self.cfg['field'])
        (self.out/'events_live.jsonl').write_text('')
        robot=self.cfg['robot']
        physical=SimRobotConfig(drive_model='ackermann',length=float(robot.get('geometry.length_m',.42)),width=float(robot.get('geometry.width_m',.30)),wheelbase=float(robot.get('geometry.wheel_base_m',.30)),max_steer=float(robot.get('drive.max_steer_rad',.52)))
        if args.scenario=='configured':
            from .world import SimCone
            source=Path(args.config_dir or default_config_dir())/'sim_cones.json'
            configured=[SimCone(float(c['x']),float(c['y']),'unknown',radius_base=float(c.get('radius',.115)),radius_lidar=min(.085,float(c.get('radius',.115))*.74)) for c in json.loads(source.read_text())]
        self.world=SimWorld(f,configured if args.scenario=='configured' else demo_cones() if args.scenario=="demo" else generate_cones(f,np.random.default_rng(args.seed)),np.random.default_rng(args.seed),
                            robot=physical)
        is_mono = not (getattr(args, 'use_card', False) or getattr(args, 'color', False))
        self.is_mono = is_mono
        if not is_mono:
            self.cfg['plan'].data.setdefault('mission', {})['monochrome'] = False
            (self.runtime_config/'plan.json').write_text(json.dumps(self.cfg['plan'].data,ensure_ascii=False,indent=2))
        if args.ai_model:
            self.cfg['vision'].data.setdefault('ai',{})['model_path']=args.ai_model
        if args.ai_backend: self.cfg['vision'].data.setdefault('ai',{})['backend']=args.ai_backend
        args.ai_backend=self.cfg['vision'].get('ai.backend','auto')
        (self.runtime_config/'vision.json').write_text(json.dumps(self.cfg['vision'].data,indent=2))
        # Validate backend/model before starting armed simulation-only nodes.
        if not is_mono:
            from racelib.ai_vision import AIVisionEngine
            AIVisionEngine(args.ai_backend, self.cfg['vision'].get('ai.model_path',''))
        self.card_to_show = ('blue' if args.route == 'A' else 'red') if getattr(args, 'use_card', False) else None
        if args.card_color: self.card_to_show=args.card_color
        self.expected_route=('A' if self.card_to_show=='blue' else 'B') if args.use_card else args.route
        self.renderer=CameraRenderer(f,CameraModel(width=320,height=240),monochrome=is_mono)
        self.scan=self.create_publisher(LaserScan,'/scan',qos_profile_sensor_data)
        self.imu=self.create_publisher(Imu,'/imu/data',qos_profile_sensor_data)
        self.odom=self.create_publisher(Odometry,'/odom',qos_profile_sensor_data)
        self.image=self.create_publisher(Image,'/usb_cam/image_raw',qos_profile_sensor_data)
        self.create_subscription(Twist,'/crossfire/sim_cmd',self.cmd,1)
        self.create_subscription(String,'/crossfire/mission/state',self.state,10)
        self.create_subscription(String,'/crossfire/mission/events',self.event,10)
        self.create_subscription(MarkerArray,'/crossfire/mission/markers',self.on_markers,10)
        self.create_subscription(String,'/crossfire/gateway_state',self.gate,10)
        self.estop_pub=self.create_publisher(Bool,'/crossfire/estop',1)
        self.received_cmd=0;self.last_cmd=0.;self.command=(0.,0.);self.last_state={};self.events=[]
        self.mission_markers=[];self.estimated_trajectory=[]
        self.gateway=[];self.trajectory=[];self.wall0=time.monotonic();self.last=self.wall0
        self.last_image=0.;self.last_scan=0.;self.done=False;self.camp_truth=False
        self.frames=[];self.last_frame=0.;self.last_echo=0.;self.failure_test=None
        self.video_proc=subprocess.Popen(['ffmpeg','-y','-loglevel','error','-f','rawvideo','-pix_fmt','rgb24','-s','1200x650','-r','4','-i','-','-an','-vcodec','libx264','-pix_fmt','yuv420p',str(self.out/'video.partial.mp4')],stdin=subprocess.PIPE)
        self.create_timer(.01,self.tick)

    def cmd(self,msg):
        self.command=(msg.linear.x,msg.angular.z);self.received_cmd+=1;self.last_cmd=time.monotonic()
        if self.failure_test is not None:
            self.failure_test['observations'].append([time.monotonic(),msg.linear.x,msg.angular.z])
    def state(self,msg):
        try:
            d=json.loads(msg.data)
            if 'state' in d:self.last_state=d
        except ValueError:pass
    def event(self,msg):
        try:self.events.append(json.loads(msg.data))
        except ValueError:pass
    def on_markers(self,msg):
        self.mission_markers=list(msg.markers)
    def gate(self,msg):
        try:self.gateway.append([time.monotonic(),json.loads(msg.data)])
        except ValueError:pass

    def tick(self):
        now=time.monotonic();dt=min(1.,now-self.last);self.last=now
        # World also models MCU stop after 0.5s with no commands.
        self.world.set_cmd(*(self.command if now-self.last_cmd<.5 else (0.,0.)))
        # Preserve elapsed wall time; substeps maintain the bicycle dynamics stability.
        remaining=dt
        while remaining>1e-7:
            sub=min(.01,remaining);self.world.step(sub);remaining-=sub
        stamp=self.get_clock().now().to_msg();t=self.world.t
        if not (self.args.fault=='scan_drop' and t>4) and now-self.last_scan>=.10:
            q=self.world.lidar_scan(t);m=LaserScan();m.header.stamp=stamp;m.header.frame_id='base_link'
            # LaserScan has a uniform nominal angular grid; the per-beam angle
            # jitter is already reflected in the measured ranges.
            nominal=np.linspace(-math.pi,math.pi,len(q.ranges),endpoint=False)
            m.angle_min=float(nominal[0]);m.angle_max=float(nominal[-1]);m.angle_increment=float(nominal[1]-nominal[0]);m.range_min=.12;m.range_max=12.;m.ranges=q.ranges.astype(float).tolist();self.scan.publish(m);self.last_scan=now
        q=self.world.imu_sample(t);m=Imu();m.header.stamp=stamp;m.header.frame_id='base_link'
        m.orientation.z=math.sin(q.yaw/2);m.orientation.w=math.cos(q.yaw/2)
        m.angular_velocity.z=float(q.yaw_rate);m.linear_acceleration.x=float(q.accel[0]);m.linear_acceleration.y=float(q.accel[1]);m.linear_acceleration.z=float(q.accel[2]);self.imu.publish(m)
        q=self.world.odom_sample(t);m=Odometry();m.header.stamp=stamp;m.header.frame_id='odom';m.child_frame_id='base_link';m.twist.twist.linear.x=float(q.v);m.twist.twist.angular.z=float(q.w);self.odom.publish(m)
        if now-self.last_image>=.2:
            card_now = self.card_to_show if t < 2.5 else None
            a=self.renderer.render(self.world.cones,self.world.pose,card_color=card_now);m=Image();m.header.stamp=stamp;m.height=a.shape[0];m.width=a.shape[1];m.encoding='rgb8';m.step=a.shape[1]*3;m.data=a.tobytes();self.image.publish(m);self.last_image=now
        self.trajectory.append([t,*self.world.pose,self.world.v,self.world.w,*self.command])
        state_pose=self.last_state.get('pose')
        if isinstance(state_pose,list) and len(state_pose)>=3:
            self.estimated_trajectory.append([t,*state_pose[:3]])
        if now-self.last_frame>=.25:
            card_now = self.card_to_show if t < 2.5 else None
            self.save_frame(a if 'a' in locals() else self.renderer.render(self.world.cones,self.world.pose,card_color=card_now));self.last_frame=now
        camp=self.world.field.zone('camp_a' if self.expected_route=='A' else 'camp_b')
        if camp.distance_to_center(self.world.pose)<.25:self.camp_truth=True
        if now-self.last_echo>5:
            print(json.dumps(dict(t=round(t,1),pose=[round(v,2) for v in self.world.pose],state=self.last_state.get('state'),estimate=self.last_state.get('pose'),gate=self.gateway[-1][1] if self.gateway else None,cmd=self.command)),flush=True);self.last_echo=now
        if self.last_state.get('state') in ('DONE','FAILED'):self.done=True

    def save_frame(self,a):
        from PIL import Image as PI,ImageDraw,ImageFont
        canvas=PI.new('RGB',(1200,650),'#10151e');d=ImageDraw.Draw(canvas)
        try:
            d.font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',18)
            small=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',13)
        except OSError:
            d.font=ImageFont.load_default();small=d.font
        # Fixed scale layout; all field materials are grey in this online mode.
        scale=min(525/self.world.field.length,350/self.world.field.width);ox=30;oy=590
        def xy(x,y):return (ox+x*scale,oy-y*scale)
        f=self.world.field
        d.polygon([xy(*p) for p in (f.boundary or [[0,0],[f.length,0],[f.length,f.width],[0,f.width]])],fill='#d8d8d8',outline='white')
        for name,z in f.zones.items():
            d.polygon([xy(*p) for p in z.corners()],outline='#505050');d.text(xy(z.cx-.36,z.cy),{'stronghold_a':'Base A','stronghold_b':'Base B','camp_a':'Camp A','camp_b':'Camp B'}[name],fill='#111111',font=small)
        # Top-down panel: mission markers/estimated pose only.  World truth is
        # kept in world.json/summary for scoring, never drawn as robot state.
        for m in self.mission_markers:
            if m.type != Marker.CYLINDER:
                continue
            x,y=xy(m.pose.position.x,m.pose.position.y)
            r=max(4.,float(m.scale.x)*scale*.5)
            fill=tuple(int(255*max(0.,min(1.,c))) for c in (m.color.r,m.color.g,m.color.b))
            d.ellipse((x-r,y-r,x+r,y+r),fill=fill,outline='#dddddd')
        pose_est=self.last_state.get('pose')
        if isinstance(pose_est,list) and len(pose_est)>=3:
            x,y,yaw=map(float,pose_est[:3])
            from racelib.geometry import rect_corners
            corners=[xy(p[0],p[1]) for p in rect_corners(x,y,self.world.robot.length,self.world.robot.width,yaw)]
            d.line(corners+[corners[0]],fill='#007d9c',width=4)
            if len(self.estimated_trajectory)>2:
                d.line([xy(p[1],p[2]) for p in self.estimated_trajectory[::4]],fill='#007d9c',width=2)
            # Distinguish the physical front camera from the signed travel direction.
            def arrow(angle,length,color):
                a0=xy(x,y);b=xy(x+length*math.cos(angle),y+length*math.sin(angle))
                ux,uy=math.cos(angle),-math.sin(angle)
                d.line([a0,b],fill=color,width=3)
                d.polygon([b,(b[0]-9*ux+4*uy,b[1]-9*uy-4*ux),
                             (b[0]-9*ux-4*uy,b[1]-9*uy+4*ux)],fill=color)
            for angle in (yaw-math.pi/4,yaw+math.pi/4):
                d.line([xy(x,y),xy(x+.85*math.cos(angle),y+.85*math.sin(angle))],fill='#bf7527',width=1)
            arrow(yaw,.48,'#e59626')
            movement='STOP'
            if abs(self.command[0])>.025:
                movement='FORWARD' if self.command[0]>0 else 'REVERSE'
                arrow(yaw if self.command[0]>0 else yaw+math.pi,.75,'#008eb8')
        else:
            movement='WAIT'
        cam=PI.fromarray(a).resize((600,450));canvas.paste(cam,(590,180))
        d.text((30,25),f'ROS2 Humble | Route {self.expected_route} | {"MONOCHROME" if self.is_mono else "COLOR"} | ACKERMANN',fill='white')
        d.text((30,65),f't={self.world.t:.1f}s  state={self.last_state.get("state","WAITING")}  sensor map {sum(1 for m in self.mission_markers if m.type==Marker.CYLINDER)} cones',fill='white')
        d.text((30,100),f'command SI: v={self.command[0]:.2f} m/s, w={self.command[1]:.2f} rad/s',fill='white')
        d.text((30,135),f'drive={movement} | orange: body front / camera | cyan: estimated travel',fill='white',font=small)
        d.text((590,155),'FRONT CAMERA - fixed to body heading',fill='#e59626',font=small)
        self.frames=[canvas]
        self.video_proc.stdin.write(canvas.tobytes())

    def finish(self,reason):
        s=self.last_state;done=s.get('state')=='DONE'
        is_mono = getattr(self, 'is_mono', True)
        from racelib.geometry import rect_corners
        camp=self.world.field.zone('camp_a' if self.expected_route=='A' else 'camp_b')
        body_in_camp=all(camp.contains(p) for p in rect_corners(self.world.x,self.world.y,
            self.world.robot.length,self.world.robot.width,self.world.yaw))
        selected_route=s.get('metrics',{}).get('route')
        summary=dict(route=self.expected_route,selected_route=selected_route,
                     simulation_namespace=os.environ.get('CROSSFIRE_SIM_NAMESPACE',''),
                     node_launcher='direct_python' if os.environ.get('CROSSFIRE_NODE_ENTRY') else 'ros2_run',
                     physical_output_publishers_at_end={t:self.count_publishers(t) for t in ('/car_cmd_vel','/cmd_vel')},
                     finish_body_in_camp_truth=body_in_camp,
                     initial_route=self.args.route,seed=self.args.seed,transport='real ROS2 Humble DDS',monochrome=is_mono,
                     use_card=getattr(self.args, 'use_card', False),ai_backend_requested=self.args.ai_backend,ai_backend=s.get('metrics',{}).get('ai_backend'),
                     drive_model='ackermann',
                     final_state=s.get('state'),success=done and body_in_camp and selected_route==self.expected_route and abs(self.world.v)<.025 and not self.world.contacts and not self.world.wall_contacts,
                     reached_camp_truth=self.camp_truth,cone_contacts=len(self.world.contacts),wall_contacts=self.world.wall_contacts,
                     sim_time=self.world.t,metrics=s.get('metrics'),received_commands=self.received_cmd,reason=reason)
        forward=reverse=0.
        for a,b in zip(self.trajectory,self.trajectory[1:]):
            distance=math.hypot(b[1]-a[1],b[2]-a[2])
            if b[4]<-.01:reverse+=distance
            elif b[4]>.01:forward+=distance
        summary['travel_distance_m']={'forward':round(forward,3),'reverse':round(reverse,3)}
        (self.out/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2))
        (self.out/'events.json').write_text(json.dumps(self.events,ensure_ascii=False,indent=2))
        (self.out/'gateway.json').write_text(json.dumps(self.gateway,indent=2))
        (self.out/'world.json').write_text(json.dumps([dict(x=c.x,y=c.y,role=c.color,line=c.line) for c in self.world.cones],indent=2))
        with (self.out/'trajectory.csv').open('w') as f:
            w=csv.writer(f);w.writerow(['t','x','y','yaw','v','w','cmd_v','cmd_w']);w.writerows(self.trajectory)
        self.save_frame(self.renderer.render(self.world.cones,self.world.pose))
        if self.frames:
            self.frames[-1].save(self.out/'final.png')
        self.video_proc.stdin.close()
        code=self.video_proc.wait(timeout=30)
        if code: raise RuntimeError(f'ffmpeg encoding failed ({code}); partial video retained')
        (self.out/'video.partial.mp4').replace(self.out/'video.mp4')
        (self.out/'completed.json').write_text(json.dumps({'video_finalized':True,'reason':reason}))
        return summary


SIM_TOPICS = ('/scan','/imu/data','/odom','/usb_cam/image_raw','/crossfire/sim_cmd',
    '/crossfire/desired_cmd','/crossfire/mission/state','/crossfire/mission/events',
    '/crossfire/mission/announce','/crossfire/mission/markers','/crossfire/gateway_state',
    '/crossfire/estop','/teleop_cmd_vel')


def simulation_remaps(env):
    prefix=env.get('CROSSFIRE_SIM_NAMESPACE','')
    if not prefix:return []
    result=['-r','__ns:='+prefix]
    for topic in SIM_TOPICS:result+=['-r',topic+':='+prefix+topic]
    return result


def launch_node(executable,params,log,env):
    entry=env.get('CROSSFIRE_NODE_ENTRY')
    cmd=([sys.executable,entry,executable] if entry else ['ros2','run','racecar_mission',executable])+['--ros-args']
    cmd+=simulation_remaps(env)
    for k,v in params.items():cmd+=['-p',f'{k}:={str(v).lower() if isinstance(v,bool) else v}']
    return subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,env=env,start_new_session=True)


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--scenario',choices=['demo','random','configured'],default='demo')
    ap.add_argument('--config-dir')
    ap.add_argument('--motion-preset',choices=list(MOTION_PRESETS))
    ap.add_argument('--route',choices=['A','B'],default='A')
    ap.add_argument('--use-card',action='store_true',default=False)
    ap.add_argument('--color',action='store_true',default=False)
    ap.add_argument('--ai-backend',choices=['auto','npu','onnx','fast_color','cpu'],default='')
    ap.add_argument('--ai-model',default='')
    ap.add_argument('--card-color',choices=['blue','red'])
    ap.add_argument('--planner-engine',choices=['auto','native','python'],default='auto')
    ap.add_argument('--seed',type=int,default=3)
    ap.add_argument('--outdir',default='/output/ros_'+time.strftime('%Y%m%d-%H%M%S'))
    ap.add_argument('--max-time',type=float,default=180)
    ap.add_argument('--fault',choices=['none','mission_kill','scan_drop','estop','competing_publisher'],default='none')
    args=ap.parse_args()
    os.environ['CROSSFIRE_PLANNER_ENGINE']=args.planner_engine
    rclpy.init(args=['--ros-args',*simulation_remaps(os.environ)] if os.environ.get('CROSSFIRE_SIM_NAMESPACE') else None);node=WorldNode(args);procs=[];logs=[];env=os.environ.copy();env['CROSSFIRE_RUN_DIR']=args.outdir
    try:
        for executable,params in [
            ('gateway',dict(armed=True,hardware_watchdog_confirmed=True,calibration_confirmed=True,output_topic='/crossfire/sim_cmd',output_mode='yaw_rate',allow_reverse=True,wheelbase=node.world.robot.wheelbase,max_steer=node.world.robot.max_steer,body_length=node.world.robot.length,body_width=node.world.robot.width,max_speed=node.motion['forward_mps'],reverse_limit=node.motion['reverse_mps'],brake_decel=node.motion['brake_mps2'])),
            ('mission_node',dict(route=args.route,use_card=args.use_card,monochrome=node.is_mono,ai_backend=args.ai_backend,publish_cmd=True,tts='none',config_dir=str(node.runtime_config),**{'topics.scan':'/scan','topics.imu':'/imu/data','topics.odom':'/odom','topics.image':'/usb_cam/image_raw','topics.cmd_vel':'/crossfire/desired_cmd'}))]:
            prefix=env.get('CROSSFIRE_SIM_NAMESPACE','')
            if prefix:
                # Keep the mission's private output contract literal; DDS remaps
                # its publisher into the isolated simulation namespace.
                params={k:(prefix+v if isinstance(v,str) and v.startswith('/') and k!='topics.cmd_vel' and (k.endswith('_topic') or k.startswith('topics.')) else v) for k,v in params.items()}
            log=(node.out/f'{executable}.log').open('w');logs.append(log);procs.append(launch_node(executable,params,log,env))
        deadline=time.monotonic()+args.max_time;fault_start=None;observations=[]
        while not node.done and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.01)
            if args.fault!='none' and node.world.t>4 and (fault_start is not None or abs(node.command[0])>.08):
                if fault_start is None:
                    fault_start=time.monotonic();node.failure_test=dict(start=fault_start,observations=[],initial_command=list(node.command))
                    if args.fault=='mission_kill':os.killpg(procs[1].pid,signal.SIGKILL)
                    if args.fault=='estop':node.estop_pub.publish(Bool(data=True))
                    if args.fault=='competing_publisher':node.competing_pub=node.create_publisher(Twist,'/crossfire/sim_cmd',1)
                if time.monotonic()-fault_start>2:break
            if any(p.poll() is not None for i,p in enumerate(procs) if not(args.fault=='mission_kill' and i==1)):
                raise RuntimeError('ROS2 child exited; inspect logs')
        result=node.finish('fault_test' if fault_start else 'mission_done' if node.last_state.get('state')=='DONE' else 'mission_failed' if node.done else 'timeout')
        if fault_start:
            obs=node.failure_test['observations'];zeros=[a[0]-fault_start for a in obs if a[0]>=fault_start and abs(a[1])+abs(a[2])<1e-6]
            late=[a for a in obs if a[0]>fault_start+.65]
            report=dict(fault=args.fault,initial_command=node.failure_test['initial_command'],stopped_speed=abs(node.world.v),first_zero_s=min(zeros) if zeros else None,stayed_zero=bool(late) and all(abs(a[1])+abs(a[2])<1e-6 for a in late),gateway_faults=sorted(set(s[1]['fault'] for s in node.gateway if s[0]>fault_start+.65)))
            report['passed']=abs(report['initial_command'][0])>.08 and bool(zeros) and min(zeros)<.65 and report['stayed_zero'] and report['stopped_speed']<.025
            (node.out/'fault_test.json').write_text(json.dumps(report,indent=2));print(report,flush=True)
            return 0 if report['passed'] else 1
        print(json.dumps(result,ensure_ascii=False,indent=2));return 0 if result['success'] else 1
    finally:
        for p in reversed(procs):
            if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
        end=time.monotonic()+4
        for p in procs:
            try:p.wait(timeout=max(.1,end-time.monotonic()))
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
        for f in logs:f.close()
        if node.video_proc.poll() is None:
            node.video_proc.stdin.close()
            node.video_proc.wait(timeout=10)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':sys.exit(main())
