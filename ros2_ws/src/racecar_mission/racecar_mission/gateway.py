"""Separate wall-clock safety process. Only this optional node touches chassis output.
Default is unarmed and creates NO chassis publisher (including zero commands).
"""
from __future__ import annotations
import math
import time
import json
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String


from racelib.safety_policy import CommandPolicy


class Gateway(Node):
    def __init__(self):
        super().__init__('crossfire_gateway')
        defaults = dict(armed=False, hardware_watchdog_confirmed=False, calibration_confirmed=False,
                        output_topic='/car_cmd_vel', desired_topic='/crossfire/desired_cmd',
                        output_mode='legacy_steering', scan_topic='/scan', imu_topic='/imu/data',
                        odom_topic='/odom', command_timeout=.30, sensor_timeout=.50,
                        wheelbase=.30, max_steer=.52, max_speed=.50, reverse_limit=.20,
                        body_length=.42, body_width=.30, allow_reverse=False,brake_decel=.6)
        for k,v in defaults.items():
            self.declare_parameter(k,v)
        self.p = {k:self.get_parameter(k).value for k in defaults}
        self.armed = bool(self.p['armed'])
        if self.armed and not self.p['calibration_confirmed']:
            raise RuntimeError('拒绝实车输出：尚未确认几何尺寸、舵机映射和传感器坐标标定')
        if self.armed and not self.p['hardware_watchdog_confirmed']:
            raise RuntimeError('拒绝实车输出：须先验证底盘驱动失联后自行停车')
        if self.p['output_mode'] not in ('legacy_steering','yaw_rate'):
            raise ValueError('未知底盘指令模式')
        for k in ('command_timeout','sensor_timeout','wheelbase','max_steer','max_speed','body_length','body_width'):
            if not math.isfinite(self.p[k]) or self.p[k] <= 0:
                raise ValueError('非法安全参数: '+k)
        if self.p['desired_topic'] == self.p['output_topic']:
            raise ValueError('输入输出话题不能相同')
        if not .1<=self.p['brake_decel']<=.6:raise ValueError('制动估计必须在0.1～0.6 m/s²内')
        self.policy = CommandPolicy(self.p['wheelbase'],self.p['max_steer'],self.p['max_speed'],self.p['reverse_limit'])
        self.pub = self.create_publisher(Twist,self.p['output_topic'],1) if self.armed else None
        self.status = self.create_publisher(String,'/crossfire/gateway_state',1)
        self.last = {}; self.command=(0.,0.); self.points=None
        self.estop=False; self.fault=''; self.odom_v=0.
        self.start=time.monotonic(); self._last_status=0.
        self.create_subscription(Twist,self.p['desired_topic'],self.on_cmd,1)
        self.create_subscription(LaserScan,self.p['scan_topic'],self.on_scan,qos_profile_sensor_data)
        self.create_subscription(Imu,self.p['imu_topic'],lambda m:self.on_sensor('imu',m),qos_profile_sensor_data)
        self.create_subscription(Odometry,self.p['odom_topic'],self.on_odom,qos_profile_sensor_data)
        self.create_subscription(Bool,'/crossfire/estop',self.on_estop,1)
        self.create_timer(.02,self.tick)

    def on_estop(self,msg):
        # Latch: restarting the explicitly armed run is required to clear emergency stop.
        if msg.data:self.estop=True

    def stamp_valid(self,msg):
        stamp=msg.header.stamp
        t=stamp.sec+stamp.nanosec*1e-9
        now=self.get_clock().now().nanoseconds*1e-9
        return t > 0 and -.15 <= now-t <= self.p['sensor_timeout']

    def on_sensor(self,key,msg):
        if not self.stamp_valid(msg):return
        if key=='imu' and not all(math.isfinite(v) for v in (msg.angular_velocity.z,msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z)):
            return
        self.last[key]=time.monotonic()

    def on_odom(self,msg):
        if not math.isfinite(msg.twist.twist.linear.x):return
        self.odom_v=msg.twist.twist.linear.x
        self.on_sensor('odom',msg)

    def on_scan(self,msg):
        if not self.stamp_valid(msg) or len(msg.ranges)<30 or msg.angle_increment==0:return
        r=np.array(msg.ranges,float)
        valid=np.isfinite(r)&(r>=msg.range_min)&(r<=msg.range_max)
        if np.sum(valid)<len(r)*.35:return
        a=msg.angle_min+np.arange(len(r))*msg.angle_increment
        self.points=np.stack([r[valid]*np.cos(a[valid]),r[valid]*np.sin(a[valid])],axis=1)
        self.scan_a=a[valid]
        self.last['scan']=time.monotonic()

    def on_cmd(self,msg):
        if not all(math.isfinite(x) for x in (msg.linear.x,msg.angular.z)):
            self.estop=True;return
        self.command=(msg.linear.x,msg.angular.z); self.last['cmd']=time.monotonic()

    def collision(self,v,w):
        from racelib.ackermann import integrate
        if self.points is None:return True
        if v < 0 and not self.p['allow_reverse']:return True
        # Need observed scan coverage in travel direction; no return does not mean clear.
        a=self.scan_a
        if v >= 0:
            coverage=np.sum(np.abs(a)<math.pi/3)
        else:
            coverage=np.sum(np.abs(np.abs(a)-math.pi)<math.pi/3)
        if coverage<10:return True
        k=w/v if abs(v)>.01 else 0.
        # Use measured speed too, so current momentum is covered after lowering a command.
        speed=max(abs(v),abs(self.odom_v))
        distance=speed*.50+speed*speed/(2*self.p['brake_decel'])+.06
        for d in np.linspace(0,math.copysign(distance,v),12):
            x,y,yaw=integrate((0.,0.,0.),float(d),k)
            c,s=math.cos(yaw),math.sin(yaw)
            q=self.points-np.array([x,y])
            bx=c*q[:,0]+s*q[:,1];by=-s*q[:,0]+c*q[:,1]
            if np.any((np.abs(bx)<self.p['body_length']/2+.06)&(np.abs(by)<self.p['body_width']/2+.06)):
                return True
        return False

    def tick(self):
        now=time.monotonic();reason=''
        if self.estop:reason='latched_estop'
        for key in ('scan','imu','odom','cmd'):
            limit=self.p['command_timeout'] if key=='cmd' else self.p['sensor_timeout']
            if now-self.last.get(key,-1e9)>limit:reason='stale_'+key
        if self.armed:
            # Do not stop or reconfigure other nodes. Refuse shared control instead.
            if self.count_publishers(self.p['output_topic'])>1 or self.count_publishers('/teleop_cmd_vel')>0:
                self.estop=True;reason='competing_publisher'
            if self.count_subscribers(self.p['output_topic']) != 1:
                reason='expected_one_chassis_subscriber'
        v,w=self.command
        if not reason and (abs(v)>.001 or abs(w)>.001) and self.collision(v,w):
            reason='swept_footprint_or_coverage'
        if reason:v=w=0.
        v,w=self.policy.convert(v,w,self.p['output_mode'])
        self.fault=reason
        if self.pub is not None:
            msg=Twist();msg.linear.x=float(v);msg.angular.z=float(w);self.pub.publish(msg)
        if now-self._last_status>.25:
            self.status.publish(String(data=json.dumps(dict(armed=self.armed,fault=reason,output=[v,w]))))
            self._last_status=now

    def stop(self):
        if self.pub is not None:
            for _ in range(5):self.pub.publish(Twist());time.sleep(.02)


def main(args=None):
    from rclpy.signals import SignalHandlerOptions
    import signal
    def interrupted(signum,frame):raise KeyboardInterrupt
    rclpy.init(args=args,signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT,interrupted)
    signal.signal(signal.SIGTERM,interrupted)
    node=None
    try:
        node=Gateway();rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if node is not None:
            node.stop();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
