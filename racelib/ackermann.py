"""Bicycle-constrained Hybrid A* and mission. Commands are SI (m/s, rad/s).
The legacy PWM/steering conversion belongs ONLY in the ROS safety gateway.
"""
from __future__ import annotations
import os
import heapq
import math
import time
import numpy as np
from . import geometry as G
from .mission import Mission, State
from .settings import motion_settings,validate_field


def integrate(pose, distance, curvature):
    x, y, a = pose
    da = distance * curvature
    if abs(curvature) < 1e-8:
        return (x + distance * math.cos(a), y + distance * math.sin(a), a)
    return (x + (math.sin(a + da) - math.sin(a)) / curvature,
            y + (math.cos(a) - math.cos(a + da)) / curvature,
            G.wrap_pi(a + da))


class CollisionWorld:
    def __init__(self, field, cones, length=.42, width=.30, margin=.06):
        self.field, self.length, self.width = field, length, width
        self.margin = margin
        self.obstacles = np.array([(c.x, c.y, max(.115, c.radius)) for c in cones], float).reshape(-1, 3)
        self._cones = self.obstacles.tolist()

    def clear(self, p):
        x, y, a = p
        ca, sa = math.cos(a), math.sin(a)
        hl, hw = self.length / 2 + self.margin, self.width / 2 + self.margin
        ex, ey = abs(ca) * hl + abs(sa) * hw, abs(sa) * hl + abs(ca) * hw
        if x < ex or y < ey or x > self.field.length - ex or y > self.field.width - ey:
            return False
        if self.field.boundary:
            # A convex measured quadrilateral contains the full rectangle iff all corners fit.
            for dx,dy in ((hl,hw),(hl,-hw),(-hl,hw),(-hl,-hw)):
                if not self.field.inside_arena((x+ca*dx-sa*dy,y+sa*dx+ca*dy)):return False
        bound=math.hypot(hl,hw)
        for cx,cy,r in self._cones:
            dx,dy=cx-x,cy-y
            if abs(dx)>bound+r or abs(dy)>bound+r:continue
            bx,by=ca*dx+sa*dy,-sa*dx+ca*dy
            qx,qy=max(abs(bx)-hl,0.),max(abs(by)-hw,0.)
            if qx*qx+qy*qy<=r*r:return False
        return True

    def edge(self, p, distance, curvature):
        n=max(1,int(abs(distance)/.04)+1)
        for i in range(n+1):
            if not self.clear(integrate(p, distance*i/n, curvature)):
                return False
        return True


def hybrid_astar(start, goal, world, wheelbase=.30, steer_limit=.52,
                 allow_reverse=True, max_nodes=45000, timeout=3.0,
                 max_reverse_m=.66, prefer_forward=True):
    """Search (x,y,heading,direction); every edge checks swept rectangular footprint.
    Try a forward-only path first. Reverse is a bounded maneuver fallback, not
    a cheaper way to traverse the whole field with the front camera facing away.
    Goal is position-only: arrival heading is free. No zero-radius rotation edges.
    """
    if not math.isfinite(max_reverse_m) or max_reverse_m < 0:
        raise ValueError('倒车路径长度上限必须为非负有限数')

    if not world.clear(start):
        return None
    engine = planner_engine()
    if engine == 'cpp_native':
        from .native_planner import c_hybrid_astar
        return c_hybrid_astar(start, goal, world, wheelbase, steer_limit, allow_reverse,
                             max_nodes, timeout, max_reverse_m, prefer_forward)
    return _python_hybrid_astar(start, goal, world, wheelbase, steer_limit, allow_reverse,
                               max_nodes, timeout, max_reverse_m, prefer_forward)


def planner_engine():
    from .native_planner import is_native_available
    requested = os.environ.get('CROSSFIRE_PLANNER_ENGINE', 'auto')
    if requested not in ('auto','native','python'):
        raise ValueError('planner engine must be auto, native or python')
    if requested == 'native' and not is_native_available():
        raise RuntimeError('C++ planner unavailable; build the native library for this environment')
    return 'cpp_native' if requested != 'python' and is_native_available() else 'python'


def _python_hybrid_astar(start, goal, world, wheelbase, steer_limit,
                         allow_reverse, max_nodes, timeout, max_reverse_m, prefer_forward):
    begin = time.monotonic()
    if allow_reverse and prefer_forward:
        started=time.monotonic()
        forward=_python_hybrid_astar(start,goal,world,wheelbase,steer_limit,False,
                             max_nodes,timeout*.6,max_reverse_m,False)
        if forward is not None:return forward
        timeout-=time.monotonic()-started
        if timeout<=0:return None
        begin=time.monotonic()
    curvature = math.tan(steer_limit) / wheelbase
    step = .22
    reverse_budget=int((max_reverse_m+1e-8)/step)
    def key(p, gear, reversed_steps):
        return (round(p[0] / .09), round(p[1] / .09),
                round(G.wrap_pi(p[2]) / (math.pi / 24)) % 48, gear, reversed_steps)
    # Obstacle-aware 2D cost-to-go is only a heuristic; footprint safety is exact below.
    resolution=.12
    nx,ny=int(world.field.length/resolution)+1,int(world.field.width/resolution)+1
    gx,gy=np.meshgrid(np.arange(nx)*resolution,np.arange(ny)*resolution,indexing='ij')
    occupied=np.zeros((nx,ny),bool)
    for cx,cy,cr in world.obstacles:
        occupied |= (gx-cx)**2+(gy-cy)**2 < (cr+.19)**2
    dist=np.full((nx,ny),np.inf)
    ix,iy=min(nx-1,max(0,round(goal[0]/resolution))),min(ny-1,max(0,round(goal[1]/resolution)))
    dist[ix,iy]=0.;heap=[(0.,ix,iy)]
    while heap:
        value,x,y=heapq.heappop(heap)
        if value>dist[x,y]+1e-8:continue
        for dx,dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
            xx,yy=x+dx,y+dy
            if not(0<=xx<nx and 0<=yy<ny) or occupied[xx,yy]:continue
            nv=value+resolution*math.hypot(dx,dy)
            if nv<dist[xx,yy]:
                dist[xx,yy]=nv;heapq.heappush(heap,(nv,xx,yy))
    def heuristic(p):
        euclidean=math.hypot(p[0]-goal[0],p[1]-goal[1])
        ix,iy=min(nx-1,max(0,round(p[0]/resolution))),min(ny-1,max(0,round(p[1]/resolution)))
        return max(euclidean,float(dist[ix,iy])) if math.isfinite(dist[ix,iy]) else euclidean
    # Separate node IDs prevent parent changes corrupting continuous edge chains.
    nodes = [(tuple(start), 1, -1, 0., 0., 0)]
    best = {key(start, 1, 0): 0.}
    q = [(heuristic(start), 0)]
    while q and len(nodes) < max_nodes:
        if time.monotonic() - begin > timeout:
            return None
        _, idx = heapq.heappop(q)
        p, gear, parent, cost, last_k, reversed_steps = nodes[idx]
        if cost > best.get(key(p, gear, reversed_steps), math.inf) + 1e-6:
            continue
        if heuristic(p) < .12:
            path = []
            while idx >= 0:
                p, gear, parent, _, k, _ = nodes[idx]
                if parent >= 0:
                    pp = nodes[parent][0]
                    path.extend([(integrate(pp, d, k), gear) for d in np.linspace(step*gear, .04*gear, 6)])
                idx = parent
            path.append((tuple(start), path[-1][1] if path else 1))
            return list(reversed(path))
        for direction in ((1, -1) if allow_reverse else (1,)):
            next_reversed=reversed_steps+int(direction<0)
            if next_reversed>reverse_budget:continue
            for k in (-curvature, -.5*curvature, 0., .5*curvature, curvature):
                if not world.edge(p, direction * step, k):
                    continue
                nxt = integrate(p, direction * step, k)
                nc = cost + step * (1.0 if direction == 1 else 1.5)
                nc += (.45 if direction != gear else 0) + .025 * abs(k-last_k)
                ky = key(nxt, direction, next_reversed)
                if nc + 1e-6 >= best.get(ky, math.inf):
                    continue
                best[ky] = nc
                nodes.append((nxt, direction, idx, nc, k, next_reversed))
                heapq.heappush(q, (nc + 1.8 * heuristic(nxt), len(nodes)-1))
    return None


class AckermannMission(Mission):
    """All cones are obstacles. No card, ground-color, or obstacle-color dependency."""
    def __init__(self, io, cfgs, mission_cfg=None):
        super().__init__(io, cfgs, mission_cfg)
        errors,_=validate_field(cfgs["field"].data,cfgs["robot"].get("geometry"))
        if errors:raise ValueError("; ".join(errors))
        self.motion=motion_settings(cfgs["plan"])
        self.cfg.cross_speed_mps=self.motion["forward_mps"]
        self.wheelbase = float(cfgs['robot'].get('geometry.wheel_base_m', .30))
        self.steer_limit = float(cfgs['robot'].get('drive.max_steer_rad', .52))
        self.allow_reverse = bool(cfgs['robot'].get('drive.allow_reverse', True))
        self.max_reverse_path_m=float(cfgs['robot'].get('drive.max_reverse_path_m',.66))
        if not math.isfinite(self.max_reverse_path_m) or not 0<=self.max_reverse_path_m<=1.1:
            raise ValueError('drive.max_reverse_path_m 必须在0～1.1m内')
        self._hybrid_path = None
        self._segment = []
        self._next_index = 0
        self._last_goal = None
        self._last_attempt = -1e9
        self._last_direction = 0
        self._switch_until = -1e9
        self._search_backoff = -1e9
        self._speed = 0.
        self._plan_future = None
        self._executor = None
        self._failed_plans = 0
        self.async_planning = bool(getattr(io, "params", {}).get("async_planning", False))
        self.metrics['drive_model'] = 'ackermann'
        self.metrics['monochrome'] = bool(self.cfg.monochrome)
        self.metrics['use_card'] = bool(self.cfg.use_card)
        self.metrics['planner_engine'] = planner_engine()
        if hasattr(self.vision, "backend_name"):
            self.metrics['ai_backend'] = 'disabled' if self.cfg.monochrome else self.vision.backend_name
            self.metrics['ai_details'] = (dict(requested=cfgs['vision'].get('ai.backend','auto'),
                actual='disabled',providers=[],fallback_reason='monochrome mode') if self.cfg.monochrome
                else self.vision.ai_engine.diagnostics)
        self.metrics['motion_strategy'] = 'forward_first_bounded_reverse'
        self.metrics["motion_settings"]=dict(self.motion)
        self.cones.seed_prior(self.io.now())

    def _world(self):
        return CollisionWorld(self.field, self.cones.snapshot(), self.car_len, self.car_wid, .06)

    def _zone_reached(self, zone):
        # Center-only arrival, tighter than the 1m square region.
        tolerance=min(self.motion["arrival_m"],min(zone.length,zone.width)/2-math.hypot(self.car_len,self.car_wid)/2-.055)
        return tolerance>0 and zone.distance_to_center(self.pose()) < tolerance

    def _on_bump(self, ev):
        if self.cfg.monochrome:
            self.log('bump', detail=ev.detail, hit_color='unknown')
            self._fail('检测到碰撞：无颜色模式统一停车')
        else:
            super()._on_bump(ev)

    def _start_cross(self):
        self.state, self.state_t0 = State.CROSS, self.io.now()
        self._hybrid_path = None
        self._segment = []
        self.log('cross_start', mode='hybrid_astar')

    def _st_hold(self, dt):
        # Three seconds begin after measured longitudinal and angular motion settles.
        now=self.io.now()
        stopped=(self._last_odom is not None and abs(self._last_odom.v)<.025
                 and self._last_imu is not None and abs(self._last_imu.yaw_rate)<.08)
        if not stopped:
            self._hold_started=None
        elif self._hold_started is None:
            self._hold_started=now
        elif now-self._hold_started>=self.cfg.hold_seconds:
            self.metrics['hold_actual_s']=round(now-self._hold_started,3)
            self.log('hold_done',seconds=self.metrics['hold_actual_s'])
            self._start_cross()
        return 0.,0.

    def _st_cross(self, dt):
        if self.io.now() - self.state_t0 > self.cfg.stage_timeout_s.get('cross_mine', 240):
            self._fail('穿越雷区超时')
            return 0., 0.
        camp = self._target_zone('camp')
        if self._zone_reached(camp):
            self._enter_finish()
            return 0., 0.
        return self._drive_to(camp.center, self.cfg.cross_speed_mps, dt=dt)

    def _apply_close_guard(self, v, w):
        # No spin-in-place escape. Swept-footprint check is in _drive_to.
        return v, w

    def _apply_safety(self, v, w):
        if self.state in (State.FAILED, State.DONE, State.HOLD, State.FINISH):
            self._speed = 0.
            return 0., 0.
        return v, w

    def _update_progress(self):
        pass  # Legacy differential escape logic is intentionally not used.

    def _drive_to(self, goal, speed_cap, goal_tolerance=.18, replan=True, dt=.02):
        dt=self._last_dt
        now, pose = self.io.now(), self.pose()
        if now < self._switch_until:
            return 0., 0.
        world = self._world()
        changed = self._last_goal is None or np.linalg.norm(np.array(goal)-self._last_goal) > .2
        blocked = bool(self._segment and any(not world.clear(p) for p in self._segment[self._next_index::2]))
        if changed or blocked:
            self._hybrid_path, self._segment = None, []
            self._speed = 0.
        if not self._segment and self._hybrid_path:
            self._select_segment()
        if not self._segment:
            self._speed = 0.
            if now - self._last_attempt < .6:
                return 0., 0.
            self._last_attempt = now
            self._last_goal = np.array(goal)
            plan_world = CollisionWorld(self.field, self.cones.snapshot(), self.car_len, self.car_wid, self.motion["planning_margin_m"])
            if not plan_world.clear(pose):
                # A smaller tracking reserve is allowed, never below the execution guard.
                plan_world = world
            if self.async_planning:
                from concurrent.futures import ThreadPoolExecutor
                if self._executor is None:self._executor=ThreadPoolExecutor(max_workers=1)
                if self._plan_future is None:
                    self._plan_origin=pose
                    self._plan_future=self._executor.submit(hybrid_astar,pose,tuple(goal),plan_world,self.wheelbase,self.steer_limit,self.allow_reverse,max_reverse_m=self.max_reverse_path_m)
                    return 0.,0.
                if not self._plan_future.done():return 0.,0.
                path=self._plan_future.result();self._plan_future=None
                if np.linalg.norm(np.array(pose[:2])-self._plan_origin[:2])>.15:return 0.,0.
            else:
                path = hybrid_astar(pose, goal, plan_world, self.wheelbase, self.steer_limit, self.allow_reverse,max_reverse_m=self.max_reverse_path_m)
            if path is None:
                self._failed_plans+=1
                if self._failed_plans>=5:self._fail('无法找到满足车体和转弯半径约束的路径')
                self.log('plan_failed', mode='hybrid_astar',start=list(pose),goal=list(goal),
                         start_clear=plan_world.clear(pose),margin=plan_world.margin,obstacles=plan_world._cones)
                return 0., 0.
            # A background result may have been computed from an older scan snapshot.
            if any(not world.clear(p) for p, _ in path):
                self.log('path_discarded', reason='map_changed')
                return 0., 0.
            self._failed_plans=0
            self._hybrid_path = path[1:]
            self._select_segment()
            if not self._segment:
                return 0., 0.
            engine_name = self.metrics.get('planner_engine', 'cpp_native')
            self.log('path_planned', n=len(path), mode='hybrid_astar', engine=engine_name,
                     start=list(pose),goal=list(goal),planning_margin_m=plan_world.margin)
        arr = np.asarray(self._segment)
        stop = min(len(arr), self._next_index + 30)
        nearest = self._next_index + int(np.argmin(np.linalg.norm(arr[self._next_index:stop, :2]-pose[:2],axis=1)))
        self._next_index = nearest
        remaining = float(np.linalg.norm(arr[-1,:2]-pose[:2]))
        if remaining < .10:
            self._segment = []
            self._speed = 0.
            self._switch_until = now + self.motion["shift_pause_s"]
            return 0., 0.
        target_index = nearest
        arc=0.
        while target_index<len(arr)-1 and arc<self.motion["lookahead_m"]:
            arc+=float(np.linalg.norm(arr[target_index+1,:2]-arr[target_index,:2]));target_index+=1
        tx, ty = G.world_to_body(pose, arr[target_index,:2])
        dist2 = tx*tx + ty*ty
        k = 2*ty/max(dist2,.02)
        kmax = math.tan(self.steer_limit)/self.wheelbase
        k = float(np.clip(k,-kmax,kmax))
        # Small speed while turning, braking before a cusp. Reverse explicitly bounded.
        cap = min(speed_cap, self.motion["forward_mps"], self.motion["corner_mps"] if abs(k)>1. else self.motion["forward_mps"])
        if self._direction < 0:
            cap = min(cap,self.motion["reverse_mps"])
        cap = min(cap, math.sqrt(max(.0, 2*self.motion["brake_mps2"]*max(0.,remaining-.06))))
        self._speed = min(cap,self._speed + self.motion["accel_mps2"]*dt)
        def safe_at(speed):
            horizon = speed*.7 + speed*speed/(2*self.motion["brake_mps2"]) + .04
            return world.edge(pose, self._direction*horizon, k)
        if not safe_at(self._speed):
            low, high = 0., self._speed
            for _ in range(10):
                mid=(low+high)/2
                if safe_at(mid): low=mid
                else: high=mid
            self._speed=low
        if self._speed < .025 and not safe_at(.025):
            self._hybrid_path, self._segment, self._speed = None, [], 0.
            self.log('predicted_collision_stop')
            return 0., 0.
        v = self._direction * self._speed
        self._path = arr[:,:2]  # diagnostics / visualization
        return v, v*k

    def _select_segment(self):
        self._segment = []
        if not self._hybrid_path:
            return
        self._direction = self._hybrid_path[0][1]
        end = 0
        while end < len(self._hybrid_path) and self._hybrid_path[end][1] == self._direction:
            end += 1
        self._segment = [p for p, _ in self._hybrid_path[:end]]
        del self._hybrid_path[:end]
        self._next_index = 0
