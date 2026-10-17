"""Pure command conversion; no ROS or chassis side effects."""
import math

class CommandPolicy:
    def __init__(self, wheelbase=.30, steer_limit=.52, speed_limit=.50, reverse_limit=.20):
        self.wheelbase, self.steer_limit = wheelbase, steer_limit
        self.speed_limit, self.reverse_limit = speed_limit, reverse_limit

    def convert(self, v, w, mode):
        if not all(math.isfinite(x) for x in (v,w)):
            return 0.,0.
        if abs(v) < .01:
            return 0.,0.
        curvature=w/v
        v = max(-self.reverse_limit,min(self.speed_limit,v))
        steer = max(-self.steer_limit,min(self.steer_limit,math.atan(self.wheelbase*curvature)))
        if mode == 'legacy_steering':
            return v,steer
        return v,v*math.tan(steer)/self.wheelbase

