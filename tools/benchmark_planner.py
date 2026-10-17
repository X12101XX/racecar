"""Compare planner latency on this machine; no projection to vehicle hardware."""
import json
import statistics
import time
from racelib.ackermann import CollisionWorld, _python_hybrid_astar
from racelib.config import load_all
from racelib.field import Field, Cone
from racelib.native_planner import c_hybrid_astar,is_native_available
from sim.scenarios import demo_cones


def main():
    world=CollisionWorld(Field.from_config(load_all()['field']),
        [Cone(c.x,c.y,'unknown',.115) for c in demo_cones()],margin=.06)
    report={}
    for name,fn in [('python',_python_hybrid_astar),('cpp_native',c_hybrid_astar)]:
        if name=='cpp_native' and not is_native_available():continue
        times=[];counts=[]
        for _ in range(5):
            started=time.perf_counter()
            path=fn((.75,3.1,1.95),(5.,1.),world,.30,.52,True,45000,3.,.66,True)
            times.append((time.perf_counter()-started)*1000)
            if path is None or not all(world.clear(p) for p,_ in path): raise RuntimeError(name+' invalid or missing path')
            counts.append(len(path))
        report[name]=dict(mean_ms=round(statistics.mean(times),3),min_ms=round(min(times),3),max_ms=round(max(times),3),path_points=counts)
    print(json.dumps(report,indent=2))

if __name__=='__main__':main()
