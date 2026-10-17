import sys,time,json,math
from pathlib import Path
from types import SimpleNamespace
from racelib.config import load_all
from racelib.field import Field
from racelib.ackermann import CollisionWorld,hybrid_astar
cfg=load_all('profiles/generated/measured_example','A');field=Field.from_config(cfg['field'])
cones=[SimpleNamespace(x=c['x'],y=c['y'],radius=c['radius']) for c in json.loads(Path('profiles/measured_example.json').read_text())['cones']]
world=CollisionWorld(field,cones,margin=.1)
for pose,goal in [((.79,3.1,1.9),(6.25,.75)),((.5,2.,.05),(.75,3.25))]:
 for reverse in (False,True):
  t=time.monotonic();p=hybrid_astar(pose,goal,world,allow_reverse=reverse)
  length=sum(math.dist(a[0][:2],b[0][:2]) for a,b in zip(p or [],(p or [])[1:]))
  back=sum(math.dist(a[0][:2],b[0][:2]) for a,b in zip(p or [],(p or [])[1:]) if b[1]<0)
  print(pose,'allow_reverse',reverse,'found',bool(p),'seconds',time.monotonic()-t,'length',length,'reverse',back,flush=True)
