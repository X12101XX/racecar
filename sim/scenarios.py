"""Published reproducible 7x4m demo, compliant counts and spacing.
Only blue barricade positions may be copied into the controller's published map;
red-role obstacle truth stays in the simulator. All materials render grey.
"""
from .world import SimCone

def demo_cones():
    cones=[SimCone(x,y,'blue',line=i) for i,x in enumerate((2.,3.5,5.)) for y in (.45,1.25,2.05,2.85)]
    cones += [SimCone(x,y,'red') for x,y in ((2.7,.8),(2.7,1.7),(4.2,1.1),(4.2,2.4),(4.65,.5))]
    return cones
