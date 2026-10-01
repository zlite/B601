"""Correct OAK-1 tripod offset using Luxonis enclosure CAD, mm.
Camera USB-C end faces +X in the mount CAD coordinates.
"""
from pathlib import Path
import cadquery as cq
from cadquery import exporters
from build_full_width_attachment import create as previous, print_orientation

HERE=Path(__file__).resolve().parent
# Luxonis OAK-1_ENCLOSURE.step: long-axis extent -24.5 .. 30.0;
# threaded tripod axis Y=-10.0. Offset from midpoint 2.75 is -12.75.
# In our sideways orientation camera -Y maps to mount +X.
TRIPOD_X=12.75
TRIPOD_Y=5.0
HOLE_D=6.8

def create():
    mount=previous()
    # Close only the old camera hole through its 5.4 mm bearing stack.
    fill=cq.Solid.makeCylinder(HOLE_D/2,5.4,cq.Vector(0,TRIPOD_Y,25.6))
    mount=mount.fuse(fill)
    hole=cq.Solid.makeCylinder(HOLE_D/2,10,cq.Vector(TRIPOD_X,TRIPOD_Y,24))
    return mount.cut(hole).clean()

if __name__=='__main__':
    s=create()
    assert s.isValid() and len(s.Solids())==1
    exporters.export(s,str(HERE/'OAK1_B601_offset_tripod.step'))
    exporters.export(print_orientation(s),str(HERE/'OAK1_B601_offset_tripod_PRINT.stl'),tolerance=0.03,angularTolerance=0.1)
    print('Valid single solid. Tripod hole centre (12.75, 5.0), diameter 6.8 mm.')
