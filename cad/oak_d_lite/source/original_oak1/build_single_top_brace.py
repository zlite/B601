"""Round tripod hole and one continuous top brace. Units mm.
Derived from the preceding low-support mount, CERN-OHL-W-2.0.
"""
from pathlib import Path
import cadquery as cq
from cadquery import exporters
from build_low_support_mount import create as previous, print_orientation
from build_direct_top_mount import box

HERE=Path(__file__).resolve().parent
HOLE_X=0.0  # centre of the former slot; editable for a measured socket offset
HOLE_Y=5.0
HOLE_D=6.8
BRACE_Y=24.0
BRACE_T=3.5
BRACE_TOP=44.0

def create():
    mount=previous()
    # Remove all four side stops and their angled undersides.
    for x in (-31.06,27.54):
        mount=mount.cut(box(x,-22,30,3.52,51,20))
    # Restore the material removed by the old slot, within the original pad.
    fill=(cq.Workplane('XY',origin=(0,HOLE_Y,25.6))
          .slot2D(40,6.8).extrude(5.4).val())
    mount=mount.fuse(fill)
    mount=mount.cut(cq.Solid.makeCylinder(HOLE_D/2,10,cq.Vector(HOLE_X,HOLE_Y,24)))
    # Extend the plate at its far edge so the single top bar has full contact.
    mount=mount.fuse(box(-31.05,23.8,25.6,62.1,BRACE_T+0.2,4.4))
    mount=mount.fuse(box(-31.05,BRACE_Y,29.8,62.1,BRACE_T,BRACE_TOP-29.8))
    return mount.clean()

if __name__=='__main__':
    mount=create()
    assert mount.isValid() and len(mount.Solids())==1
    exporters.export(mount,str(HERE/'OAK1_B601_single_top_brace.step'))
    p=print_orientation(mount)
    exporters.export(p,str(HERE/'OAK1_B601_single_top_brace_PRINT.stl'),tolerance=0.03,angularTolerance=0.1)
    b=p.BoundingBox()
    print(f'Valid single solid. Print dimensions: {b.xlen:.1f} x {b.ylen:.1f} x {b.zlen:.1f} mm')
