"""Full-width B601 attachment for long-side-down printing, mm.
CadQuery; derived generator chain and Seeed reference STEP required.
"""
from pathlib import Path
import cadquery as cq
from cadquery import exporters
from build_single_top_brace import create as previous
from build_direct_top_mount import box

HERE=Path(__file__).resolve().parent
HALF_WIDTH=31.05
OVERLAP_X=18.0
FRONT_Y=-20.869
BACK_Y=-4.2175
WEB_FRONT_Y=-10.717
LOWER_INNER_Z=2.8
UPPER_INNER_Z=23.2
UPPER_OUTER_Z=26.0

def create():
    mount=previous()
    # Extend BOTH flanges and their connecting web. Extending just the flanges
    # would leave the original narrow web starting as a bridge above the bed.
    for x in (-HALF_WIDTH,OVERLAP_X):
        width=HALF_WIDTH-OVERLAP_X
        lower=box(x,FRONT_Y,0,width,BACK_Y-FRONT_Y,LOWER_INNER_Z)
        upper=box(x,FRONT_Y,UPPER_INNER_Z,width,BACK_Y-FRONT_Y,
                  UPPER_OUTER_Z-UPPER_INNER_Z)
        web=box(x,WEB_FRONT_Y,LOWER_INNER_Z-0.1,width,BACK_Y-WEB_FRONT_Y,
                UPPER_INNER_Z-LOWER_INNER_Z+0.2)
        mount=mount.fuse(lower,upper,web)
    return mount.clean()

def print_orientation(s):
    # Original X=-31.05 side lies flat on the build plate; no floating clip.
    return s.rotate((0,0,0),(0,1,0),-90).translate((0,0,HALF_WIDTH))

if __name__=='__main__':
    mount=create()
    assert mount.isValid() and len(mount.Solids())==1
    exporters.export(mount,str(HERE/'OAK1_B601_full_width_attachment.step'))
    p=print_orientation(mount)
    exporters.export(p,str(HERE/'OAK1_B601_full_width_attachment_PRINT.stl'),tolerance=0.03,angularTolerance=0.1)
    b=p.BoundingBox()
    print(f'Valid single solid; print bounds {b.xlen:.1f} x {b.ylen:.1f} x {b.zlen:.1f} mm; minimum Z {b.zmin:.7f}')
