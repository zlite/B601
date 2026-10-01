"""Direct-top mount optimized for Y-min-down printing as in user's screenshot.
CadQuery; imports adjacent original generator and Seeed reference STEP.
"""
from pathlib import Path
import cadquery as cq
from cadquery import exporters
from build_direct_top_mount import create as original, box

HERE=Path(__file__).resolve().parent
BED_Y=-21.5

def create():
    mount=original()
    for sign in (-1,1):
        x=27.55 if sign==1 else -31.05
        # Replace flat-bottom stops with 45-degree printable profiles. These
        # remain outside the camera opening: no braces across its seating area.
        mount=mount.cut(box(x-0.001,-21.5,30,3.502,46,20))
        lower=[(BED_Y,29.8),(BED_Y,30),(-7.5,44),(-2,44),(-2,29.8)]
        upper=[(12,29.8),(12,30),(24,42),(24,29.8)]
        for profile in (lower,upper):
            rail=(cq.Workplane('YZ',origin=(x,0,0)).polyline(profile)
                  .close().extrude(3.5).val())
            mount=mount.fuse(rail)
    # Two tiny permanent bed-contact feet on the outside of the lower flange.
    # They bridge the original 0.63 mm gap to the chosen tray-edge bed plane.
    for x in (-17,17):
        mount=mount.fuse(box(x-1,BED_Y,0.5,2,1.3,1.8))
    return mount.clean()

def print_orientation(s):
    return s.rotate((0,0,0),(1,0,0),90).translate((0,0,-BED_Y))

if __name__=='__main__':
    mount=create()
    assert mount.isValid() and len(mount.Solids())==1
    exporters.export(mount,str(HERE/'OAK1_B601_low_support.step'))
    p=print_orientation(mount)
    exporters.export(p,str(HERE/'OAK1_B601_low_support_PRINT.stl'),tolerance=0.03,angularTolerance=0.1)
    b=p.BoundingBox()
    print('Valid single solid. Print bounds',b.xlen,b.ylen,b.zlen,'bed Z',b.zmin)
