"""OAK-1 sideways directly above B601-DM attachment. Units mm.
CadQuery; Seeed-derived attachment licensed CERN-OHL-W-2.0.
"""
from pathlib import Path
import cadquery as cq
from cadquery import exporters

HERE=Path(__file__).resolve().parent
GAP=55.1
WALL=3.5
TRAY_BOTTOM=25.6  # 0.4 mm overlap into original top flange
TRAY_TOP=30.0
SEAT_Z=31.0
STOP_TOP=44.0  # same 13 mm engagement above seat
SCREW_Y=5.0
YMIN,YMAX=-21.5,24.0
MOUNT_HOLE_Y=-14.717

def box(x,y,z,dx,dy,dz):
    return cq.Solid.makeBox(dx,dy,dz,cq.Vector(x,y,z))

def create():
    src=cq.importers.importStep(str(HERE/'D435_Gemini2_Mount.step'))
    arm=next(s for s in src.solids().vals() if s.BoundingBox().xlen<50)
    clip=arm.intersect(box(-60,-60,-40,120,55.7825,100)).translate((0.014,0,-13.224))
    width=GAP+2*WALL
    mount=clip.fuse(box(-width/2,YMIN,TRAY_BOTTOM,width,YMAX-YMIN,TRAY_TOP-TRAY_BOTTOM))
    for sign in (-1,1):
        x=GAP/2 if sign==1 else -GAP/2-WALL
        for a,b in ((-13,SCREW_Y-7),(SCREW_Y+7,24)):
            rail=box(x,a,TRAY_TOP-0.2,WALL,b-a,STOP_TOP-TRAY_TOP+0.2)
            mount=mount.fuse(cq.Workplane(obj=rail).faces('>Z').edges().chamfer(0.6).val())
    boss=(cq.Workplane('XY',origin=(0,SCREW_Y,TRAY_TOP-0.2))
          .slot2D(48,15).extrude(SEAT_Z-TRAY_TOP+0.2).val())
    mount=mount.fuse(boss)
    screw=(cq.Workplane('XY',origin=(0,SCREW_Y,TRAY_BOTTOM-1))
           .slot2D(40,6.8).extrude(9).val())
    mount=mount.cut(screw)
    # Extend access through the new tray into the original two upper holes.
    # Use countersunk M3 screws; upper grip thickness increases by 4 mm.
    for x in (-11.001,10.999):
        bore=cq.Solid.makeCylinder(1.75,8,cq.Vector(x,MOUNT_HOLE_Y,23))
        sink=cq.Solid.makeCone(1.75,3.25,1.5,cq.Vector(x,MOUNT_HOLE_Y,TRAY_TOP-1.5))
        mount=mount.cut(bore).cut(sink)
    return mount.clean()

if __name__=='__main__':
    mount=create()
    assert mount.isValid() and len(mount.Solids())==1
    for ext in ('step','stl'):
        exporters.export(mount,str(HERE/f'OAK1_B601_direct_top.{ext}'),tolerance=0.03,angularTolerance=0.1)
    b=mount.BoundingBox()
    print(f'Valid single solid: {b.xlen:.1f} x {b.ylen:.1f} x {b.zlen:.1f} mm')
