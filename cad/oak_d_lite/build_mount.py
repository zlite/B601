"""OAK-D Lite replacement for OAK1_B601_snug_1mm. Units mm.
SPDX-License-Identifier: CERN-OHL-W-2.0
Run with Python + cadquery + trimesh. No robot hardware access.
"""
from pathlib import Path
import json
import cadquery as cq
import trimesh

HERE=Path(__file__).resolve().parent
STEM='OAK_D_Lite_B601_v2'
WIDTH=95.0
PLATE_BOTTOM=25.6
PLATE_TOP=30.0
SEAT_Z=31.0
M4_PITCH=75.0
M4_ROW_Y=5.0
M4_CLEARANCE=4.5
# Exact wrist relief measured from the inherited arm clip's cylindrical faces.
WRIST_CENTER=(-0.000937686570375,-45.21714881219)
WRIST_RADIUS=28.65

def wrist_relief():
 return cq.Solid.makeCylinder(WRIST_RADIUS,SEAT_Z-PLATE_BOTTOM+1,
                             cq.Vector(*WRIST_CENTER,PLATE_BOTTOM))

def box(x,y,z,dx,dy,dz):
 return cq.Solid.makeBox(dx,dy,dz,cq.Vector(x,y,z))

def build():
 old=cq.importers.importStep(str(HERE/'source/OAK1_B601_snug_1mm.step')).val()
 # Keep the proven arm clip intact; replace the seating plate and camera wall.
 lower_region=box(-100,-100,-10,200,200,PLATE_BOTTOM+10)
 lower=old.intersect(lower_region)
 plate=box(-WIDTH/2,-21.5,PLATE_BOTTOM,WIDTH,40,PLATE_TOP-PLATE_BOTTOM)
 mount=lower.fuse(plate)
 for x in (-M4_PITCH/2,M4_PITCH/2):
  mount=mount.fuse(cq.Solid.makeCylinder(5,1.2,cq.Vector(x,M4_ROW_Y,29.8)))
  mount=mount.cut(cq.Solid.makeCylinder(M4_CLEARANCE/2,7,cq.Vector(x,M4_ROW_Y,25)))
 # Narrow ventilation slots: short bridges in the supplied print orientation.
 for x in (-27,-18,-9,0,9,18,27):
  slot=(cq.Workplane('XY',origin=(x,3,PLATE_BOTTOM)).slot2D(22,6,90).extrude(6).val())
  mount=mount.cut(slot)
 # Preserve the two original arm fasteners and their countersunk access.
 for x in (-11.001,10.999):
  mount=mount.cut(cq.Solid.makeCylinder(1.75,5,cq.Vector(x,-14.717,25.6)))
  mount=mount.cut(cq.Solid.makeCone(1.75,3.25,1.5,cq.Vector(x,-14.717,28.5)))
 # Continue the arm clip's curved wrist clearance through the camera plate.
 # Start at the old/new interface so the proven lower attachment is untouched.
 mount=mount.cut(wrist_relief()).clean()
 assert mount.intersect(wrist_relief()).Volume()<1e-5
 assert mount.isValid() and len(mount.Solids())==1
 assert lower.cut(mount).Volume()<1e-5
 assert mount.intersect(lower_region).cut(lower).Volume()<1e-5
 # Official enclosure coordinate system: rear is +Z, optical direction -Z.
 # Flip around Y to face the same direction as the former OAK-1 (+mount Z).
 # Rear mounting axes (3.5,15.5) and (78.5,15.5) -> (+/-37.5,5).
 camera=cq.importers.importStep(str(HERE/'source/DM9095_enclosure.stp')).val()
 camera=camera.rotate((0,0,0),(0,1,0),180).translate((41,-10.5,39.5))
 collision=sum(mount.intersect(s).Volume() for s in camera.Solids())
 assert collision<1e-4,collision
 for x in (-37.5,37.5):
  assert mount.intersect(cq.Solid.makeCylinder(2.24,5.4,cq.Vector(x,5,25.6))).Volume()<1e-5
 # Conservative cable plug envelope beyond the port at original (56.55,-3,-3.22).
 plug=box(-24,-40,35,17,26.5,16)
 assert mount.intersect(plug).Volume()<1e-5
 # Printed body is entirely behind the camera front, including all lens windows.
 assert mount.BoundingBox().zmax<camera.BoundingBox().zmax
 return mount,camera,collision

if __name__=='__main__':
 mount,camera,collision=build()
 cq.exporters.export(mount,str(HERE/f'{STEM}.step'))
 # Front tray edge on bed, same orientation used by the earlier low-support mount.
 printed=mount.rotate((0,0,0),(1,0,0),90).translate((0,0,21.5))
 cq.exporters.export(printed,str(HERE/f'{STEM}_PRINT.stl'),tolerance=.03,angularTolerance=.1)
 cq.exporters.export(mount,str(HERE/f'{STEM}_assembly_coordinates.stl'),tolerance=.03,angularTolerance=.1)
 cq.exporters.export(camera,str(HERE/'camera_positioned.stl'),tolerance=.08,angularTolerance=.2)
 assembly=cq.Assembly(name=STEM)
 assembly.add(mount,name='PRINT_mount',color=cq.Color(.22,.64,.47))
 assembly.add(camera,name='REFERENCE_camera_do_not_print',color=cq.Color(.2,.22,.25))
 assembly.export(str(HERE/f'{STEM}_assembly.step'))
 mesh=trimesh.load(HERE/f'{STEM}_PRINT.stl',force='mesh')
 assert mesh.is_watertight and mesh.is_winding_consistent and len(mesh.split())==1
 assert abs(mesh.bounds[0,2])<1e-4
 report={'valid_single_solid':True,'watertight_single_mesh':True,
 'wrist_relief_radius_mm':WRIST_RADIUS,'wrist_relief_center_xy_mm':WRIST_CENTER,
 'wrist_relief_through_camera_plate':True,'wrist_relief_intersection_mm3':mount.intersect(wrist_relief()).Volume(),
 'arm_attachment_below_z_25_6_unchanged':True,'camera_mount_intersection_mm3':collision,
 'camera_rear_hole_pitch_mm':M4_PITCH,'camera_screw_clearance_mm':M4_CLEARANCE,
 'camera_seating_stack_mm':SEAT_Z-PLATE_BOTTOM,'rear_ventilation_gap_mm':1.0,
 'usb_plug_envelope_clear':True,'print_dimensions_mm':mesh.extents.tolist(),
 'mount_volume_cm3':mount.Volume()/1000,'physical_fit_tested':False,
 'note':'CAD clearance is not a full robot swept-volume or optical occlusion test.'}
 (HERE/f'{STEM}_checks.json').write_text(json.dumps(report,indent=2)+'\n')
 print(json.dumps(report,indent=2))
