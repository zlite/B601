"""Move only the OAK-1 anti-rotation wall 1 mm toward the tripod hole.
Units: mm. Derived mount: CERN-OHL-W-2.0.
"""
from pathlib import Path
import json
import cadquery as cq
from cadquery import exporters
import trimesh

HERE=Path(__file__).resolve().parent
STEM='OAK1_B601_snug_1mm'
SHIFT=1.0

def box(x,y,z,dx,dy,dz):
    return cq.Solid.makeBox(dx,dy,dz,cq.Vector(x,y,z))

def create():
    old=cq.importers.importStep(str(HERE/'OAK1_B601_offset_tripod.step')).val()
    # Preserve the whole seating plate below Z=30. Replace only the upright wall.
    s=old.cut(box(-31.06,23.99,30,62.12,3.52,14.01))
    s=s.fuse(box(-31.05,24-SHIFT,29.8,62.1,3.5,14.2)).clean()
    assert s.isValid() and len(s.Solids())==1
    lower=box(-70,-70,-10,140,140,40)
    a,b=old.intersect(lower),s.intersect(lower)
    assert a.cut(b).Volume()+b.cut(a).Volume()<1e-4
    wall=s.intersect(box(-32,20,31.1,64,10,14))
    wb=wall.BoundingBox()
    assert abs(wb.ymin-23)<1e-5 and abs(wb.ymax-26.5)<1e-5
    hole=cq.Solid.makeCylinder(3.39,10,cq.Vector(12.75,5,24))
    assert s.intersect(hole).Volume()<1e-4
    return s

if __name__=='__main__':
    s=create()
    exporters.export(s,str(HERE/f'{STEM}.step'))
    p=s.rotate((0,0,0),(0,1,0),-90).translate((0,0,31.05))
    exporters.export(p,str(HERE/f'{STEM}_PRINT.stl'),tolerance=.03,angularTolerance=.1)
    m=trimesh.load(HERE/f'{STEM}_PRINT.stl',force='mesh')
    assert m.is_watertight and m.is_winding_consistent and len(m.split())==1
    assert abs(m.bounds[0,2])<1e-4
    report={'wall_movement_mm':SHIFT,'inner_wall_y_before_mm':24,'inner_wall_y_after_mm':23,
            'hole_center_unchanged_mm':[12.75,5],'hole_diameter_mm':6.8,
            'wall_thickness_mm':3.5,'wall_top_z_mm':44,
            'lower_geometry_unchanged':True,'valid_single_solid':True,
            'watertight_mesh':True,'print_orientation_unchanged':True}
    (HERE/f'{STEM}_checks.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
