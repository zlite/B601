REVISION 2 - WRIST CLEARANCE CORRECTION
Use the v2 print file. Revision 1 extended into the wrist clearance and does
not fit. The exact inherited 28.65 mm radius cutout, centered at CAD
X=-0.0009376866, Y=-45.2171488122, now continues through the entire camera
plate. CAD comparison shows 473.77 mm^3 removed inside that clearance and
zero geometry changes outside it. Lower attachment and all fasteners are
unchanged. Fit still needs a physical check. Superseded v1 files are archived
under revisions/v1; do not print them. Unversioned export aliases now contain
this corrected geometry as well.

OAK-D Lite mount for the Seeed reBot B601-DM

PRINT: OAK_D_Lite_B601_v2_PRINT.stl (millimeters, already oriented on the bed).
EDIT: OAK_D_Lite_B601_v2.step or build_mount.py.
VIEW: OAK_D_Lite_B601_v2_preview.png / OAK_D_Lite_B601_v2_assembly.step.
The assembly contains the reference camera. Do not print the assembly or camera.

This replaces the complete installed OAK1_B601_snug_1mm bracket. The proven
arm attachment below CAD Z=25.6 mm is unchanged. The camera plate is now
95 mm wide, with two 4.5 mm clearance holes at 75 mm centers, seven ventilation
slots, and 1 mm raised camera seats. Two rear M4 screws replace the old
OAK-1 tripod screw and anti-rotation wall. The original mount is preserved.

Hardware and installation
- Reuse the existing arm attachment hardware. Install the bracket on the arm
  before adding the camera, for access to the original countersunk screws.
- Use the two REAR M4 camera mounting threads, not the small case screws.
- The printed seating stack is 5.4 mm. Aim for about 3-4 mm of screw projection
  into the camera. M4 x 10 mm screws with approximately 1 mm washers give
  3.6 mm projection; verify free thread depth on your unit before tightening.
  Tighten gently; never force a screw that bottoms out.
- Install the camera as shown: all three lenses face out; the USB port faces
  the open edge above the attachment. Provide a cable slack loop and strain
  relief on the arm so cable load does not twist the camera or snag the wrist.

Print starting point: PETG, 0.2 mm layers, 4-5 perimeters, 35-40% infill.
Use the supplied orientation with the 95 mm tray edge on the bed and a brim
if needed. Inspect the slicer preview: narrow vent roofs bridge 6 mm, and the
original clip and horizontal screw bores may need local support depending on
printer settings. Keep support off the camera seating pads where possible.
Overall print envelope: approximately 95 x 31 x 40 mm. Solid CAD volume 25.3 cc.

Verification
Valid single CAD solid; watertight, consistently wound, single-component STL;
unchanged lower arm attachment; zero interference with the official Luxonis
camera enclosure; M4 bores open and USB plug clearance envelope unobstructed.
The mount stays behind the camera front face. These checks do not certify the
camera field of view past the gripper, the full robot travel envelope, strength,
or print fit. Physically test the fit and inspect clearance before powered use.
The earlier OAK-1 camera-to-wrist calibration must be replaced after fitting
this camera; no robot software/calibration or motor state was changed here.

Rebuild: python build_mount.py (CadQuery 2.8.0, trimesh used for this export).
Optional preview: python render_preview.py (VTK). Reference camera is a Luxonis
STEP model, reoriented using its measured mounting-hole axes. Lens circles in
the PNG are schematic markers; the enclosure geometry is the official CAD.

Source provenance
Installed source: OAK1_B601_snug_1mm.step, copied from the prior local CAD work.
Its original generator chain and Seeed reference STEP are included in source/.
Luxonis official enclosure index:
https://github.com/luxonis/oak-hardware/blob/master/DM9095_OAK-D-LITE_DepthAI_USB3C/Mechanical/README.md
Enclosure downloaded 2026-09-30:
https://oak-files.fra1.cdn.digitaloceanspaces.com/OAK-D-Lite/DM9095_enclosure.stp
Measured official enclosure dimensions: 91 x 28 x 17.45 mm.
Rear M4 axes: (3.5,15.5) and (78.5,15.5), rear seating plane Z=8.5 mm.
Derived mount/source: CERN-OHL-W-2.0, see included license. Third-party camera
reference CAD remains under its upstream terms; it is not a printable part.
