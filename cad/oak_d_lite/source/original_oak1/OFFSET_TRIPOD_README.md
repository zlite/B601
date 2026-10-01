# Corrected OAK-1 tripod socket location

This replaces the incorrectly centered camera screw hole in the full-width
attachment revision. Hole diameter remains 6.8 mm. The old central hole is filled;
the new hole is at CAD X=+12.75, Y=5.0 mm. All other geometry is preserved.

Source measurement: Luxonis's OAK-1 enclosure STEP has long-axis bounds
Y=-24.5 to +30.0 mm and tripod thread axis at Y=-10.0 mm. Therefore the socket
is 12.75 mm from the overall enclosure midpoint, toward the USB-C end (14.5 mm
from that end). The camera sits sideways, with its USB-C end toward the mount's
+X edge. In the supplied print orientation this corresponds to the end away
from the build plate. Reverse the camera roll only if using a mirrored hole.

Official CAD link published in Luxonis's repository:
https://github.com/luxonis/oak-hardware/blob/master/BW1093_USB3C/3D_Models/README.md
Enclosure model:
https://oak-files.fra1.cdn.digitaloceanspaces.com/OAK-1/OAK-1_ENCLOSURE.step

The PDF drawing confirms the off-center rear socket but does not explicitly
dimension its offset. The 12.75 mm value is measured from the CAD, not estimated
from the illustration. This applies to the standard USB OAK-1 enclosure.

Print the supplied pre-oriented STL on its long side as before. Full-width
attachment flanges and the single top bar are unchanged. Prior print and hardware
notes apply except any statement that the tripod hole is centered.
CAD validity, closed mesh, filled old hole and open new hole were checked.
Actual fit and printer performance have not been physically tested.

Editable source: build_offset_tripod_mount.py, with the included generator chain
and Seeed reference STEP. Derivative mount source is CERN-OHL-W-2.0 as included.
