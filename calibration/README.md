# Second-arm reference calibration

This procedure records software coordinates without enabling motors, changing
modes, clearing faults, or resetting motor zeros. The active motion configuration
stays blocked. The old first-arm range file is not used.

## First step: record the reference

1. Keep the base fixed. Support the arm, with all motors disabled and other motor
   controllers disconnected. Do not force an enabled joint or push into a stop.
2. Open [Seeed's reference photograph](seeed_reference_pose.jpg). Arrange the arm
   by hand to match the folded geometry: the two long links are horizontal and
   parallel, the upper one folds back over the lower one, and the wrist/gripper
   projects horizontally away from the elbow. Match the wrist housings as well
   as the links. Use a level on the long-link edges; a casual visual match is only
   an initial estimate. Keep the camera mount and cable clear.
3. Mark the base yaw direction on the fixed base/table as the local reference.
   The side photo cannot uniquely establish base yaw or all wrist alignment.
   Photograph the actual arm from the side and above for later geometry review.
   The stock gripper in the photo differs from the installed fingers: don't close
   or force custom fingers to copy it. Joint 7 is excluded from this calibration.
4. While the pose is supported, run:

   ```bash
   uv run calibrate_arm.py reference
   ```

   Type `REFERENCE` when positioned. Ten position samples are taken; more than
   0.5° of movement rejects the capture. Motor status must report disabled.
   The recorder uses fresh position-register queries, rather than treating cached
   feedback as proof of a new position reply.
5. Rest the arm after the capture completes. Review with:

   ```bash
   uv run calibrate_arm.py show
   ```

The saved file is `calibration/arm2_reference.json`; replacements archive the old
file with a timestamp. No current pose is silently treated as reference. The zero
model angles are a candidate based on operator alignment to Seeed's reference;
physical alignment and joint directions remain to be independently checked.

## Direction checks and validation

The guided version now handles direction-only measurements without requiring a
protractor or signed angle input:

```bash
uv run calibrate_arm.py directions
```

It prompts for each of the six joints, always starting at the recorded reference.
It checks the starting angles before asking for a small manual movement in a named
physical direction. It accepts 3–15° of movement on the selected joint and rejects
more than 2° of movement on others. Support the arm throughout. Type `DONE` only if
you performed the described physical direction. Stop if the instruction is unclear
or obstructed. The program sends only read requests; it does not make the movement.

Progress saves after every joint and resumes automatically. To redo one joint:
`uv run calibrate_arm.py directions --joint 3`. This establishes a sign from your
observed direction, **not** an independently measured angular scale or a validation
of the zero/reference pose. The more precise measured-angle method below remains
available. Neither method marks the arm ready for motion.

One reference measurement cannot distinguish a reversed joint direction. For each
joint, return to the reference and move only that joint through an independently
measured signed model angle of 5–20°. Use the URDF axis and right-hand convention,
not the sign shown by the motor. Do not guess the sign from a photo or from whether
an arbitrary movement raised the gripper. Then record, for example:

```bash
uv run calibrate_arm.py direction --joint N --model-delta-deg ANGLE
```

Replace N and ANGLE with the joint and actual measured signed angle. The tool
rejects excessive movement of other joints and mismatched angle magnitudes. These
are measurement commands, not instructions to make a particular physical move.
Have the selected joint's axis/positive direction identified before this step.

The software mapping is `q_model = sign * q_raw + offset`. Signs and offsets stay
unset until measured; recorded travel endpoints cannot supply these quantities.
After all six checks, compare predicted link positions with at least two additional
measured physical poses. This tool intentionally does not certify its own data or
turn on motion. Reference capture alone is not Cartesian calibration.

## Camera and fingertip steps after joint mapping is verified

Keep the 60 mm tag fixed relative to the base. Collect synchronized, stationary
joint and tag observations across at least 15 varied camera poses with rotations
about multiple axes. Solve hand-eye calibration and check a held-out set of poses;
repeated views with nearly identical orientation are insufficient. The existing
`outputs/arm_held.json` and `outputs/tag_pose.json` are not synchronized calibration
pairs. Do not use them as a fitted transform.

Measure/calibrate the actual installed fingertip contact point relative to the
chosen tool frame and retain the corresponding gripper opening. A camera-to-tool
transform alone does not locate the fingertip. Validate at a visible stand-off
before any contact approach. Joint ranges, collision clearance, tracking and
feedback handling must also be checked before executing a new trajectory.

## Sources

Reference photo: https://files.seeedstudio.com/wiki/robotics/projects/lerobot/b601dm_zeroposition.jpg

[Seeed reference-pose instructions](https://wiki.seeedstudio.com/rebot_arm_b601_dm_lerobot/)
show hardware zeroing. Our recorder uses the pose photograph only and **does not**
run those zeroing commands.

[Seeed grasping calibration workflow](https://wiki.seeedstudio.com/rebot_arm_b601_dm_grasping_demo/).
The checked-out DM URDF is at
`vendor/reBotArm_control_py/urdf/DM/urdf/ReBot_Arm_DM.urdf`, commit
`6415d43130d1e143c70dc106096a857ac5556f81`.

## Guided camera-to-wrist collection

All six direction checks have been recorded for this arm. `arm_geometry.py` uses
those software offsets and the pinned DM URDF to calculate the `link6` wrist pose.
The camera capture tool never commands the arm.

1. Fix tag 0 firmly in place and leave the robot base fixed. Its black square is
   configured as 60 mm. Keep the same camera mount and gripper configuration.
2. Run from the project folder:

   ```bash
   uv run calibrate_camera.py capture
   ```

3. Support and move the disabled arm by hand so the whole tag is visible in the
   window. Let it settle; press **Space** to record. Read the message in the window:
   unsuccessful captures explain what to change.
4. Collect **at least 20 different poses**. Keep the tag visible while varying both
   viewing position and wrist orientation: include tilted views in more than one
   direction, not just a left/right sweep. Only use clear, comfortable poses.
   The tag and base must stay fixed across every capture, including resumed runs.
5. Press **Q** to finish, gently rest the arm, then run:

   ```bash
   uv run calibrate_camera.py solve
   ```

Samples save immediately to `calibration/arm2_handeye_samples.json`, with annotated
images. Rerunning capture resumes; if the tag/base moved between sessions, archive
that sample file first and start a new set. A changed joint reference, model, or
tag definition is rejected automatically using a stored fingerprint.

The solver uses OpenCV's Park hand-eye method and holds out every fifth pose for
validation. It rejects inadequate rotation diversity. The candidate's held-out
maximum errors must be no more than 5 mm and 2 degrees. These are consistency
checks, not proof of collision clearance, correct physical zero alignment, or
contact accuracy. Results are saved to `calibration/arm2_handeye_candidate.json`;
`motion_ready` stays false even if validation passes. The actual fingertip contact
point is still a separate measurement. There is no automatic motor enable or
approach in these tools.

[OpenCV hand-eye transform conventions](https://docs.opencv.org/4.12.0/d9/d0c/group__calib3d.html)
use base-from-wrist and camera-from-tag observations to estimate wrist-from-camera.

### First 10-view diagnostic

The first dataset has useful rotational diversity, but its camera/arm relative
rotations disagree by up to 8.29 degrees. Multiple hand-eye solvers show inconsistent
fixed-tag poses. This cannot be repaired simply by changing the camera-to-wrist
translation. Saved photos show curling paper near the tag; a non-flat tag is a
possible contributor, not a proven sole cause. Physical reference alignment and
camera pose estimation remain possible sources.

Mount the tag flat on a rigid board, secure the board, and check that the printed
black square measures 60 mm on both sides. Avoid glare and views nearly edge-on.
If the tag/base placement changes, start a new session:

```bash
uv run calibrate_camera.py capture --new-session
```

This archives the previous JSON without deleting its images. New sessions use
unique image filenames and save unannotated images, corner coordinates, calibration
metadata and frame timestamps for diagnosis. Existing captures remain available;
do not combine observations taken before and after repositioning the board.

### Flat-tag dataset review

The replacement session contains 20 views. The Park fit's four held-out position
errors are 6.46, 1.67, 7.64, and 8.73 mm; orientation errors are 1.08, 2.85, 1.12,
and 0.83 degrees. This is a substantial improvement, but validation still fails
at the original 5 mm / 2 degree thresholds. Candidate files are not activated.

`uv run refine_handeye.py` performs an offline training-only corner reprojection
fit and writes a separate candidate. It does not change the reference, capture
set, camera configuration, or motor settings. The current refinement also fails
validation; it is diagnostic, not an approved replacement. Synthetic tests verify
transform recovery and that held-out corners do not influence fitting. Physical
reference alignment and installed fingertip geometry still need checking; further
captures alone are not currently the prescribed next step.

### External photographs and custom tool geometry

The supplied side/overhead images are recorded in
`calibration/tool_geometry_candidate.json`. The folded pose broadly matches the
reference, but perspective does not establish precision angular alignment. The
installed elongated fingers differ from the stock URDF geometry. Matching-looking
CAD was found in the earlier design workspace: `B601_long_braced45` and
`B601_clean_braced45` share the same contact geometry. Part identity is awaiting
confirmation. Nominal pad points were derived directly from the CAD generator and
are stored in **finger CAD coordinates only**. They must not be used as link6 or
camera-frame targets. Mount registration, actual pad dimensions, gripper opening,
and physical contact-point verification remain outstanding. No motion is enabled.

### Operator-taught contact attempt

`record_contact.py` captured five bracketed joint/image observations in
`calibration/contact_20260930T002532864375Z/`. Contact was reported by the operator;
there is no force-sensor confirmation. The tag is clipped by the image top edge
and blurred in the contact view, so none of the five frames decoded. The raw joint
endpoint is preserved, but this is not a valid visual-servo contact target and
no approach or replay path has been validated. `motion_ready` remains false.
