# B601-DM + OAK-1 hello-world

A gentle wrist wave while reading RGB images from the wrist-mounted Luxonis OAK-1.
Uses Seeed's motorbridge driver and Luxonis DepthAI v3. No recalibration or zero-position writes.

## Visual leader pairing workbench (October 2, 2026)

```bash
.venv/bin/python pairing_dashboard.py
```

Open `http://127.0.0.1:8765` on this computer. The page combines live wrist and
workspace RGB views, tag visibility/quality, follower joint readings, a rotatable
B601 joint model, and a cyan preview driven by the FashionStar REbot 102 leader.
It sends **only read queries** to the arms. There is no motor activation or
movement endpoint. Stop the server with Ctrl+C before starting another device
client. `--demo --port 8766` previews the page without accessing hardware.

Match the small leader to the page's Seeed reference photo, confirm its pose,
and click **Save software reference**. After one second of steady readings, the
page saves a session-specific software reference. It does not change servo origins
or the follower reference. The six guided direction checks then record signs from
operator-confirmed visual movements. Each check starts from the current comfortable
pose; returning to the folded reference is unnecessary. Other joints and the
gripper can move freely during a check. Live movement bars show all six joints,
and a focused view freezes the other model joints on screen so the selected hinge
is easier to compare. **Flip it** reverses only that check's preview until the
operator confirms **Looks right · Save direction**. The selected hinge must move
at least 3 degrees (at most 45 in one check) and settle within a 1-degree span
for approximately half a second; other joints do not veto capture. No direction
is inferred from whichever joint happens to move most. Unchecked preview signs come from the
[upstream LeRobot REbot 102 configuration](https://github.com/huggingface/lerobot/blob/main/src/lerobot/teleoperators/rebot_102_leader/config_rebot_102_leader.py)
and remain unverified. Gripper mapping, ranges, scale, motion and collision
validation are not performed. Saved candidates are named
`calibration/leader_pairing_<session>.json` and retain `motion_ready: false`.
References are not automatically loaded on a new run, and leader connection loss
or a large discontinuity invalidates the current reference.

The CH340 leader adapter is
`/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`, with servo IDs 0–6 at 1 Mbaud.
If Linux denies access, run `sudo setfacl -m u:chris:rw /dev/ttyUSB0` in your own
terminal. This temporary permission may need repeating after USB reconnection.
`leader_read.py` uses checksummed FashionStar monitor queries with per-query
timeouts, reply-ID checks and no cached-angle fallback. The installed optional
SDK 0.0.4 marks this unit's stationary -0.1° shoulder reading unreliable, so it
is retained only for comparison with `leader_read.py --sdk`; the dashboard uses
fresh monitor transactions. Protocol reference:
[MotorBridge FashionStar protocol](https://github.com/motorbridge/motorbridge-smart-servo/blob/main/smart_servo_vendors/fashionstar/src/protocol.rs).
To recreate that optional SDK: `UV_CACHE_DIR=/tmp/b601-uv-cache uv pip install
--python .venv/bin/python --target vendor/leader_sdk -r requirements-leader.txt`.

The cameras provide visual feedback, not independent link-angle measurements.
The model is not registered as an overlay on either camera. Streams are not
synchronized hand-eye calibration samples. Camera-to-wrist positional validation
and fingertip geometry remain unresolved. The camera startup discards frames
until wrist focus reaches the configured position. Leader pose and direction
captures require an attended operator; the UI cannot establish physical alignment
on its own.

## Run

### Attended wrist-only leader following

After the October 2 ±3° independent wrist wave passed and the operator confirmed
smooth motion, `wrist_follow.py` provides the next bounded experiment:

```bash
.venv/bin/python wrist_follow.py --pairing calibration/leader_pairing_20261002T202428971124Z.json
```

Stop the pairing dashboard first; both applications own the same hardware. Open
`http://127.0.0.1:8765`, confirm support/cable clearance, and **press and hold to
follow**. Startup is read-only. Only the saved, checked wrist direction is reused;
old absolute leader origins are not loaded. Each hold captures fresh relative
baselines. Other leader joints may move freely. Only follower joint6 is enabled;
the other five follower joints must remain supported within 1° of startup.
The first physical following trial revealed opposite wrist rotation with the
preview-derived mapping. The saved pairing now includes a wrist-only raw-to-raw
sign of -1 in `relative_follow_checks`, which overrides that preview mapping for
following. The operator confirmed the corrected physical wrist direction.

Targets remain within ±15° of the **whole session's** starting wrist position,
even across repeated holds. Target speed is capped at 12°/s and trajectory
acceleration at 40°/s², with braking before the fixed travel boundary.
Holds have no duration cap while the button remains held and heartbeats continue.
The former 20-second cap was removed at the operator's request. Confirm room for ±15° of wrist/cable rotation
using the page's clearance checkbox before holding the button.
The initial 1°/s trial showed a steady ~68 ms control cycle but substantial target
lag on hand reversals. After physical direction confirmation, the cap was raised
to 3°/s, the leader polling pause reduced to 20 ms, and the follower loop pause
reduced to 5 ms. The operator then explicitly requested substantially wider
bounds after the measured motor-speed guard tripped at 8°/s. The present limits
are ±15°, 12°/s target speed, 35°/s measured-speed cutoff, 4° tracking-error
cutoff and 2° measured envelope margin. The trajectory ramps acceleration;
MIT gains remain unchanged. All six follower position/status checks are retained.
Actual cycle duration, leader sample age, trajectory velocity and motor-reported
velocity are logged; speed faults now include the triggering measured value.
The operator confirmed the expanded wrist trial worked well. The recorded trial
reached 14.22° measured travel and 21.41°/s peak motor-reported velocity without
a reported fault (`outputs/wrist_follow_20261002T211544376166Z.jsonl`).
Release, Escape, browser blur/hiding, a 400 ms heartbeat expiry, stale leader or
camera frames, a control cycle over 300 ms, or a tracking fault ends following.
Release removes torque without an automatic return to center. Keep the arm
supported and its physical power switch accessible. Software stop depends on a
responsive process/transport; it is not a safety-rated emergency stop.

The wrist must already be in the MIT mode used by the successful independent
wave. On first hold, its communication timeout is set to 500 ms in RAM and read
back before enable; the previous timeout is restored on clean server shutdown.
The raw timeout register uses 50 µs ticks: 500 ms reads back as 10,000.
`--check-timeout` verifies setup and restoration without enabling any motor,
then exits (stop the dashboard before running it).
No zeros, flash parameters, or other motor settings are written. Motor API
reference: [MotorBridge Damiao API](https://github.com/motorbridge/motorbridge/blob/main/motor_cli/DAMIAO_API.md).
Faults latch until server restart; reconnects cannot resume motion. Both cameras,
leader input, and six follower positions are monitored. Target/measured offsets
and events are logged in `outputs/wrist_follow_<session>.jsonl`.
This does not validate whole-arm following, gripper mapping or camera geometry.

### Guided camera-versus-joint check

The axis dashboard now opens a four-view camera guide: starting view, wrist roll,
wrist yaw, and wrist bend. Keep the tag and robot base fixed. Use **All joints
follow**, briefly press Follow, then release to powered hold. Click **Capture
when steady** once; it waits for a usable view instead of requiring repeated
clicks. For each later view, move the highlighted wrist joint 6–15° in either
direction using the leader, release, and capture. A schematic, live movement
bars, tag quality, settling progress, and saved-view thumbnails show progress.
Coupled movement is included in the forward-kinematics comparison.

Capture requires fresh joint readings and a stationary arm (powered hold or external support),
a continuous one-second image-time stationary window (joint span ≤0.25°), stable camera
poses, and joint readings bracketing the image timestamp. Camera timestamps are
mapped from the SDK host clock to Python's monotonic clock. Joint reads are
sequential host-timed transactions, not hardware-synchronized exposures; the
stationary checks bound this limitation. Measured video delays up to 1.5 seconds
are handled by bracketing exposure timestamps in joint history and requiring
stationarity from that image window through the present; delayed images are never
paired with a newly moved pose. The page displays image delay. Invalid/ambiguous tag estimates and
stale streams prevent capture. Capture actions never enable or move motors.

The dashboard uses NV12 camera transport at 10 fps, retaining 1280×800 wrist
images and using 640×400 for the workspace preview. On the current shared USB 2
connection, the measured median wrist image age fell from 762 ms to 98 ms, and
workspace age from 1053 ms to 46 ms. Wrist intrinsics, calibrated focus (76), and
sensor coordinates were unchanged. `camera_latency_probe.py` reproduces the
camera-only comparison after stopping other camera clients; measurements are
saved in `outputs/camera_latency/report.json`. These are host image-age measurements,
not end-to-end browser display latency.

`outputs/camera_checks/<session>/report.json` and four annotated images preserve
the measurements. Starting a new check preserves earlier sessions. The result
compares relative rotation magnitudes, which do not require a known rigid camera
mount transform; differences over 2° are flagged for investigation. Agreement
is diagnostic only: it does not validate rotation direction, translation,
hand-eye calibration, stereo depth, or fingertip geometry. No calibration files
are changed. Workspace-image rotation is presentation-only.

Completed checks can be reviewed after restarting the dashboard by adding
`--camera-report outputs/camera_checks/<session>/report.json`. Only completed
checks from the same geometry are accepted; loading one does not reuse old
camera frames, apply calibration, or enable motors. Start over begins a new check
while preserving the old report and images.

`camera_alignment_trial.py` provides a separate bounded diagnostic for the
authorized clear workspace and supported folded pose. Stop the dashboard before
running it. Without arguments it previews an example without hardware access.
With `--execute`, it keeps the base, shoulder, and elbow at their starting angles,
moves roll +8°, yaw +8°, and bend −8°, captures stationary views, then reverses
the wrist path and removes torque at the starting folded pose. Targets are limited
to 4°/s and 12°/s². Fresh motor status, temperature, measured speed, tracking error,
camera freshness, existing joint limits, and a small local envelope are checked.
Capture uses measured joint angles, with the same stationary-window checks as
the dashboard. A settling timeout with healthy feedback attempts the reverse
path; a motor/control fault invokes torque-off cleanup. The script is not a
general home planner or collision checker. Wrist joints may settle slightly after
torque removal; both pre-release and final readings are recorded.

`--execute --recover-trial outputs/camera_diagnosis/autonomous_trial_<session>.json`
returns only wrists to that trial's start, rejecting displacements over 10° or
large-joint changes over 0.1°. It holds the large joints at their current angles.
It requires the same supported folded pose and hardware exclusivity. Reports are
saved under `outputs/camera_diagnosis`; no motor zeros, flash settings, or camera
calibration are written.

The OAK-D Lite's BMI270 was verified operational. `camera_alignment_trial.py
--execute --imu` records its raw gyro and accelerometer at 100 Hz alongside the
camera and arm readings; IMU firmware updating is explicitly disabled. Add
`--bend-repeat` for outward/return/repeated bend measurements with all other joints
fixed. `imu_camera_analysis.py <trial.json>` integrates body-frame gyro samples,
subtracting interpolated stationary endpoint biases and rejecting gaps over 30 ms.
Gravity tilt is a separate check; it does not measure yaw. The resulting
`imu_comparison_<session>.json` appears in the dashboard when that completed camera
report is loaded, including a visual comparison of motor, image, and gyro angles.

October 4 independent results changed the diagnosis: the bend-only trial reported
8.01°, 8.03°, and 8.04° from motor readings, but the camera-body gyro measured
6.14°, 6.19°, and 6.19°. Tag estimates differed from gyro by only 0.02–0.35°;
gravity tilt was 6.42–6.46°. This establishes a physical motion discrepancy for
this pose, without identifying a particular mechanical cause. The user's selected
approach is to treat camera measurements as the reference and calibrate the
motor-to-motion mapping. A local bend scale of 0.787 camera degrees per motor
degree fits the outward/return/repeat set within 0.22°, but leaves up to 1.54°
error in earlier wrist poses. This candidate is saved in
`outputs/camera_diagnosis/camera_reference_scale_candidate.json`; it is not an
active global correction. More paired observations across pose and direction are
needed to distinguish a stable mapping from history-dependent motion. Mechanical
inspection is not established as a prerequisite by these measurements.
Original raw images and stationary pose estimates are now
saved with captures for further analysis. Higher resolution and alternate corner
detectors did not consistently remove the discrepancy.

### Autonomous camera-referenced joint measurements

`calibrate_joint_motion.py` previews a bounded route; `--execute` collects it
using both cameras and all six arm motors. Stop the dashboard first so only one
process owns the hardware. Starting near the folded reference, each joint moves
at most 8° at 3°/s with the other joints held, then returns before the next joint.
Fresh camera views, motor feedback, existing joint limits, tracking error, speed,
and temperature checks remain active. Captures use the same stationary gate as
the dashboard. Original images, measured poses, motion logs, return verification,
and motor-off verification are saved under `outputs/joint_calibration/<session>`.
No motor zeros, firmware, global kinematics, or live following mapping are changed.

The default sequence fits separate outward/return angular curves from five views
and tests four additional views. `--dense --joints 5 6` instead fits full camera
rotation with 2° knots and tests intermediate positions in both directions: nine
training views and ten validation views per joint. Rotation interpolation follows
the shortest path between measured orientations. Validation views never enter
the fit. Saved models are local to the recorded reference pose and motion
history; prediction helpers reject extrapolation.

October 4 autonomous runs captured 110 views. Base, shoulder, elbow, and bend
passed local projected-angle checks with largest held-out errors of 0.18°, 0.57°,
0.44°, and 0.71°. Yaw and roll candidates had full-rotation errors of 0.81° and
1.11°, exceeding the unchanged 0.75° target. They remain unvalidated candidates.
These are two different error metrics, not six full-workspace accuracy claims.
All three runs returned to their folded starting pose before motor-off verification.

Load the visual six-joint report with:

```bash
.venv/bin/python axis_follow.py --pairing calibration/leader_pairing_20261002T202428971124Z.json --joint-calibration calibration/camera_joint_motion_summary_20261004.json
```

The panel shows measured curves, separate validation points, tested raw ranges,
and earlier wrist trials. Loading a report checks its geometry fingerprint and
does not enable motors or apply its corrections to live control. The aggregate
report links every original measurement file. Full workspace, absolute joint
zero, hand-eye translation, and fingertip calibration remain separate work.

### Leader following with powered pause

```bash
.venv/bin/python axis_follow.py --pairing calibration/leader_pairing_20261002T202428971124Z.json
```

Stop other arm/camera clients first. The local dashboard starts with **all motors
disabled**, **All joints follow** selected, and wrist yaw selected for inspection.
The gripper is never registered or commanded. No saved absolute pose is replayed.

Press and hold to follow. **Release, Escape, browser blur, heartbeat expiry,
or stale leader/camera input pauses following and holds the measured position
with motor power on.** The motor thread continues checking feedback and sending
hold commands independently of the browser, leader, and cameras. Input recovery
does not resume motion: a new press captures a fresh leader reference. You can
move the leader while paused to reposition it without moving the follower.
There is no duration cap while following or holding.

**Pause is not torque-off.** To remove motor power, physically support the arm,
check the separate support checkbox, then use **Remove motor power**. Motor faults,
tracking/velocity/temperature violations, a stalled motor loop, server shutdown,
or controller/USB/power failure can still remove torque and let the arm drop.
Powered holding depends on a functioning motor controller; it is not a mechanical
brake or a guarantee against falling. Support must remain available.

| Axis | Local range | Target speed | Target acceleration | Measured speed cutoff |
| --- | --- | --- | --- | --- |
| Base, shoulder | ±60° | 18°/s | 60°/s² | 60°/s |
| Elbow | ±90° | 18°/s | 60°/s² | 60°/s |
| Wrist bend, yaw, roll | ±45° | 24°/s | 80°/s² | 75°/s |

These expanded settings passed offline tests; they still require a supported
physical trial. Ranges are intersected with the existing model limits and shown
in the dashboard. They do not establish self-collision, workspace, or cable
clearance. A starting pose up to 1° outside a software model limit may be held
where it already is, but targets can only move inward. Larger discrepancies
block setup. Startup encoder variation up to 0.1° is clipped inside the command
range; larger deviations report the affected joint and allowed interval.

The incremental leader mapping discards excess movement beyond a limit instead
of accumulating an unreachable target. Reversing the leader moves the requested
target inward immediately (the bounded trajectory still accelerates/brakes).
Rows display **AT LIMIT**. Pause/resume re-establishes the leader reference without
expanding follower bounds. Changing selection or mode while disabled captures a
new starting window; reversing a direction preserves that window.

Target smoothing uses an 80 ms response constant. Non-roll motor velocity commands
allow 1.5 times target speed for catch-up. Tracking error is limited to 8°, and
measured positions must remain within 2° of the command envelope. Motor feedback,
temperature checks, and the 500 ms motor watchdog remain active while paused.

**Selected joint only** powers only that joint; all other links require external
support and may drift no more than 2°. **All joints powered · selected joint
follows** holds the other five positions. **All joints follow** accepts coupled
leader movement on all six axes. Support and remove power before changing mode,
selection, or direction. Direction confirmation still requires at least 2° of
leader and 1° of follower movement. Saved physical direction overrides are retained.

Roll retains its tested MIT gains. Other axes use position/velocity mode with
`B601_GAINS` on the first three and existing positive gains on bend/yaw. Initial
enabling begins with 0.8 seconds of slow load take-up and a 1° settling check;
leader movement during that period is ignored. Resume from powered pause does
not re-enable motors. Modes, gains, and timeouts are changed only in RAM with
readback checks and restored after torque-off on exit. No zeros or flash writes.
`--check-settings` verifies configuration/restoration without enabling motors.

Logs in `outputs/axis_follow_*.jsonl` distinguish requested pauses, actual powered
pauses, resume, explicit torque-off, faults, and server shutdown. Browser pause
requests include their event reason; heartbeat expiry is identified server-side.
Samples include which joints are limited. Fault cleanup attempts to disable every
attempted/active motor even when one disable fails. A failed cleanup is reported
as an attempt, not confirmation that motor power is off.

Dependencies are installed in `.venv`; `uv.lock` pins them. To recreate: `uv sync`.

The OAK and B601 serial controller need Linux USB permissions configured. Run once:

```bash
bash scripts/setup_usb.sh
```

Enter your sudo password in your terminal. Reconnect the OAK if necessary, then:

```bash
uv run hello_world.py --preview
```

### Migration to this computer (September 30, 2026)

Current camera roles are OAK-D-LITE `19443010C104077E00` on the wrist and
OAK-1 `19443010A155785A00` on the tripod, recorded in `config/cameras.json`.
Arm demos, tag observations, and hand-eye capture explicitly select the wrist
camera and refuse to substitute the tripod if it is unavailable. The earlier
OAK-1 wrist camera documentation below describes the previous setup. Saved
hand-eye candidates and prior physical route verification must not be assumed
valid for the current camera and mounting. Start a new hand-eye session when
collecting measurements with this setup.

Wrist previews and saved viewing images rotate 180 degrees for the upside-down
mount. Labels are drawn upright after rotation. Raw calibration images, depth
arrays, tag coordinates and metric estimates retain original sensor coordinates;
reports identify the display rotation. Offline `tag_view.py --image` leaves the
supplied image orientation unchanged.

The locked Python environment and pinned DM URDF have been restored. The URDF
fingerprint matches the saved hand-eye capture dataset. All 44 offline tests pass,
and `continuous_approach.py` previews successfully. The most recent documented
hardware experiment was the continuous noncontact lift/scan/approach/return below.
Contact and hand-eye validation remain unresolved.

USB enumeration finds the HDSC arm controller at the existing persistent serial
path, one Luxonis OAK device, and a CH340 serial adapter. Joint reads and camera
capture currently fail on device permissions. `scripts/setup_usb.sh` now covers
both the OAK and this specific HDSC controller, using your account's primary group.
Run it in your terminal with your sudo password, then perform read-only checks:

```bash
.venv/bin/python calibrate_arm.py inspect
.venv/bin/python tag_view.py --seconds 5 --output outputs/migration_tag.jpg
```

No motors were enabled during migration. Before replaying a taught route or
executing a lift, establish the current supported starting pose and tag/base
placement. Saved observations do not establish their positions after relocation.
The recreated interpreter is stored in ignored `.python/` so it survives temporary
directory cleanup. Previous ignored `outputs/` run telemetry was not included in
this checkout.

### OAK-D Lite stationary revalidation (October 1, 2026)

`revalidate_setup.py` reads the disabled arm before/after capturing synchronized
RGB and RGB-aligned stereo depth, excluding 30 warm-up frames. The current setup
passed: 60/60 tag detections, 100% valid depth in the tag interior, approximately
3 mm median stereo/tag-plane disagreement, 0.43 mm tag Z sample variation, and
less than 0.6 ms RGB/depth timestamp difference. Joint positions did not change.
These checks use the configured 60 mm tag size; they are consistency measurements,
not independent dimensional or hand-eye validation. All 47 offline tests pass.

Summary: `calibration/wrist_setup_validation.json`. Original measurements and
images: `outputs/wrist_revalidation/`. Current RGB-only observation check:

```bash
.venv/bin/python tag_pose.py
```

The current arm pose is outside the lift demo's five-degree reference tolerance.
Physical route validation and camera-to-wrist calibration remain incomplete for
the new mount. Hand-eye sessions now record camera identity and reject resuming
legacy samples or samples from another camera. To collect a new session in an
attended terminal, keep the tag/base fixed, manually support the disabled arm,
and capture at least 20 distinct stationary views with varied wrist orientations:

```bash
.venv/bin/python calibrate_camera.py capture --new-session
.venv/bin/python calibrate_camera.py solve
```

Space captures a pose; Q finishes. `--new-session` archives the old sample JSON
and preserves its images. The solver still requires held-out errors within
5 mm and 2 degrees and does not enable motion.

The October 1 OAK-D session `20261001T234501896017Z` contains 20 poses with
useful rotation diversity. Its held-out translation errors are 12.31, 5.55,
26.35 and 10.34 mm; orientation errors are 1.46, 0.79, 0.67 and 0.85 degrees.
Position validation fails. Training-only corner refinement also fails and is
preserved as a diagnostic candidate; neither candidate is activated. Verify
tag/base stability, measured black-square size, rigid flat backing, camera mount
rigidity and physical joint reference before prescribing a replacement session.
The original capture images remain available; review montage:
`outputs/handeye_review.jpg`.

The operator confirms the black square measures 60 by 60 mm, is mounted on a
rigid flat board and stayed fixed. A fresh 12-sample stationary check measured
less than 0.07 mm translation standard deviation per axis and unchanged joint
readings. Choosing before versus after joint readings does not resolve the
held-out error (maximum approximately 25.9 versus 26.9 mm). Small training-only
reference-offset and scale adjustments also fail; these diagnostics do not modify
the arm reference or activate a transform. The root cause remains unresolved.
Next compare the physical arm and camera mount against the recorded reference
and model; repeating an unchanged full capture session is not yet prescribed.

The supported reference check showed shoulder/elbow/wrist-bend differences from
the saved reference of approximately 0.32/0.003/0.19 degrees; the reference was
not overwritten. The operator also confirms the robot base was securely fixed.
The OAK-D Lite RGB sensor has autofocus, with observed lens positions changing
while its intrinsic metadata remained constant. Both stored and factory
calibration specify lens position 76. `config/cameras.json` now pins that focus;
RGB clients apply it in RAM after startup and hand-eye captures record actual
lens position. Sessions from the previous autofocus configuration cannot resume
into a fixed-focus session. This may contribute to calibration error, but is not
yet established as the sole cause. RGB-only manual-focus capture works. Full
hand-eye and motion validation remain incomplete. Focus changes do not write
camera EEPROM or motor settings.

USB reconnection did not resolve DepthAI 3.10's stereo-source firmware assertion.
The same IMX214/OV7251 failure is described in Luxonis issue
https://github.com/luxonis/depthai/issues/1252. An isolated DepthAI 2.32 environment
produced 60 synchronized RGB/stereo frames without a crash and with measured
lens position 76 throughout. Main arm dependencies remain on their pinned v3
versions. Reproduce the camera-only stream check:

```bash
UV_CACHE_DIR=/tmp/b601-uv-cache uv venv .venv-depth-v2 --python .venv/bin/python
UV_CACHE_DIR=/tmp/b601-uv-cache uv pip install --python .venv-depth-v2/bin/python -r requirements-depth-v2.txt
.venv-depth-v2/bin/python depth_probe_v2.py
```

`outputs/wrist_depth_v2_native/report.json` shows stable streams, but sparse depth
in the tag region and failed tag/depth metric consistency. RGB/depth alignment
also remains unvalidated. This stream workaround is not an accepted depth
calibration or motion activation. The v3 `revalidate_setup.py` stereo graph remains
available for diagnosis but currently triggers this firmware error on this unit.
After focus locking, the RGB-only 20-observation check measured approximately
0.10 mm Z standard deviation.

The replacement fixed-focus session `20261002T003620145947Z` now contains 20
poses, all verified at lens position 76. Held-out translation errors are 3.97,
7.68, 12.12 and 4.28 mm; orientation errors are 0.81, 0.72, 1.27 and 1.27 degrees.
The worst position error improved from 26.35 to 12.12 mm, but still fails the
unchanged 5 mm limit. Individual tag corner RMS errors are 0.031–0.256 pixels.
Training-only corner refinement also fails and is saved separately in
`calibration/arm2_handeye_fixed_focus_refined_candidate.json`. Before/after
joint-read choice and diagnostic reference-offset fitting do not resolve the
disagreement; reference settings remain unchanged. The operator confirms the
camera mount is firmly fixed with no wobble. Independently measure joint/model
agreement before another full capture.
Review images: `outputs/handeye_fixed_focus_review.jpg`. Current results are
recorded in `calibration/wrist_setup_validation.json`; no candidate is activated
and motion remains disabled.

The camera starts first, followed by a three-second countdown. Joint 6 rotates ±8°
from its measured starting position, twice over 12 seconds, then returns to the
starting target and disables the wrist. Other joints and the gripper are never
commanded. Place the arm in a stable, supported pose and leave room for the wrist
and camera cable. The wrist must initially be disabled; an already enabled wrist
is rejected to avoid taking over another controller.

Press **Q**, **Esc**, or **Ctrl+C** to stop and disable the wrist immediately.
Stopping early does not run a return trajectory. A disabled wrist can rotate under
its load. Keep the hardware emergency stop accessible; software cannot stop a motor
if the USB link or process is lost.

Without `--preview`, the same demo runs headlessly. The latest RGB frame is saved
to `outputs/hello.jpg` on exit. OAK-1 provides RGB, not stereo depth.

## Individual checks

```bash
uv run hello_world.py --mode probe        # Read joint6 only; no enable or mode writes
uv run hello_world.py --mode camera --preview
uv run hello_world.py --mode dry-run      # Print offsets; no hardware access
uv run hello_world.py --amplitude 5 --cycles 1 --preview
uv run python -m unittest discover -s tests -v
```

`--port` overrides the default persistent serial path.
`--output` changes the snapshot path. Amplitude is capped at 10°, period at least
6 seconds, and cycles at most five. The demo checks joint6 travel limits, fault
status, temperature, and tracking error. It is a small joint-space demo, with no
collision planning. The motorbridge state API has no receive timestamp, so cached
feedback cannot prove a live bus connection.

## Implementation references

- [Seeed Python SDK](https://github.com/Seeed-Projects/reBotArm_control_py), inspected at commit `6415d43130d1e143c70dc106096a857ac5556f81`. Reference checkout is in ignored `vendor/`; the demo does not import it.
- DM joint6: motor ID `0x06`, feedback `0x16`, model `4310`, MIT gains Kp=18/Kd=2, URDF limits ±3.14 rad. The demo adds a 0.1 rad limit margin.
- [Luxonis Camera API](https://docs.luxonis.com/software-v3/depthai/depthai-components/nodes/camera/)
- [Luxonis Linux USB setup](https://docs.luxonis.com/hardware/platform/deploy/usb-deployment-guide)

## Verification on this machine

Serial feedback from joint6 succeeded (about 8.86°, disabled, 27°C MOS / 25°C rotor).
Motion bounds and cleanup tests pass. Live camera capture and the physical wave
remain unverified until the OAK USB permission rule is installed. The demo fails
before enabling the wrist when the camera is unavailable.

## AprilTag detection (no arm motion)

```bash
uv run tag_view.py --preview
uv run tag_view.py --image outputs/tag_initial.jpg
```

Searches for ID 0 in the four OpenCV AprilTag dictionaries (or specify
`--family 36h11`). Saves an annotated image and JSON corner coordinates. Detection
is in pixels only; it does not establish a metric gripper target or confirm contact.
Synthetic marker tests cover all four families and rejection of other IDs.

The initial live image did not contain a detected tag. Full-arm lift/search and
contact are pending the tag dimensions, camera-to-tool calibration, and resolution
of the joint coordinate mapping: measured joint2=+0.981 rad and joint3=+1.765 rad,
while the inspected DM URDF limits both to [-3.14, 0]. No arm motion was commanded
during this inspection. Do not negate these angles without verifying the physical
joint directions and calibration offsets.

Tag settings are now saved in `config/tag_task.json`: family **36h11**, ID **0**,
side **0.060 m**. `tag_view.py` defaults to that family and ID. The teaching tool
was found at `/home/chris/rebot-hello/teach_ranges.py`; its range file is copied to
`config/taught_joint_ranges.json` with the source path recorded. It contains raw
motor travel ranges, not joint-to-model offsets, hand-eye calibration, or a taught
20 cm lift. Those missing transforms remain explicitly unset. The older folder
also describes a second arm, so the range file's physical-arm identity needs to
be resolved before using it to bound arm motion.

The connected arm is confirmed by the user to be the **second arm**. The expected
`/home/chris/rebot-hello/joint_ranges_arm2.json` was not found. The copied first-arm
ranges are therefore excluded from the active task configuration. A recollection
that the elbow test raised the gripper is not a measured Cartesian calibration.
Before autonomous lift/contact, establish this arm's joint-to-model mapping and
camera-to-gripper/contact-point geometry. A manually supported viewing pose with
the tag visible allows camera/tag observations without guessing a lift trajectory.

## Metric tag observation

`uv run tag_pose.py` captures 20 observations of the configured 60 mm tag with
per-frame OAK intrinsics and distortion metadata at 1280×800. It saves
`outputs/tag_pose.json` and an annotated image. Each sample retains both planar
pose candidates when valid; estimates are in the camera frame (x right, y down,
z forward), not the robot base or fingertip frame. No motors are accessed.
A synthetic projection test checks scale, distortion handling, and corner order.

The user manually raised the second arm; its read-only joint snapshot is saved
in `outputs/arm_raised.json`. Tag36h11 ID 0 was detected in that view, saved as
`outputs/tag_raised.jpg`. This observation does not establish a safe replay path
or a calibrated contact point.

A subsequent supported capture succeeded: 20 usable observations put tag 0 at
approximately (0.0576, 0.0236, 0.2445) m in camera coordinates. Across these frames,
translation standard deviations were (0.00088, 0.00050, 0.00321) m; these measure
sample variation, not absolute accuracy. The tag pose and image are in
`outputs/tag_pose.json` and `outputs/tag_pose.jpg`. The separately read joint
snapshot is `outputs/arm_held.json` and is not time-synchronized to the images.
All motors remained disabled and no movement commands were sent. Metric camera
observation is verified; fingertip contact and Cartesian control remain uncalibrated.

## Second-arm software reference calibration

See [the calibration procedure](calibration/README.md) and the included official
[reference photograph](calibration/seeed_reference_pose.jpg). Run
`uv run calibrate_arm.py reference` only while manually supporting the arm in that
reference pose. The tool records software calibration candidates, never writes
motor zeros or enables motors, and leaves motion blocked pending validation.
`uv run calibrate_arm.py inspect` reads current positions without saving a pose.

## Lift, find tag 0, and approach without contact

The current demo is `rise_approach.py`:

```bash
uv run rise_approach.py             # geometry preview; no hardware access
uv run rise_approach.py --execute   # attended physical run
```

Start with the second arm near its folded resting reference, tag/base fixed,
and the space above/in front clear. The first centimeter moves slowly and settles
so the wrist takes the load; the remaining lift and camera scan use a 0.06 rad/s
trajectory limit, six times the initial demo's 0.01 rad/s. Faster motion is blocked
until that load take-up completes. The 1-degree tracking limit remains in force.
The camera scans downward until tag36h11 ID 0 is reliably visible. `APPROACH`
performs a 2 cm noncontact move, with observations after each 5 mm step; `REST`
returns without approaching. It returns along the executed path afterward.

`arm_control.py` applies the supplied B601 position-loop settings to the three
large arm motors in RAM, retains the existing wrist settings, verifies each write,
and restores the original gains when stopping. No motor zero, protection or
flash settings are written. Correction-speed headroom is separate from the
trajectory's movement speed.

The full hardware run completed a commanded 20 cm lift, 25-degree downward scan,
and 2 cm approach. The observed camera-to-tag range decreased from 407.9 mm to
390.4 mm. This verifies a short approach; it does not calibrate fingertip contact.
Images, tracking samples and the run result are saved under `outputs/rise_*`.

### Faster run and closer approach

```bash
uv run fast_approach.py             # preview without hardware
uv run fast_approach.py --execute   # attended faster run
```

This retains the slow first centimeter and two-second settling interval, then
ramps native waypoint speed limits through 0.09, 0.12, 0.18 and 0.24 rad/s.
The maximum is four times the preceding demo's 0.06 rad/s travel limit; startup,
settling and camera observations still take their own time. `waypoint_motion.py`
checks measured deviation from each bounded joint-space segment against a
1-degree limit, along with speed, fresh feedback, endpoint settling and timeout.
It also refuses faster moves before the load take-up completes.

The camera continues tilting until the full tag is near its center. After
`APPROACH`, this version advances up to 6 cm in 5 mm steps, stopping early on
observation loss, a workspace limit, or the 25 cm camera-to-tag-plane standoff.
The approach remains slower than free-space travel. The executed path is reversed
before shutdown. This is still a noncontact demo.

The first full faster run reached the 0.24 rad/s waypoint setting, centered tag 0
at 40 degrees of downward tilt, and completed the 6 cm approach. Measured
camera-to-tag range decreased from 379.3 mm to 318.1 mm. Results and waypoint
telemetry are in `outputs/fast_approach_result.json`.

### Continuous lift and approach

`uv run continuous_approach.py` previews without hardware access; `uv run continuous_approach.py --execute` runs the supervised route from folded rest. It blends a 20 cm lift, 40° camera tilt, up to 6 cm noncontact approach, and return without waypoint dwells. The first centimeter takes up load gently while monitoring stability. Free-space speed is capped at 0.48 rad/s, with acceleration limiting and an earlier slowdown during the final camera tilt to acquire eight fresh tag observations. Vision, clearance, joint bounds, and 1° path-deviation checks remain active; missing vision causes a return. Do not move the tag or base, obstruct the taught workspace, or run another motor/camera client during execution.

Verified hardware run: completed 6 cm approach and returned in 49.35 s, measured peak joint speed 0.337 rad/s and maximum path deviation 0.481°. Camera-to-tag range at the approach endpoint was 324 mm (camera-to-tag-plane distance 303 mm). All six arm motors were verified disabled and original gains restored afterward. This is a noncontact demonstration, not a verified fingertip contact routine. The doubled speed cap does not mean the whole cycle runs twice as fast. Results: `outputs/continuous_result.json`; endpoint image: `outputs/continuous_closest.jpg`. All 44 offline tests passed before this run.

### October 4: faster stereo approach and first confirmed tag touch

The current camera-controlled runner is `camera_visual_approach.py`. It uses the
working `AxisArm` controller, the calibrated/fixed-focus RGB view for the first
20 cm, then the OAK-D Lite B/C stereo pair for incremental fingertip positioning.
The operator witnessed the padded fingertip touch the AprilTag paper on October 4.
The complete run returned to folded rest and verified all six arm motors off.
Contact record: `calibration/first_confirmed_tag_touch_20261004.json`;
full telemetry: `outputs/visual_approach/20261004T200403261403Z/report.json`.

```bash
# Preview only (default):
.venv/bin/python camera_visual_approach.py --fast --forward-m .20 --bend-view 6
# Clear workspace, same fixed arm/tag/camera/gripper setup, no competing clients:
.venv/bin/python camera_visual_approach.py --execute --fast --forward-m .20 --bend-view 6 --stereo-servo
# Explicit bounded touch attempt, followed by retraction:
.venv/bin/python camera_visual_approach.py --execute --fast --forward-m .20 --bend-view 6 --stereo-servo --touch
```

Fast traversal merges waypoints only when their joint-space chord differs by at
most 0.2 degrees from the dense route. It preserves the initial viewing pose and
lift corner, reduces stationary captures from 52 to four, and requests at most
12 degrees/s with 36 degrees/s² acceleration. Measured-motion pacing reduces
speed when tracking lag grows. The free-space tracking limit is 4 degrees;
stereo traversal restores the 2-degree limit and slower settings. The verified
20 cm waypoint was reached in about 40 seconds including camera startup, versus
164 seconds in the preceding capture-at-every-5-mm run. The complete touch run
kept motors enabled for 190 seconds; maximum recorded tracking error was 2.294
degrees. Near the paper, steps decrease to 0.5 mm with a 1 degree/s joint cap.
These are phase-specific settings, not global teleoperation changes.

`stereo_contact_observer.py` runs in `.venv-depth-v2` (DepthAI 2.32), publishing
fresh, synchronized, metric B/C tag poses. At the tested 20 cm endpoint, tag-edge
errors were below about 0.4 mm. DepthAI 3.10 simultaneous RGB/mono capture crashed
the device during motor-off diagnostics; RGB alone works. No firmware or EEPROM
was changed. The v2 RGB preview's intrinsic metadata did not describe its actual
crop; stereo control therefore uses B coordinates directly. Diagnostic evidence
is under `outputs/contact_camera/20261004T190921818454Z` and
`outputs/contact_camera/20261004T194430628210Z`.

The touch target is the white border 5 mm beyond the black square, preserving
stereo visibility. Contact occurred with the candidate fingertip landmark still
reporting about 5 mm clearance; camera advance stalled and motor load changed.
The operator confirmed physical touch. The exact onset and force were not
measured, so the saved contact is a calibration observation, not a globally
validated fingertip/hand-eye transform. Final approach never requests penetration
past the estimated paper plane, checks both fingers, and retracts on visibility
loss, near-paper stalled advance, or load change. The stalled-advance guard and
closest-view snapshots were added after the confirmed touch run and passed
offline checks; they have not yet been exercised on another hardware run.

The longer-settling wrist refinement did not pass: roll held-out full-rotation
error was 1.94 degrees, and the yaw repeat lost tag visibility. Previous results
and the unsuccessful repeat remain in the dashboard summary. No local calibration
curve was globally activated. All 141 offline tests passed after these changes.

#### Operator-requested 2× cruise profile

Add `--speed-scale 2` to the `--fast` command for a 24 degrees/s cruise cap and
144 degrees/s² acceleration. Native position/velocity motor command limits remain
27 degrees/s for the first three axes and 36 degrees/s for wrist bend/yaw.
Measured-speed protection is 40 degrees/s for this run, below the existing
teleoperation profiles. The known free-space route permits 6 degrees tracking
error and paces at 4.5 degrees lag; stereo uses tighter limits, and final contact
retains the previous slow/small-step settings. Three-frame stereo checks replace
seven-frame checks only during the coarse approach; final approach still uses
seven frames. Load changes pause advancement immediately and require three
settled readings before ending the attempt.

Hardware run `outputs/visual_approach/20261004T201108905356Z/report.json` completed
the route and returned with all six motors verified off. Peak shoulder speed rose
from 7.29 to 13.94 degrees/s; elbow from 7.48 to 12.85 degrees/s. Maximum measured
joint speed was 19.62 degrees/s and maximum tracking error 4.463 degrees. This
run retracted on the load guard at 9.03 mm candidate-landmark clearance; a second
physical touch was not confirmed. The prior run remains the operator-confirmed
contact. Enabled duration was 126.5 seconds versus 190.3 seconds previously, but
these whole-cycle times include different final-approach endpoints, so they do
not establish a 2× complete-cycle speedup. All 142 offline tests pass.

#### 48 degrees/s noncontact trial (another requested doubling)

`--fast --speed-scale 4 --forward-m .20 --bend-view 6` is restricted to the known
noncontact route. It ramps the viewing/lift portion at 36 degrees/s, then requests
48 degrees/s at 288 degrees/s² on forward/return travel. A transient per-command
native velocity ceiling of 60 degrees/s is set only on this trial's AxisArm
instance (first five joints); normal dashboard defaults, motor gains and flash
remain unchanged. Measured-speed protection is 60 degrees/s. Tracking protection
is 8 degrees, matching the existing teleoperation tracking ceiling. A bounded
trajectory-clock advance reserves 0.5 degrees below that ceiling without changing
the joint-space segment. The profile rejects stereo/contact options so this
experiment does not propagate the new speed to the paper approach.

Run `outputs/visual_approach/20261004T201749820363Z/report.json` completed the 20 cm
route and return, with all six motors verified off. Shoulder peak increased from
13.94 to 19.98 degrees/s (43%); elbow from 12.85 to 14.44 degrees/s; wrist bend
from 19.62 to 24.98 degrees/s. This did not produce another doubling of actual
speed. Tracking error reached 6.643 degrees; maximum motor temperature was 43°C.
The motors were enabled for 41.8 seconds on this noncontact-only run; that is not
directly comparable with the earlier 126.5-second stereo/contact trial. Short
segments still accelerate, brake and settle individually, and tracking pacing
also limits progress. Smooth waypoint transitions are the next performance
refinement, before further cap increases. All 144 offline tests pass.

Manufacturer reference for configurable native POS_VEL speed limiting:
https://wiki.seeedstudio.com/rebot_arm_b601_dm_pinocchio_meshcat/
Manufacturer load/temperature test reference (V4 hardware; not a certification of
this particular arm/setup):
https://github.com/Seeed-Projects/reBot-DevArm/blob/main/hardware/reBot_B601_DM/performance_testing/Performance_Testing.md

#### Blended motion, multi-angle observations and plate approach (October 4)

`--blend --sweep` adds shape-preserving continuous waypoint interpolation and
bounded six-axis observations at two distances. Run
`outputs/visual_approach/20261004T202540602851Z` completed 29 of 48 requested
offset views (92 observations including centres/transit); remaining views lost
the tag. The blended 20 cm segment took 9.31 s without intermediate stops.
The final folded return missed the strict roll tolerance by 0.418 degrees;
the large joints returned and motor-off state was subsequently verified.
This is a sampled working region, not validation of the entire mechanical range.

`calibration/stereo_sweep_handeye_20261004.json` is a **camera B** candidate,
with 31 distinct poses and held-out errors up to 3.69 mm / 1.19 degrees.
It is not globally activated. Extended plate poses exposed a larger model/visual
disagreement, so final approach uses camera-relative plate measurements.

`plate_survey.py` records full-resolution wrist and tripod images, plus optional
DepthAI 2.32 RGB/stereo triplets. The plate holder has tag 36h11 ID 1, approximately
30 mm square; the upright reference remains ID 0, 60 mm. At some wrist angles a
finger occludes tag 1 in camera C. Small-tag upsampling helps only when the whole
tag is visible. `--execute --hover --lower` uses the well grid to reach a 25 mm
noncontact standoff and return; that descent succeeded in
`outputs/plate_hover/20261004T210850400559Z`.

The grid uses nominal 9 mm well spacing from ANSI/SLAS 4-2004:
https://www.slas.org/SLAS/assets/File/public/standards/ANSI_SLAS_4-2004_WellPositions.pdf
Its reference points are manually initialized; tracked reprojection consistency
is not an independent absolute-accuracy measurement. A visible flange feature
was also triangulated in the close-up stereo views. The tripod provides an
external clearance view; its single-tag extrinsic remains a candidate.

Gripper tests preserve mode/gain/timeout settings and leave the six arm motors
off. Increasing motor angle closes the jaws. The estimated usable opening is
about 89.5 mm, anchored to the operator's earlier 80 mm ruler measurement. Opening
tests stopped on speed/load guards rather than pushing farther. A small native
force-position trial at ratio 0.02 succeeded and returned in
`outputs/gripper/20261004T211110804141Z`. This ratio limits motor torque/current;
it is not a calibrated contact-force measurement.

Two faults found during the plate work were corrected:

- OpenCV's rotation-vector conversion rounded some 1e-5 rad finite differences
  to zero, breaking the angular Jacobian. `rotation_math.py` retains those small
  rotations; regression tests cover all six revolute derivatives.
- A roll-settling failure during nested retreat left the controller short of the
  hover pose. Attempting the next reverse blend then raised a start-mismatch
  error and disabled the extended arm. The operator supported and reset it.
  Plate motion now uses `PlateArm`: a software exception with healthy motor
  feedback keeps the extended arm holding, awaiting explicit physical-support
  confirmation before power removal. Drive/feedback faults retain the driver's
  shutdown behavior. Plate-route roll settling allows 0.8 degrees while visual
  orientation checks remain active.

`--execute --hover --pickup` approaches an 8 mm review point. It does not close
until the current run's camera views have been inspected and `grasp.txt` is
created in its output directory. The prepared routine uses the tested force
limit, attempts only a 5 mm test lift, places the plate back and verifies jaw
reopening before retreat. Do not infer a successful pickup from this command's
availability; inspect the run's `pickup_report.json` and both camera views.

Post-reset plate attempts on October 4 returned to rest with motor-off checks:

- `outputs/plate_hover/20261004T213530664490Z`: camera-guided 8 mm review
  succeeded. A closure after 4 mm descent reached its bounded travel without
  a grip. Jaws reopened and settings were restored; no lift was attempted.
- `outputs/contact_camera/20261004T214034370728Z`: close stereo review, with
  approximate manually matched flange/table landmarks in
  `plate_clearance_review.json`. Flange landmarks were near grid z=0; the
  tabletop point was near -14.7 mm. These sparse measurements do not establish
  clearance for the entire finger body or holder.
- `outputs/plate_hover/20261004T214245856952Z`: final descent stopped for
  lateral drift, before gripper activation.
- `outputs/plate_hover/20261004T214538164304Z`: final visual descent remained
  near the target centre but did not converge. Comparing settled arm torques
  at pregrasp and stop showed substantial unloading/reversed loading:
  `[-.47,11.59,-6.43,-1.67,.07,.17]` versus
  `[-4.55,-1.22,1.91,2.71,1.65,.22]` Nm. This suggests physical contact with
  the plate/holder, not merely a kinematic-model discrepancy. No gripper
  closure or test lift occurred. The arm returned and all motors were off.

`plate_contact_guard.py` now rejects large changes in settled arm loading during
final descent, with a regression against those actual readings. This is a
contact indication, not calibrated tool-force sensing. All 156 offline tests
pass. The wrist-roll visual correction no longer accumulates commands against
stiction when the observed orientation is already within 0.5 degrees.

**No plate pickup has been verified.** Further contact motion needs the actual
finger/holder interference resolved. An operator observation of what touched
was requested; both cameras and the dashboard were restored with motors off.
The current 8 mm review and lower descent code are experimental, not a validated
plate-handling procedure. Do not infer collision clearance solely from the two
distal pad landmarks or manually initialized well-grid pose.

Requested observation repeat `outputs/plate_hover/20261004T215514431160Z`
used `observe_only.txt` to save both views at each final-descent step and prevent
closure. The new load check stopped at a measured distal-landmark height near
6.2 mm, with settled torque changes
`[-1.552,-4.800,3.303,0.784,0.359,-0.151]` Nm. Both stop images are saved as
`pickup_stop_view.jpg` and `pickup_stop_tripod.png`; contact object is not yet
confirmed. The gripper remained open. Return to rest and motors-off were
verified, and the live dashboard was restored.

Operator follow-up and pitch correction (October 4, ~15:24 PDT):

- The operator identified contact with the **white holder**, then clarified the
  hover was suitable for closure and confirmed the jaws contacted the clear
  plate in `20261004T220104487958Z`. The old travel-limit message did not prove
  absence of contact. Contact is not equivalent to retention during lifting.
- The operator observed pitch along the fingers (one end higher). Offline
  rectified stereo on the wood table in `20261004T214034370728Z` found a 3.63°
  difference from the tracked well-grid plane (8,620/10,302 points within 1 mm).
  `table_plane_review.json` records the estimate. `plate_grasp_geometry.py`
  applies this local correction after the 25 mm standoff. It also records the
  operator-confirmed contact motor angle, -0.14885509 rad.
- `20261004T221134303812Z`: corrected pitch, close to that angle, 5 mm commanded
  test lift. The plate moved 3.96 mm relative to the wrist camera during 4.31 mm
  model fingertip travel; secure retention was not verified. Jaws reopened.
- `20261004T221646489828Z`: up to four short tests, closure targets from the
  confirmed angle +0.04 through +0.16 rad, same 0.02 force ratio. None passed
  the retention check. The release image shows the plate skewed in the holder.
  Historical `released_verified` in this and earlier reports meant only that
  the jaws reopened; it did **not** verify plate seating or clearance.
- `20261004T222123433555Z`: the next approach found an empty holder. It stopped
  on grid acquisition before descent, returned to rest and verified motors off.
  The plate was visible off to the side in the tripod entry view. The operator
  was asked whether they moved it or it moved during the previous return.

Release now checks the plate's observed pose against its pre-grasp pose before
retreat, in addition to jaw opening. Unexpected displacement/rotation or lost
plate tracking raises a healthy-arm hold for review. Regression tests cover a
skewed plate with open jaws, displacement and invalid feedback. All 160 offline
tests pass. No secure plate pickup has yet been verified. Current proposed
next trial uses a 5.5 mm grid standoff (2.5 mm below review), corrected pitch,
the same force limit and holder-load guard; it was **not executed** because
there was no plate in the holder. Dashboard restored, motors off.

Follow-up, October 4 ~15:47 PDT:

- Operator confirmed the plate moved during the earlier return, then reseated
  it. A cleanup bug in `plate_lower.py` could still retrace after a failed
  release check. That path now propagates directly to the powered fault hold;
  regression tests cover both ValueError and RuntimeError from pickup.
- Release requires both plate-pose consistency and a bounded open-jaw
  withdrawal that visibly separates camera and plate. Jaw opening alone is
  insufficient. Hover trials now record both cameras every 0.5 seconds.
- Trial `20261004T223713038569Z` reached gripper stall at -0.17907 rad, but lost
  well-grid tracking during the 5 mm test lift. Its release image shows a
  skewed plate. Secure retention and release were not verified. The revised
  cleanup correctly held the extended arm instead of returning or dropping
  motor power. Operator supported it; the process exited and the dashboard
  subsequently confirmed motors disabled. It was physically reset.
- Operator reports the pads need to descend to holder resistance, back off
  1 mm, then close. `plate_touchdown.py` implements a maximum 8 mm search in
  0.5 mm steps after the centered 8 mm review. Motor-load change is only a
  contact indication, not calibrated fingertip force or object identification.
  Sustained resistance, observed 0.5–1.5 mm withdrawal, reduced loading and
  retained lateral alignment are required before closure. Large loads,
  lateral drift, lost vision or missing contact stop in powered hold without
  lateral recovery. These thresholds remain experimental.
- All 169 offline tests passed before the first touchdown trial
  `20261004T224651472906Z`. That hardware trial is in progress; no successful
  pickup has been established by this entry.

Touchdown trial outcome, October 4 ~15:52 PDT:

- `20261004T224651472906Z` stopped with jaws open. After the first 0.5 mm
  descent command, settled shoulder torque changed by -3.36 Nm while the
  observed pad midpoint barely moved. Sustained resistance triggered backoff,
  but the observed withdrawal did not meet the requested 1 mm clearance.
  `touchdown_replay.json` preserves an offline replay of the recorded images;
  movement lag is visible around the direction reversal. This does not
  establish whether the resistance was holder contact or drivetrain friction.
- Fault hold inhibited closure and return. Operator physically supported the
  arm again; cleanup completed and the restored dashboard confirmed motors
  disabled. The plate remained in the holder. Dashboard process is active.
- Backoff now corrects against measured camera clearance, aiming at 1 mm
  (0.75–1.25 mm acceptance), at most four corrections / 3 mm total command.
  It still requires reduced load and centered upward movement before closure.
  This revised backoff has passed offline undertravel/no-motion/overshoot
  tests but has **not yet been validated on hardware**. All 172 tests pass.
- Fault-hold diagnostics now preserve chained causes, and touchdown reports
  preserve backoff measurements before rejecting them. No secure plate pickup
  has yet been verified. Do not resume motion from an off-center fault pose.

Validation follow-up, October 4 ~16:10 PDT:

- `20261004T225954054329Z` measured 0.793 mm backoff, but stopped before
  closure because the load-release check used absolute torque difference.
  That was incorrect when gravity/friction torque reversed relative to the
  contact direction. `remaining_contact_load` now checks the signed projection
  onto the measured contact-load change; independent overload bounds remain.
  Operator supported/reset the arm and motors-off was verified.
- `20261004T230406165888Z` passed 0.878 mm backoff and reduced contact-direction
  loading. The 5 mm test lift failed retention (2.43 mm plate motion relative
  to camera). Plate displacement on release prevented return. **This run is
  still in powered fault hold, jaws open, session 89584.** A support confirmation
  has been requested and not yet received. Do not interrupt the owner, remove
  power, or start another hardware owner until supported recovery completes.
- Torque alone triggered contact while downward motion was still following
  commands. Touchdown now additionally requires two consecutive observed
  descent shortfalls (<0.25 mm during each 0.5 mm command). This refinement is
  not yet hardware validated. Unit coverage includes friction without an
  observed obstruction.
- User supplied `operator_side_view.jpg` in the latter run directory. It shows
  longitudinal finger tilt despite the model's 0.17 degree target residual.
  The earlier parallelism review is explicitly rejected in `parallel_review.json`.
  No further contact trial should precede a corrected physical finger alignment.
  Preliminary offline stereo edge-direction estimates in
  `outputs/contact_camera/20261004T214034370728Z/finger_axis_review.json`
  disagree across edges; they are marked NOT VALIDATED and have not changed
  the orientation calibration. Secure pickup remains unverified.

Automatic recovery validation, October 4 ~16:36 PDT:

- The supported recovery of `20261004T230406165888Z` completed; the old active
  hold note above is historical. `plate_recovery.py` now handles explicitly
  typed task faults with the existing motor owner. It verifies open jaws,
  both fresh cameras, bounded vertical motion and visible plate separation,
  then reaches 30 mm observed clearance before joining the saved 25 mm anchor
  and returning along the checked route. Motor/feedback faults are not
  converted into task-recovery requests.
- `20261004T232008105070Z`: injected task fault at 25 mm. Observed 7.09 mm
  upward withdrawal, automatic return to rest, motor disable verified.
- `20261004T232314260214Z`: closer 18 mm rehearsal stopped when the old
  symmetric descent-load check rejected a normal +4.17 Nm shoulder change
  during upward reversal. Operator supported recovery. This exposed the
  need for a resumable powered hold and direction-aware withdrawal limits.
- Withdrawal from an observed >=15 mm clearance now permits the recorded
  lifting direction's shoulder/elbow load change up to 6 Nm while retaining
  other limits, visibility and separation checks. Near-contact withdrawal
  retains the earlier load bounds. This is a local experimental guard, not
  calibrated tool-force sensing.
- `PlateArm` fault hold can now process `recovery_request.json` containing
  `{"action":"recover_to_rest","request_id":"unique-id"}`. It does not
  restart the process, change motor settings, or drop holding torque. It
  resumes the preserved plate tracker and validates the saved recovery route.
  At most two distinct requests are attempted per hold; duplicate requests
  are ignored. Failed partial recovery holds the NEW measured pose, not the
  original one. Malformed/non-object request files cannot terminate the hold.
  This interface takes no arbitrary joint targets or executable code.
- `20261004T233141970460Z`: deliberately injected a failure AFTER the first
  upward recovery step from the 18 mm approach. The motor owner entered hold;
  a recovery request then reacquired vision, withdrew clear and returned all
  the way to rest without operator support or power interruption. Report flags
  `recovered_from_powered_hold`, `returned_to_rest` and
  `motors_disabled_verified` are all true; maximum final joint error 0.012°.
  This validates the fallback after a recovery failure, not just normal return.
- 187 full-suite tests passed before the successful hardware rehearsal;
  additional focused tests cover status/reporting and malformed requests.
  Final dashboard restored at http://127.0.0.1:8765 with motors disabled.
  Physical finger pitch and secure plate pickup are still NOT validated.

Noncontact reproduction commands:

```bash
.venv/bin/python plate_survey.py --execute --hover --recovery-test
.venv/bin/python plate_survey.py --execute --hover --recovery-test --recovery-height-mm 18 --recovery-inject-failure
```

The second command intentionally enters powered hold. Inspect its fresh views
and fault/recovery reports, then submit the checked return request above to that
run directory. Never write `supported_remove_power.json` without actual physical
support confirmation. Keep one hardware owner at a time.

October 5 plate continuation:
- `20261005T161455329882Z`: open-jaw 18 mm alignment review, stereo capture,
  automatic return and motors-off verification. Fresh stereo showed that the
  old fixed table-normal correction did not hold across viewpoints.
- `plate_surface.py` now tracks wood landmarks from an independently measured
  stereo table plane (`calibration/plate_surface_20261005.json`). The transparent
  well grid still locates the plate; it no longer supplies the approach normal.
  Periodic fixed-reference reacquisition prevents permanent landmark attrition.
- Open-finger longitudinal direction comes from the visible outer rubber/white
  seams in `outputs/contact_camera/20261005T161611058175Z/finger_axis_review_v2.json`.
  Inner silhouettes were partly occluded by the plate and were rejected.
- `20261005T162429978416Z`: independent stereo validation after live table-based
  alignment measured 0.1–0.9 degrees of longitudinal slope across four nearby
  wood-plane regions. Returned to rest, motors disabled. This bounds local pitch;
  it is not a claim of exact parallelism or secure plate retention.
- `20261005T162757290959Z`: resistance plus blocked descent detected, but a table
  tracking rejection during backoff prevented closure. Same-process recovery
  verified open-jaw separation, reached clearance, and returned to rest with
  motors off. No manual support was needed. The rejected live frame was not
  retained in the 2 Hz recording; replay alone cannot establish its exact cause.
- Operator changed the contact policy: contact completes the approach. Pickup
  now holds the measured contact pose and closes directly; there is no 1 mm
  backoff. Load limits and sustained resistance plus observed descent blockage
  remain required. Closing alone is not a verified pickup.
- `20261005T163731351872Z`: grip-at-contact produced a verified short lift
  (0.29 mm plate motion relative to the wrist camera), then the old release
  check flagged 2.49 mm. Independent wood registration showed only 0.63 mm
  plate displacement; camera movement caused most of the old estimate. Open
  separation and automatic return succeeded. `plate_in_surface_frame` now
  removes camera motion before testing the original 2 mm / 3 degree limits.
- `20261005T164732289899Z`: first complete live cycle with corrected release
  verification, at operator-requested 2x speed. Lift following, jaw opening,
  release, and withdrawal all passed: 0.26 mm wrist-relative plate change,
  0.093 mm table-relative placement change, 0.16 degree orientation change,
  and 2.94 mm observed separation. These are local camera estimates, not
  independently certified robot accuracy. Returned to rest; motors off verified.
- Plate runs now default to 2x commanded motion speed: transit 32 deg/s,
  local approach 16 deg/s, recovery 10 deg/s, gripper ramp 0.08 rad/s and
  velocity limit 0.10 rad/s. Acceleration scales by 4; short trajectory minimum
  duration scales by 1/2. Original tracking, load, travel, and watchdog guards
  remain. Observation/settling time is not halved.
- Validated pickup runs proceed directly from fresh pregrasp checks to contact
  and grip. Use `--review` to request the former file-based review pause,
  `--alignment-review` for noncontact stereo, or `--speed-scale 1` for old speed.
  Default repeat: `.venv/bin/python plate_survey.py --execute --hover --pickup`.
  No full-height transfer has yet been validated; these are short test lifts.
- `20261005T165356668614Z`: second consecutive complete pickup/replacement at
  2x motion speed, this time without the review pause. Active powered cycle
  144.8 seconds; lift following, jaw opening, release and separation passed.
  Measured placement change 0.090 mm, separation 3.10 mm. Returned to rest and
  verified all motors disabled. The previous faster cycle also passed.
  Replay: `outputs/plate_hover/20261005T164732289899Z/pickup_return_replay.gif`.
  Software validation: 53 relevant tests passed after speed and table-frame
  release changes; 3 cleanup/automatic-entry checks passed after removing the
  review pause. No higher lift or lateral plate transfer has been tested.

October 5 faster-cycle follow-up:
- `--speed-scale 4` raises approach speed to 32 deg/s and transit to the
  existing 48 deg/s ceiling (previous transit was 32 deg/s). Acceleration is
  capped at 288 deg/s². It does not halve settling or observation time.
- `20261005T170200835915Z`: stopped before gripping because blocked descent
  produced less load change than the original contact threshold. Powered
  recovery returned to rest on its second attempt. Contact now also accepts
  three consecutive blocked descent samples with sustained load score >=0.5;
  zero-load blockage, excess load and lateral drift still stop the approach.
- `20261005T170934631260Z`: contact and short lift passed; the plate returned
  to the placement pose, but faster jaw opening triggered the gripper guard.
  The offending feedback sample was not recorded, so the exact guard input
  is unknown. Manual jaw opening allowed withdrawal; grid tracking then
  failed at the recorded return anchor, exhausting two recovery attempts.
  The operator supported the arm for shutdown. Dashboard subsequently
  confirmed motors disabled. This was NOT a successful complete 4x cycle.
- Gripper speed is now capped at the demonstrated 2x profile even when arm
  speed scale is 4. Guard diagnostics now save position, velocity, torque,
  status and temperature before rejecting the sample.
- Recovery can finish partial jaw opening only with a healthy disabled
  gripper, the recorded placement joint pose, fresh cameras and verified
  table-relative plate placement. It opens gradually, then disables/restores
  the gripper. Faulted motors are not reset. At the recorded clear anchor,
  open jaws and matching joint readback permit the checked return prefix
  without grid reacquisition. Both camera streams remain required.
  These changes passed 57 plate software tests; hardware validation is pending.
- Timing in the successful 144.8-second cycle: about 75.7 seconds marked
  moving and 69.1 seconds holding/settling/checking. Motor-loop median was
  49.6 ms; six sequential 5 ms feedback waits plus a 5 ms loop wait account
  for 35 ms. Tracking lag frequently reaches the 1.8-degree pacing threshold.
  Raising transit from 32 to 48 deg/s reduced entry-to-hover time only from
  20.7 to 19.3 seconds in the first faster trial. Cameras run at 10 fps wrist
  and 5 fps tripod. CPU saturation has not been profiled. Likely next gains
  are feedback scheduling and measured settling, not merely higher speed caps.

Lower-wall holder and 20 mm lift (October 5):
- Operator replaced the holder with a lower-wall design. In
  `20261005T173024832559Z`, the old well tracker rejected acquisition; no
  contact occurred and the arm returned to rest with motors off. Several
  optical-flow matches had shifted by a well row. The tracker now rejects
  these through undistorted planar consensus before PnP, retaining the 80%
  consensus and 2 px correspondence bounds. The recorded new-holder image
  gives 36 accepted wells and 0.09 px reprojection RMS.
- `20261005T173437502929Z`: contact at the new holder and short lift passed.
  Release stopped at measured gripper velocity -0.359 rad/s (guard 0.35),
  with low torque and normal temperatures. Automatic reopening could not
  verify the tabletop; the operator opened the disabled jaws. Same-owner
  withdrawal and return then completed; all motors were verified off.
- Operator requests approximately 20 mm for future lift-and-replace cycles.
  Pickup now targets 19–23 mm modeled vertical rise using joint readback,
  in 2.5 mm increments with a 30 mm command budget. Plate following still
  requires <2 mm wrist-relative plate change. Height is a kinematic estimate,
  not independently certified metrology.
- `20261005T174326211366Z`: scale-4 alignment stopped on a 4.725 Nm shoulder
  load change before gripping. Baseline was already settled; the change
  followed a small alignment reversal. Returned to rest, motors off.
- `20261005T174631735971Z`: scale-2 trial passed the 20 mm lift check:
  19.73 mm modeled vertical rise, 25 mm accumulated commands, and 0.402 mm
  wrist-relative plate change. The plate returned to placement. Ramped
  release still reached -0.359 rad/s, triggering the guard. Automatic
  recovery successfully completed jaw opening, but its first withdrawal
  stopped on load and the second lost grid tracking near the return anchor.
  Operator support was required for shutdown. Dashboard verified motors off.
  This validates the local lift check, NOT a complete unassisted cycle.
- Release now uses a 0.04 rad/s target ramp and 0.05 rad/s requested velocity;
  closure retains its demonstrated scale-2 profile. Velocity, load and
  temperature guards are unchanged. Gradual opening alone at the faster
  requested velocity did not eliminate the measured velocity spike.
- Recovery now switches from grid tracking to fresh-camera checks after
  observed separation and 30 mm clearance, verifies the recorded anchor by
  joint readback and open-jaw position, and retraces the checked route. This
  prevents disappearance of wells at that anchor from consuming another
  recovery attempt. 61 plate tests pass, including this regression. These
  latest release and return changes still need hardware validation.

Recovery validation and progress dashboard:
- `20261005T175841476303Z`: live noncontact recovery from the 25 mm standoff
  passed. After observed upward separation and >30 mm clearance, the test
  deliberately stopped the well tracker. The arm verified its recorded
  standoff, retraced to rest, and verified motors disabled without assistance.
  This validates loss of well tracking after clearance, not every recovery
  case or loss of the camera streams themselves.
- `20261005T185123742274Z`: the closer 18 mm test did not acquire the well
  grid in the changed-lighting view. It aborted before descent/contact and
  automatically returned to rest with motors off. The 18 mm recovery case
  remains unvalidated with the latest changes.
- Read-only experiment history: `python3 progress_dashboard.py`, then open
  http://127.0.0.1:8766. It reads saved run reports and timestamped camera
  frames, never motor/camera hardware. It stays available during standalone
  experiments. The control dashboard remains on port 8765.
  Run selection and stage buttons show archived approach/lift/recovery views;
  the wrist image is rotated 180 degrees for upright display. Milestones
  distinguish verified local lift, fault-injected return, and complete cycle.
  Four outcome-classification tests pass; the page was rendered and visually
  inspected in headless Chrome. Reports are calibration/test records, not
  learned model weights. A complete unassisted 20 mm cycle is still pending.

Variable-lighting tracking and repeated-cycle validation (October 5):
- Well and wood trackers now normalize local contrast in both reference and
  current frames. Planar correspondence consensus rejects well-row aliases
  and inconsistent wood matches before pose estimation. A rejected frame
  no longer permanently kills the observer: it retries while preserving the
  original timestamp of the last accepted synchronized plate/table pose.
  The 500 ms freshness bound remains in effect.
- `calibration/plate_surface_dense_20261005.json` adds 1,000 reference corners
  in verified wood-only regions of the original calibrated image. Their
  metric locations are ray intersections with the original stereo-measured
  plane, independent of arm kinematics and the transparent plate. The table
  tracker requires at least 30 consistent landmarks, 80% inliers, <=0.8 px
  RMS, and estimated local normal uncertainty <=0.2 degrees under a minimum
  0.5 px noise model. That uncertainty is a local model, not certified accuracy.
- Two recorded shadow-affected six-frame sequences now track throughout;
  synthetic dimming, brightening, and a shadow gradient also pass. All 68
  plate tests pass. The earlier live lighting stops at
  `20261005T185718424449Z` and `20261005T190011333297Z` both returned to rest
  with motors off before contact.
- `20261005T190906767335Z` and `20261005T191241457230Z`: complete direct-entry
  cycles passed without intervention. Estimated lifts were 19.96 and
  19.47 mm; placement shifts were 0.37 and 0.35 mm. Release, jaw opening,
  return to rest, and motor shutdown were verified in both runs.
- `--approach direct|left|right` varies the clear overhead entry, converging
  to the same visual alignment pose before descent. Offset entries pass
  through a point 20 mm to either side and 20 mm higher than the final hover.
  This does not vary final grasp yaw or tilt. The first left trial
  (`20261005T191616559968Z`) rejected smoothing across the intermediate turn
  and returned before approaching the plate. The route now settles at that
  turn; every forward/reverse blend is preflighted before torque is enabled.
  All four blend tests pass, including all three approaches in both directions.
- `20261005T191803928385Z`: left approach acquired the plate and wood pose,
  but a -4.089 Nm shoulder-load change stopped touchdown before closure.
  First withdrawal also tripped the +4 Nm relative limit. A second powered
  recovery succeeded; rest and motor shutdown were verified without assistance.
- Contact search and initial recovery now cap speed at 4/5 deg/s respectively,
  acceleration at 36 deg/s², and minimum move duration at 0.25 s. This retains
  all load bounds and leaves overhead travel unchanged. The 68 plate tests
  still pass, including restoration of the previous recovery motion profile.
- `20261005T192204983772Z`: lighting tracking remained valid, but contact was
  not confirmed within the 8 mm search. Slower withdrawal still repeatedly
  exceeded the shoulder's relative +4 Nm bound (4.13–4.23 Nm), exhausting
  recovery. At this writing the process remains in powered hold with open
  jaws, awaiting physical support for shutdown. This is NOT a passed cycle;
  reduced acceleration did not resolve the load/recovery problem. Repeated
  left/right validation remains incomplete. Do not treat the two direct
  passes as evidence of general approach or recovery reliability.
- Operator subsequently confirmed physical support; the owner disabled the
  motors and exited. The restored control dashboard independently verified
  `powered=false`, no fault, and fresh follower/camera feedback. This later
  verification is saved separately as `shutdown_verification.json`; the
  dashboard labels it a supported shutdown, never a successful full cycle.
  Batch results are recorded in `outputs/plate_validation_20261005.json`.

Recovery and offset-approach refinements (October 5, continued):
- Near-holder upward withdrawal can now tolerate up to +6 Nm relative
  shoulder load only with verified open jaws and a camera-verified stationary
  plate relative to the wood reference saved at the 25 mm standoff. Plate
  movement >2 mm or >3 degrees rejects recovery. Absolute shoulder load is
  capped at 13.5 Nm in this near-holder branch; other near-holder load bounds,
  the 5 mm separation probe, vertical corridor, and 30 mm clearance requirement
  remain. Successful prior cycles reached 12.53 Nm absolute shoulder load;
  failed reversals peaked below 12.28 Nm. This is a bounded motor-load policy,
  not calibrated fingertip force sensing.
- `--recovery-height-mm 8` adds a closer open-jaw rehearsal. The first attempt
  (`20261005T193148950781Z`) exhausted alignment steps before reaching that
  height and returned to rest/off. Lateral correction gain was reduced from
  3 to 1.5 after repeated alternating corrections; low-height recovery tests
  now receive the same 90-step budget as pickup alignment.
- `20261005T193419653224Z`: the 8 mm left-entry rehearsal passed, withdrew to
  >30 mm, verified the stationary plate, returned to rest and verified motors
  off without assistance. This validates near-holder withdrawal, not pickup.
- Pickup touchdown allows up to 10 mm summed commands in 0.5 mm steps,
  retaining the original <=9 mm observed travel, minimum height, lateral,
  sustained-load and blocked-motion requirements. This provides additional
  observations when the lower holder is reached late in the search; zero
  load and freely moving fingers still cannot count as contact. A touchdown
  load guard now dispatches checked open-jaw recovery instead of requiring a
  file request from a powered hold. Motor/feedback failures retain their
  existing behavior. All 72 plate tests pass; full pickup validation follows.
- `20261005T193702193441Z`: complete left-entry 20 mm cycle passed, with
  19.39 mm estimated rise, 0.28 mm wrist-relative plate movement, 0.66 mm
  placement shift, verified jaw opening/separation, return to rest, and motors
  off. Touchdown confirmed after 8.5 mm summed commands, one step beyond the
  previous budget, with five blocked steps and sustained load.
- `20261005T194021551333Z`: the repeat stopped before gripping at a shoulder
  change of -4.007 Nm / elbow +3.2 Nm. Automatic open-jaw withdrawal succeeded
  from ~1.9 mm minimum observed height. During recovery shoulder load rose
  5.63 Nm relative to its start; plate displacement stayed within 1.13 mm.
  The arm returned to rest and verified motors off without a hold request or
  manual assistance. This exercised the new near-holder withdrawal policy.
- Touchdown now uses a separate directional guard when independent table
  tracking verifies the plate remains seated: shoulder decrease <=6 Nm,
  elbow increase <=6 Nm, original limits in all other directions, and a
  13.5 Nm absolute shoulder cap. Alignment retains the original symmetric
  guard. Contact still requires blocked motion plus sustained load; this
  allowance is not itself a contact detection. All 74 plate tests pass.
- `20261005T194924981632Z`: contact reached three blocked steps and sustained
  load, but table tracking expired during confirmation. No jaws closed.
  Same-owner recovery reacquired vision, withdrew, returned to rest and
  verified motor shutdown. No physical support was required.
- Contact observation now permits a <=2 s stationary reacquisition pause:
  freeze measured joint targets, maintain feedback/torque, enforce <=0.3 deg
  movement (0.8 deg roll), load-change bounds and the shoulder absolute cap.
  Only a newly accepted observation satisfying the unchanged 500 ms freshness
  bound can resume the contact check. Expiry requests recovery; motor faults
  propagate immediately. Four pause tests bring the plate suite to 78 tests.
- `20261005T195412343726Z`: second complete left cycle passed (20.09 mm
  estimated lift, 0.37 mm wrist-relative change, 0.40 mm placement shift),
  with release, rest and shutdown verified. No vision pause was needed.
- `20261005T195731101233Z`: right trial stopped before arm enable because jaw
  position had drifted to -1.099965 rad, outside the measured open interval.
  Release formerly stopped within 0.015 rad of each run's initial position,
  accumulating error across cycles. Release now reaches its full opening
  target and stays within 0.004 rad after settling. A bounded automatic jaw
  correction opens to -1.17 rad only with the arm disabled at rest, fresh
  cameras, valid feedback and initial position in (-1.22,-1.07) rad. No other
  arm motor is enabled. Four reset tests bring the plate suite to 82 tests.
- `20261005T195925074787Z`: jaw reset succeeded at -1.1685 rad; right-entry
  grasp/lift passed (21.05 mm estimated rise, 0.51 mm wrist-relative change).
  Placement shift was 0.27 mm and jaws opened precisely, but the first 5 mm
  commanded withdrawal did not show 2 mm separation. Automatic recovery
  verified separation and >30 mm clearance with plate movement <=0.91 mm,
  then returned to rest/off. This was a recovered placement, not a clean
  full-cycle pass under the existing dashboard criterion.
- Normal release withdrawal now permits one additional 2.5 mm command
  (7.5 mm total maximum), requiring the plate to stay seated and >=2 mm
  observed separation before retreat. Insufficient separation still dispatches
  recovery. The full 82-test plate suite passes after this change.

Smoother returns requested by the operator:
- Above the recorded 25 mm standoff, normal and recovery returns now use
  continuous, shape-preserving blends through the recorded route instead of
  calling stop-and-settle moves at every waypoint. The time profile has zero
  speed and acceleration at each blended segment's endpoints. A turn that
  cannot meet the existing 0.2 degree curve-deviation bound is partitioned;
  its corner is retained. Paths and endpoints are not skipped.
- The local withdrawal below the standoff and the contact/release observation
  pauses remain. Motion limits, tracking margin, route envelope and joint
  bounds are unchanged. Blend planning ticks the motor owner between checks.
- Offline replay converted a recorded 20-point left return into one segment;
  a sharper right return required five bounded segments. Six blend and six
  route tests pass along with all 82 plate tests. Live validation follows.
- `20261005T200740441635Z`: live 25 mm open-jaw rehearsal passed. The clear
  return used six blended segments for 19 recorded waypoints, with maximum
  curve deviation 0.138 degrees. Rest and motor shutdown were verified. The
  operator explicitly reported the return was "Noticeably smoother."
- The pickup handoff now uses the same 1 mm / 1 degree alignment criterion
  as final grasp alignment, avoiding a needless extra corrective move after
  the grasp load baseline is captured. All 82 plate tests still pass.

- `20261005T201103982927Z`: full right-entry cycle with the smoother return
  passed: 20.56 mm estimated lift, 0.27 mm wrist-relative movement, 0.25 mm
  placement shift, verified release, rest and motors off. The local clear
  return took 6.66 seconds across six bounded blends.

Clear-travel speed adjustment:
- `--travel-speed-scale 2` (new default) increases only clear travel. With
  `--speed-scale 2`, overhead travel is 48 deg/s (previously 32, capped at
  the existing trajectory ceiling); visual approach to the 25 mm anchor
  and its clear return are 32 deg/s (previously 16), acceleration 288 deg/s².
- At the anchor, descent switches back to 16 deg/s and 144 deg/s². Contact,
  initial recovery withdrawal, gripping, lift and placement retain their
  previous profiles. Retrace below the anchor remains slow before the clear
  travel profile is restored. `--travel-speed-scale 1` selects prior speeds.
- Existing tracking, measured velocity and motor-command limits remain.
  Camera observations and settling still take time, so these settings do
  not imply halving total cycle duration. All 84 plate tests and six blend
  tests pass before hardware validation.

- `20261005T201735449489Z`: faster-cap 25 mm open-jaw rehearsal passed,
  rest and shutdown verified. Total powered time 97.52 s versus the prior
  96.37 s: higher caps alone did not improve total time on these different
  visual correction paths.
- `20261005T201931865227Z`: faster-cap full right cycle passed: estimated
  lift 19.76 mm, release, return and shutdown verified. Total powered time
  177.56 s versus 177.55 s previously. This completes two clean cycles per
  direct/left/right entry across development revisions, not six consecutive
  trials of a single fixed controller.
- Logs identify tracking-paced travel as the main speed constraint: median
  loop interval 50 ms, 90th-percentile moving shoulder lag 1.96 degrees,
  measured shoulder/elbow peaks below 8 deg/s despite the 48 deg/s cap.
  The clear-travel pacing threshold is now 2.4 degrees instead of 1.8,
  while the existing 3 degree error cutoff and 2.5 degree proposed-target
  margin remain unchanged. Below the 25 mm anchor and during initial recovery,
  pacing remains 1.8 degrees. New live validation follows. A separately
  evaluated tighter blend acceleration bound was not deployed.

- `20261005T202425970529Z`: revised-pacing open-jaw rehearsal passed,
  returned to rest and verified motors off. Powered time 89.50 s (previous
  baseline 96.37 s); overhead entry-to-hover 16.81 s (previously 21.02 s).
  Peak logged tracking error 2.482 degrees, under the unchanged 3 degree
  cutoff. This is a measured improvement, not a demonstrated 2x speedup.

- `20261005T202620981270Z`: final faster-pacing full right cycle passed,
  20.57 mm estimated lift and 0.72 mm placement shift;
  jaw opening, release, return to rest and motor shutdown all verified.
  Peak tracking error 2.485 degrees. Total powered time
  184.31 s; near-plate visual correction counts varied between trials.
  Final aggregate: direct 2, left 2, right 3 clean cycles across revisions.
- Comparable clear sections improved: entry-to-hover 21.02 -> 16.71 s;
  final departure from hover through return/rest settling 30.36 -> 24.99 s.
  The full cycle was 184.31 s versus 177.55 s because local corrections and
  retreat differ between trials. Actual clear travel saved about 18–20% time.
- Control dashboard restored on port 8765; fresh wrist/tripod frames, no fault,
  motors disabled. Read-only progress dashboard remains on port 8766.


2026-10-05 one-hour validation session (20:39:16–21:39:16 UTC requested):
- New holder cycles `203916986415Z` (left), `204306756718Z` (direct),
  `204638000440Z` (right) each verified lift, placement, release, rest and
  motor shutdown. The development series reached ten complete holder cycles.
- Blended paths now check endpoint settling directly instead of executing a
  redundant minimum-duration zero-distance move. The endpoint tolerance,
  camera requirement, tracking cutoff and absolute limits are unchanged.
  Seven route tests and 87 plate tests passed before table-route work.
- `plate_training.py` records complete-cycle evidence and writes a reuse
  manifest separating hardware/mount calibration from workspace-specific
  geometry. It gates table-route work on ten verified holder cycles.
- `205454171293Z`: first empty-jaw table rehearsal suffered a watchdog fault
  at the 25 mm standoff, before any lateral table motion. A duplicated history
  scan in the powered survey path exceeded the motor service interval. All
  six drives subsequently reported communication-loss status 13. This was
  not a controlled return. Later read-only inspection found a stationary
  folded pose and all drives disabled; the plate appeared seated.
- The evidence scan is now done only before enabling motors; camera-thread
  waits in the new survey tick the existing motor owner. No watchdog limit
  was increased. `scripts/plate_status.py` supports read-only raw fault and
  camera inspection; its optional timeout-clear operation requires a stable
  folded pose, only normal/communication fault codes and low temperatures,
  and never enables motors.
- `211835804990Z`: corrected controller returned normally after the original
  well-grid tracker could not acquire. New imagery showed a shifted repeated
  grid. Whole-grid photometric registration, anchored to the visible bottom
  row, produced `outputs/plate_grid_reseed_candidate.json` (ECC 0.945,
  fitted reprojection 0.047 px over 56 wells). This is an image-registration
  candidate, not an independent recalibration of physical geometry.
- `--grid-reference PATH` permits a checked image/pose bootstrap while keeping
  existing live tracking freshness, consensus and reprojection thresholds.
  The original reference remains the default. Live noncontact validation is
  in progress; adjacent-table carry/placement has NOT yet been validated.

- The 61×61 LK window (previously 31×31) retains broader well-pattern context.
  A previously failing 16-frame recorded sequence passed all frames, retaining
  43 wells at the end; seven grid tests pass without relaxing consensus or
  reprojection thresholds. `plate_reacquire.py` produces offline local-image
  registration candidates with explicit small seed shifts and row-identity
  requirements. It does not operate motors or certify a relocated workspace.
- `212724750321Z`: empty-jaw adjacent-table rehearsal passed after live grid
  reacquisition. Sideways offset 115 mm, minimum planned full-pad clearance
  35.6 mm, 68 independent wood landmarks at the destination. Returned to its
  anchor, rest and verified motors off. The plate stayed in its holder.
- The successful survey's return refitted densely time-sampled commands,
  creating excessive spline duration. The next survey reuses the original
  checked reverse curves after full completion; partial stops retain the dense
  actually-visited retrace. This change still needs its live repeat.

- `213039160412Z`: complete left-entry holder trial with the larger-window
  tracker and checked bootstrap passed. Estimated lift 20.53 mm, placement
  shift 1.06 mm, observed separation 4.01 mm after the allowed extra withdrawal,
  autonomous rest and verified shutdown. The series now has eleven verified
  holder cycles across development revisions.
- Actual adjacent-table release/regrasp/return remains incomplete. The proven
  adjacent route is an EMPTY-JAW survey only. Do not interpret this as plate
  transfer validation. After relocation, reacquire workspace references and
  perform an empty-jaw rehearsal before reusing these motion routines; the
  existing base-coordinate target and table reference are workspace-specific.

- `213434955742Z`: final right-entry table rehearsal lost vision readiness
  partway through the lateral movement. Automatic partial retrace returned
  to the anchor, then to rest, and verified motors disabled. No manual
  recovery was requested. The optimized full-route reverse branch was not
  reached, so its hardware speed improvement remains unvalidated.
- Motion experiments ended after the requested hour. Total verified holder
  cycles: 11 (direct 3, left 4, right 4), including four new successes in
  this session. These are development-series totals across code revisions.
  Final checks: 89 plate tests, seven route tests, five dashboard tests pass.
  Adjacent table transport/placement/regrasp is not complete; lateral vision
  tracking was not repeatable enough to proceed with a carried plate.
- Final arm state: controlled return to rest and all motors disabled verified.
  Control/camera dashboard restored at 8765, read-only progress at 8766.
  Detailed session evidence: `outputs/plate_session_20261005_hour.json`.

2026-10-05 resumed adjacent-table experiments (setup confirmed unchanged):
- `214911092531Z`: empty-jaw table survey reached the 115 mm destination and
  returned autonomously to the anchor, rest and verified motor shutdown. The
  two original-curve reverse segments took 4.46 s and 0.80 s, validating the
  previously untested faster survey return. Minimum planned pad clearance
  was 35.59 mm.
- Added `--table-carry`: after the verified 20 mm lift, plan a further 20 mm
  clearance rise, carry 115 mm beside the holder and retrace before source
  placement/release. This mode does not release on the table. History gates
  remain pre-power only. Partial travel reverses its visited segment before
  the completed original segments; the pickup owner keeps servicing jaws.
- `215254487309Z`: grasp failed before any carry. The jaws stalled at -0.852
  rad, much wider than the seven recent successful stalls (-0.362 to -0.336).
  The plate did not follow the test lift (16.65 mm camera-relative movement),
  so lateral travel was inhibited. Release, rest and motors off verified.
- Early stalls now require a bounded 0.5 mm relief/reclose, at most three
  times, with the same force limit and independent seated-plate checks.
  This conditional recovery is not a mandatory backoff after touchdown.
  Ninety-five plate tests and six dashboard tests pass; new carry and relief
  hardware validation is pending. Table release/regrasp remains unimplemented.

- `215716019347Z`: verified pickup (0.17 mm plate/camera movement) and partial
  carry. The paired tracker rejected too few wood landmarks during lateral
  movement. The carried plate returned along the partial route and stayed
  secure (0.37 mm camera-relative change at source), then source release,
  rest and shutdown passed without assistance. Early-stall relief was not
  needed on this run.
- Clear carry now requires live plate retention and both fresh cameras, using
  the independently checked preflight table-clearance route. Wood tracking
  remains mandatory for contact and source placement. Optional wood failure
  never substitutes a stale transform; the observation explicitly omits it.
  Ninety-six plate tests pass.

- `220145766180Z`: stopped before closure because the pregrasp handoff
  failed its unchanged geometry check; final descent sample had only
  6.068 mm minimum tip height, near the 6 mm threshold. Returned/rested and
  motors off verified. Pregrasp now waits for three distinct accepted frames
  within two seconds; the same clearance limits apply. This change has unit
  coverage but has not been retried on hardware.
- User ended trials to relocate the arm. All four resumed runs returned to
  rest and verified motor shutdown without physical assistance. A final
  read-only motor inspection confirmed disabled drives; no motion owner was
  left running. Full lateral carry was not reached, and no table release or
  regrasp occurred. Conditional early-stall relief remains hardware-unproven.
- `calibration/workspace_revalidation_required.json` now blocks plate-hover
  execution before hardware startup. Reacquire workspace references and
  validate fresh paths at the new location before clearing this marker.
  Joint/mount calibration remains reusable only if the hardware/mounts stay
  unchanged. Detailed local evidence: `outputs/plate_session_20261005_resumed.json`.

Repository snapshot notes:
- Bulk `outputs/` recordings remain local. A small explicit set of historical
  reference images/JSON under that directory is versioned because the current
  tracking/calibration loaders require it. These references belong to the old
  workspace and do not remove the relocation block.
- Final resumed-session checks: 99 plate unit/regression tests and six
  progress-dashboard tests passed. Clear carry with optional wood tracking,
  conditional jaw relief and the revised pregrasp handoff still require live
  validation in the newly acquired workspace.
- Full repository test discovery passed: 259 tests before the GitHub push.
