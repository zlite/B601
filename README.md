# B601-DM + OAK-1 hello-world

A gentle wrist wave while reading RGB images from the wrist-mounted Luxonis OAK-1.
Uses Seeed's motorbridge driver and Luxonis DepthAI v3. No recalibration or zero-position writes.

## Run

Dependencies are installed in `.venv`; `uv.lock` pins them. To recreate: `uv sync`.

The OAK currently needs its Linux USB permissions configured. Run once:

```bash
bash scripts/setup_usb.sh
```

Enter your sudo password in your terminal. Reconnect the OAK if necessary, then:

```bash
uv run hello_world.py --preview
```

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
