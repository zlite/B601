# Short taught approach

This is a supervised local replay for the second arm, not a completed autonomous
AprilTag search or visual-servo controller. It uses the operator-taught contact
endpoint because the tag leaves view there. No camera or hand-eye calibration is
silently marked valid. No first-arm limits are used. Motor zeros are never changed.

## Teach a retreat (read-only)

```bash
uv run teach_approach.py teach
```

Keep the tag, base, camera mounting, gripper opening and motor zeros unchanged
throughout this recording and its replay. Support the disabled arm and gently
place a padded fingertip at the desired contact point. This captures a **fresh
contact endpoint and jaw opening**. It no longer requires reproducing the previous
contact angles. Hold steady for capture and wait for `RECORDING` before withdrawing.

For one continuous teaching/replay session, use:

```bash
uv run teach_approach.py teach-replay
```

After saving, keep the arm supported at the retreat endpoint. The script prints a
path preview and proceeds to the attended replay prompts, avoiding a later manual
return to those exact angles. Motors stay disabled until the REPLAY prompt is
accepted; the startup pose checks and all other motion checks still apply.

Follow the terminal prompts. When recording starts, slowly withdraw only a few
centimeters along a clear route, away from the tag. Avoid joint stops and reverse
bends; keep all links, fingers and cables clear. Press Enter to finish while holding
the retreat endpoint steady. This captures the intermediate raw motor angles,
not just the endpoints. Progress is saved only after a valid complete recording.
It rejects abrupt sample jumps above 1 degree, excursions above 10 degrees from
contact, movements too small to distinguish, and a changed gripper opening.

The capture must be done in an attended terminal. It sends only motor read queries.
For `teach`, gently rest the arm after saving. For `teach-replay`, keep it supported at the retreat endpoint until instructed. The file is `calibration/arm2_short_approach.json`;
old completed paths are archived before replacement. Ctrl+C abandons the current
recording without changing torque state.

## Preview and supervised replay

```bash
uv run teach_approach.py preview
uv run teach_approach.py replay
```

Preview does not access hardware. Replay requires manually placing the supported
arm at the taught **retreat endpoint** within 0.5 degrees per joint. It does not
move there from rest. Ensure the entire recorded route and contact target remain
unchanged. The terminal displays the concrete path and starting-pose requirements.
Type REPLAY only after reviewing them. Mode setup and enabling take place with
the arm supported; existing motor gains are not rewritten. No gripper control
commands are sent.

After enabling, the arm holds the retreat endpoint. Move supporting hands clear
and type GO. It follows the recorded retreat in reverse, capped at 0.01 rad/s,
using smooth interpolation between samples. At 80% of the recorded joint-space
arc it holds and asks for TOUCH or BACK. **This pause is not a measured physical
clearance**, and recorded joint-space interpolation is not a collision guarantee.
If there is unexpected contact, use the stop immediately, not the final prompt.

TOUCH permits the remaining taught segment at 0.005 rad/s. The arm immediately
retraces the path to the retreat endpoint after reaching the taught contact angles;
there is no additional forward travel or dwell. Reaching those angles does not
prove contact or control force. BACK (or another response at that pause) retreats
along only the portion already traversed. At the final retreat hold, support the
arm and type SUPPORT to disable and exit.

Ctrl+C, SIGTERM, a fault, a detected control-loop stall or a tracking error attempts
to disable all six arm motors. **Torque-off can let the arm fall.** Keep the hardware
cutoff accessible. Software cannot guarantee a stop after lost communication.
There is no force sensor or force limit in this demo, and no unattended operation.
Do not use it for a rigid collision, an unstable target, or a payload.

## Validation status

Offline tests cover path bounds, bad samples, start mismatch, command-envelope
rejection, partial-enable cleanup and read-only cleanup. Physical teaching/replay
has not yet been performed. The previously recorded single contact endpoint alone
does not define a safe approach path.

New version-2 path files contain their own contact endpoint; older contact captures
are preserved. Tests exercise fresh-contact teaching without any previous contact
file and reject disagreement between the embedded contact and path endpoint.
