# Pickup and wall recovery: 1 October 2026

This update addresses pushing a reachable stone instead of closing the jaws,
turning into a nearby stone, and recovery blocked by detections of the robot itself.
The gripper close angle is **70 degrees**, in the planner, simulator, firmware
source and firmware-check tool. Opening remains 0 degrees.

## 1 October: crowded floor-pile additions

`vision.py` now attempts to split a connected same-colour region only when its
distance-transform peaks have a clear saddle and the resulting basins fit the
existing area/extent limits. It reports every passing jaw-entry heading; a blob
with no defensible split or clear 60×80 mm jaw lane remains unpickable.

The planner ranks those candidates using confidence, number of passing headings,
route distance and recent failed attempts. `pile_navigation.py` routes around
observed bounding circles inflated by the robot's full circumscribed footprint
(shifted by the configured axle offset) and a 20 mm pile-planning margin. It
applies this route to staging and carry motion. If no path exists inside the
conservative field bounds, the planner stops
with a route reason. This circle uses the provisional 235 mm front and 75 mm side
footprint values, so it can reject a passage the physical robot might fit through.

After the configured close servo position is reported, `PICK_RETREAT` reverses
on the entry heading in camera-measured pulses: 120 mm minimum, up to 320 mm
while checking whether a full turn is clear. The robot does not turn or carry if
the return path or turning sweep is blocked. The gripper has no force sensor, so
the commanded 70-degree angle still cannot prove that a stone is held.

These additions use the current configuration values only as provisional inputs:
the recorded `[197, -4]` mm grip offset, 235 mm front footprint and 60×80 mm jaw
lane still need measurement on the assembled robot. The 20 mm default obstacle
radius is the simulator's nominal radius (`STONE_R=20`), not a physical stone
measurement. The 20 mm watershed seed spacing is likewise derived from that
simulator model. The run used one overhead camera and has no wheel encoders or
gyro, so all retreat distance comes from calibrated camera pose.

## What the supplied recordings show

| Evidence | Finding | Change |
| --- | --- | --- |
| `20260930-062142-447604`, frames 1230–1280, stationary held sky-blue stone | Its center is approximately `[197, -4]` mm from the tag. The run used `[135, 0]`; that point sits too far back, so driving it to the stone pushes the jaws past the stone. | Provisional grip offset `[197, -4]`; measure it on the actual gripper before driving. |
| `20260930-062920-757c74`, approach around frames 760–777 | The robot pushes a nearby lime stone; approach/backoff transitions include lateral errors. | Stop before contact, measure in the current jaw frame, and use short straight approach pulses. Retreat before a large correction close to a stone. |
| `20260930-062142-447604`, frames 1227 and 1288 | Roof/body/gripper detections are reported as unknown obstacles and block retreat. | Mask the projected robot volume, including roof parallax and fingers. Allow retreat away from an object already touching the collision padding, while rejecting paths that move closer to it. |
| Same run, `reference_moved` first reported at frame 1341 | Scoring-zone positions shift in the image. A visible AprilTag alone does not make the old floor calibration valid. | Keep the stop and show an explicit instruction to recalibrate corners/background. |

The offset estimate uses 51 stationary frames before the reference shift. Its
median is `[196.92, -4.12]` mm, with a 90th-percentile deviation below 1 mm on
each axis. These are repeatability figures within one camera view, **not** a
measurement of absolute accuracy. The new local jaw detector recognizes the
held blue stone in 50 of those 51 frames.

## How pickup now works

An already-reachable known-color stone takes priority over driving to a staging
point, parking or recovering at the wall when the robot is empty. The planner
enters `CAPTURE`, commands zero wheel speed, waits for camera delay/coasting,
and checks that the camera shows the robot at rest. It then rechecks the stone
position before closing. It does not close based on an extrapolated pose.

The initial capture window extends 35 mm behind and 12 mm ahead of the measured
held-stone center. The supplied configuration keeps the lateral tolerance at
11 mm. The green rectangle in the rectified camera view shows this window.
Confirm that the whole window lies between the real open jaws; the actual
clear jaw width and the stone size determine the useful lateral tolerance.

Far from the stone, approach can drive continuously with bounded steering.
Near contact, it brakes according to observed motion, then uses 0.04–0.20 s
straight pulses with a stop/measure interval between pulses. The sender stops
a pulse at its deadline even if camera processing stalls. Pulse displacement
updates the approach speed estimate. A close, off-center stone triggers a
checked retreat before another alignment turn; a blocked retreat stays stopped.

Local color detection inside the jaws bypasses the robot mask and the wall's
target exclusion margin. These detections remain separate from navigation
targets and obstacles, so a held stone does not block recovery. Static scoring
zone/fixture exclusions, ambiguous-color rejection, valid vision, fresh tag
feedback and firmware state checks still apply. This does not authorize driving
into a wall to collect an unreachable stone in a pile.

If firmware reports a servo angle that never reaches the configured 70-degree
close position, `GRIP_BLOCKED` stops the mission instead of assuming a successful
grab after a timeout. Check the firmware/configuration and restart the run.
The reported servo angle is commanded position; there is no grasp-force sensor.

## Apply and check on the robot

1. Use the updated project together. The root configuration retains the recorded
   `min_duty: 0.65`, `cruise: 0.3` and `approach_max_side_mm: 11` settings. Copying
   Python files does not update the ESP32; its `grip close` command must target 70
   degrees too. The supplied run logs already report 70, and the firmware source
   now matches that setting for future uploads.
2. Fix the camera and field in place. On an empty field, recapture corners and
   background, then identify/check all six zone labels:

   ```bash
   python calibrate_arena.py 1
   python find_zones.py
   ```

3. Place one known-color stone in the jaws, closed at 70 degrees, arm down and
   stone at floor level. For a sky-blue stone (color 5), measure while stationary:

   ```bash
   python calibrate_grip.py 1 --color 5
   python calibrate_grip.py 1 --color 5 --write
   ```

   Repeat at another position/heading; the offsets should agree within about
   5 mm. `--color` prevents another colored part/stone being chosen; the tool
   refuses to save when its 90th-percentile spread exceeds 10 mm. The supplied
   `[197, -4]` offset is marked `grip_calibrated: false` until measurement.

   Measure `robot_tag.footprint_mm` from the tag center to the outermost body and
   open fingers. The supplied front extent, **235 mm**, is a provisional
   conservative estimate. Do not confuse this extent with the held-stone center.
   Confirm the rear/side extents too. The separate image-mask padding defaults
   to 20 mm (`vision.robot_mask_padding_mm`).
4. Preview without connecting to the robot:

   ```bash
   python autonomy.py --dry-run --camera 1 --show-full-frame
   ```

   Check the magenta mask covers the robot, the green capture rectangle lies
   inside its jaws, and a stone at the jaw base appears as a local jaw detection.
5. For an operator-controlled field trial, start with one stone and record it:

   ```bash
   python autonomy.py <ESP_IP> --camera 1 --record --show-full-frame
   ```

   Confirm `CAPTURE` stops before closing, 70 degrees is reported, and a wall
   retreat moves away from the nearby pile. New traces include `jaw_observations`,
   `pickup_command_until`, and `vision_diagnostics` alongside the wall reasons.
   If `vision_reference_moved` appears, stop and repeat field calibration;
   recovery cannot use the shifted reference. A missing tag also stops motion.

## Software validation

For the 30 September pickup/wall changes, **189 tests passed** with
`python -m unittest discover -s tests`. Those results predate the 1 October pile
additions; no automated tests or build were run for those changes.
Regressions cover capture at the jaw
base, coasting, moved stones, mismatched servo angles, pulse expiry, blocked
retreats, mask/exclusion interactions and recorded wall-obstacle positions.
Existing clean/noisy sorting simulations retain their completion and wrong-zone
checks. The small fixture contains positions from the supplied recording; raw
videos and run logs are not included in the repository.

The video check exercises jaw detection and geometry, not a full autonomous
replay: the recordings do not include their background image, and a recording
cannot show how the robot would respond to different commands. Simulation and
tests do not validate gripper mechanics, tire slip, battery-dependent coasting,
or the provisional measurements. No physical robot trial or firmware upload
was performed for this change.
