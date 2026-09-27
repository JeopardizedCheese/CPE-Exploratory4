# Review and implementation plan

## Rules extracted from First Project Details.pdf

Source: user-provided PDF, pages 2-6. The fictional story on page 1, including
the rainbow stone, is not an additional tested color class.

| Requirement | Consequence |
| --- | --- |
| Fully assembled robot no larger than A4, 210 x 297 mm | Check CAD footprint including mechanism; PDF gives no separate numerical height limit. |
| Provided two-wheel/two-motor base and ESP32 main controller | ESP32 owns motor/servo actuation, the run timer and safety stops; selection and planning run on the PC. |
| Optional HuskyLens on request | Not used. Color is confirmed from the overhead camera before pickup; uncertain stones are skipped. |
| Student-designed laser-cut acrylic and printed plastic parts; two servos provided | Design a simple intake and retention/release mechanism within the footprint. |
| Stones: 50 x 35 x 35 mm; two shapes 37 x 25 x 25 mm | Tune gripper dimensions and vision size checks on actual orientations. |
| Approximately 2100 x 1200 mm arena; supplied overhead USB camera | Calibrate actual floor dimensions and camera geometry. |
| 54 stones, nine each of six colors, piled centrally | Visible detections cannot be assumed to count every stone. Touching stones merge; pick from the pile edge (`pile_mode`) or spread the pile away from the zones. |
| Start anywhere in designated start zone | Select and verify robot starting pose; the photo places the zone on the right. |
| Three attempts, five minutes each | ESP32 needs a local run timer and stop state. |
| Round 1 custom gesture remote control; round 2 autonomous; round 3 choice | A keyboard/UDP camera demo does not fulfill manual mode. |
| Correct manual +1; correct autonomous +5; incorrect -1 in both | Prefer abstention and reinspection over low-confidence placement. |
| Mechanical 35%, code 35%, achievement 30% | Allocate development and validation to all three, not vision alone. |

The PDF does not explicitly approve extra arena lights, camera-mounted filters,
floor fiducials, or moving field equipment. Confirm event permission before
altering the shared setup. The scoring zones' exact geometry must be measured;
the included photographs are not calibration data.

## Original findings and implemented response

| Finding | Change |
| --- | --- |
| Four HSV classes only | Explicit missing crimson/sky-blue calibration; all-six readiness gate. |
| Each colored fragment treated as a gem | Separate reference-based silhouette from color voting; fill highlight holes. |
| White stones disappear from color masks | Unknown foreground blocks clearance even without a color label. |
| Zone paint and gems share color | Empty-field reference plus explicit exclusions. |
| Pixel sizes vary with perspective | Four-corner floor mapping and mm-based size/clearance checks. |
| Every frame immediately sent as a target | Consecutive confirmation; immediate removal on disappearance. |
| Center distance ignores stone edges/occluders | Distance from entire object silhouette to other foreground and boundaries. |
| Hardcoded camera controls differ between tools | Shared configured settings applied consistently, with rejected-setting warnings. |
| Receiver can keep obsolete observations | Versioned protocol, sequence check, stop messages and reception timeout. |
| Network credentials in source | Ignored local secrets header and committed placeholder example. |

## Lighting and top-view limitations

White highlights lose hue and saturation. Do not widen every color threshold to
include white: that would make white floor/glare match several classes. Here,
visible colored pixels classify a whole foreground component only when evidence
is adequate. A fully white object remains unknown. Software cannot reconstruct
clipped color or see the underside of an occluded stone from one top view.

Try broad diffuse illumination and reposition glare-producing light sources
where the event setup permits. Lower exposure until colored faces retain detail,
then lock exposure, focus and white balance if the actual camera supports it.
Changing exposure also changes calibration. Compare actual captured frames,
including shadows and motion, rather than assuming a property setter succeeded.
OpenCV documents hardware/backend dependencies for camera controls:
[OpenCV video properties](https://docs.opencv.org/4.12.0/d4/d15/group__videoio__flags__base.html).

If allowed, test a rotatable lens polarizer; with controllable lights, crossed
polarizers on the light and camera may reduce specular glare. Measure the loss
of usable light and retune exposure. This is an optional experiment, not an
assumption about the supplied camera or stone finish.
[Edmund Optics polarization guidance](https://www.edmundoptics.com/knowledge-center/application-notes/illumination/successful-light-polarization-techniques/).

The robot, gripper, hands, pile and walls can hide stones. Large/mixed foreground
is withheld and lost detections are not reused. However, a small visible fragment
or a same-color cluster can still resemble one stone. This pipeline has no depth
sensor or trained instance model. The next physical step is single-stone intake
and reobservation from the overhead camera before pickup, plus robot-pose-based
dynamic masking (`footprint_polygon_mm` in `robot_pose.py` provides the outline). Avoid gem
colors on the robot's visible upper surfaces. A hidden obstacle or an object
indistinguishable from the reference cannot be guaranteed detected.

A floor homography is exact only on its plane (and does not correct lens
distortion). Stone tops are 25-35 mm above the floor, so off-axis localization
has parallax error. For a level camera at height H above the floor, radial floor
projection error is approximately r*h/(H-h), for object height h and radial
distance r from the optical axis. Measure error at the corners and for all stone
heights. `robot_pose.py` applies this height correction to the roof AprilTag
(`robot_tag.height_mm`, `camera_height_mm`, `camera_floor_xy_mm`); stone positions
are not yet corrected. Add lens calibration/height compensation for stones or rely
on a final close-range alignment step before grabbing. The current 60 mm clearance is a tunable local
margin, not a navigation clearance for an A4-sized robot.

The reference method assumes a fixed camera and mostly unchanged background.
It deliberately abstains on major changes; small camera shifts or local light
patches may still yield false foreground. Check overlays, recapture only when
the arena is empty, and test under the real lighting. Do not adapt the reference
online while stones sit still, or they may be absorbed into the background.

## Remaining robot implementation sequence

| Step | Status |
| --- | --- |
| 1. Specify motor driver, ESP32 pins, encoders, servo geometry and power; measure the 210 x 297 mm envelope | Open. `config.h` holds placeholders. |
| 2. Local ESP32 stop, watchdog, five-minute timer, manual input with dead-man condition | Firmware written (`robot_ctrl`), simulated by `fake_robot.py`; not compiled or bench-tested. Keyboard teleop only; gesture input open. |
| 3. Robot pose, footprint, collision-free approach | Pose from roof AprilTag implemented and tested synthetically; tag mounting waits for the final roof. Per-stone approach direction (`approach_deg`) implemented in `pile_mode`. Path planning open. |
| 4. select -> approach -> reobserve -> single intake -> matching zone -> release -> verify | Selection helper (`target_lock.py`) implemented. Planner loop open; needs the arm and robot pose. |
| 5. Gesture control; complete timed manual/autonomous runs; choose round 3 mode | Open. |

Decisions taken since the first review:

- Planning runs on the PC in one program (vision, pose, lock, controller) and sends
  v3 commands at 20 Hz; the ESP32 does not receive targets in this design.
- The robot handles one stone per trip. Prefer short robot -> stone -> zone trips
  and skip uncertain stones rather than risk -1.
- When no edge stone is pickable, push the pile toward the empty start-zone side,
  never toward the colored zones.

These items are a proposed next implementation plan, not completed features.

## Field acceptance checklist

Record videos and ground-truth labels under bright light, shadow, glare, mixed
light and robot movement. Include all 54 physical stones, sizes and orientations.
For each class report precision/recall, unknown rate, false-target count,
localization error in mm, and end-to-end target age. Keep separate calibration
and evaluation scenes; particularly inspect cyan versus sky blue.

Check white/glared stones near a colored target, touching same/different colors,
hidden stones, gripper overlap, colored zone edges, camera vibration, and stones
near walls. Test unplugged camera, blocked camera reads, Wi-Fi loss, malformed,
duplicate and reordered UDP packets, application restart and empty target lists.
Measure watchdog behavior on ESP32; no successful bench simulation substitutes
for this. Proposed release criteria: no unsafe pickup in the challenge set,
error below the measured intake tolerance, and repeatable complete five-minute
runs. Set numeric accuracy/latency criteria after measuring the actual mechanism.

## Home test findings

A 300 x 200 mm home setup with violet, cyan and crimson stones under dim, warm
light confirmed the full vision -> UDP path: stable targets (position noise about
0.1 mm), correct labels, removal while moving, exclusion zones, stop packet.

- Crimson and violet sat only about 4-6 hue units apart; the sampler's +/-6 padding
  made them overlap and crimson became unknown. Moving the boundary by hand
  (violet `hi` 171, crimson `lo` 172) separated them. When the light dimmed further,
  crimson lost most of its votes again. Crimson is the class most sensitive to
  lighting; test it alone at several arena positions and again later in a session.
- Shadow inside the blob keeps `color_fraction` low; a higher `background_delta`
  (45) shrank blobs toward the stone size.
- Unsampled classes are mislabeled as the nearest sampled class when detection
  is allowed to run with incomplete calibration.
- Home HSV values and relaxed thresholds are not field values. The absolute scale
  was not verified with a ruler; verify it in the arena.

## Validation performed in this workspace

The synthetic suite contains 43 test cases (vision 22, pile 7, target lock and
sticky targets 8, robot pose 6). With the local `vision.py` test-time edits, two
vision cases fail as expected (see README). `fake_robot.py` was exercised with the
`teleop.py` link: start, drive, 300 ms stop, time-up, reset and stop behave as the
firmware state machine describes. Neither ESP32 sketch was compiled or flashed:
the Arduino toolchain and physical board are not available here. HSV values,
size limits, confidence thresholds, clearance and gripper dimensions require
field validation.