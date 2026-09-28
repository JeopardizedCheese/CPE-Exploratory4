# Gemstone sorting: overhead perception and robot link

**Color-first update:** start with [COLOR_FIRST.md](COLOR_FIRST.md). The
root `calib.json` is the calibration for the 2100 x 1200 mm arena and every tool
reads it by default. `color_preview.py` provides camera/image/video diagnostics
without connecting to a robot.

This improves the camera/ESP32 prototype from `CPE-Exploratory1-main.zip`.
The implemented scope is perception, camera calibration, target communication,
robot pose from an AprilTag, a manual control/recording tool, and ESP32 motor/servo
firmware with local safety stops. The ZIP also contains an autonomy loop and the
`CHROMA-Gesture-Control` subproject. Hardware-dependent tuning and field validation
are still required; software and synthetic tests do not establish a complete
working robot. This color update does not flash or test the firmware on hardware.

Robot-side setup, wiring, AprilTag and test order are described in Thai in
[README_ROBOT.md](README_ROBOT.md).

## Run

Use Python 3.12 and a virtual environment. From this project directory:

```powershell
python -m venv .venv
.venv\Scripts\python -m pip install -r requirements.txt
.venv\Scripts\python calibrate_arena.py 1
.venv\Scripts\python sample_hsv.py 1
.venv\Scripts\python find_zones.py
.venv\Scripts\python color_preview.py 1
```

On Linux/macOS use `.venv/bin/python` instead of `.venv\Scripts\python`.
All tools read the root `calib.json` unless `--config` names another file.

Use the normal GUI OpenCV package in requirements.txt for camera windows.
`detect_live.py --debug` prints one line per blob (votes, dominance, margin) for
color tuning; it is off by default because printing slows every frame.

**Speed.** Detection reads the camera in a background thread and always processes
the newest frame, so a slow frame never builds a backlog. The display shows fps
and processing time. Processing cost grows with the rectified image size: a
640 x 480 camera covers a 2.1 m field at about 3.3 mm per pixel, so
`arena.mm_per_px: 3` loses no detail and is roughly three times faster than 2.
Changing `mm_per_px` requires rerunning `calibrate_arena.py` and `find_zones.py`.
The workspace `.runtime` directory is an ignored, headless test dependency only.
Press `q` to quit detection, `m` to display foreground segmentation.

| Tool | Purpose |
| --- | --- |
| `sample_hsv.py` | Freeze frames, save labeled patches across sessions, calibrate or evaluate |
| `color_preview.py` | Color masks, rejection reasons, reference checks and saved evidence; no robot link |
| `calibrate_arena.py` | Floor corners, empty-field reference, exclusion zones |
| `camera_probe.py` | Reports an unknown camera's settings, tests which ones it accepts, measures real fps |
| `find_zones.py` | Finds the six color zones in `background.png`, writes circular exclusions with margin and zone centers |
| `detect_live.py` | Live/replayed detection, UDP v2 targets to port 4210 |
| `fake_esp32.py` | Prints v2 target packets received on localhost:4210 |
| `robot_pose.py` | Robot x, y, heading from the roof AprilTag; detection rate |
| `teleop.py` | Keyboard driving over UDP v3 (port 4211) plus run recording |
| `fake_robot.py` | Simulates the `robot_ctrl` firmware for `teleop.py` tests |
| `target_lock.py` | Planner helper: keep one chosen target despite flicker |

## Calibrate in the actual arena

1. Fix the camera, focus and resolution. Use the same lighting for sampling,
   reference capture and operation. Set tested camera properties in
   `camera_properties` (e.g. `AUTO_WB`, `AUTO_EXPOSURE`, `EXPOSURE`). Values are
   backend dependent; an accepted setting is not proof the hardware applied it.
   All tools apply the same configuration. Do not change it mid-run.
2. Calibrate the **empty field first** with `calibrate_arena.py`.
   Your latest measured size is 2100 x 1200 mm. SPACE freezes; select corners
   TL, TR, BR, BL, then Enter. The long side corresponds to the horizontal axis.
   Press `s` to save without outlining zones; label circles in step 5.
3. Run `sample_hsv.py`. SPACE freezes. Select 1 violet,
   2 cyan, 3 crimson, 4 orange, 5 skyblue, 6 lime; click real stone faces.
   Avoid glare, floor and scoring paper. Collect several sessions as described in
   [COLOR_FIRST.md](COLOR_FIRST.md). `u` undoes; `s` applies ranges; `q` quits.
4. Inspect sample-fit diagnostics for crimson/orange and cyan/skyblue. Do not
   assign ambiguous pixels by guessing a boundary. Keep a separate evaluation session.
5. Exclude the six scoring zones. Either run `find_zones.py` after the reference
   is saved (automatic circles: zone + white ring + `--margin-mm`, default 20; it
   also stores each zone's color and center in mm under `zones`; in
   `calibrate_arena.py` press `s` without drawing any zone to skip this step), or outline each
   zone by hand on the rectified view, clicking outside the white ring, pressing
   Enter per polygon and `s` to save. Anything inside an exclusion is invisible to
   detection. The reference must include the permanent markings but no stones,
   robots, cables or people.
6. Place stones, inspect the foreground with `m`, and verify coordinates against
   ruler measurements. Rectified coordinates start at the top-left floor corner:
   x right, y down, in millimeters. Recalibrate after any camera move/settings change.

`calibrate_arena.py` loads the config when it starts and rewrites the whole file
when it saves; do not edit the file by hand while it is running. It writes
`background.png` into the **same folder as the config file**.

Missing background, corner mapping, or color calibration results in
`setup_required` and no transmitted targets. Empty color ranges are never guessed.

### Checking color boundaries

`sample_hsv.py` pads measured hue percentiles by +/-4 and measures both ends of
saturation and brightness. Classes can still overlap; pixels claimed by two
classes vote for neither, so a stone can become unknown. Saving checks the labeled
sample fit. `detect_live.py --debug` prints one line per blob:

```
candidate=crimson area=2712.0 unit=mm2 size_ok=True color_fraction=0.14 dominance=0.97 margin=0.97 votes={1: 12, 2: 0, 3: 393}
```

| Field | Meaning | Default rule |
| --- | --- | --- |
| `color_fraction` | winning votes / blob area | >= 0.3 (`min_color_fraction`) |
| `dominance` | winning votes / all votes | >= 0.85 |
| `margin` | (1st - 2nd) / 1st | >= 0.65 |
| `votes` | voting pixels per class id | only saturated, bright, single-class pixels vote |

`candidate` is printed before the rules are applied. Place each stone **alone**
and read its line. If a stone gets votes for a neighboring class, inspect the
patch labels, camera settings and lighting. Collect representative labeled faces
and check both classes on a separate evaluation session. A low `color_fraction`
can come from glare, shadows, overlap or unmatched pixels; inspect the masks.

### Home test setup

For tests away from the arena, keep a separate config in its own folder so the
field `background.png` is not overwritten:

```bash
mkdir home
cp calib.json home/calib_home.json
# edit arena.size_mm (e.g. [300, 200] for a 30 x 20 cm sheet), mm_per_px, exclude_polygons: []
python calibrate_arena.py 0 --config home/calib_home.json
python sample_hsv.py 0 --config home/calib_home.json
python detect_live.py 0 127.0.0.1 --config home/calib_home.json
```

Home HSV values and relaxed thresholds do not transfer to the field.

## Values to set before a real run


| Where | Value | Set from |
| --- | --- | --- |
| `calib.json` `arena` | `size_mm`, `corners_px`, `background.png` | `calibrate_arena.py` in the arena, empty field |
| `calib.json` | `exclude_polygons` | outline the six zones in the arena |
| `calib.json` | `hsv` (all six classes) | `sample_hsv.py` under arena light, then the boundary check |
| `calib.json` | `camera_properties` | lock exposure / white balance if the camera allows |
| `calib.json` | `gem_area_mm2`, `vision.max_gem_extent_mm` | measured single stones in the arena |
| field config `vision` | `pile_mode: false`, `sticky_frames: 0` initially | enable only after separate-stone validation |
| `calib.json` `vision` | `gripper_width_mm`, `approach_length_mm`, `own_radius_mm`, `clearance_mm` | the finished gripper |
| `calib.json` `vision` | `min_color_fraction` (keep 0.3), `background_delta` (try 45 if shadows inflate blobs) | arena test |
| `calib.json` `robot_tag` | `size_mm` (120, measured print), `height_mm`, `grip_offset_mm`, `footprint_mm` | final roof and arm |
| `calib.json` `robot_tag` | `camera_height_mm`, `camera_floor_xy_mm` | measured; recheck if the camera moves |
| `vision.py` | all-six-color readiness and stone size checks | restored in this update; retain them |
| `firmware/robot_ctrl/config.h` | driver type, motor pins, `L/R_INVERT`, `MAX_DUTY`, `MIN_DUTY`, `ESTOP_PIN` | actual wiring, wheels-lifted test |
| `firmware/robot_ctrl/config.h` | servo pin, `SERVO_MIN/MAX/START_DEG`, `GRIP_*` | calibrating the gripper with the `servo` command |
| `firmware/robot_ctrl/config.h` | `RUN_TIME_MS` back to 300000 if shortened for testing | before every match |
| included firmware | `secrets.h` Wi-Fi of the team hotspot | venue network |

Values in `home/calib_home.json` are for home tests only; do not copy them.

## Implemented changes

- Foreground segmentation against the empty field groups white highlights and
  colored faces into one connected object. Unchanged floor markings disappear.
- Only sufficiently saturated, visible pixels vote for color. Overlapping HSV
  classes, mixed-color objects, and insufficient evidence become unknown (0).
  Confidence is an evidence fraction, **not a calibrated probability**.
- Unknown foreground, large objects, borders, and excluded zones block pickup
  clearance.
- Targets require four consecutive consistent detections; missing objects are
  removed immediately unless `sticky_frames` is set.
- Large overall exposure/foreground changes suppress output. Small additive
  shifts are compensated for segmentation only, not used to invent color.
- Perspective mapping uses physical-area thresholds; sampling handles red hue
  wrap, edge clicks, outliers, and exact patch undo. Modules are import-safe.
- UDP v2 caps output at eight candidates and 10 packets/second. Receiver clears
  targets on empty/stop messages, rejects reordered packets and expires silence
  after 300 ms. Embedded network credentials were replaced with a local header.

### Pile mode and sticky targets (optional, off by default)

The match starts with all stones in one pile. Touching stones merge into one
blob, and a 60 mm all-around clearance leaves almost nothing pickable. With
`vision.pile_mode` enabled, a blob that fails the normal checks gets a second test:

- **Merged blob:** split into single-color regions. Regions that look like one
  stone (area, length) are candidates. A candidate is pickable if a strip as wide
  as the gripper, leading **away from the pile center** (up to +/-67.5 deg), is
  free of foreground, excluded zones and the arena edge.
- **Single known stone with something nearby:** pickable if any direction has
  such a free strip, preferring the side away from the nearest obstacle.

Pickable stones get `approach_deg`: the heading the robot should drive **along
the strip toward the stone** (0 = +x, 90 = +y, same convention as `robot_pose.py`).
It is added to the v2 target; the ESP32 receiver ignores unknown fields.
`detect_live.py` draws it as an arrow. Buried stones and same-color pairs are
not split.

`vision.sticky_frames` keeps a stable target for up to N frames after it fails a
check, unless a different known color appears at its position.

| `vision` option | Default | Meaning |
| --- | --- | --- |
| `pile_mode` | false | Enable edge picking and single-side clearance |
| `gripper_width_mm` | 60 | Width of the free strip (outer gripper width) |
| `approach_length_mm` | 80 | Length of the free strip |
| `approach_start_mm` | 6 | Strip starts this far ahead of the stone center |
| `own_radius_mm` | 25 | Pixels within this radius count as the stone itself |
| `region_min_mm2` / `region_max_mm2` | 150 / 2000 | Voting area of one-stone regions |
| `max_gem_extent_mm` | 70 | Longest side of one-stone regions |
| `max_approach_turn_deg` | 67.5 | Allowed deviation from straight out of the pile |
| `sticky_frames` | 0 | Frames to hold a target after it drops out |
| `min_obstacle_mm2` | off | Ignore foreground blobs smaller than this (background speckles); e.g. 150 |
| `edge_margin_mm` | 0 | Ignore a strip this wide along the arena border (walls, fence, mat edge) |
| `debug_blobs` | false | One printed line per blob; set by `detect_live.py --debug` |

The gripper-related values are placeholders until the arm exists.

### Checks restored in this update

Detection requires all six configured colors before emitting targets. Stone
area and extent filters are active. The new field profile also requires agreement
with fixed scoring-circle positions; missing or displaced reference markers
withhold targets. See [VALIDATION.md](VALIDATION.md) for measured test results.

## Protocols and firmware

**v2, vision targets (`detect_live.py` -> port 4210).** Each JSON message contains
`version:2`, sender `session`, increasing `seq`, host wall-clock `t`, `ttl_ms:300`,
`units:"mm"`, `status`, and `targets`. Each target has `color`, `x`, `y`,
`confidence`, and `approach_deg` in pile mode. An empty target list revokes old
targets. v2 is intentionally incompatible with the original pixel-coordinate
`gems` protocol: update sender and receiver together.

The historical `firmware/esp_link/` receiver is not included in the supplied ZIP.
Use `fake_esp32.py` for local v2 packet inspection. The included `robot_ctrl`
firmware uses v3 robot commands, not v2 vision targets.

**v3, robot commands (`teleop.py` or a planner -> port 4211).**

```json
{"v":3, "s":"a1b2c3d4e5f6", "q":42, "c":"drive", "l":0.5, "r":0.5}
```

`s` is a 12-character session, `q` an increasing sequence. Commands: `drive`
(`l`, `r` in -1..1), `start`, `stop`, `reset`, `grip` (`p`: open/close),
`servo` (`i`, `deg`). There is no lift: the gripper only opens and closes. `firmware/robot_ctrl/` runs a local
state machine (IDLE -> RUNNING -> DONE after 5 minutes; ESTOP by button or `stop`),
stops the wheels when no `drive` arrives for 300 ms, and returns a status packet
every 200 ms to the sender. Pins and servo angles in `config.h` are placeholders.

For the included sketch, copy `secrets.example.h` to `secrets.h`, enter Wi-Fi
credentials, and compile with the ESP32 Arduino core (3.x for `robot_ctrl`) and
ArduinoJson 7. Use a controlled local network (for example your own hotspot);
networks with client isolation block PC -> ESP32 traffic.

The watchdogs measure time **since reception**, not network transit age. No
clock synchronization, authentication or end-to-end acknowledgement is provided.
Robot pose, collision-free approach and a close-range recheck must gate actual
pickup. `isolated` and `approach_deg` describe local silhouette clearance, not
proof that the robot's full route or footprint is clear.

## Tests and recorded video

```powershell
.venv\Scripts\python -m unittest discover -s tests -v
.venv\Scripts\python detect_live.py --video arena.mp4 --headless
```

Replay requires the same camera geometry/reference and never sends UDP.
`teleop.py --camera <n>` records `runs/<time>/video.mp4` for replay, together with
`log.jsonl` (one row per control tick: action, events, ESP32 status, pose) and
`meta.json`.

| Test file | Covers |
| --- | --- |
| `test_vision.py` (22) | six colors, whitening, ambiguous color, unknown neighbors, robot-sized occluders, zones, borders, temporal loss, missing calibration, exposure change, packets, hue wrap |
| `test_pile.py` (7) | edge picking, buried/surrounded stones, same-color pairs, walls, packet field, off by default |
| `test_target_lock.py` (8) | lock through flicker, timeout, occlusion, color change, sticky vision targets |
| `test_robot_pose.py` (6) | position, heading, parallax, size check, wrong id, grip offset |
| `test_find_zones.py` (3) | six zones and colors found among stones, missing zone reported, grown exclusion circles |

All 84 root-project tests pass after this update: 58 original tests plus 26 new
calibration, reference and supplied-photo checks. The two supplied photos establish
a reference mismatch, not real-camera classification accuracy. See
[VALIDATION.md](VALIDATION.md) for evidence and hardware limitations.

See [REVIEW_AND_PLAN.md](REVIEW_AND_PLAN.md) for PDF rules, design decisions,
physical-lighting improvements, remaining robot work, and field acceptance tests.
## Autonomy (autonomy.py)

The latest fix prevents forward steering from reversing one wheel close to the
stage point. It preserves the locked stone while that stone is under the robot,
advances to approach when the jaws are already near it, and reports why the planner
stopped. Each run saves its config and a per-frame trace under
`runs/autonomy/<time>/`. See [AUTONOMY_FIX_TH.md](AUTONOMY_FIX_TH.md) for
deployment and status meanings. Simulation does not validate real motor response;
start field checks with one stone.

```bash
python autonomy.py --sim --show                   # simulated robot + field, watch it (q quits)
python autonomy.py --sim --noise --scenario pile  # with pose noise, latency, dropped tags, failed grabs
python autonomy.py --sim --field-physics          # + MIN_DUTY, spin stalls, walls, tag loss near edges (fitted to field traces)
python autonomy.py <ESP_IP> --camera 1 # after field and firmware validation
python autonomy.py <ESP_IP> --camera 1 --record   # also saves video.mp4 (raw frames) beside trace.jsonl
python -m unittest discover -s tests              # includes simulated runs and safety tests
```

| File | Role |
| --- | --- |
| `autonomy.py` | Planner state machine (SEARCH → GOTO_STAGE → ALIGN → APPROACH → GRIP → CARRY → RELEASE → BACKOFF) and the sim/real runners |
| `perception.py` | Camera frame → robot pose + stones in mm; masks the robot's footprint out of detection |
| `sim.py` | Simulated robot/field (firmware behaviour, wheels, gripper, zones, simple camera) |
| `fake_robot.py` | Firmware stand-in on UDP 4211 for `teleop.py` tests without hardware |
| `calibrate_grip.py` | Measures `robot_tag.grip_offset_mm` (stone in closed jaws) and `axle_offset_mm` (spin on the spot) |

Safety rules in the planner: the gripper only opens over the stone's own zone; a missed
grab (stone still visible at its old spot) is dropped outside every zone; no fresh pose
means the wheels stop; failed approaches retry once, then the stone is skipped for 25 s.
Tuning values go in `calib.json` → `"autonomy"` (defaults at the top of autonomy.py).
The servo angles there must equal `firmware/robot_ctrl/config.h`.

### Before the first real autonomous run (autonomy.py warns until 1 and 2 are done)

1. `python calibrate_grip.py <camera> --write`: jaws closed on a stone, arm down. Run twice at different spots; results should agree within ~5 mm.
2. `python calibrate_grip.py <camera> --axle --write`: start it, then spin the robot slowly on the spot with teleop for one full turn. In simulation, a 40 mm axle offset the planner didn't know about cut pile runs from 19 stones to 4.
3. Measure the real drive speed (teleop at a fixed speed for a few seconds) and adjust `autonomy.cruise`/`creep` if the robot is much faster or slower than the ~300 mm/s the defaults assume.
4. Check `robot_tag.footprint_mm` covers the robot in `detect_live.py` (the magenta outline): no blobs on the robot body.
