# Color-first project update

Based on `CPE-Exploratory3-main(1).zip`. This update prioritizes stone-color
recognition, explains rejected detections, and checks whether the empty-field
reference still matches the scene. It does not claim measured field accuracy yet.

## Findings from your files

- Cyan and sky-blue overlap at H=78..101, S=71..255, V=98..255. Pixels inside
  this intersection match both classes and are discarded as ambiguous. Crimson
  and orange also overlap. Expanding both ranges makes this worse.
- Stone area/extent filters were commented out. The original suite failed its
  long-cluster rejection test. The filters are restored.
- `examples/image.png` is a 640 x 480 raw view with stones. `examples/image1.png`
  is a 767 x 563 empty-field image with the historical rectified dimensions.
  After applying the supplied historical homography to the raw view, their six
  scoring-circle centers differ by roughly 25..49 rectified pixels. This pair
  is not a matching frame/background. The images do not establish whether camera
  movement, field/circle movement, or the supplied calibration caused the mismatch.
- The old detector still reported `ok` on that pair. With reference checking
  enabled, the update reports `reference_moved` and withholds targets while
  retaining the pixel color display. The photos also differ in brightness/color.

## Changes

- Sampling freezes frames before clicks, starts with a 5 x 5 pixel patch,
  rejects mostly-white/background samples, supports display zoom and exact undo,
  and persists patches across sessions.
- Saturation and brightness have measured upper and lower bounds. Previously
  both upper bounds were always 255, making nearby classes needlessly overlap.
  Red hue wrap remains supported. No missing color boundaries are guessed.
- Saving checks per-class sample coverage. Too few patches or conflicting
  calibration labels prevent applying bad ranges; the samples are retained.
- Evaluation sessions never change ranges. Reports include wrong, ambiguous and
  unmatched fractions. These are correlated patch pixels, not independent stones.
- The color preview shows names, masks, rejection reasons and reference checks;
  it handles camera/image/video and saves evidence without any robot connection.
- Fixed scoring-circle positions provide an optional stale-reference guard,
  enabled in the new field profile. Missing markers mean unverified; geometry
  is never silently updated or repaired.
- Zone colors are no longer guessed from the closest hue. Manual circle labels
  handle differences between printed paper and actual stone colors.
- Camera properties are also applied in autonomy and gripper calibration, where
  they were missing. Arena recalibration clears stale destination coordinates.

Firmware, teleop, simulator, fake robot, stress test and `firmware_check.py` come from
the team's current version (two states, fixed IP, silent servo at boot); see CHANGELOG.

## Current field profile

Use `field/calib.json` consistently in every command. It contains your latest
measurements: arena 2100 x 1200 mm, lens height 1810 mm, and lens-floor projection
[1100, 600] mm assuming top-left origin and X along the 2100 mm side.

Tag size/height, gripper offset and footprint are copied starting values requiring
physical verification. Corners, background, colors and zones start uncalibrated.
The root `calib.json` is preserved as a historical record for the photo comparison.

## Field workflow

Use Python 3.12 and the project's GUI OpenCV dependencies in a virtual environment:

```bash
python -m pip install -r requirements.txt
```

1. Fix the camera and choose its final resolution, focus, exposure and white
   balance. The profile requests 640 x 480 to match the existing setup. If the
   camera supports a sharper higher-resolution mode, choose it before calibration
   and sampling. Small stones at 181 cm need enough camera detail. Property
   values depend on the camera/backend; check warnings and the actual image.

2. Empty the field while keeping its permanent scoring circles:

```bash
python calibrate_arena.py 1 --config field/calib.json
```

SPACE freezes; click top-left, top-right, bottom-right, bottom-left, then Enter.
The 2100 mm side corresponds to left-to-right. Press `s` to skip outlining zones;
step 4 will identify them. This writes `field/background.png`. Repeat this step
after camera/field movement or a camera-setting change.

3. Place separate real stones and collect all six colors:

```bash
python sample_hsv.py 1 --config field/calib.json
```

SPACE freezes; select 1 violet, 2 cyan, 3 crimson, 4 orange, 5 skyblue, 6 lime.
Click a colored stone face. Avoid white glare and scoring paper. The cursor square
shows the patch; `+/-` adjusts size, `u` undoes the selected color's last patch in
this session, `s` applies ranges, and `q` exits. `--zoom 2` enlarges the display
while samples keep their native pixel size.

Five patches per color is only a minimum save gate. As a practical start, collect
three calibration sessions with different physical stones, positions and normal
shadows. Close and reopen the sampler between sessions. Earlier patches persist
in `field/color_samples.json`; config backups are saved before applying changes.
Keep camera settings and geometry fixed. If they change, the sampler rejects
mixing incompatible records: use a new `--samples field/color_samples_new.json`.
Use that same samples path when recording its evaluation session.

If cyan/skyblue or crimson/orange remain ambiguous, inspect the labels and use
smaller patches on actual faces. If the camera genuinely renders the colors
indistinguishably, improve image acquisition and retain UNKNOWN. White reflections
contain no usable hue. Repeatedly clicking the same stone is not new variation.

4. Label all six scoring circles and create exclusion areas:

```bash
python find_zones.py --config field/calib.json
```

Click a circle, press its correct 1..6 ID, and repeat. Press `s` after all six
different labels match the real paper circles. Unknown/duplicate/missing labels
cannot be saved. For a scripted mapping, `--labels` accepts six IDs in the printed
zone order. Paper colors may differ from stones; do not widen stone ranges just
to recognize scoring paper.

5. Preview without motor or servo commands:

```bash
python color_preview.py 1 --config field/calib.json
```

Left: objects when the reference is valid. Right: raw pixel color evidence,
including painted zones; WHITE means multiple matching classes, black means
insufficient/unmatched evidence. `m` switches to foreground, SPACE pauses, `s`
saves a raw image, overlay, masks, config and JSON, and `q` quits. Reports retain
rejected blobs even when hidden from the display for readability.

| Status/reason | Next action |
|---|---|
| `setup_required` | Inspect diagnostics for missing background, corners or colors |
| `reference_moved` | Check mounting/layout, then recapture and recalibrate the empty field |
| `reference_unverified` | Ensure at least three fixed circles are visible; inspect lighting/occlusion |
| `lighting_or_camera_change` | Check camera settings/reference before resampling |
| `overlapping_color_ranges` | Inspect samples from the conflicting classes |
| `insufficient_color` | Check glare, focus, dark faces and patch coverage |
| `mixed_colors` | Separate touching stones; pile handling is a later stage |
| `size_out_of_range` | Check scale and stone size; a pile is not one stone |
| `insufficient_clearance` | The color can be correct while pickup remains blocked |

The circle guard is a heuristic, not full camera calibration. It cannot detect
every camera/lens change. Its default threshold is 8 mm displacement of at least
two circles, with at least three circles verifiable. Validate this threshold on
your setup; it does not establish 8 mm position accuracy. It never updates geometry.

6. Evaluate new scenes without updating calibration:

```bash
python sample_hsv.py 1 --config field/calib.json --purpose evaluation
python color_preview.py --audit --config field/calib.json --samples field/color_samples.json
```

Use different stones/placements. `s` saves the evaluation report. These are labeled
patch-pixel measurements, not whole-stone detection accuracy. After tuning against
an evaluation scene, reserve another session for the final test.

Also test one separate stone of each color at five measured field locations,
normal shadows/glare, an empty field and robot occlusion. Log correct labels,
wrong labels, UNKNOWNs, missed stones, false objects and latency. Choose acceptance
limits from the actual gripper tolerance and scoring penalty.

## Reproduce your supplied-photo check

```bash
python color_preview.py --audit --config calib.json
python color_preview.py --image examples/image.png --background examples/image1.png --config calib.json --check-reference --headless --output evidence/provided-photos
```

This intentionally uses the historical config. Do not copy `examples/image1.png`
into the current field profile. A fresh empty reference from the current setup
is still required. Millimeter shifts calculated with the historical wrong field
dimensions are not physical measurements; the pixel mismatch is the evidence.

## Remaining priorities and training

1. Complete and measure color recognition first; save representative scenes.
2. Verify AprilTag pose, height correction and tape-measured positions/headings.
3. Calibrate tag-to-jaw offset and repeat a single-stone grip/release.
4. Resolve the previously reviewed firmware/PC state mismatch, then test autonomous
   sorting and enable pile handling only after separate-stone tests pass.

A trained detector is worth evaluating if calibrated color thresholds still fail
on varied scenes. Collect labeled stone images from multiple sessions and keep
an unseen session for comparison. A neural detector predicts image locations and
classes; camera-to-field calibration and AprilTag checks remain necessary. No
neural network has been trained or claimed more accurate in this update.

## Validation

```bash
python -m unittest discover -s tests -v
```

See `VALIDATION.md` for the final test count and real-photo evidence. Passing
synthetic regressions does not establish accuracy or robot reliability in the field.
