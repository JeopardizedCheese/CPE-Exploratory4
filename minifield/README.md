# Mini practice field (common room)

`calib_minifield.json` is a separate config for the small practice field: about 1650 x 1100 mm, with only a
red (3 crimson) and a green (6 lime) zone. The competition `calib.json` is not touched. Every script takes
`--config minifield/calib_minifield.json`. `background.png` and `color_samples.json` are written next to
the config, in this folder; both are gitignored.

`python autonomy.py --config minifield/calib_minifield.json --check-config` lists what is still missing.

## What differs from calib.json

| Key | Value | Why |
|---|---|---|
| `zone_colors` | `[3, 6]` | only two zones; `find_zones.py` and the start check expect exactly these |
| `autonomy.color_alias` | `{"1": 3, "4": 3, "2": 6, "5": 6}` | violet + orange go to red, cyan + skyblue go to green, so every stone gives a test grab. Remove it (or `--set color_alias={}`) to pick only red and green stones; the others are then avoided like any obstacle |
| `arena.size_mm` | `[1650, 1100]` | x = the 165 cm side |
| `robot_tag.camera_height_mm`, `camera_floor_xy_mm` | `null` | the camera is not centred: **measure both** (below). The run refuses to start while they are null |
| `robot_tag.footprint_mm.front` | 175 | tag centre to the jaw tips (new arm) |
| `robot_tag.grip_offset_mm` | `[140, 0]` | **estimate** (tips 175 minus ~35 mm, as on the old arm): measure with a ruler (below) |
| `autonomy.grip_close` | 100 | must equal `GRIP_CLOSE_DEG` in `firmware/robot_ctrl/config.h` |
| `autonomy.approach_max_side_mm` | 25 | about half of (jaw tip gap 105 - stone ~40), minus a little for pose noise |
| `autonomy.min_duty` | 0.65 | sent with every drive packet, no reflash |
| `vision.reference_guard` | `min_markers` 1, `min_shifted_markers` 1 | with two zones, the robot covering one zone would otherwise stop vision |
| `camera_properties` | 640 x 480, FPS 30 | change to what the camera supports (`camera_probe.py`), then redo `calibrate_arena.py` |

Unchanged: `mm_per_px` 3 (every pixel-based vision threshold stays the same), tag, back/left/right footprint,
`footprint_margin_mm` 30, `axle_offset_mm`, all `vision` values including `gripper_width_mm` 60, wall margins.

## Setup order

1. Camera: `python camera_probe.py 1 --try-config --config minifield/calib_minifield.json` (real fps, accepted size).
2. Field: `python calibrate_arena.py 1 --config minifield/calib_minifield.json`. Empty field. Click the corners
   **clockwise as seen in the picture, the first two along the 165 cm side**. If the long side runs up/down in
   the picture, start at a corner such that corner 1 -> 2 is still a long side; the rectified view is then turned
   90 degrees, which is fine. Press `s` right away (no hand-drawn zones).
3. Zones: `python find_zones.py --config minifield/calib_minifield.json`. It must find exactly the red and the green
   zone. If a zone is found but labelled `?`, click it and press 3 or 6, or use `--labels 3 6` in the printed order.
   If no zone is found, the paper circles may be outside `zone_area_mm2` (radius about 70 to 160 mm).
4. Colours: `python sample_hsv.py 1 --config minifield/calib_minifield.json` under this room's light (at least
   5 patches per colour you use; unsampled colours keep the competition ranges).
5. Camera position (the camera is not centred):
   - `camera_height_mm`: floor to the lens, tape measure.
   - `camera_floor_xy_mm`: the floor point straight under the lens (plumb line), in field mm: x along the
     edge from clicked corner 1 to corner 2, y from corner 1 toward corner 4. Negative or larger than the field
     is fine if the camera hangs outside it.
   - Why it matters: the tag is 185 mm above the floor, so its position is corrected toward the point under the
     lens by 185 / height (about 10 %). A floor point 500 mm off puts the robot about 50 mm off; 50 mm off is
     about 5 mm. Stones (20 mm high) are 10 times less sensitive.
   - Check: `python robot_pose.py 1 --config minifield/calib_minifield.json`. `side` should read about 120 mm
     everywhere on the field. Put the tag centre on a taped point and compare x, y.
6. Grip offset (instead of `calibrate_grip.py`): close the jaws on a stone, measure from the tag centre to the
   stone centre along the robot's centre line (first number) and sideways, right positive (second number).
7. Firmware: flash `firmware/robot_ctrl` (servo pin 33, close 100).

## Run

```bash
.venv/bin/python autonomy.py 10.178.188.50 --camera 1 --config minifield/calib_minifield.json --record
.venv/bin/python autonomy.py --dry-run --camera 1 --config minifield/calib_minifield.json    # no robot commands
```

The floor is lower-friction than the competition field: expect longer coasting after stops and
different pulse turns (`pulse_gain` learns the turn factor during the run).
