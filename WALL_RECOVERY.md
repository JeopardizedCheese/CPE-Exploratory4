# Return from a wall with a visible AprilTag

The supplied `(2).zip` did not implement wall recovery. Its pose detector already
searched the **full camera image**, including outside the calibrated field, but
the autonomous planner had no boundary recovery state. This update adds one to
`autonomy.py`; it uses the existing firmware `drive` command.

This is for a tag that is still detectable in the full image. If the tag disappears
completely, the robot stops. It does not reverse using a remembered position.

## What happens

1. A live tag within 200 mm of an edge, a body corner within 20 mm of an edge, or
   predicted motion crossing those limits interrupts the current drive or backoff.
   Recovery also accepts live tag coordinates up to 200 mm outside the rectangle.
2. The robot stops for at least 350 ms **and until the camera shows it at rest**
   (below 80 mm/s). The camera reports motion ~0.2 s late and the wheels coast
   after a stop, so movement from before the stop is not held against the
   recovery: "moving outward" is judged only after its own first pulse. It then
   chooses forward or reverse motion that increases clearance. A robot facing
   the wall normally reverses.
3. It moves for a 150 ms drive pulse (or a 200 ms turn pulse at turn power 0.5),
   stops, waits until at rest, and measures again. A drive pulse needs a clear
   path of 120 mm, about what one pulse travels on a charged battery. A tangent
   heading turns toward facing inward, or toward facing the wall to reverse out,
   whichever keeps every body corner at least 20 mm inside the field over the
   whole arc one pulse may sweep (the planned turn + 30°, at least 100°), avoids
   observed stones, and preferably keeps the tag off the edge. A physically
   pinned robot with no clear route stays stopped.
4. Once the tag is at least 260 mm inside and the body has at least 80 mm
   clearance, normal autonomy resumes. An interrupted pickup is skipped briefly;
   an already held stone remains assigned to its original color. Recovery sends
   no gripper commands. A grip/release already in progress finishes while stopped.
5. No progress for 3 seconds, 12 seconds total recovery, motion substantially
   farther outward, or no clear escape produces `WALL_BLOCKED`. The robot stays
   stopped for 3 seconds (`wall_retry_s`), then tries again from what the camera
   shows: a stone may have been pushed aside, or the robot moved by hand. If it
   is already safely inside, normal autonomy resumes.

Staging and pickup poses are checked against the boundary before acquiring a
target, and the park destination is clamped inside it. Recovery also interrupts
an unsafe backoff. This is a local escape controller, not a general obstacle
path planner: it will stop if a stone blocks the only available retreat.

The sender has an independent pulse deadline, so a slow camera-processing frame
cannot keep refreshing a recovery pulse. Stop packets go out on the next 20 Hz
sender tick; Python scheduling and Wi-Fi are not real-time guarantees. The existing
firmware drive watchdog still handles loss of communication.

## Our camera: where recovery can act

On our field the calibrated rectangle fills the camera picture (0 px border left
and right, roughly 25-100 mm top and bottom), and the tag was lost 100-170 mm from
an edge in the 2026-09-29 runs. So recovery normally acts between the point where
the tag disappears and the 200 mm margin; a robot whose tag is already gone
stays stopped (no blind driving). The keep-out checks on staging, pickup and park
are what keep the robot out of that band in the first place.

## Run with the full camera view

Keep your current field calibration, HSV settings, background and robot geometry.
Run from the extracted project folder, with the same Python environment as before:

```powershell
python autonomy.py --dry-run --camera 1 --show-full-frame
python autonomy.py YOUR_ROBOT_IP --camera 1 --show-full-frame --record
```

The first command sends **no robot commands**. In the full-camera window, yellow
marks the calibrated field and magenta outlines a detected tag. Check that a tag
outside yellow still appears and that its indicated heading matches the robot.
The second command starts ordinary autonomy with automatic recovery enabled.
`q`, `x`, or Escape stops the run. It resumes sorting after recovery.

Start the physical check at one clear edge, with the robot facing the wall and its
tag visible, then repeat at each edge. Check the body/gripper measurements and
the real distance travelled by one pulse before placing stones along the escape
route. The camera must see a border around the calibrated field. A tag-size or
calibration rejection is a stop condition; widening the display does not fix it.

Logs in `runs/autonomy/<run>/trace.jsonl` contain `wall_phase`, `wall_reason`,
`wall_clearance_mm` and `wall_command_until`, as well as the existing tag reason,
pose, firmware state and requested wheel commands. `wall_clearance_mm` is the
smallest clearance **relative to the configured margins**, not distance to the wall.

## Tuning

Settings live under `calib.json` → `autonomy`, or use `--set KEY=VALUE` for one run.

| Setting | Default | Meaning |
| --- | ---: | --- |
| `wall_margin_mm` | 200 | Minimum tag distance from each edge |
| `wall_body_margin_mm` | 20 | Minimum body-corner distance |
| `wall_resume_mm` | 60 | Extra clearance before resuming |
| `wall_recovery_speed` | 0.18 | Forward/reverse drive command |
| `wall_recovery_turn` | 0.5 | Turn drive command (= `pulse_power`; at 0.25 a spin often never starts) |
| `wall_pulse_s` | 0.15 | Drive pulse; limited to at most 0.20 s |
| `wall_turn_pulse_s` | 0.20 | Turn pulse; limited to at most 0.40 s |
| `wall_path_mm` | 120 | Path checked for stones and wrong zones before a drive pulse |
| `wall_settle_s` | 0.35 | Stopped observation period; at least camera delay + 0.10 s |
| `wall_rest_mm_s` | 80 | The camera must show the robot slower than this before a pulse |
| `wall_retry_s` | 3 | Stopped time in `WALL_BLOCKED` before a new attempt |
| `wall_pose_max_age_s` | 0.25 | Maximum age accepted for recovery |
| `wall_no_progress_s` | 3 | No-motion timeout |
| `wall_timeout_s` | 12 | Total recovery timeout |
| `wall_max_outside_mm` | 200 | Limit on recovery outside calibration |
| `wall_recovery_enabled` | true | Enable boundary recovery and goal checks |

These are motor **commands**, not volts or measured mm/s. Existing `min_duty`
still applies: with a 0.71 floor, command 0.18 maps to about 0.76 duty, so it can
move fast on a charged battery. The short pulse limits time, not exact distance.
Use a floor that reliably moves your actual robot; too low can stall, too high
can travel too far during a pulse. Reduce pulse duration and increase margins if
needed after measuring. Battery-independent speed requires measured feedback;
no encoder or IMU was added or assumed.

No calibration values or firmware were changed by this update. The existing
camera-guided turn/speed tool is also unchanged. The changed perception contract
returns `None` for a missed tag frame immediately; only the image mask briefly
retains the previous pose.

## Verification and limits

`python -m unittest discover -s tests` passes 163 tests. New checks cover visible
tags outside all four calibrated edges, forward/reverse recovery, corners,
rotation clearance, obstacle and wrong-zone checks, payload retention, freshness
and firmware gates, pulse deadlines, timeouts, simulated camera latency, motion
still arriving after the stop, waiting for rest, the arc turn next to a wall,
and retry after `WALL_BLOCKED`.
Mission simulation tests still exercise sorting with the guard enabled. One
historical zone-routing-only test disables the guard because it deliberately
starts outside the new wall margin.

Simulator comparison (2026-09-30; charged-battery physics, min_duty 0.65, tag lost
within 130 mm of an edge; 30 seeds each of practice/competition layout x
scattered/pile, 180 s):

| | stones correct | wrong | runs frozen in `WALL_BLOCKED` |
| --- | ---: | ---: | ---: |
| before the wall guard | 499 | 84 | - |
| PR #2 as submitted | 461 | 62 | 26 / 120 |
| with the fixes above | 515 | 72 | 0 (brief blocks retry) |
| before, with a 0.12 s coast model | 101 | 34 | - |
| PR #2, coast model | 92 | 31 | 54 / 120 |
| fixes, coast model | 118 | 35 | 0 |

Robot started 100-195 mm from each wall at 12 headings (coast model): 232 of
240 back inside (PR as submitted: 206), median 2.0 s. The 8 others are parallel
to the wall at 130 mm; any turn there would bring a corner within 9 mm of it.

These checks validate software decisions, not real-world release friction,
traction, camera calibration outside its rectangle, or physical clearance.
The existing simulator simplifies collisions and does not model stone pushing.
Hardware performance has not been verified in this update.
