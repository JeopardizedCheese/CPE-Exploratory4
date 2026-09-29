# Handoff: gemBot autonomy tuning (session of 2026-09-29)

Repo: `~/Projects/CPE-Explo/gemBotV4/CPE-Exploratory4` (GitHub `JeopardizedCheese/CPE-Exploratory4`, branch `main`, pushed at `92d8e1d`).
A teammate may pick this up on another machine; they get the code via `git pull`, but NOT `runs/` (gitignored) or `firmware/robot_ctrl/secrets.h` (gitignored, Wi-Fi credentials — never commit; `secrets.example.h` must keep placeholders).

## Where things stand

- Robot: overhead-camera + AprilTag autonomous stone sorter (`autonomy.py` planner, `perception.py`, ESP32 firmware `firmware/robot_ctrl/`). Gripper servo is **broken**; a new gripper is coming. Until then only driving/turning can be judged — grab outcomes are meaningless.
- Latest work (commit `3de65de`, see its message for detail): pulse turning (`turn_mode: 'pulse'`, `PULSE_TABLE` in `autonomy.py`), pose prediction over `camera_delay_s` 0.2 s, BACKOFF by distance (`backoff_mm`), crash-safe recording (`--record` → MJPG `video.avi`), simulator refit (`sim.FIELD_PARAMS` = charged battery, `sim.LOW_BATTERY_PARAMS` = earlier fit), `sim_bench.py`.
- All of it is **simulation-validated only**. Tests: `.venv/bin/python -m unittest discover -s tests` (134 pass). Gesture subproject tests need `CHROMA-Gesture-Control/.venv-gesture` (sklearn).
- New gripper is fitted (2026-09-29): front of body 135 mm from the tag centre, grip offset measured [140, 0].
- Codex PR (`5b98762`): `motion_control.py` + firmware `duty` packets (real PWM, no floor; bench-verified ramp after the fix to `rampDuty`). Autonomy does NOT use `duty`: its 0.6/s ramp from zero is too slow for turn pulses.
- Drive floor per run: `autonomy.py --set min_duty=X` sends `m` with every `drive` packet; firmware uses it in place of `MIN_DUTY` (legacy ramp unchanged) and reports `min_duty` in status. Autonomy stops at once if the firmware doesn't report it. Default (unset) = firmware `MIN_DUTY` 0.71, identical to before. `--set` works for any autonomy option (typos rejected).
- The old stall measurement behind `MIN_DUTY 0.71` is **not trusted** (partly measured on USB power, per the user).
- Commit history for the whole arc: `git log --oneline 2f23e6e..92d8e1d`.

## Key findings (evidence behind the current design)

Detailed in the commit messages (`git log 2f23e6e..92d8e1d`). On the original author's laptop only, also in Claude memory (`charged-battery-motion.md`, `field-debug-2026-09-29.md`); on any other machine the summary below is the full record. Short version:
- Command → camera delay ≈ 0.2 s. On a charged battery the robot turns 150–300°/s once moving and drives 430–530 mm/s even at "creep" (MIN_DUTY 0.71 floor dominates). Held at turn power 0.6 it often did not turn at all (one 13 s case) — cause unknown (wheel slip vs motor current).
- Flat-battery runs (`runs/autonomy/20260929-02*`) behaved very differently (spin stalls, ~190 mm/s) — don't tune from them.
- Tag lost whenever robot is ~100–170 mm from an arena edge; every such loss happened in GOTO_STAGE (stage point 335 mm behind top/bottom-of-pile stones lands near the wall).
- The five `runs/autonomy/20260929-07*/video.mp4` were unfinalized; recovered copies are `video_recovered.avi` in the same folders (user's laptop only). Trace rows map to frames via `video_frame`.

## Open work, in the user's priority order

1. **Analyse the next recorded field run** (user/teammate will supply `runs/autonomy/<time>/` with `trace.jsonl` + `video.avi`): check pulse behaviour via trace keys `turn_phase`, `turn_pulse_s`, `turn_planned_deg`, `turn_moved_deg`, `pulse_gain`, `stalled`, `predict_mm`, `backoff_mm`. Compare real pulse rotation with `PULSE_TABLE`.
2. **Drive floor from the duty sweep**: `motion_control.py --test-duty` (forward/left/right, `runs/motion/`) gives the lowest duty that reliably moves and turns the robot on a charged battery. Use it as `--set min_duty=X` for autonomy (no reflash); once settled, put it in `calib.json` or `MIN_DUTY`. `PULSE_TABLE` and `sim.FIELD_PARAMS` (stall_duty 0.65 assumed) are fitted at 0.71 and need a refit from the new runs.
3. **Wall keep-out** (agreed design, not built): vision `_find_approach` in `vision.py` should also require the robot's staging point (tag) ≥ ~200 mm from every wall (corners follow automatically); planner clamps its own goals (park, safe drop, carry waypoints) and BACKOFF to the same margin; `pose_timeout_s` 0.5 → 0.25. Zones near walls (skyblue ~139 mm from bottom) must stay reachable. User confirmed 200 mm is acceptable-ish but asked to verify on the competition layout first — add that layout (zone centres estimated from `actual_start_zone.jpg`, table in chat: lime (169,419), violet (192,831), cyan (690,236), crimson (1245,188), skyblue (627,1061), orange (1209,1033), pile ≈ (850,640), start zone x ≥ ~1706) to the simulator.
4. **CARRY no-progress watchdog** (agreed, not built): if distance to zone hasn't shrunk ~50 mm in ~5 s → existing DISCARD (never opens inside a wrong zone).
5. New gripper follow-up: `GRIP_*` in `config.h` must equal `grip_open/close` in `calib.json` (they differ: close 40 vs 90), confirm `robot_tag.height_mm`, then discuss `approach_max_side_mm` (currently 11).
6. Minor: `teleop.py` still records mp4 (same unfinalized-file risk).

## User preferences / decisions (must respect)

- **No blind driving, ever** ("I will not accept such blasphemy"): no motion without a fresh pose; no tag-loss nudge/recovery. Prevention only.
- Wary of reversing into stones; accepted the short CARRY back-out (`backout_inside_reach`) and 120 mm BACKOFF.
- Prefers changes that are certain; wants data first, then discussion; likes being "grilled" on decisions. Concise answers when asking for commands.
- Team recalibrates the field every run (`calibrate_arena.py` → `find_zones.py`). Competition layout differs from practice.
- Commit per logical change; user asked to merge to `main` and push this time.
- Robot ESP32 is on the team hotspot at a fixed private IP (see `firmware/robot_ctrl/config.h`); camera index 1.

## Useful commands

```bash
.venv/bin/python sim_bench.py [--physics charged|low-battery|ideal] [--set key=value]
.venv/bin/python autonomy.py --sim --field-physics [charged|low-battery] --show
.venv/bin/python autonomy.py <ESP_IP> --camera 1 --record      # real run
.venv/bin/python autonomy.py <ESP_IP> --camera 1 --record --set min_duty=0.5   # real run, lower floor
.venv/bin/python motion_control.py <ESP_IP> --camera 1 --test-duty 0.4 --test-motion left --max-duty 0.8
```
Recovering an unfinalized mp4v file: take the 54-byte MPEG-4 VOL header from a same-settings reference mp4 (`ffmpeg -c copy -bsf:v dump_extra -f m4v`), prepend to bytes after `mdat`, decode with `ffmpeg -f m4v`.

## Suggested skills

- `mattpocock-skills:diagnosing-bugs` or `debug-mantra` — when the next field traces show misbehaviour.
- `mattpocock-skills:grilling` — the user explicitly enjoys being grilled before design decisions (wall keep-out, MIN_DUTY, approach tolerance).
- `scrutinize` — review the keep-out / watchdog design before building.
- `mattpocock-skills:tdd` — keep-out and CARRY watchdog are well suited to test-first in `tests/test_autonomy.py`.
- `dataviz` — if plotting trace data (pulse rotation vs table, speed vs command) for the user.
