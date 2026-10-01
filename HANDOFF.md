# Handoff: gemBot autonomy tuning (sessions of 2026-09-29 and 2026-09-30)

Repo: `~/Projects/CPE-Explo/gemBotV4/CPE-Exploratory4` (GitHub `JeopardizedCheese/CPE-Exploratory4`, branch `main`; last pushed 2026-09-30 with the PR #2 merge).
A teammate may pick this up on another machine; they get the code via `git pull`, but NOT `runs/` (gitignored) or `firmware/robot_ctrl/secrets.h` (gitignored, Wi-Fi credentials — never commit; `secrets.example.h` must keep placeholders).

## Where things stand

- Robot: overhead-camera + AprilTag autonomous stone sorter (`autonomy.py` planner, `perception.py`, ESP32 firmware `firmware/robot_ctrl/`). Gripper servo is **broken**; a new gripper is coming. Until then only driving/turning can be judged — grab outcomes are meaningless.
- Latest work (commit `3de65de`, see its message for detail): pulse turning (`turn_mode: 'pulse'`, `PULSE_TABLE` in `autonomy.py`), pose prediction over `camera_delay_s` 0.2 s, BACKOFF by distance (`backoff_mm`), crash-safe recording (`--record` → MJPG `video.avi`), simulator refit (`sim.FIELD_PARAMS` = charged battery, `sim.LOW_BATTERY_PARAMS` = earlier fit), `sim_bench.py`.
- **Wall keep-out + wall recovery (2026-09-30)**: PR #2 (Mickmocca, `24817b1`) merged at `20fbecc`, fixed in `eb46613`. `wall_guard.py`: stones are chosen only if tag ≥ 200 mm / body ≥ 20 mm from every wall at stage and pickup; park clamped; `pose_timeout_s` 0.25; perception no longer returns a held pose. Recovery (`WALL_RECOVERY`/`WALL_BLOCKED`): stop, wait until at rest, 0.15 s drive / 0.2 s turn pulses with a fresh pose each, retry after 3 s. Trace keys `wall_phase`, `wall_reason`, `wall_clearance_mm`. Details and sim numbers: `WALL_RECOVERY.md`, `CHANGELOG.md`.
- All of it is **simulation-validated only**. Tests: `.venv/bin/python -m unittest discover -s tests` (163 pass). Gesture subproject tests need `CHROMA-Gesture-Control/.venv-gesture` (sklearn).
- New gripper is fitted (2026-09-29): front of body 135 mm from the tag centre, grip offset measured [140, 0].
- Codex PR (`5b98762`): `motion_control.py` + firmware `duty` packets (real PWM, no floor; bench-verified ramp after the fix to `rampDuty`). Autonomy does NOT use `duty`: its 0.6/s ramp from zero is too slow for turn pulses.
- Drive floor per run: `autonomy.py --set min_duty=X` sends `m` with every `drive` packet; firmware uses it in place of `MIN_DUTY` (legacy ramp unchanged) and reports `min_duty` in status. Autonomy stops at once if the firmware doesn't report it. Default (unset) = firmware `MIN_DUTY` 0.71, identical to before. `--set` works for any autonomy option (typos rejected).
- The old stall measurement behind `MIN_DUTY 0.71` is **not trusted** (partly measured on USB power, per the user).
- Commit history for the whole arc: `git log --oneline 2f23e6e..92d8e1d`.
- **2026-10-01**: vision 23.5 → 14.0 ms per frame, byte-identical output (`b44d4ba`; it was `2013775` before the runs commit was removed, and that hash survives only on the backup branch). **Next to build: the pile-mode fix, fully planned and decided in [PILE_FIX_PLAN.md](PILE_FIX_PLAN.md)** (touching stones show as "?" and the robot parks beside them). Commit `0c394d9` (run traces + videos, 1.7 GB incl. one 185 MB file, too big for GitHub) was removed from `main` on 2026-10-01 at the user's request; it is kept only on the local branch `backup/with-runs-0c394d9`, never pushed. The files are still in `runs/` on disk (gitignored). The 09:15 empty-field reference the replays need is `background_0915.png` (repo root, untracked, this laptop only; `background.png` was never in git).

## Key findings (evidence behind the current design)

Detailed in the commit messages (`git log 2f23e6e..92d8e1d`). On the original author's laptop only, also in Claude memory (`charged-battery-motion.md`, `field-debug-2026-09-29.md`); on any other machine the summary below is the full record. Short version:
- Command → camera delay ≈ 0.2 s. On a charged battery the robot turns 150–300°/s once moving and drives 430–530 mm/s even at "creep" (MIN_DUTY 0.71 floor dominates). Held at turn power 0.6 it often did not turn at all (one 13 s case) — cause unknown (wheel slip vs motor current).
- Flat-battery runs (`runs/autonomy/20260929-02*`) behaved very differently (spin stalls, ~190 mm/s) — don't tune from them.
- Tag lost whenever robot is ~100–170 mm from an arena edge; every such loss happened in GOTO_STAGE (stage point 335 mm behind top/bottom-of-pile stones lands near the wall).
- **Stop overshoot** (field run `runs/autonomy/20260929-235410-bc918b`, min_duty 0.65): after a stop command the camera shows the robot at near full speed for 0.15–0.2 s (latency), then it coasts down for another ~0.15–0.2 s (firmware `drive(0,0)` = coast; InEngMotor has an unused `brake()`). Coasting leaves ~40–60 mm beyond pose prediction. It only hurts in APPROACH: the grip point ended 30–134 mm (median ~70) past the stone in 9 of 10 grabs (stone pushed aside → empty carries). CARRY overshoot is harmless (lands nearer the zone centre). "creep" 0.18 still runs ~350–400 mm/s because of the floor.
- The simulator has latency but no coasting. An eval-only coast model (time constant 0.12 s) drops sim sorting from 499 to 101 correct over 120 runs — the same APPROACH overshoot symptom as the field.
- The calibrated field fills our camera picture (≈0 px border left/right), so a tag outside the calibrated rectangle is normally out of the picture too; wall recovery acts between tag loss (~100–170 mm from an edge) and the 200 mm margin.
- The five `runs/autonomy/20260929-07*/video.mp4` were unfinalized; recovered copies are `video_recovered.avi` in the same folders (user's laptop only). Trace rows map to frames via `video_frame`.

- **2026-10-01, later: mini practice field.** Fellow students built a ~1650 x 1100 mm field in the common room with only a red and a green zone and an off-centre camera. Config `minifield/calib_minifield.json`, steps in `minifield/README.md` (camera height/floor point must be measured; grip offset 140 is an estimate). New keys: `zone_colors` (fields with fewer zones) and `autonomy.color_alias` (deliver zone-less colours to another zone). Arm remounted: firmware `config.h` servo pin 33, swapped pulse widths, close 100 (repo matches the user's board; needs a flash); `calib.json` `grip_close` set to 100 to match (uncommitted working copy). The user tunes duty (~0.65; the practice floor has less friction).
- **2026-10-01, gesture: PR #3 merged locally** (`ad770db`, not pushed). The one-hand lever `CHROMA-Gesture-Control/lever_control.py` replaces the two-hand system (kept as a backup; its core files are imported by the lever and must stay). Added `+`/`-` speed keys and `--min-duty`. Software-tested only. Gesture tests: `cd CHROMA-Gesture-Control && ./.venv-gesture/bin/python -m unittest discover -s tests` (75 pass).

## Open work, in the user's priority order

0. **V1 / V2 (built 2026-10-01, local, not pushed).** `autonomy.py` = V1 (unchanged), `autonomy2.py` = V2 (pile fix per [PILE_FIX_PLAN.md](PILE_FIX_PLAN.md) + outermost pile stone + buried-stone observations + target commitment + `skip_alone_s`); switches in `profiles.py`, same `calib.json`, same flags. Evidence in CHANGELOG "V1 / V2 split". Open: the user reviews the crop sheets in `evidence/v2-replay/` (acceptance bar item 3) before anything is pushed; then a field run of each version. Still undecided from the 2026-10-01 study: wall keep-out testing only vision's one `approach_deg`, parallax robot mask (hull), shadow rule, and the prototype `vision.robot_approach "free"` (local branch `proto/robot-side-approach`).
1. **Analyse the next recorded field run** (user/teammate will supply `runs/autonomy/<time>/` with `trace.jsonl` + `video.avi`): check pulse behaviour via trace keys `turn_phase`, `turn_pulse_s`, `turn_planned_deg`, `turn_moved_deg`, `pulse_gain`, `stalled`, `predict_mm`, `backoff_mm`. Compare real pulse rotation with `PULSE_TABLE`. New: check wall recovery on the real robot (`wall_phase`, `wall_reason`, `wall_clearance_mm`; how far one 0.15 s drive pulse and one 0.2 s turn pulse really move; whether a robot pinned on a wall gets free). `WALL_RECOVERY.md` has a field check procedure.
2. **Stop overshoot in APPROACH** (analysed 2026-09-30, user has not chosen yet): options offered, in the suggested order: (a) bench test firmware short-brake (`inengmotor.brake()`) on a zero command vs coast — needs a reflash, and check the driver chip really brakes with both inputs high; (b) speed-proportional stop lead in APPROACH only (`grip_tol + speed × stop_lead_s`), test-first; (c) a coast term in `sim.py` so (b) can be validated. Fallback: pulse-driving APPROACH like pulse turning (precise, slower).
3. **Drive floor from the duty sweep**: `motion_control.py --test-duty` (forward/left/right, `runs/motion/`) gives the lowest duty that reliably moves and turns the robot on a charged battery. Use it as `--set min_duty=X` for autonomy (no reflash); once settled, put it in `calib.json` or `MIN_DUTY`. `PULSE_TABLE` and `sim.FIELD_PARAMS` (stall_duty 0.65 assumed) are fitted at 0.71 and need a refit from the new runs.
4. **Wall keep-out follow-ups** (the keep-out itself is done, see above): it is checked in the planner (`Planner._approach_inside`), not in vision `_find_approach`; safe drop and carry waypoints are not clamped (recovery catches them); BACKOFF is interrupted by recovery rather than clamped. The competition layout (zone centres estimated from `actual_start_zone.jpg`: lime (169,419), violet (192,831), cyan (690,236), crimson (1245,188), skyblue (627,1061), orange (1209,1033), pile ≈ (850,640), start zone x ≥ ~1706; radius ~110 assumed) was simulated on 2026-09-30 with these zones patched into `calib.json` in a scratch script: zones near walls stayed reachable. It is still not a built-in simulator layout. In that layout, delivering near a wall often triggers recovery with the only escape blocked by already-placed stones (brief `wall_no_clear_escape` blocks, ~8 s per run in total).
5. **CARRY no-progress watchdog** (agreed, not built): if distance to zone hasn't shrunk ~50 mm in ~5 s → existing DISCARD (never opens inside a wrong zone).
6. New gripper follow-up: `GRIP_*` in `config.h` must equal `grip_open/close` in the config (now both 100). The competition `calib.json` still has the old arm's geometry (grip [205,0], footprint front 135): update it from the mini-field measurements (front 175, measured grip offset) before the match.
7. Minor: `teleop.py` still records mp4 (same unfinalized-file risk).

## User preferences / decisions (must respect)

- **No blind driving, ever** ("I will not accept such blasphemy"): no motion without a fresh pose; no tag-loss nudge/recovery. 2026-09-30: the user accepted wall recovery **with a visible tag** (PR #2 + fixes), on the condition that it does not degrade performance (checked in the simulator before merging). A robot whose tag is lost still just stops.
- Wary of reversing into stones; accepted the short CARRY back-out (`backout_inside_reach`) and 120 mm BACKOFF.
- Prefers changes that are certain; wants data first, then discussion; likes being "grilled" on decisions. Concise answers when asking for commands.
- Team recalibrates the field every run (`calibrate_arena.py` → `find_zones.py`). Competition layout differs from practice.
- Commit per logical change; user asked to merge to `main` and push (2026-09-29 and again 2026-09-30). Write CHANGELOG entries (EN + TH) for changes.
- **With every command to run a script, remind the user of the valid flags and switches** (list in PILE_FIX_PLAN.md; keep it current).
- Match start: all gems start in one big pile and the team will make the robot crash into it. **The user will provide that code; don't write it.** Vision changes must keep the big pile's colour-0 observation.
- No field testing time is left (2026-10-01): judge changes by replay of recorded runs and tests; offer config switches as the fallback at the match.
- Robot ESP32 is on the team hotspot at a fixed private IP (see `firmware/robot_ctrl/config.h`); camera index 1.

## Useful commands

```bash
.venv/bin/python autonomy2.py <ESP_IP> --camera 1 --record   # V2 (same flags as autonomy.py)
.venv/bin/python detect_live.py 1 --v2                        # preview V2 vision
.venv/bin/python sim_bench.py --v2                            # V2 planner switches in the simulator
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
