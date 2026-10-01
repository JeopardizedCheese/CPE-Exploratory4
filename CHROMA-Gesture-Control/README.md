# CHROMA Gesture Control (two hands, one command set)

Drive the robot with hand poses in front of a webcam. **Either hand gives the same commands**, so the
driver can switch hands when one gets tired. You train the poses yourselves:
record → train / validate / test → check live → drive.
Thai training guide: [TRAIN_GESTURES_TH.md](TRAIN_GESTURES_TH.md).

| Command | Suggested pose (you may train others) | Robot |
| --- | --- | --- |
| `STOP` | open palm facing the camera | wheels stop |
| `FORWARD` | index finger pointing up | forward |
| `BACK` | thumb pointing down | reverse |
| `LEFT` / `RIGHT` | index finger pointing screen-left / screen-right | turn in place (0.6 x speed) |
| `GRIP_OPEN` | V sign | open the gripper (once) |
| `GRIP_CLOSE` | fist | close the gripper (once) |
| `NONE` | anything else: relaxed hand, half-open, moving between poses | wheels stop |

How the two hands combine (every camera frame):
- No hand, a stale frame (> 0.2 s), only `NONE`: **wheels stop at once**.
- Any hand shows `STOP`: stop. The two hands show **different** commands: stop ("CONFLICT").
- Otherwise the one command shown drives. The other hand may rest (`NONE`) or show the same command,
  so you can hand over: show the command with the new hand, then drop the old one.
- A motion command must be steady 0.15 s before the wheels move; a grip command 0.4 s, and it is
  sent once (change pose to send it again). Low-confidence frames count as `NONE`.
- Keyboard: **G** start (waits for the robot's RUNNING status), **SPACE / X** stop (robot to IDLE),
  **+ / -** speed 0.1..1.0 in steps of 0.1 (like `teleop.py`), **ESC** quit.

## Setup (once)

Windows: `.\setup_gesture.ps1` (Python 3.12). Linux: the existing `.venv-gesture`.
Below, `PY` = `.\.venv-gesture\Scripts\python.exe` (Windows) or `./.venv-gesture/bin/python` (Linux).
Run everything from this folder.

## 1. Record (gesture_collect.py, the trainer)

```bash
PY gesture_collect.py --split train --note "Mick, common room, evening"
PY gesture_collect.py --split validation --note "..."
PY gesture_collect.py --split test --note "..."
```
Flags: `--camera N` (default 0), `--split train|validation|test` (fixed per session), `--samples N`
(per take, default 60 at 5 per second), `--note TEXT`, `--session ID` (resume), `--data DIR`
(default `gesture_data/`).

One run = one **session**. Record **all 8 commands with both hands** in every session.
Recommended: 3 train sessions, 1 validation, 1 test, on different days/lighting/people if you can.
**The test session must be recorded last and never looked at while tuning.**

In the trainer: **1-8** command, **H** hand, **N** next missing command/hand, **R** record (2 s
countdown, then 60 samples in 12 s), **S** save early, **D** discard take, **U** undo the last saved take,
**M** live check with the newest model, **ESC** quit. The screen shows the counts for this session and
for the whole split, and whether each split is ready (>= 30 samples per command, hand and split).
Show one hand only. It warns when the camera sees the other hand than the one selected.
While recording, move slowly: angle, distance, height, a little rotation. Record `NONE` while you
move between poses, scratch, wave, rest. That is what keeps half-poses from driving the robot.

## 2. Train, validate, test (gesture_train.py)

```bash
PY gesture_train.py --inspect      # counts per split; says what is missing
PY gesture_train.py                # trains -> models/gesture_commands_<time>.npz + .report.json
```
Flags: `--data DIR`, `--output PATH`, `--inspect`, `--seed N` (42), `--iterations N` (500).

- TRAIN sessions fit the network (MLP 64-32 on 63 hand-landmark numbers).
- VALIDATION sessions choose the confidence gate (threshold, margin): the safest gate (fewest
  *dangerous* mistakes = a wrong motion or grip command), then the one that recognises most commands.
- TEST sessions are scored once with that gate: accuracy, commands recognised, read as NONE,
  dangerous mistakes, per hand, recall per command and the confusion table.
Nothing is overwritten; each training writes a new timestamped model.

## 3. Check, then drive (gesture_control.py)

```bash
PY gesture_control.py --demo                                       # keyboard only: 1-8 pose, 0 none, 9 second hand
PY gesture_control.py --camera 0                                   # preview: real model, no robot
PY gesture_control.py --camera 0 --live --robot 172.20.10.2        # real robot
PY gesture_control.py --camera 0 --live --robot 172.20.10.2 --min-duty 0.65 --speed 0.3
```
Flags: `--camera N`, `--live`, `--robot IP` (required with `--live`), `--port 4211`, `--speed X`
(start speed 0.1..1, default 0.3), `--min-duty X` (drive floor sent with every packet; firmware must
report `min_duty`), `--model PATH` (default: newest `models/gesture_commands_*.npz`),
`--threshold X` / `--margin X` (override the trained gate: higher = fewer false commands, more NONE),
`--demo`.
Test against the PC stand-in first: `../.venv/bin/python ../fake_robot.py`, then
`--live --robot 127.0.0.1`.

## Safety

- No fresh hand → stop. Status from the robot lost (> 0.6 s) or another controller took over →
  stop, press G again. Firmware stops the wheels itself after 300 ms without packets.
- Only one controller at a time: close teleop/autonomy first.
- Frame-level test scores are not robot results; check the model live in preview first.
  The gripper angle in the status is what the firmware commanded, not a measurement.

## Files

`gesture_control.py` driver app + dashboard (`gesture_ui.py`), `gesture_logic.py` command logic and
UDP link, `gesture_link.py` start/stop/RUNNING gate, `hand_camera.py` webcam + MediaPipe hand
landmarks, `gesture_collect.py` trainer, `gesture_train.py` training/validation/test,
`gesture_model.py` labels, features, portable model. Old data: `gesture_data_v1/` (old labels, not used).
Tests: `PY -m unittest discover -s tests`.
