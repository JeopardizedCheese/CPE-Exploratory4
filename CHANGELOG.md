# CHANGELOG: CHROMA gemstone-sorting robot (CPE102)

## 2026-09-30: changing fields and iPhone hotspot preparation

**EN**
- Correct a missed firmware limit: close was set to 70 degrees but `SERVO_MAX_DEG` still clamped it to 55. Set the limit to 70 and fail compilation for out-of-range open/close targets; expose the configured angles/limits in status.
- Default to DHCP instead of the old phone's static subnet. Announce the assigned address after connection/reconnection, retry UDP binding, and clear wheel commands/old status peer when the link is lost or its IP changes.
- Add `network_check.py`, which sends only protocol pings and checks status and reported servo limits, without actuating the robot.
- Document fresh field/camera/color calibration, measured gripper geometry, an iPhone setup sequence and the remaining dense-pile routing/segmentation work in `FIELD_PREPARATION_TH.md`. Correct obsolete firmware/button/setup instructions in `README_ROBOT.md`.
- Validation: 189 tests pass, including actual servo/network helper code compiled against host stubs and a loopback UDP checker test. No ESP32 build/upload or physical iPhone/robot trial.

**TH**
- แก้เพดาน servo ที่ยังเป็น 55° ให้รองรับปิด 70° และตรวจค่าขัดกันตั้งแต่คอมไพล์
- เปลี่ยนค่าเริ่มต้นเป็น DHCP สำหรับ hotspot ใหม่ พร้อมแสดง IP จริงหลังเชื่อมต่อ และเพิ่มเครื่องมือตรวจเครือข่ายที่ส่งเฉพาะ ping
- เพิ่มคู่มือภาษาไทยสำหรับย้ายสนาม/กล้อง ใช้ iPhone hotspot และแผนทำเส้นทางอ้อมกองใหญ่ โดยระบุชัดว่างานกองส่วนใดยังไม่เสร็จ

## 2026-09-30: stop before gripping and recover beside piles

**EN**
- Use `CAPTURE` to stop and remeasure a stone already between the jaws before closing. Near-contact approach uses bounded straight pulses, waits for the camera to show rest, and retreats before turning when a stone is too close and off-center.
- Detect local jaw stones separately from navigation masks; project the raised robot body into the image so roof/finger blobs do not block wall recovery. Allow an escape that moves away from an object already inside collision padding.
- Set close to 70 degrees throughout firmware, planner, simulator and firmware check. A reported close-angle mismatch stops in `GRIP_BLOCKED`.
- Supply provisional grip offset `[197, -4]` mm and front footprint 235 mm, explicitly uncalibrated. The stationary video supports the offset estimate; confirm it on the robot. The reference movement later in the video still requires field recalibration.
- Add regression coverage and a small recorded-position fixture. See [PICKUP_RECOVERY.md](PICKUP_RECOVERY.md) for evidence, setup and validation limits.

**TH**
- เพิ่ม `CAPTURE` ให้หยุดและวัดตำแหน่งหินอีกครั้งก่อนคีบ เมื่อเข้าใกล้หินจะขยับสั้น ๆ แล้วรอกล้องยืนยันว่าหยุด หากหินอยู่ชิดปากคีบแต่เยื้องข้าง จะตรวจทางถอยก่อนหมุนจัดแนวใหม่
- แยกการตรวจหินในปากคีบจากพื้นที่บังตัวหุ่น และชดเชยภาพตัวหุ่นที่สูงจากพื้น เพื่อไม่ให้หลังคาหรือปากคีบกลายเป็นสิ่งกีดขวางปลอม อนุญาตให้ถอยออกห่างจากหินที่แตะขอบระยะเผื่อได้
- ตั้งค่าปิดปากคีบเป็น 70 องศาให้ตรงกัน หากเฟิร์มแวร์รายงานมุมไม่ตรงจะหยุดที่ `GRIP_BLOCKED`
- ค่า grip offset `[197, -4]` มม. และขอบหน้าหุ่น 235 มม. เป็นค่าประมาณที่ต้องวัดยืนยัน กล้องหรือสนามที่ขยับยังต้อง calibrate ใหม่ รายละเอียดอยู่ใน `PICKUP_RECOVERY.md`

## 2026-09-30: wall keep-out and wall recovery (PR #2 + fixes)

**EN**
- Merged PR #2 (Mickmocca): the planner keeps the robot away from the walls. A stone is chosen only if the robot's tag stays at least 200 mm and its body at least 20 mm from every wall, both at the staging point and at the pickup point. The park point is kept inside the same margin. `pose_timeout_s` 0.5 → 0.25 s. A missed tag frame now stops the wheels at once; the previous pose only keeps masking the robot in the image.
- New wall recovery (`wall_guard.py`, states `WALL_RECOVERY` and `WALL_BLOCKED`): when the tag comes within 200 mm of a wall (or a body corner within 20 mm, or the predicted pose would), the robot stops, waits until the camera shows it at rest, and moves back inside in short pulses, measuring after each one. It moves only with a fresh tag pose (no blind driving). Settings are the `wall_*` keys in `calib.json` → `autonomy`; see [WALL_RECOVERY.md](WALL_RECOVERY.md).
- Fixes on top of the PR: motion from before the stop (camera delay and coasting) no longer counts as "moving outward", which froze the robot for the rest of the run; pulses are long enough to move the robot (drive 0.15 s, turn 0.2 s at power 0.5); a turn next to a wall checks only the arc it may sweep, so a robot parallel to a wall can turn to face it and reverse out; `WALL_BLOCKED` tries again after 3 s instead of stopping for the whole run.
- `autonomy.py --show-full-frame` also shows the uncropped camera picture with the calibrated field outline.
- Simulator, 120 runs of 180 s (charged battery, min_duty 0.65, practice and competition layouts): 515 stones in the right zone vs 499 before, 72 wrong vs 84; no scenario significantly worse. With a coast model: 118 vs 101. The PR as submitted placed 461 and froze in 26 of 120 runs (54 of 120 with coasting). The simulator lets the robot slide along walls, so how recovery frees a robot on a real wall still needs a field check.
- Tests: 163 pass.

**TH**
- รวม PR #2 (Mickmocca): planner กันหุ่นไม่ให้เข้าใกล้ขอบสนาม จะเลือกหินก็ต่อเมื่อแท็กห่างขอบทุกด้านอย่างน้อย 200 mm และตัวหุ่นห่างอย่างน้อย 20 mm ทั้งที่จุดเตรียมและจุดคีบ จุดจอด (park) อยู่ในระยะเดียวกัน `pose_timeout_s` 0.5 → 0.25 s ถ้าเฟรมไหนไม่เห็นแท็ก ล้อหยุดทันที (pose เก่าใช้แค่บังตัวหุ่นในภาพ)
- ระบบพาหุ่นกลับจากขอบ (`wall_guard.py`, สถานะ `WALL_RECOVERY` และ `WALL_BLOCKED`): เมื่อแท็กเข้าใกล้ขอบภายใน 200 mm (หรือมุมตัวหุ่นภายใน 20 mm หรือ pose ที่คาดการณ์จะเข้าใกล้) หุ่นจะหยุด รอจนกล้องเห็นว่าหยุดนิ่ง แล้วขยับกลับเข้าสนามเป็นช่วงสั้น ๆ และวัดใหม่ทุกครั้ง ขยับเฉพาะเมื่อเห็นแท็กสด ๆ เท่านั้น (ไม่ขับแบบตาบอด) ค่าตั้งคือคีย์ `wall_*` ใน `calib.json` → `autonomy` อ่าน [WALL_RECOVERY.md](WALL_RECOVERY.md)
- แก้เพิ่มจาก PR: การเคลื่อนที่ที่เกิดก่อนหยุด (กล้องหน่วงและล้อไหลต่อ) ไม่ถูกนับว่า "ออกไปทางขอบ" อีกแล้ว (เดิมทำให้หุ่นค้างทั้งรอบ); ช่วงขยับยาวพอให้หุ่นขยับจริง (เดิน 0.15 s, หมุน 0.2 s ที่กำลัง 0.5); การหมุนข้างขอบตรวจเฉพาะส่วนโค้งที่หมุนจริง หุ่นที่ขนานกับขอบจึงหันหน้าเข้าขอบแล้วถอยออกได้; `WALL_BLOCKED` ลองใหม่หลัง 3 วินาที แทนการหยุดทั้งรอบ
- `autonomy.py --show-full-frame` แสดงภาพกล้องเต็มพร้อมกรอบสนามที่ calibrate
- ตัวจำลอง 120 รอบ รอบละ 180 s (แบตเต็ม, min_duty 0.65, สนามซ้อมและสนามแข่ง): วางถูกโซน 515 ก้อน เทียบ 499 ก่อนแก้ ผิด 72 เทียบ 84 ไม่มีกรณีไหนแย่ลงอย่างมีนัยสำคัญ เมื่อจำลองล้อไหลต่อหลังหยุด: 118 เทียบ 101 PR ตามที่ส่งมาวางได้ 461 และค้าง 26 จาก 120 รอบ (54 จาก 120 เมื่อมีล้อไหล) ตัวจำลองให้หุ่นไถลไปตามขอบได้ จึงยังต้องทดสอบในสนามจริงว่าหุ่นที่ติดขอบหลุดออกมาได้
- เทสต์: ผ่าน 163

## 2026-09-28: autonomy stage movement fix

**EN**
- Fixed forward steering near GOTO_STAGE so the planner doesn't reverse one wheel while trying to advance.
- If the gripper is already close to a stone, proceed to ALIGN and APPROACH instead of turning back toward a staging point behind the robot.
- Preserve one target while it's inside the configured robot footprint; return to SEARCH before selecting a new target and its approach direction.
- Show the stop reason, target lock state, stage distance, heading error and wheel command; save a per-frame trace and the runtime config under `runs/autonomy/<time>/`.
- Add `--check-config` and `--dry-run`; reject missing runtime calibration before real autonomy starts.
- Add 11 autonomy regressions. The root suite now contains 84 tests; simulation is not hardware validation.
- See [AUTONOMY_FIX_TH.md](AUTONOMY_FIX_TH.md) for symptoms, deployment and diagnostic steps.

**TH**
- แก้การเลี้ยวใกล้ GOTO_STAGE ให้ล้อทั้งสองข้างยังเดินหน้า แทนการกลับทิศล้อข้างหนึ่ง
- ถ้าปากคีบอยู่ใกล้หินแล้ว ให้เข้า ALIGN และ APPROACH แทนการหันกลับไปจุดเตรียมที่อยู่ด้านหลัง
- ล็อกเป้าหมายเดิมขณะหินอยู่ในกรอบตัวหุ่น เมื่อเป้าหมายหาย ให้กลับ SEARCH ก่อนเลือกหินและทิศทางเข้าใหม่
- แสดงเหตุผลที่หยุด, target lock, ระยะถึงจุดเตรียม, มุมคลาดและคำสั่งล้อ พร้อมบันทึก trace ทุกเฟรมและ config ที่ใช้งานไว้ใน `runs/autonomy/<time>/`
- เพิ่ม `--check-config` และ `--dry-run`; ตรวจ calibration ที่ขาดก่อนเริ่มโหมดหุ่นจริง
- เพิ่ม regression tests ของ autonomy 11 ข้อ รวมเป็น 84 tests; ผล simulation ไม่ใช่การทดสอบหุ่นจริง
- อ่าน [AUTONOMY_FIX_TH.md](AUTONOMY_FIX_TH.md) สำหรับวิธีอัปเดตและอ่านอาการ

## 2026-09-28: merge of the color-first update

**EN**
- Kept from the color-first update: new color sampler (measured lower *and* upper S/V limits, patch sampling, sample checks), `color_preview.py`, `reference_guard.py` (detects a moved camera or field), `camera_io.py`, manual zone labels, the `field/calib.json` profile, and its tests.
- Kept from our version: firmware (IDLE/RUNNING only, fixed IP, silent servo at boot, reset reason, `MIN_DUTY` 0.71, grip 0/40), `teleop.py`, `sim.py`, `fake_robot.py`, `stress_test.py`, `firmware_check.py`, autonomy grip defaults.
- `vision.py`: the stone size check is a setting again (`vision.size_check`, off in `field/calib.json`); `--debug` prints `hsv_median` again.
- `field/calib.json`: measured values ported (tag height 185, grip offset [270, 0], footprint front 300, `autonomy` grip 0/40, pile mode on). Camera 1810 mm at [1100, 600] kept from the update.
- `secrets.example.h` placeholders restored (it contained the real hotspot password).
- At the color-first merge: 73 root tests pass; gesture tests pass; `firmware_check.py` 15/15 on the fake robot. The later autonomy fix raises the root suite to 84 tests.

**TH**
- เก็บจาก color-first: เครื่องมือเก็บตัวอย่างสีแบบใหม่ (วัดขอบบนและล่างของ S/V), `color_preview.py`, `reference_guard.py` (ตรวจว่ากล้องหรือสนามขยับ), `camera_io.py`, ป้ายสีวงแบบกำหนดเอง, โปรไฟล์ `field/calib.json` และเทสต์
- เก็บจากเวอร์ชันของเรา: firmware (IDLE/RUNNING, IP คงที่, servo เงียบตอนเปิดเครื่อง, `MIN_DUTY` 0.71, ปากคีบ 0/40), teleop, ตัวจำลอง, fake robot, stress test, `firmware_check.py`
- `vision.py`: การเช็กขนาดหินเป็นตัวตั้งค่า (`vision.size_check` ปิดใน `field/calib.json`) และ `--debug` แสดง `hsv_median`
- ย้ายค่าที่วัดแล้วเข้า `field/calib.json` และคืนค่าตัวอย่างใน `secrets.example.h` (เดิมมีรหัส hotspot จริง)

## 2026-09-27

[English](#english) · [ภาษาไทย](#ภาษาไทย)

---

## English

### Added

- **`autonomy.py`**: the autonomous planner. It is a state machine that repeats one cycle: find a stone, drive to a point behind it, turn to face it, creep in, grip, carry it to its color's zone, release, and back off. It runs against the real robot (UDP v3) or the simulator (`--sim --show`).
- **`perception.py`**: combines stone detection and the robot's pose from the tag into one step, in arena millimetres. It hides the robot's footprint from detection, so the robot body no longer creates false blobs. It also corrects stone positions for their height (parallax).
- **`sim.py`**: a simulated robot and field. It copies the firmware's behavior (states, 300 ms watchdog, 5-minute timer, servos) and adds wheel movement, a gripper, zones and scoring. It can also add pose noise, missed tag frames, latency and failed grabs.
- **`fake_robot.py`**: a firmware stand-in on UDP port 4211, so `teleop.py` can be tested without the robot. The docs already referred to this file, but it was missing from the repo.
- **`calibrate_grip.py`**: measures `grip_offset_mm` (stone in the closed jaws) and `axle_offset_mm` (robot spins in place) with the overhead camera, and saves them with `--write`.
- **Tests:** `tests/test_autonomy.py` and `tests/test_perception.py`. The full suite is 58 tests, all passing.
- **Printable tag:** AprilTag 36h11, ID 0, with a 120 mm square and a 112 mm fallback, cut lines, a front arrow and a 100 mm check scale.

### Changed

- **`vision.py`**: the size check (`plausible`) and the all-six-colors check (`calibrated`) are back on. `process()` now accepts extra areas to hide, which is used for masking the robot. Detection results are otherwise unchanged.
- **`detect_live.py`**: hides the robot and draws its outline in magenta. `--no-robot-mask` turns this off. The `m` foreground-window key was removed.
- **`calib.json`**: the color ranges were adjusted to remove overlaps. Cyan and sky blue are now split by saturation (sky blue up to 165, cyan from 169), and violet ends at hue 169, below crimson's 170. `pile_mode`, `sticky_frames`, `min_obstacle_mm2` and `edge_margin_mm` are on.
- **`README.md`**: new autonomy section and pre-run checklist.
- **Lift removed everywhere:** the planner (no LIFT/LOWER states; CARRY → RELEASE), the simulator, the firmware (`SERVO_COUNT 1`, no `LIFT_*`, no `lift` command), teleop (`u`/`j` keys), and the gesture remote (thumb up/down no longer send anything). Stones are slid along the floor in the closed jaws.

### Fixed (found in simulation, before any real run)

- **Wrong-zone drop:** after a missed grab, the robot could open the gripper while standing inside another color's zone, which is a −1. A stone it isn't sure about is now carried to a spot outside every zone first. The gripper only opens over the correct zone.
- **Latency made grabs miss by up to 5 cm:** the final approach now follows the approach line through the stone, instead of steering relative to the robot's own heading.
- **Axle offset:** if the tag isn't above the wheel axle, turning in place slides the gripper sideways. With a 40 mm offset, pile runs dropped from 19 stones to 4. The planner now aims with the axle point, given `axle_offset_mm`.
- **Stage point oscillation:** the robot no longer circles a point that ends up behind it.

### Simulation results (5 minutes, zero wrong placements in every run)

| Condition | Pile start | Scattered |
| --- | --- | --- |
| Clean | 17–19 | 16–17 |
| Realistic noise | 15–16 | 14–16 |
| Harsh noise | 8–9 | 11–12 |

The simulator does not model the robot pushing stones, the real drive speed, or whether the jaws really hold each stone shape. Real tests must confirm these.

### Decisions

- **No lift.** The gripper is the "Robot Gripper 9g Micro Servo" design (SMT_M, Creality Cloud), which only opens and closes, with one SG90. Stones are slid along the floor to the zone. The lift code has been removed.
- **LeRobot is dropped.** `teleop.py` stays, for driving, bench tests and recording.
- **Vision is frozen** apart from recalibration.
- **`camera_properties` stays `{}`.** The FFMPEG backend can't read or set camera controls, and the colors were sampled with automatic settings.
- **Old path to remove:** `fake_esp32.py` and `firmware/esp_link/`. Nothing uses vision packets on port 4210 any more.

### Values still to fill

#### `firmware/robot_ctrl/config.h` (hardware team and bench tests)

| Variable | What it represents | How to get it |
| --- | --- | --- |
| `MOTOR_DRIVER` | Motor driver type: `DRIVER_IN_IN_PWM` (TB6612, L298N) or `DRIVER_TWO_PWM` (DRV8833, MX1508) | Driver board name or example code |
| `L_IN1`, `L_IN2`, `L_PWM`, `R_IN1`, `R_IN2`, `R_PWM` | ESP32 pins for the left and right motors | Hardware team |
| `MOTOR_STBY` | TB6612 standby pin, or `-1` | Hardware team |
| `L_INVERT`, `R_INVERT` | Flip a wheel that spins backwards | Press `w` in teleop and watch |
| `MAX_DUTY`, `MIN_DUTY` | Top motor power; the power where the wheels just start moving | Bench test (start with 0.60 / 0.00) |
| `SERVO_PINS` | Pin for the gripper servo | Hardware team |
| `GRIP_OPEN_DEG`, `GRIP_CLOSE_DEG` | Jaw angles, a few degrees short of the mechanical stops | Raw `servo` command |
| `SERVO_START_DEG`, `SERVO_MIN_DEG`, `SERVO_MAX_DEG` | Safe angle at power-on; mechanical limits | Bench test |
| `ESTOP_PIN` | Stop button: `0` for the BOOT button, or `-1` for none | Team decision |

#### `firmware/robot_ctrl/secrets.h`

| Variable | What it represents |
| --- | --- |
| `WIFI_SSID`, `WIFI_PASS` | The team's own hotspot, not the university Wi-Fi |

#### `calib.json`: written by scripts at the field

| Key | What it represents | Script |
| --- | --- | --- |
| `arena.corners_px`, `arena.size_mm` | 4 clicked points and their real distance apart. Verify `size_mm` (2300×1690 now) against a measured zone diameter and stone area | `calibrate_arena.py` |
| `background.png` | Empty-field reference image | `calibrate_arena.py` |
| `exclude_polygons`, `zones` | Hidden zone circles; each color's zone center in mm | `find_zones.py` |
| `robot_tag.grip_offset_mm` | Tag center → center of a stone held in the closed jaws, `[forward, right]` mm | `calibrate_grip.py --write` |
| `robot_tag.axle_offset_mm` | How far the tag center is ahead of the wheel axle | `calibrate_grip.py --axle --write` |

#### `calib.json`: measured by hand

| Key | What it represents | Now |
| --- | --- | --- |
| `robot_tag.size_mm` | Printed black square, measured with a ruler | 100 (guess) |
| `robot_tag.height_mm` | Floor → tag surface (±5 mm matters) | 150 (guess) |
| `robot_tag.camera_height_mm` | Floor → camera lens | **1815 (measured)** |
| `robot_tag.camera_floor_xy_mm` | Floor point directly under the lens (plumb line), in arena mm. `null` = arena center | [285, 975] (check) |
| `robot_tag.footprint_mm` | Robot outline from the tag center: front (open jaw tips + stone), back, left, right. Too big is harmless | 170/110/105/105 (guess) |
| `robot_tag.heading_offset_deg` | Correction if the tag's front doesn't point at the gripper. Facing image-right should read 0° | 0 (verify) |

#### `calib.json`: new `autonomy` block (defaults in `autonomy.py`)

| Key | What it represents |
| --- | --- |
| `grip_open`, `grip_close` | Must equal `GRIP_OPEN_DEG` and `GRIP_CLOSE_DEG` in `config.h` (the planner waits for these angles) |
| `cruise`, `creep`, `turn` | Drive commands. The defaults assume about 300 mm/s at full command; adjust after measuring |
| `approach_max_side_mm` | Allowed sideways miss at the grab: about half of (open jaw gap − 35 mm) |

#### `calib.json` → `vision` (from the real jaws)

| Key | What it represents |
| --- | --- |
| `gripper_width_mm` | Outer width of the open jaws; the free strip the robot needs to reach a stone |
| `approach_length_mm` | How long that free strip must be |

### Next

1. Print the tag. Get the servo pins, servo power and wheel-driver info from the hardware team.
2. Bench: fill `config.h`, flash, safety checks, grip angles, measure the jaws, 10 grabs per stone shape.
3. Field: calibrate → zones → verify scale → colors → camera point → tag rate → `calibrate_grip.py` (both modes) → drive speed → first `autonomy.py` run.
4. Round 1: gesture remote hardware and practice.

---

## ภาษาไทย

### เพิ่มใหม่

- **`autonomy.py`**: ตัววางแผนสำหรับโหมดอัตโนมัติ เป็น state machine ที่ทำซ้ำรอบเดิม คือหาหิน ขับไปจุดหลังหิน หันเข้าหา ค่อย ๆ เข้าไป หนีบ ลากไปวงสีของมัน ปล่อย แล้วถอยออก ใช้ได้ทั้งกับหุ่นจริง (UDP v3) และตัวจำลอง (`--sim --show`)
- **`perception.py`**: รวมการตรวจจับหินกับตำแหน่งหุ่นจาก tag ไว้ในขั้นเดียว เป็นหน่วยมิลลิเมตรบนสนาม และซ่อนพื้นที่ของตัวหุ่นก่อนตรวจจับ ตัวหุ่นจึงไม่สร้าง blob หลอกอีก นอกจากนี้ยังแก้ตำแหน่งหินจากความสูงของหิน (parallax)
- **`sim.py`**: หุ่นและสนามจำลอง ทำงานเหมือน firmware (state, watchdog 300 ms, จับเวลา 5 นาที, servo) และเพิ่มการเคลื่อนที่ของล้อ ปากคีบ วงสี กับการนับคะแนน ใส่ noise ของตำแหน่ง, เฟรมที่ tag หาย, ความหน่วง และการหนีบพลาดได้
- **`fake_robot.py`**: firmware จำลองบน UDP พอร์ต 4211 ใช้ทดสอบ `teleop.py` โดยไม่ต้องมีหุ่น (เอกสารเดิมอ้างถึงไฟล์นี้ แต่ไม่มีอยู่ใน repo)
- **`calibrate_grip.py`**: วัด `grip_offset_mm` (หนีบหินไว้) และ `axle_offset_mm` (หมุนหุ่นอยู่กับที่) ด้วยกล้องบนสนาม แล้วบันทึกด้วย `--write`
- **เทสต์:** `tests/test_autonomy.py` และ `tests/test_perception.py` รวมทั้งหมด 58 เทสต์ ผ่านทั้งหมด
- **Tag สำหรับพิมพ์:** AprilTag 36h11 ID 0 ขนาด 120 mm และสำรอง 112 mm มีเส้นตัด ลูกศรบอกด้านหน้า และสเกล 100 mm ไว้ตรวจ

### เปลี่ยนแปลง

- **`vision.py`**: เปิดการเช็กขนาด (`plausible`) และการเช็กว่ามีครบ 6 สี (`calibrated`) กลับมาแล้ว `process()` รับพื้นที่ที่ต้องซ่อนเพิ่มได้ (ใช้ซ่อนตัวหุ่น) ส่วนผลการตรวจจับอื่นเหมือนเดิม
- **`detect_live.py`**: ซ่อนตัวหุ่นและวาดขอบหุ่นเป็นสีม่วงชมพู ปิดได้ด้วย `--no-robot-mask` เอาปุ่ม `m` (หน้าต่าง foreground) ออก
- **`calib.json`**: ปรับช่วงสีไม่ให้ทับกัน cyan กับ sky blue แยกกันด้วยค่าความอิ่มสี (sky blue ถึง 165, cyan ตั้งแต่ 169) และ violet จบที่ hue 169 ต่ำกว่า crimson ที่เริ่ม 170 เปิด `pile_mode`, `sticky_frames`, `min_obstacle_mm2` และ `edge_margin_mm`
- **`README.md`**: เพิ่มหัวข้อ autonomy และเช็กลิสต์ก่อนรันจริง
- **เอาการยกแขนออกทั้งหมด:** planner (ไม่มี state LIFT/LOWER แล้ว; CARRY → RELEASE), ตัวจำลอง, firmware (`SERVO_COUNT 1`, ไม่มี `LIFT_*` และคำสั่ง `lift`), teleop (ปุ่ม `u`/`j`) และ gesture remote (ยกนิ้วโป้งขึ้น/ลงไม่ส่งคำสั่งแล้ว) หินถูกลากไปตามพื้นในปากคีบที่ปิดอยู่

### แก้ไข (เจอใน simulation ก่อนรันจริง)

- **ปล่อยหินผิดวง:** หลังหนีบพลาด หุ่นอาจเปิดปากคีบขณะยืนอยู่ในวงสีอื่น ซึ่งโดน −1 ตอนนี้หินที่ไม่แน่ใจจะถูกพาไปวางนอกทุกวงก่อน ปากคีบจะเปิดเฉพาะเหนือวงที่ถูกเท่านั้น
- **ความหน่วงทำให้หนีบพลาดได้ถึง 5 cm:** ช่วงเข้าหาหินตอนท้ายตอนนี้วิ่งตามเส้นแนวเข้าที่ผ่านหิน แทนการเลี้ยวตามทิศของตัวหุ่นเอง
- **ระยะ tag กับเพลาล้อ:** ถ้า tag ไม่อยู่เหนือเพลาล้อ การหมุนอยู่กับที่จะทำให้ปากคีบเลื่อนไปด้านข้าง ถ้าห่าง 40 mm รอบกองหินลดจาก 19 ก้อนเหลือ 4 ก้อน ตอนนี้ planner เล็งด้วยจุดเพลาล้อเมื่อรู้ค่า `axle_offset_mm`
- **วนรอบจุดเตรียม:** หุ่นไม่วนรอบจุดที่ไปอยู่ข้างหลังตัวเองอีก

### ผล simulation (5 นาที, ไม่มีการวางผิดวงเลยทุกรอบ)

| เงื่อนไข | เริ่มจากกองหิน | หินกระจาย |
| --- | --- | --- |
| ไม่มี noise | 17–19 | 16–17 |
| noise สมจริง | 15–16 | 14–16 |
| noise หนัก | 8–9 | 11–12 |

ตัวจำลองไม่ได้จำลองการที่หุ่นดันหิน ความเร็วจริงของหุ่น และการที่ปากคีบจับหินแต่ละทรงได้จริงหรือไม่ ต้องยืนยันด้วยการทดสอบจริง

### การตัดสินใจ

- **ไม่มีการยกแขน** ปากคีบใช้แบบ "Robot Gripper 9g Micro Servo" (SMT_M, Creality Cloud) ซึ่งเปิด-ปิดได้อย่างเดียว ใช้ SG90 ตัวเดียว หินจะถูกลากไปตามพื้นจนถึงวง เอาโค้ดส่วนยกแขนออกแล้ว
- **ตัด LeRobot ออก** ยังเก็บ `teleop.py` ไว้ใช้ขับ ทดสอบบนโต๊ะ และบันทึกวิดีโอ
- **ไม่แก้ vision อีก** นอกจาก calibrate ใหม่
- **`camera_properties` ให้เป็น `{}`** เพราะ backend FFMPEG อ่านหรือตั้งค่ากล้องไม่ได้ และเก็บตัวอย่างสีไว้ตอนกล้องตั้งค่าอัตโนมัติ
- **ของเก่าที่จะลบ:** `fake_esp32.py` และ `firmware/esp_link/` ไม่มีอะไรใช้ packet vision บนพอร์ต 4210 แล้ว

### ค่าที่ยังต้องกรอก

#### `firmware/robot_ctrl/config.h` (จากทีม hardware และการทดสอบบนโต๊ะ)

| ตัวแปร | ความหมาย | หาได้จาก |
| --- | --- | --- |
| `MOTOR_DRIVER` | ชนิด driver มอเตอร์: `DRIVER_IN_IN_PWM` (TB6612, L298N) หรือ `DRIVER_TWO_PWM` (DRV8833, MX1508) | ชื่อบอร์ด driver หรือโค้ดตัวอย่าง |
| `L_IN1`, `L_IN2`, `L_PWM`, `R_IN1`, `R_IN2`, `R_PWM` | ขา ESP32 ของมอเตอร์ซ้ายและขวา | ทีม hardware |
| `MOTOR_STBY` | ขา standby ของ TB6612 หรือ `-1` | ทีม hardware |
| `L_INVERT`, `R_INVERT` | กลับทิศล้อที่หมุนถอยหลัง | กด `w` ใน teleop แล้วดู |
| `MAX_DUTY`, `MIN_DUTY` | กำลังมอเตอร์สูงสุด และกำลังที่ล้อเริ่มหมุน | ทดสอบบนโต๊ะ (เริ่มที่ 0.60 / 0.00) |
| `SERVO_PINS` | ขาของ servo ปากคีบ | ทีม hardware |
| `GRIP_OPEN_DEG`, `GRIP_CLOSE_DEG` | องศาปากคีบ ให้ขาดจากจุดชนกลไกไม่กี่องศา | คำสั่ง `servo` แบบดิบ |
| `SERVO_START_DEG`, `SERVO_MIN_DEG`, `SERVO_MAX_DEG` | องศาที่ปลอดภัยตอนเปิดเครื่อง และขีดจำกัดกลไก | ทดสอบบนโต๊ะ |
| `ESTOP_PIN` | ปุ่มหยุด: `0` ใช้ปุ่ม BOOT หรือ `-1` ไม่ใช้ | ทีมตัดสินใจ |

#### `firmware/robot_ctrl/secrets.h`

| ตัวแปร | ความหมาย |
| --- | --- |
| `WIFI_SSID`, `WIFI_PASS` | hotspot ของทีมเอง ไม่ใช่ Wi-Fi มหาวิทยาลัย |

#### `calib.json`: สคริปต์เขียนให้ที่สนาม

| คีย์ | ความหมาย | สคริปต์ |
| --- | --- | --- |
| `arena.corners_px`, `arena.size_mm` | 4 จุดที่คลิก และระยะจริงระหว่างจุด ต้องตรวจ `size_mm` (ตอนนี้ 2300×1690) เทียบกับเส้นผ่านศูนย์กลางวงและพื้นที่หินที่วัดได้ | `calibrate_arena.py` |
| `background.png` | ภาพสนามว่าง ใช้เป็นภาพอ้างอิง | `calibrate_arena.py` |
| `exclude_polygons`, `zones` | วงกลมที่ซ่อนจากการตรวจจับ และจุดกลางวงของแต่ละสีเป็น mm | `find_zones.py` |
| `robot_tag.grip_offset_mm` | กลาง tag → กลางหินที่หนีบอยู่ `[ไปข้างหน้า, ไปทางขวา]` mm | `calibrate_grip.py --write` |
| `robot_tag.axle_offset_mm` | กลาง tag อยู่หน้าเพลาล้อเท่าไร | `calibrate_grip.py --axle --write` |

#### `calib.json`: วัดเอง

| คีย์ | ความหมาย | ตอนนี้ |
| --- | --- | --- |
| `robot_tag.size_mm` | ขนาดสี่เหลี่ยมดำที่พิมพ์ วัดด้วยไม้บรรทัด | 100 (เดา) |
| `robot_tag.height_mm` | พื้น → ผิว tag (คลาด ±5 mm ก็มีผล) | 150 (เดา) |
| `robot_tag.camera_height_mm` | พื้น → เลนส์กล้อง | **1815 (วัดแล้ว)** |
| `robot_tag.camera_floor_xy_mm` | จุดบนพื้นใต้เลนส์พอดี (ใช้ลูกดิ่ง) เป็น mm บนสนาม `null` = กลางสนาม | [285, 975] (ต้องเช็ก) |
| `robot_tag.footprint_mm` | ขอบหุ่นจากกลาง tag: หน้า (ปลายปากคีบตอนเปิด + หิน), หลัง, ซ้าย, ขวา ใหญ่ไปไม่เสียหาย | 170/110/105/105 (เดา) |
| `robot_tag.heading_offset_deg` | ค่าแก้ ถ้าด้านหน้าของ tag ไม่ได้ชี้ไปที่ปากคีบ หันไปทางขวาของภาพต้องได้ 0° | 0 (ต้องเช็ก) |

#### `calib.json`: บล็อก `autonomy` ใหม่ (ค่าเริ่มต้นอยู่ใน `autonomy.py`)

| คีย์ | ความหมาย |
| --- | --- |
| `grip_open`, `grip_close` | ต้องเท่ากับ `GRIP_OPEN_DEG` และ `GRIP_CLOSE_DEG` ใน `config.h` (planner รอจน servo ถึงองศานี้) |
| `cruise`, `creep`, `turn` | คำสั่งความเร็ว ค่าเริ่มต้นคิดจากประมาณ 300 mm/s ที่คำสั่งเต็ม ปรับหลังวัดความเร็วจริง |
| `approach_max_side_mm` | ระยะพลาดด้านข้างที่ยอมได้ตอนหนีบ ประมาณครึ่งหนึ่งของ (ช่องปากคีบตอนเปิด − 35 mm) |

#### `calib.json` → `vision` (จากปากคีบจริง)

| คีย์ | ความหมาย |
| --- | --- |
| `gripper_width_mm` | ความกว้างด้านนอกของปากคีบตอนเปิด คือแนวว่างที่หุ่นต้องใช้เข้าถึงหิน |
| `approach_length_mm` | แนวว่างนั้นต้องยาวเท่าไร |

### ขั้นต่อไป

1. พิมพ์ tag และขอข้อมูลขา servo, การจ่ายไฟ servo และ driver ล้อจากทีม hardware
2. บนโต๊ะ: กรอก `config.h`, flash, ทดสอบความปลอดภัย, หาองศาปากคีบ, วัดปากคีบ, หนีบหินทรงละ 10 ครั้ง
3. ที่สนาม: calibrate → หาวง → ตรวจสเกล → สี → จุดใต้กล้อง → อัตราการเจอ tag → `calibrate_grip.py` (ทั้งสองโหมด) → ความเร็วขับ → รัน `autonomy.py` จริงครั้งแรก
4. รอบที่ 1: เลือกอุปกรณ์ gesture remote และฝึกขับ
