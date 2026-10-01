# CHANGELOG: CHROMA gemstone-sorting robot (CPE102)

## 2026-10-01 (night, later): gesture UI: ERA-ONE name, left/right hand tag fixed

**EN**
- Both gesture screens and window titles say **ERA-ONE** with an elephant 🐘 (drawn with an emoji font through Pillow: colour on Windows/macOS, a tinted outline on this Linux laptop, text only if no emoji font exists).
- The L/R tag on each hand (driver dashboard, trainer warning) was reversed on the team's webcam: raising the left hand showed R. MediaPipe's handedness is now swapped (`hand_camera.operator_side`). Only the tag changes: commands, recorded data and the screen picture are unaffected.

**TH**
- หน้าจอ gesture ทั้งสองและชื่อหน้าต่างเปลี่ยนเป็น **ERA-ONE** พร้อมช้าง 🐘 (วาดด้วยฟอนต์อีโมจิผ่าน Pillow: สีบน Windows/macOS, เส้นขอบสีเดียวบนโน้ตบุ๊ก Linux นี้, ถ้าไม่มีฟอนต์อีโมจิจะแสดงแค่ตัวหนังสือ)
- ป้าย L/R ของแต่ละมือ (หน้าจอขับ, คำเตือนในโปรแกรมเก็บข้อมูล) กลับข้างบนเว็บแคมของทีม: ยกมือซ้ายแล้วขึ้น R ตอนนี้สลับค่าจาก MediaPipe แล้ว (`hand_camera.operator_side`) เปลี่ยนแค่ป้าย คำสั่ง ข้อมูลที่อัดไว้ และภาพบนจอไม่เปลี่ยน

## 2026-10-01 (night): gesture control rebuilt: two hands, one trained command set

**EN**
- `CHROMA-Gesture-Control/` starts over. The one-hand lever (`lever_*.py`, its docs, tests and screenshots) and the old two-hand system (drive hand in a centre box + command hand, MediaPipe canned gestures) are removed; git history keeps them.
- Commands = the classifier's classes: `NONE`, `STOP`, `FORWARD`, `BACK`, `LEFT`, `RIGHT`, `GRIP_OPEN`, `GRIP_CLOSE`. **Either hand gives the same commands**, so the driver can swap hands. Each hand is classified separately, then combined: no fresh hand / only `NONE` → stop at once; any `STOP` → stop; two different commands → stop (`CONFLICT`); otherwise the one command shown. No centre box: the pose alone is the command. Motion needs 0.15 s steady, a grip 0.4 s and fires once (3 UDP copies, wheels at zero).
- `+` / `-` change the speed by 0.1 in 0.1..1.0 (as `teleop.py`), at once; turns use 0.6 × speed. `--min-duty` floor, the start/RUNNING/session gate and the 300 ms firmware watchdog work as before (`gesture_link.py`, ported from the lever).
- Trainer `gesture_collect.py`: one session per run with a fixed `--split train|validation|test`; counts per command and hand for the session and the whole split, "ready to train?" per split, next-missing (N), undo, a warning when the camera sees the other hand, and a live model check (M). Landmarks only.
- `gesture_train.py`: fits on train sessions, picks the confidence gate on validation (fewest dangerous mistakes, i.e. a wrong motion or grip command, then most commands recognised), scores test once: accuracy, recall per command, NONE rate, dangerous mistakes, per hand, confusion table. New timestamped model each time (`models/gesture_commands_<time>.npz`); the apps take the newest.
- `gesture_control.py`: 1280 × 720 dashboard: camera with both hand skeletons and their labels, big command tile with hold bar, speed gauge, wheel bars, per-hand status, robot status. Preview by default; `--live --robot IP` sends; `--demo` = keyboard hands.
- `fake_robot.py` now reports `session` in its status like the firmware (`robot_ctrl.ino`), so session-checking controllers can run against it.
- Checked: 51 gesture tests (logic, session gate, apps, trainer, train/validate/test on synthetic sessions); the webcam + MediaPipe worker runs on this laptop (~17 fps, no hand in view); against `fake_robot.py`: start → RUNNING, FORWARD 0.5/0.5, LEFT −0.3/0.3, GRIP CLOSE moves the servo, no hand → 0/0, `+` → 0.6. **No model trained yet, not tested with real hands or the robot.**

**TH**
- เริ่ม `CHROMA-Gesture-Control/` ใหม่ ลบคันโยกมือเดียว (`lever_*.py` พร้อมเอกสาร เทสต์ และภาพ) และระบบสองมือแบบเก่า (มือขับในกรอบกลาง + มือสั่งงาน, ท่าสำเร็จรูปของ MediaPipe) ยังกู้คืนได้จากประวัติ git
- คำสั่ง = คลาสของตัวจำแนก: `NONE`, `STOP`, `FORWARD`, `BACK`, `LEFT`, `RIGHT`, `GRIP_OPEN`, `GRIP_CLOSE` **มือไหนก็สั่งได้เหมือนกัน** สลับมือได้เมื่อเมื่อย จำแนกทีละมือแล้วรวมกัน: ไม่เห็นมือ/เห็นแต่ `NONE` → หยุดทันที, มือไหนทำ `STOP` → หยุด, สองมือทำคำสั่งต่างกัน → หยุด (`CONFLICT`), นอกนั้นใช้คำสั่งที่เห็น ไม่มีกรอบกลางแล้ว ท่ามืออย่างเดียวคือคำสั่ง คำสั่งขับต้องนิ่ง 0.15 s คำสั่งหนีบต้องนิ่ง 0.4 s และส่งครั้งเดียว (ส่ง UDP 3 ชุด ล้อเป็นศูนย์)
- `+` / `-` ปรับความเร็วทีละ 0.1 ในช่วง 0.1..1.0 (เหมือน `teleop.py`) มีผลทันที เลี้ยวใช้ 0.6 × ความเร็ว `--min-duty`, การรอสถานะ RUNNING/session และ watchdog 300 ms ของ firmware ทำงานเหมือนเดิม (`gesture_link.py` ยกมาจากคันโยก)
- โปรแกรมเก็บข้อมูล `gesture_collect.py`: เปิดหนึ่งครั้ง = หนึ่ง session กำหนด `--split train|validation|test` ตายตัว แสดงจำนวนต่อคำสั่งต่อมือของ session และของทั้งชุด, บอกว่าแต่ละชุดพร้อมเทรนหรือยัง, ไปคำสั่งที่ขาด (N), ย้อน take, เตือนเมื่อกล้องเห็นเป็นอีกมือ และลองโมเดลสด (M) เก็บแค่ landmark
- `gesture_train.py`: สอนด้วยชุด train, เลือกเกณฑ์ความมั่นใจด้วยชุด validation (คำสั่งผิดที่อันตราย คือขับผิดหรือหนีบผิด น้อยที่สุดก่อน แล้วจำคำสั่งได้มากที่สุด), วัดชุด test ครั้งเดียว: accuracy, recall ต่อคำสั่ง, สัดส่วน NONE, คำสั่งผิดอันตราย, แยกมือ, ตาราง confusion ได้ไฟล์โมเดลใหม่พร้อมเวลาทุกครั้ง (`models/gesture_commands_<เวลา>.npz`) โปรแกรมใช้ไฟล์ล่าสุด
- `gesture_control.py`: หน้าจอ 1280 × 720: ภาพกล้องพร้อมโครงมือทั้งสองข้างและป้ายคำสั่ง, ช่องคำสั่งใหญ่พร้อมแถบนับเวลานิ่ง, มาตรวัดความเร็ว, แถบล้อ, สถานะแต่ละมือ, สถานะหุ่น ค่าเริ่มต้นเป็น preview; `--live --robot IP` ส่งจริง; `--demo` = มือจำลองจากคีย์บอร์ด
- `fake_robot.py` รายงาน `session` ใน status แบบเดียวกับ firmware (`robot_ctrl.ino`) โปรแกรมที่ตรวจ session จึงทดสอบกับมันได้
- ตรวจแล้ว: เทสต์ gesture 51 ข้อ (ตรรกะ, การรอ session, แอป, โปรแกรมเก็บข้อมูล, train/validate/test บนข้อมูลสังเคราะห์); กล้องเว็บแคม + MediaPipe ทำงานบนโน้ตบุ๊กนี้ (~17 fps, ไม่มีมือในภาพ); กับ `fake_robot.py`: start → RUNNING, FORWARD 0.5/0.5, LEFT −0.3/0.3, GRIP CLOSE ขยับเซอร์โว, ไม่เห็นมือ → 0/0, `+` → 0.6 **ยังไม่มีโมเดลที่เทรนแล้ว ยังไม่ได้ทดสอบกับมือจริงหรือหุ่นจริง**

## 2026-10-01 (evening): V3 = V2 + grip check by the gripper camera (autonomy3.py)

**EN**
- New hardware: a HuskyLens 1 (the *gripper camera*) on a fixed tilt looks into the closed jaws. Object classification (colour recognition was too unreliable), trained on the device: ID1–2 empty jaws, ID3–4 one stone (a stone touching the tips of the closed jaws counts: they push it), ID5–6 two stones. `autonomy.grip_check_ids` maps IDs to verdicts if the training order differs.
- Firmware (`2e8abdd`): HuskyLens on Serial2, RX GPIO 25 / TX 32, 115200 baud, board 5 V. Status `gripcam` ok/none; `{"c":"look","n":N}` waits 150 ms, then returns 5 class IDs from distinct camera frames (gives up after 700 ms) in status `look`. Own non-blocking parser (the Arduino library waits up to 100 ms per reply). V1/V2 never send `look`. Compiled; parser host-tested against the protocol document's byte examples; **not bench-tested**.
- Also committed (`0e83e4a`): the team's `config.h`: 360 gripper servo, `GRIP_CLOSE_DEG` 250, Mick's hotspot IP `172.20.10.2`. `calib.json` and `minifield/calib_minifield.json` `grip_close` 250 (working copies), `firmware_check.py --grip-close` default 250 and a 1.8 s wait (a 250° close takes ~1.25 s).
- Planner, V3 only (`grip_check` true): after the jaws close, GRIP asks once (re-sent after 0.4 s if the firmware never saw it). **Empty** (5 of 5 readings) → open, back off, skip that spot (a pushed stone is retried at its new spot). **Single / Multiple** (4 of 5) or anything else, or no answer in 1.5 s → carry exactly as V2, **overhead pick check included**. Trace keys `grip_verdict`, `grip_check_ids`, `grip_check_s`.
- Changed during the build, against the grilled plan (Q21: skip the pick check on Single): in the simulator that raised wrong placements from 0 to 11 (robot stopping short) and from 8 to 23 (overshoot model). The camera counts stones but cannot see colour; a neighbour of another colour in the jaws is exactly what the pick check catches. With the pick check kept: 0 and 5 wrong.
- `autonomy3.py` refuses to start unless the firmware reports `gripcam: ok` and tells you to run `autonomy2.py` or `--set grip_check=false`. `profiles.py`: v3 = v2 + `grip_check`; V1 and V2 set it false explicitly. `sim_bench.py --v3`; the simulator answers `look` from the true held state.
- `firmware_check.py <ESP_IP> --look N`: go/no-go for the gripper camera: N looks each with a stone in the jaws, a stone at the jaw tips, and nothing. **FAIL if any stone is read as Empty → run V2.**
- Simulator, 6 seeds × scattered + pile × 180 s, scratch eval (models for the two field failures are eval-only, not in `sim.py`): normal grabs V2 128 / 0 wrong vs V3 125 / 0 (the 0.35 s look per grip); robot stops short V2 52 / 0 vs V3 53 / 0; overshoot (50 % of grips push the stone 60 mm aside) V2 90 / 8 vs V3 77 / 5. The simulator lets closed jaws pick up a stone they meet while carrying, which rewards V2's empty carries; whether V3's early exit pays off on the real robot is not shown by it.
- Tests: 224 pass (22 new: `tests/test_v3.py`). Not bench- or field-tested.

**TH**
- ฮาร์ดแวร์ใหม่: HuskyLens 1 (*กล้องที่ปากคีบ*) ติดแบบเอียงตายตัว มองลงไปในปากคีบที่หุบแล้ว ใช้โหมด object classification (การจำสีไม่น่าเชื่อถือพอ) สอนบนตัวกล้อง: ID1–2 ปากคีบว่าง, ID3–4 หินหนึ่งก้อน (หินที่ชิดปลายปากคีบที่หุบแล้วนับด้วย เพราะปากคีบดันมันไปด้วย), ID5–6 หินสองก้อน ถ้าสอนคนละลำดับให้ตั้ง `autonomy.grip_check_ids`
- firmware (`2e8abdd`): HuskyLens ต่อ Serial2, RX GPIO 25 / TX 32, 115200 baud, ไฟ 5 V จากบอร์ด status มี `gripcam` ok/none คำสั่ง `{"c":"look","n":N}` รอ 150 ms แล้วส่ง ID 5 ค่าจากคนละเฟรม (เลิกรอหลัง 700 ms) ใน status `look` ใช้ตัวอ่านโปรโตคอลที่เขียนเองไม่บล็อก loop (ไลบรารี Arduino รอได้ถึง 100 ms ต่อคำตอบ) V1/V2 ไม่เคยส่ง `look` คอมไพล์ผ่าน ทดสอบตัวอ่านบน PC กับตัวอย่างไบต์ในเอกสารโปรโตคอลแล้ว **ยังไม่ได้ทดสอบบนบอร์ดจริง**
- commit ด้วย (`0e83e4a`): `config.h` ของทีม: เซอร์โว 360 องศา, `GRIP_CLOSE_DEG` 250, IP ฮอตสปอตของ Mick `172.20.10.2` และ `grip_close` 250 ใน `calib.json` กับ `minifield/calib_minifield.json` (ไฟล์ทำงาน), `firmware_check.py --grip-close` ค่าเริ่มต้น 250 และรอ 1.8 s (หุบ 250° ใช้ ~1.25 s)
- planner เฉพาะ V3 (`grip_check` true): หลังหุบปากคีบ GRIP ถามกล้องหนึ่งครั้ง (ส่งซ้ำหลัง 0.4 s ถ้า firmware ไม่ได้รับ) **ว่าง** (ตรงกัน 5 จาก 5) → อ้าปากคีบ ถอย ข้ามจุดนั้น (หินที่ถูกดันไปจะถูกลองใหม่ที่ตำแหน่งใหม่) **หนึ่งก้อน / หลายก้อน** (4 จาก 5) หรืออย่างอื่น หรือไม่ตอบใน 1.5 s → ขนไปเหมือน V2 ทุกอย่าง **รวมการตรวจจากกล้องด้านบน (pick check)** trace มี `grip_verdict`, `grip_check_ids`, `grip_check_s`
- เปลี่ยนระหว่างสร้าง ต่างจากแผนที่ตกลงกัน (Q21: ข้าม pick check เมื่อได้หนึ่งก้อน): ในตัวจำลองทำให้วางผิดเพิ่มจาก 0 เป็น 11 (หุ่นหยุดก่อนถึงหิน) และจาก 8 เป็น 23 (แบบจำลองไถลเลย) กล้องนับจำนวนหินได้แต่มองสีไม่ออก หินข้างเคียงสีอื่นในปากคีบคือสิ่งที่ pick check จับได้ เมื่อคง pick check ไว้: ผิด 0 และ 5
- `autonomy3.py` ไม่ยอมเริ่มถ้า firmware ไม่รายงาน `gripcam: ok` และบอกให้ใช้ `autonomy2.py` หรือ `--set grip_check=false` `profiles.py`: v3 = v2 + `grip_check` V1 และ V2 ตั้งเป็น false ชัดเจน `sim_bench.py --v3` ตัวจำลองตอบ `look` จากสถานะจริงว่าคีบอะไรอยู่
- `firmware_check.py <ESP_IP> --look N`: ตรวจผ่าน/ไม่ผ่านของกล้องที่ปากคีบ ถาม N ครั้งในแต่ละแบบ: หินในปากคีบ, หินชิดปลายปากคีบ, ว่าง **ถ้ามีหินถูกอ่านเป็นว่างแม้ครั้งเดียว = ไม่ผ่าน → ใช้ V2**
- ตัวจำลอง 6 seed × กระจาย + กอง × 180 s (สคริปต์ชั่วคราว แบบจำลองความผิดพลาดสองแบบในสนามไม่ได้อยู่ใน `sim.py`): คีบปกติ V2 128 / ผิด 0 เทียบ V3 125 / 0 (เสียเวลาถาม 0.35 s ต่อการคีบ), หุ่นหยุดก่อนถึง V2 52 / 0 เทียบ V3 53 / 0, ไถลเลย (50% ของการคีบดันหินออกข้าง 60 mm) V2 90 / 8 เทียบ V3 77 / 5 ตัวจำลองยอมให้ปากคีบที่หุบแล้วเก็บหินที่ชนระหว่างขนได้ ซึ่งให้รางวัลกับการขนเปล่าของ V2 ตัวเลขนี้จึงบอกไม่ได้ว่าการออกเร็วของ V3 คุ้มบนหุ่นจริงหรือไม่
- เทสต์: ผ่าน 224 (ใหม่ 22 ข้อ: `tests/test_v3.py`) ยังไม่ได้ทดสอบบนบอร์ดหรือในสนาม

## 2026-10-01: V1 / V2 split (autonomy.py / autonomy2.py)

**EN**
- Two versions on one `calib.json` (never written): `python autonomy.py ...` = **V1**, the field-tested behaviour, unchanged; `python autonomy2.py ...` = **V2**. Same flags. V2 is a set of switches in `profiles.py`, applied in memory before `--set` (so `--set` can turn any of them back). V1 sets the same switches explicitly to their legacy values. The run's `config.json` records `"profile"` and every switch; V2 run folders end in `-v2`; the window and console show the version.
- V2 vision: `vision.pile_edge_pixels` `"nearest"`: the pile fix of PILE_FIX_PLAN.md (a stone's pale rim belongs to the nearest stone within `own_reach_mm` 15; aim point = core + rim centroid; confidence fraction × dominance as before). The default stays `"legacy"`, so V1 is unchanged (the plan's Q6 default "nearest" is replaced by V1/V2).
- V2 vision, new: `vision.pile_outermost`: if no stone of a pile has a free corridor, the stone farthest from the pile's centre whose own exit line (`outermost_width_mm` 30, about one stone) is clear gets a target, approached toward the pile's centre. Only that pile may lie in its gripper-wide corridor; walls, the robot and other objects still block it. One per pile, confidence halved. `vision.pile_regions`: every stone-like region of a pile is also reported as a coloured, not pickable observation (`reason` `pile_buried`); the pile's "?" blob stays (for the crash code). Pickable pile stones have `reason` `pile_edge` / `pile_outermost`.
- V2 planner: `autonomy.commit_target`: a locked stone stays locked while a stone of its colour is seen at its spot, even when vision no longer offers it as a target; a neighbour of another colour no longer breaks the lock (`TargetLock(track_observations=True)`, lock reason `seen`). `autonomy.skip_alone_s` 6: a stone skipped after a failed attempt is retried after 6 s instead of 25 s when it is the only stone on offer (instead of parking).
- `--set vision.KEY=VALUE` for the named vision switches (`pile_edge_pixels`, `own_reach_mm`, `pile_outermost`, `pile_regions`); typos and bad values are rejected. `detect_live.py --v2` previews V2 vision (magenta = outermost); `sim_bench.py --v2` runs V2's planner switches.
- V1 unchanged, checked: vision replay of 0917, 0920, 0922 (3090 frames) hash-identical to before, also with the V2 switches set under `pile_edge_pixels` `"legacy"`; simulator event logs (8 seeds × scattered/pile) hash-identical.
- V2 vision, replay of the same 3090 frames: targets 2057 → 2980, frames without a target 1747 → 1332, **0 V1 targets lost**. New targets: 853 frames of 8 distinct edge stones, 46 frames of 3 outermost stones, 24 frames of one stone on the V1 path (steadier tracking). Crop sheets in `evidence/v2-replay/` (script `compare_v1_v2.py`): every one is a real stone with an approach away from its neighbours. A first version of the outermost rule picked a buried middle stone when the real end stones failed the size check; the exit-line rule fixes that (`test_outermost_never_picks_a_buried_stone`).
- V2 planner, simulator (48 seeds × 180 s, charged battery; the simulator never runs `vision.py`): pile 177 correct / 88 wrong → 179 / 77, scattered 206 / 9 → 203 / 9: no difference beyond noise. `skip_alone_s` never triggered there (always other stones). Neither number says anything about the vision switches.
- Tests: 202 pass (26 new: `tests/test_pile.py`, `tests/test_v2.py`). Not field-tested.

**TH**
- มีสองเวอร์ชันที่ใช้ `calib.json` ตัวเดียวกัน (ไม่มีการเขียนทับ): `python autonomy.py ...` = **V1** พฤติกรรมเดิมที่ทดสอบในสนามแล้ว ไม่เปลี่ยน และ `python autonomy2.py ...` = **V2** ใช้ flag เหมือนกัน V2 คือชุดสวิตช์ใน `profiles.py` ที่ตั้งในหน่วยความจำก่อน `--set` (จึงใช้ `--set` ปิดทีละข้อได้) V1 ตั้งสวิตช์ชุดเดียวกันเป็นค่าเดิมอย่างชัดเจน `config.json` ของแต่ละรอบบันทึก `"profile"` และสวิตช์ทุกตัว โฟลเดอร์รอบของ V2 ลงท้ายด้วย `-v2` หน้าจอและคอนโซลแสดงเวอร์ชัน
- vision ของ V2: `vision.pile_edge_pixels` `"nearest"` คือการแก้กองหินตาม PILE_FIX_PLAN.md (ขอบซีดของหินเป็นของหินก้อนที่ใกล้ที่สุดภายใน `own_reach_mm` 15, จุดเล็ง = จุดศูนย์กลางของแกนสี + ขอบ, ความมั่นใจ = fraction × dominance เหมือนเดิม) ค่าเริ่มต้นยังเป็น `"legacy"` V1 จึงไม่เปลี่ยน (ค่าเริ่มต้น "nearest" ตามข้อ Q6 ของแผนถูกแทนด้วยการแยก V1/V2)
- vision ของ V2 ใหม่: `vision.pile_outermost` ถ้าไม่มีหินก้อนไหนในกองมีทางเข้าว่าง หินที่ไกลจากกลางกองที่สุดซึ่งมีแนวทางออกของตัวเองว่าง (`outermost_width_mm` 30 ประมาณหนึ่งก้อน) จะเป็นเป้า เข้าหาโดยวิ่งเข้าหากลางกอง ในทางเข้ากว้างเท่าก้ามยอมให้มีได้แค่กองนั้น ขอบสนาม ตัวหุ่น และวัตถุอื่นยังกั้นอยู่ หนึ่งก้อนต่อกอง ความมั่นใจลดครึ่ง `vision.pile_regions` รายงานหินแต่ละก้อนในกองเป็น observation ที่มีสีแต่หยิบไม่ได้ (`reason` `pile_buried`) blob "?" ของกองยังอยู่ (สำหรับโค้ดพุ่งชน) หินในกองที่หยิบได้มี `reason` `pile_edge` / `pile_outermost`
- planner ของ V2: `autonomy.commit_target` หินที่ล็อกแล้วยังล็อกอยู่ตราบที่ยังเห็นหินสีเดียวกันที่ตำแหน่งนั้น แม้ vision ไม่เสนอเป็นเป้าแล้ว และหินสีอื่นที่อยู่ติดกันไม่ทำให้หลุดล็อก (`TargetLock(track_observations=True)`, lock reason `seen`) `autonomy.skip_alone_s` 6 หินที่ถูกข้ามหลังพลาดจะถูกลองใหม่หลัง 6 s แทน 25 s ถ้าเป็นหินก้อนเดียวที่มี (แทนการไปจอดรอ)
- `--set vision.KEY=VALUE` สำหรับสวิตช์ vision ที่มีชื่อ (`pile_edge_pixels`, `own_reach_mm`, `pile_outermost`, `pile_regions`) พิมพ์ผิดหรือค่าผิดจะถูกปฏิเสธ `detect_live.py --v2` ดู vision ของ V2 (สีม่วงชมพู = หินก้อนนอกสุด) `sim_bench.py --v2` รันสวิตช์ planner ของ V2
- ตรวจแล้วว่า V1 ไม่เปลี่ยน: เล่นวิดีโอ 0917, 0920, 0922 (3090 เฟรม) ได้ hash เหมือนก่อนแก้ แม้ตั้งสวิตช์ V2 ไว้ภายใต้ `pile_edge_pixels` `"legacy"` และ event log ของตัวจำลอง (8 seed × scattered/pile) hash เหมือนเดิม
- vision ของ V2 บน 3090 เฟรมเดียวกัน: เป้า 2057 → 2980 เฟรมที่ไม่มีเป้า 1747 → 1332 **เป้าของ V1 ไม่หายเลย** เป้าใหม่: 853 เฟรมจากหินขอบกอง 8 ก้อน, 46 เฟรมจากหินนอกสุด 3 ก้อน, 24 เฟรมจากหิน 1 ก้อนที่มาจากเส้นทางเดิมของ V1 (ติดตามได้นิ่งขึ้น) ภาพตัดอยู่ใน `evidence/v2-replay/` (สคริปต์ `compare_v1_v2.py`) ทุกภาพเป็นหินจริง ทิศเข้าห่างจากหินข้างเคียง กฎหินนอกสุดรุ่นแรกเลือกหินกลางที่ถูกล้อมไว้เมื่อหินปลายจริงไม่ผ่านการเช็กขนาด กฎแนวทางออกแก้ปัญหานี้ (`test_outermost_never_picks_a_buried_stone`)
- planner ของ V2 ในตัวจำลอง (48 seed × 180 s แบตเต็ม ตัวจำลองไม่เรียก `vision.py`): กองหิน ถูก 177 / ผิด 88 → 179 / 77, กระจาย 206 / 9 → 203 / 9 ต่างกันไม่เกินสัญญาณรบกวน `skip_alone_s` ไม่เคยทำงานในตัวจำลอง (มีหินอื่นเสมอ) ตัวเลขนี้บอกอะไรเกี่ยวกับสวิตช์ vision ไม่ได้
- เทสต์: ผ่าน 202 (ใหม่ 26 ข้อ: `tests/test_pile.py`, `tests/test_v2.py`) ยังไม่ได้ทดสอบในสนาม

## 2026-10-01: one-hand virtual lever is the gesture system (PR #3)

**EN**
- Merged PR #3 (`codex/gesture-virtual-lever-v1`), no conflicts: `CHROMA-Gesture-Control/lever_control.py`, a one-hand lever. Move a fist to drive or turn; thumb up, then move the hand up/down to close/open the gripper; open palm, a lost hand or two hands stop the wheels. Uses MediaPipe's built-in gestures (no training).
- The lever is now the gesture system. The two-hand system (`gesture_control.py`, data collection, MLP training) stays in the folder as a backup; its docs carry a "superseded" note. `gesture_control.py`, `gesture_logic.py`, `gesture_model.py` and `models/` must stay: the lever uses their camera reader, UDP protocol and MediaPipe model.
- New in the lever: `+` / `-` change the speed by 0.05 (0.02–1.00) at once, like `teleop.py`. `--min-duty X` sends the drive floor with every drive packet, like `autonomy.py --set min_duty=X`; without it the firmware's 0.71 is used as before. With `--min-duty`, firmware that does not report `min_duty` stops control. The panel shows the PWM and floor in use.
- Fixed the README's stale gripper angles (the firmware decides: open 0, close 100) and the `.gitignore` entry in `MANIFEST.sha256`.
- Tests: gesture 75 pass (68 from the PR + 7 new), root 176 pass. Software only: no webcam, robot or motors used.

**TH**
- รวม PR #3 (`codex/gesture-virtual-lever-v1`) ไม่มี conflict: `CHROMA-Gesture-Control/lever_control.py` คันโยกมือเดียว กำมือแล้วโยกเพื่อขับ/หมุน ยกนิ้วโป้งแล้วโยกขึ้น/ลงเพื่อหนีบ/ปล่อย แบมือ มือหาย หรือเห็นสองมือ ล้อหยุด ใช้ท่ามือสำเร็จรูปของ MediaPipe (ไม่ต้องเทรน)
- คันโยกเป็นระบบควบคุมด้วยมือหลักแล้ว ระบบสองมือ (`gesture_control.py`, การเก็บข้อมูล, การเทรน MLP) ยังอยู่ในโฟลเดอร์เป็นสำรอง เอกสารเดิมมีป้าย "เลิกใช้แล้ว" ห้ามลบ `gesture_control.py`, `gesture_logic.py`, `gesture_model.py` และ `models/` เพราะคันโยกใช้ตัวอ่านกล้อง โปรโตคอล UDP และโมเดล MediaPipe จากไฟล์เหล่านี้
- เพิ่มในคันโยก: `+` / `-` เพิ่ม/ลดความเร็วทีละ 0.05 (0.02–1.00) มีผลทันที เหมือน `teleop.py` และ `--min-duty X` ส่ง floor ไปกับทุก drive packet เหมือน `autonomy.py --set min_duty=X` ถ้าไม่ใส่ใช้ 0.71 ของ firmware เหมือนเดิม ถ้าใส่ `--min-duty` แต่ firmware ไม่รายงาน `min_duty` โปรแกรมหยุดเอง หน้าจอแสดง PWM และ floor ที่ใช้
- แก้มุมก้ามที่ล้าสมัยใน README (firmware เป็นตัวกำหนด: เปิด 0 หนีบ 100) และรายการ `.gitignore` ใน `MANIFEST.sha256`
- เทสต์: gesture ผ่าน 75 (68 จาก PR + ใหม่ 7), root ผ่าน 176 ทดสอบเฉพาะซอฟต์แวร์ ไม่ได้ใช้เว็บแคม หุ่น หรือมอเตอร์จริง

## 2026-10-01: mini practice field, new arm servo

**EN**
- Firmware `config.h` for the remounted arm: servo pin 33, pulse widths swapped (2500 us at 0 deg, 500 us at 180 deg), `SERVO_MAX_DEG` 125, `GRIP_CLOSE_DEG` 100. Needs a reflash. `autonomy.grip_close` must be 100 too.
- New `minifield/calib_minifield.json` for the ~1650 x 1100 mm practice field with only a red and a green zone; steps in [minifield/README.md](minifield/README.md). Jaw tips 175 mm ahead of the tag (`footprint_mm.front`), grip offset estimated at 140 mm (measure with a ruler), `approach_max_side_mm` 25, `min_duty` 0.65. Camera height and floor point are `null` until measured (the camera is not centred); the run refuses to start while they are null.
- `zone_colors` (top level, default all six): the scoring zones a field has. `find_zones.py`, `calibrate_arena.py` and the start check (`--check-config`) use it instead of a fixed six. HSV ranges are required only for zone colours and aliased colours.
- `autonomy.color_alias` (default empty): a colour with no zone of its own is delivered to another colour's zone, e.g. `{"1": 3}` = violet to red. Colours with neither are never picked, only avoided (as before). Off per run: `--set color_alias={}`. The simulator scores an aliased delivery as wrong, as the competition would.
- Tests: 176 pass (13 new in `tests/test_field_zones.py`).

**TH**
- เฟิร์มแวร์ `config.h` สำหรับแขนที่ติดตั้งใหม่: servo ขา 33, สลับความกว้างพัลส์ (2500 us ที่ 0 องศา, 500 us ที่ 180 องศา), `SERVO_MAX_DEG` 125, `GRIP_CLOSE_DEG` 100 ต้องแฟลชใหม่ และ `autonomy.grip_close` ต้องเป็น 100 ด้วย
- เพิ่ม `minifield/calib_minifield.json` สำหรับสนามซ้อมขนาดประมาณ 1650 x 1100 mm ที่มีแค่โซนแดงและโซนเขียว ขั้นตอนอยู่ใน [minifield/README.md](minifield/README.md) ปลายปากคีบอยู่หน้าแท็ก 175 mm (`footprint_mm.front`), grip offset ประมาณ 140 mm (ควรวัดด้วยไม้บรรทัด), `approach_max_side_mm` 25, `min_duty` 0.65 ความสูงกล้องและจุดบนพื้นใต้เลนส์เป็น `null` จนกว่าจะวัด (กล้องไม่ได้อยู่กลางสนาม) ถ้ายังเป็น null โปรแกรมจะไม่ยอมเริ่ม
- `zone_colors` (ระดับบนสุด ค่าเริ่มต้นครบหกสี): สีของโซนที่มีในสนาม `find_zones.py`, `calibrate_arena.py` และการตรวจก่อนเริ่ม (`--check-config`) ใช้ค่านี้แทนการบังคับหกโซน ต้องมีช่วง HSV เฉพาะสีที่มีโซนและสีที่ alias
- `autonomy.color_alias` (ค่าเริ่มต้นว่าง): สีที่ไม่มีโซนของตัวเองจะถูกส่งไปโซนของสีอื่น เช่น `{"1": 3}` = ม่วงไปโซนแดง สีที่ไม่มีทั้งโซนและ alias จะไม่ถูกหยิบ แค่หลบ (เหมือนเดิม) ปิดเฉพาะรอบได้ด้วย `--set color_alias={}` ตัวจำลองนับการส่งแบบ alias ว่าผิดโซน เหมือนกติกาการแข่ง
- เทสต์: ผ่าน 176 (ใหม่ 13 ข้อใน `tests/test_field_zones.py`)

## 2026-10-01: faster vision, same results

**EN**
- `vision.py`: the per-pixel background difference takes the largest of the three colour channels with `np.maximum` instead of `.max(axis=2)` (5.9 → 0.3 ms per frame).
- `find_zones.py`: the colour median of each zone is taken over the zone's bounding box instead of a full-image mask. The reference guard runs this on every frame.
- Perception per frame on the development laptop: 23.5 → 14.0 ms. Replaying four recorded runs (0628, 0917, 0920, 0922) gives identical output frame by frame: every observation, target, diagnostic and foreground pixel. The field laptop took ~44 ms per frame (~20 fps), so decisions should now be based on fresher frames; there is no change in behaviour.
- Tests: 163 pass.

**TH**
- `vision.py`: การหาความต่างจากภาพพื้นหลังใช้ `np.maximum` เลือกค่ามากสุดของสามช่องสี แทน `.max(axis=2)` (5.9 → 0.3 ms ต่อเฟรม)
- `find_zones.py`: หาค่ามัธยฐานสีของแต่ละโซนเฉพาะในกรอบรอบวงกลม แทนการสร้าง mask ทั้งภาพ (reference guard เรียกฟังก์ชันนี้ทุกเฟรม)
- เวลาประมวลผลภาพต่อเฟรมบนเครื่องพัฒนา: 23.5 → 14.0 ms ผลลัพธ์เหมือนเดิมทุกเฟรมเมื่อเล่นวิดีโอที่บันทึกไว้ 4 รอบ (0628, 0917, 0920, 0922): ทุก observation, target, diagnostic และพิกเซล foreground เครื่องที่ใช้ในสนามเคยใช้ ~44 ms ต่อเฟรม (~20 fps) ตอนนี้การตัดสินใจจึงใช้ภาพที่ใหม่กว่า พฤติกรรมไม่เปลี่ยน
- เทสต์: ผ่าน 163

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
