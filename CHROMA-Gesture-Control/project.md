# CHROMA Gesture Control — Project Context

> **เลิกใช้แล้ว (1 ต.ค. 2026):** หน้านี้เป็นของระบบสองมือเดิม ระบบหลักตอนนี้คือคันโยกมือเดียว `lever_control.py` ดู [README_LEVER_TH.md](README_LEVER_TH.md) หน้านี้เก็บไว้สำรอง

สถานะ: 29 กันยายน 2026 (ควบคุมสองมือ, ตรงกับ `firmware/robot_ctrl` ปัจจุบัน)

## เป้าหมายและขอบเขต

ควบคุมหุ่นด้วยท่ามือจากกล้องโน้ตบุ๊ก/เว็บแคม และให้ผู้ใช้เก็บข้อมูลเพื่อเทรน
ตัวจำแนกท่าของตนเองได้ แพ็กเกจนี้มีเฉพาะ gesture control, เครื่องมือเก็บข้อมูล,
ตัวฝึกโมเดล, คู่มือ และ tests ไม่รวมการตรวจอัญมณี กล้องเหนือสนาม การหา robot pose,
planner, firmware, Wi-Fi credentials หรือข้อมูลฝึกส่วนตัว

เอกสารนี้เป็นบริบทและคู่มือโครงการ ไม่ใช่คำสั่งให้เชื่อมต่อหรือเริ่มเคลื่อนที่หุ่นอัตโนมัติ

## Architecture

```text
Operator webcam (mirrored image)
    -> pretrained MediaPipe Hand Landmarker: 21 landmarks
    -> wrist-relative, palm-size-normalized xyz: 63 features
    -> MediaPipe Gesture Recognizer canned gestures (default)
       OR custom MLP classifier OR original geometric rules
    -> confidence / top-two margin rejection -> UNKNOWN
    -> GestureControl: screen-half roles (right = drive, left = command),
       gears, neutral rearm, dwell, freshness checks
    -> preview HUD (default, no robot connection)
    -> optional UDP v3 -> ESP32 robot_ctrl firmware (IDLE / RUNNING)
```

MediaPipe ไม่ได้ถูกเทรนใหม่ ส่วนที่เราเทรนคือ MLP จำแนกท่าจากจุดมือ
MLP ใช้ชั้นซ่อน 64 และ 32 หน่วย ฝึกด้วย scikit-learn บน CPU
ส่งออก weights + StandardScaler + labels เป็น NPZ และใช้ NumPy ประมวลผลตอนใช้งาน
โมเดลนี้รองรับท่าค้าง ไม่รองรับการจำแนกลำดับโบก/ปัดมือ
การแปลตำแหน่งมือเป็นทิศขับและ state machine ยังเป็นตรรกะโปรแกรม

## Files

| File | Responsibility |
| --- | --- |
| gesture_control.py | Camera/inference worker, preview HUD, optional live controller |
| gesture_logic.py | Hand data, two-hand state machine, gears, UDP v3 transport |
| gesture_model.py | Shared features, confidence rejection, portable MLP loading/export |
| gesture_collect.py | Human-labeled recording, save/discard/undo/resume sessions |
| gesture_train.py | Data validation, session split, training, evaluation report |
| setup_gesture.ps1 | Create isolated environment, install packages, download hand model |
| requirements-gesture.txt | Direct dependency versions |
| README.md | Standalone quick start |
| README_GESTURE_TH.md | Control mappings and robot connection details |
| TRAIN_GESTURES_TH.md | Collection and training instructions |
| tests/test_gesture.py | Control state machine and UDP loopback tests |
| tests/test_gesture_training.py | Collection, features, split, training/export and application tests |

## Setup: Windows / Python 3.12

แตก ZIP ก่อน เปิด PowerShell ในโฟลเดอร์ CHROMA-Gesture-Control:

```powershell
.\setup_gesture.ps1
```

ต้องมีอินเทอร์เน็ตตอนติดตั้ง สคริปต์สร้าง `.venv-gesture` และดาวน์โหลดโมเดล
Google มาไว้ที่ `models/hand_landmarker.task` ZIP ไม่รวม Python environment,
โมเดลสำเร็จรูปที่ดาวน์โหลดได้ หรือโมเดลท่ามือส่วนตัว หลังติดตั้งใช้ประมวลผลในเครื่อง

## Workflow

### 1. Preview original rules

```powershell
.\.venv-gesture\Scripts\python.exe gesture_control.py --camera 0
```

### 2. Collect your data

```powershell
.\.venv-gesture\Scripts\python.exe gesture_collect.py --camera 0
```

เลือกคลาสด้วยเลข 1–8 เลือกมือด้วย H กด R มี countdown 2 วินาทีแล้วเก็บ 60 ตัวอย่าง
เก็บได้สูงสุด 5 ตัวอย่าง/วินาที บันทึกเฉพาะจุดมือ ไม่บันทึกภาพ/วิดีโอ ไม่อัปโหลด
S บันทึกและหยุด, D ทิ้ง take ที่กำลังเก็บ, U ยกเลิกไฟล์ล่าสุดโดยย้ายเข้า discarded,
Esc บันทึกส่วนที่เก็บแล้วและออก

เก็บครบทุกคลาส ทั้งมือซ้ายและขวา ในอย่างน้อย 5 sessions ที่มีความต่างจริงของมุม/ระยะ/แสง
แต่ละ session ตั้งเป้า 60 ตัวอย่าง/คลาส/มือ (16 takes); ตัวฝึกตรวจขั้นต่ำ 30 ต่อคลาสต่อมือ
ข้อมูลมือเดียวชุดเดิมย้ายไป `gesture_data_v1/` (โมเดลเดิม `models/gesture_mlp.npz`) เก็บไว้สำรอง
เริ่ม session ใหม่ด้วยการปิด–เปิดโปรแกรม หากต้องการเก็บต่อ session เดิม:

```powershell
.\.venv-gesture\Scripts\python.exe gesture_collect.py --session SESSION_ID
.\.venv-gesture\Scripts\python.exe gesture_train.py --inspect
```

### 3. Train and inspect results

```powershell
.\.venv-gesture\Scripts\python.exe gesture_train.py
```

ได้ `models/gesture_v2.npz` และ `models/gesture_v2.report.json` (รายงานแยกผลมือซ้าย/ขวาด้วย)
แบ่งเป็น train/validation/test ตาม session ทั้งกลุ่ม: 5 sessions แบ่ง 3/1/1
Scaler และน้ำหนักเรียนรู้จาก train เท่านั้น ปรับค่าตัดสินจาก validation
รายงานรวม confusion matrix, recall แต่ละคลาส, wrong active commands,
UNKNOWN-to-active rate และผลกฎเดิมบนข้อมูลชุดเดียวกัน
หากนำผล test ไปปรับโมเดลอีก ต้องเก็บ test ใหม่ที่ไม่เคยใช้
ตัวฝึกไม่เขียนทับโมเดลเดิม ให้ใช้ `--output models/gesture_v3.npz`

### 4. Preview your model

```powershell
.\.venv-gesture\Scripts\python.exe gesture_control.py --camera 0 --classifier models/gesture_v2.npz
```

หน้าจอต้องขึ้น PREVIEW | MLP ถ้าไม่ระบุ classifier จะขึ้น RULES
หากระบุโมเดลแล้วโหลดไม่ได้ โปรแกรมหยุด ไม่แอบย้อนกลับไปใช้กฎ

## Labels and control

ที่มาของท่า: ค่าเริ่มต้น MediaPipe Gesture Recognizer (`models/gesture_recognizer.task`,
คะแนน < `--min-score` 0.7 = UNKNOWN), หรือ `--classifier` MLP ที่เทรนเอง, หรือ `--rules`

จอแบ่งครึ่ง: มือครึ่งขวา = มือขับ, ครึ่งซ้าย = มือสั่งงาน (`--swap-hands` สลับ)
ตัดสินจากตำแหน่งบนจอ ไม่ใช้ handedness ของ MediaPipe ทั้งสองมือทำงานพร้อมกันได้

| Label | มือขับ (ขวา) | มือสั่งงาน (ซ้าย) |
| --- | --- | --- |
| OPEN | ในกรอบ 0.5 s = พร้อม; เลื่อนขึ้น/ลง/ซ้าย/ขวา = ขับ | ท่าพัก 0.3 s ก่อนสั่งทุกครั้ง |
| FIST | หยุดล้อ | ค้าง 0.5 s: `stop` → IDLE (ไม่ต้องพักก่อน) |
| V | หยุดล้อ | ค้าง 0.8 s: `start` → RUNNING |
| ONE | หยุดล้อ | ค้าง 0.4 s: `grip open` |
| THREE / ILOVEYOU 🤟 | หยุดล้อ | ค้าง 0.4 s: `grip close` (MediaPipe ไม่มี THREE ใช้ 🤟) |
| THUMB_UP | หยุดล้อ | ค้าง 0.4 s: เพิ่มเกียร์ |
| THUMB_DOWN | หยุดล้อ | ค้าง 0.4 s: ลดเกียร์ |
| UNKNOWN | หยุดล้อ | ไม่ทำอะไร |

เกียร์เริ่มต้น `--gears 0.15,0.35,0.6` (duty ≈ 0.75 / 0.81 / 0.88) เริ่มที่เกียร์ 1
firmware แปลง 0<|cmd|≤1 เป็น duty `MIN_DUTY 0.71`–1.0 แบบเส้นตรง จึงเป็นความเร็วต่างกันจริง
ระหว่าง PAUSED มือสั่งงานใช้ได้แค่ V และ FIST; สองมืออยู่ครึ่งเดียวกัน → หยุดทุกอย่าง
คีย์บอร์ด: G = start, Space = พักล้อฝั่ง PC, X = stop (IDLE), Esc = ออก
ทิศขับอ้างอิงหัวหุ่น ไม่ใช่พิกัดสนาม; ภาพกล้องเป็นภาพสะท้อน

## Robot interface

ใช้กับ `firmware/robot_ctrl` ใน repo นี้ (ไม่ใช่ `esp_link` v2) แพ็กเกจนี้ไม่รวม firmware
firmware ปัจจุบันมีสถานะแค่ **IDLE / RUNNING** ไม่มี DONE, ESTOP ค้าง, reset หรือตัวจับเวลา 5 นาที

```powershell
.\.venv-gesture\Scripts\python.exe gesture_control.py --camera 0 --live
```

`--robot` ค่าเริ่มต้น `10.178.188.50` (IP คงที่ใน `config.h`) UDP port 4211 ส่ง drive ~20 Hz
firmware รับผู้ส่งทีละราย ผู้ส่งใหม่ต้องรอให้ตัวเดิมเงียบเกิน 300 ms: ปิด teleop/autonomy ก่อน

แพ็กเก็ต JSON พื้นฐาน:

```json
{"v":3,"s":"012345abcdef","q":1,"c":"drive","l":0.0,"r":0.0}
```

- `s`: session ID ใหม่ต่อการเปิดโปรแกรม เป็น hex ยาว 12 ตัว; `q`: sequence เพิ่มขึ้นทุกแพ็กเก็ต
- `drive`: l/r ในช่วง -1..1 รับเฉพาะใน RUNNING; ศูนย์ = หยุด
- `start`: IDLE → RUNNING; `stop`: → IDLE (ไม่ค้างสถานะ)
- `grip`: p เป็น open (0°) / close (40°) ทำงานได้ทั้ง IDLE และ RUNNING; ไม่มี `lift`
- `servo`: มุมดิบสำหรับ calibrate (ตัวควบคุมนี้ไม่ใช้); คำสั่งอื่นแค่ต่ออายุ link
- สถานะตอบกลับทุก 200 ms: `state`, `why`, `l`, `r`, `rx_age_ms`, `rssi`, `servo` (list, ใช้ `servo[0]`)
- โปรแกรมรอสถานะ RUNNING ก่อนอนุญาตให้มือขับส่งคำสั่งเคลื่อนที่
- ทดลองโดยไม่มีหุ่น: `python fake_robot.py` ที่ root ของ repo แล้ว `--live --robot 127.0.0.1`

## Safety behavior and limitations

- ภาพ/ผลมืออายุเกิน 200 ms, ไม่มีมือขับ, มือขับข้ามเส้นกลาง หรือสองมือครึ่งเดียวกัน -> หยุดล้อและต้องกลับท่ากลาง
- Camera/inference แยก thread เพื่อให้ตัวควบคุมตรวจ timeout ได้แม้ camera read ค้าง
- สถานะหุ่นหายเกิน 600 ms หรือไม่ใช่ RUNNING -> พัก ต้องกด G / V ใหม่
- firmware: drive watchdog 300 ms และหยุดล้อเมื่อ Wi-Fi หลุด
- **ไม่มีปุ่มหยุดฉุกเฉินบนหุ่นและไม่มีตัวจับเวลาหมดรอบ** ใน firmware ปัจจุบัน
  `stop` ผ่านเครือข่ายคือทางหยุดเดียวจากซอฟต์แวร์ ต้องมีคนพร้อมยกหุ่น/ตัดไฟ
- ไม่มั่นใจ (<0.9) หรือคะแนนอันดับหนึ่งห่างอันดับสอง <0.2 -> UNKNOWN โดย default
- คะแนน softmax ไม่ใช่ความแม่นยำที่ calibrate แล้ว และอาจมั่นใจผิดกับท่าใหม่ได้
- หยุดล้อ/มือหายไม่ยกเลิกการเคลื่อนเซอร์โวที่สั่งไปแล้ว
- UDP อาจสูญหาย มุม servo ที่หุ่นรายงานคือค่าที่ firmware สั่ง ไม่ใช่มุมที่วัดจริง
- ตรวจอายุจากก่อน camera read ไม่ได้ตรวจภาพเก่าที่ driver ส่งซ้ำด้วยเวลาใหม่
- UNKNOWN training ต้องมีมือที่ตรวจพบ กรณีไม่พบมือถูกจัดการนอก classifier
- รอบถ่ายใหม่ไม่ได้เท่ากับผู้ใช้ใหม่ ต้องประเมินผู้ควบคุม/สภาพจริงที่ต้องการรองรับ

## Validation and current status

```powershell
.\.venv-gesture\Scripts\python.exe -m unittest discover -s tests -p "test_gesture*.py" -v
```

37 tests ผ่าน (29 ก.ย. 2026) รวม UDP loopback, การแบ่งมือตามครึ่งจอ, เกียร์,
คำสั่งพร้อมกันสองมือ, start/stop จากท่ามือ, collector ป้ายมือ, split ที่ต้องครบทั้งสองมือ
และ train/export/load บนข้อมูลสังเคราะห์
ทดสอบ `gesture_control.py --live` กับ `fake_robot.py` ด้วยท่ามือจำลองผ่าน:
V → RUNNING, ขับเกียร์ 1 (0.15) → 2 (0.35), ปิดคีบระหว่างขับ (servo 40), FIST → IDLE

ใช้ MediaPipe canned model เป็นค่าเริ่มต้น; ภาพตัวอย่างของ Google อ่านถูก 4/4
ข้อมูลสองมือที่เริ่มเก็บถูกลบแล้ว ยังไม่มี `models/gesture_v2.npz` (การเทรนเองเป็นทางเลือก)
ยังไม่ได้ยืนยันกับกล้องจริง, MediaPipe กับสองมือพร้อมกัน, latency, ระยะหยุด หรือหุ่นจริง

## Next steps

1. ทดลอง `gesture_control.py` (MediaPipe) ใน preview ด้วยเว็บแคมจริง ทั้งสองมือ
2. ปรับ `--min-score` ถ้ามี UNKNOWN มากไป (ลด) หรือคำสั่งหลุดผิด (เพิ่ม)
3. ถ้าท่าไหนอ่านไม่ได้จริง ค่อยเทรน MLP เองตาม TRAIN_GESTURES_TH.md
4. ตรวจท่าที่สับสนและท่าช่วงเปลี่ยน โดยเฉพาะตอนสองมืออยู่ใกล้เส้นกลาง
5. ทดสอบกับ `fake_robot.py` แล้วหุ่นจริงโดยยกล้อพ้นพื้นก่อนลงสนาม

## References

- https://developers.google.com/edge/mediapipe/solutions/vision/hand_landmarker/python
- https://scikit-learn.org/stable/modules/generated/sklearn.neural_network.MLPClassifier.html

แพ็กเกจภายนอกและโมเดลที่ดาวน์โหลดมีเงื่อนไขสิทธิ์ของเจ้าของแต่ละราย
