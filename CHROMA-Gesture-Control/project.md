# CHROMA Gesture Control — Project Context

สถานะ: 25 กันยายน 2026

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
    -> custom MLP classifier OR original geometric rules
    -> confidence / top-two margin rejection -> UNKNOWN
    -> GestureControl: mode, neutral rearm, dwell, freshness checks
    -> preview HUD (default, no robot connection)
    -> optional UDP v3 -> existing ESP32 robot_ctrl firmware
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
| gesture_logic.py | Hand data, gesture state machine, UDP v3 transport |
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

เลือกคลาสด้วยเลข 1–8 กด R มี countdown 2 วินาทีแล้วเก็บ 60 ตัวอย่าง
เก็บได้สูงสุด 5 ตัวอย่าง/วินาที บันทึกเฉพาะจุดมือ ไม่บันทึกภาพ/วิดีโอ ไม่อัปโหลด
S บันทึกและหยุด, D ทิ้ง take ที่กำลังเก็บ, U ยกเลิกไฟล์ล่าสุดโดยย้ายเข้า discarded,
Esc บันทึกส่วนที่เก็บแล้วและออก

เก็บครบทุกคลาสในอย่างน้อย 5 sessions ที่มีความต่างจริงของมุม/ระยะ/แสง
แต่ละ session ตั้งเป้า 60–100 ตัวอย่าง/คลาส; ตัวฝึกตรวจขั้นต่ำ 30
เริ่ม session ใหม่ด้วยการปิด–เปิดโปรแกรม หากต้องการเก็บต่อ session เดิม:

```powershell
.\.venv-gesture\Scripts\python.exe gesture_collect.py --session SESSION_ID
.\.venv-gesture\Scripts\python.exe gesture_train.py --inspect
```

### 3. Train and inspect results

```powershell
.\.venv-gesture\Scripts\python.exe gesture_train.py
```

ได้ `models/gesture_mlp.npz` และ `models/gesture_mlp.report.json`
แบ่งเป็น train/validation/test ตาม session ทั้งกลุ่ม: 5 sessions แบ่ง 3/1/1
Scaler และน้ำหนักเรียนรู้จาก train เท่านั้น ปรับค่าตัดสินจาก validation
รายงานรวม confusion matrix, recall แต่ละคลาส, wrong active commands,
UNKNOWN-to-active rate และผลกฎเดิมบนข้อมูลชุดเดียวกัน
หากนำผล test ไปปรับโมเดลอีก ต้องเก็บ test ใหม่ที่ไม่เคยใช้
ตัวฝึกไม่เขียนทับโมเดลเดิม ให้ใช้ `--output models/gesture_v2.npz`

### 4. Preview your model

```powershell
.\.venv-gesture\Scripts\python.exe gesture_control.py --camera 0 --classifier models/gesture_mlp.npz
```

หน้าจอต้องขึ้น PREVIEW | MLP ถ้าไม่ระบุ classifier จะขึ้น RULES
หากระบุโมเดลแล้วโหลดไม่ได้ โปรแกรมหยุด ไม่แอบย้อนกลับไปใช้กฎ

## Labels and control

| Label | Example | Function |
| --- | --- | --- |
| OPEN | แบมือ | กลางกรอบเพื่อพร้อมควบคุม; ใน DRIVE เลื่อนมือเพื่อขับ |
| FIST | กำมือ | หยุดล้อ |
| V | นิ้วชี้+กลาง | ค้าง 0.8 วินาทีสลับ DRIVE / ARM |
| ONE | นิ้วชี้ | เปิดคีบใน ARM |
| THREE | นิ้วชี้+กลาง+นาง | ปิดคีบใน ARM |
| THUMB_UP | โป้งขึ้น | ยกใน ARM |
| THUMB_DOWN | โป้งลง | ลดใน ARM |
| UNKNOWN | ท่าอื่น/ระหว่างเปลี่ยนท่า | ไม่ออกคำสั่งเคลื่อนที่ใหม่ |

G เริ่ม/กลับมาควบคุม; แบมือกลางกรอบ 0.5 วินาทีเพื่อ enable
คำสั่งแขนค้าง 0.4 วินาทีและต้องกลับท่ากลางก่อนคำสั่งต่อไป
Space พักล้อ, X ESTOP, R reset, Esc ออก
ทิศขับอ้างอิงหัวหุ่น ไม่ใช่พิกัดสนาม; ภาพกล้องเป็นภาพสะท้อน
ใช้มือเดียวและทดสอบทุกท่ากับแสงจริง

## Robot interface (external firmware required)

ใช้ได้กับ `robot_ctrl` v3 ใน CPE-Exploratory3-main.zip ที่ตรวจไว้
ไม่ใช่ `esp_link` v2 แพ็กเกจนี้ไม่รวม firmware และไม่ใช่ระบบหุ่นครบชุด
ต้องตั้งขามอเตอร์ มุมเซอร์โว compile/flash และตรวจระบบหยุดบนฮาร์ดแวร์ก่อน

เมื่อพร้อมทดสอบหุ่น จึงระบุ `--robot IP --live` เพิ่มเอง:

```powershell
.\.venv-gesture\Scripts\python.exe gesture_control.py --camera 0 --classifier models/gesture_mlp.npz --robot 192.168.1.50 --live --speed 0.25
```

แทน IP ด้วยที่อยู่หุ่น UDP port 4211 โดย default ตัวควบคุมส่ง drive ประมาณ 20 Hz
ห้ามเปิด controller อีกตัวส่งแข่งกัน; speed เป็นสัดส่วนคำสั่ง ไม่ใช่ความเร็วที่วัดได้

แพ็กเก็ต JSON พื้นฐาน:

```json
{"v":3,"s":"012345abcdef","q":1,"c":"drive","l":0.0,"r":0.0}
```

- `s`: session ID ใหม่ต่อการเปิดโปรแกรม เป็น hex ยาว 12 ตัว
- `q`: sequence เพิ่มขึ้นทุกแพ็กเก็ต
- `drive`: l/r ในช่วง -1..1; ล้อศูนย์เป็นหยุดปกติ
- `start`, `stop`, `reset`: คำสั่ง state; stop เข้า ESTOP แบบค้าง
- `grip`: p เป็น open/close; `lift`: p เป็น up/down
- firmware ต้องตอบสถานะกลับ IP/port ผู้ส่ง มี `state` เป็น IDLE/RUNNING/DONE/ESTOP
- โปรแกรมรอสถานะ RUNNING ก่อนอนุญาตส่งคำสั่งเคลื่อนที่จากมือ

## Safety behavior and limitations

- ภาพ/ผลมือเก่ากว่า 200 ms, ไม่มีมือ หรือหลายมือ -> หยุดล้อและต้องกลับท่ากลาง
- Camera/inference แยก thread เพื่อให้ตัวควบคุมตรวจ timeout ได้แม้ camera read ค้าง
- สถานะหุ่นหายเกิน 600 ms หรือไม่ใช่ RUNNING -> พัก ต้องกด G ใหม่
- firmware ที่เข้าคู่กันมี drive watchdog 300 ms, timer 5 นาที และปุ่มหยุดบนหุ่น
- ไม่มั่นใจ (<0.9) หรือคะแนนอันดับหนึ่งห่างอันดับสอง <0.2 -> UNKNOWN โดย default
- คะแนน softmax ไม่ใช่ความแม่นยำที่ calibrate แล้ว และอาจมั่นใจผิดกับท่าใหม่ได้
- หยุดล้อ/มือหายไม่ยกเลิกการเคลื่อนเซอร์โวที่สั่งไปแล้ว; X ขอ ESTOP ผ่านเครือข่าย
- UDP อาจสูญหายและไม่มีการยืนยันว่าแขนถึงตำแหน่งจริง ต้องคงปุ่มหยุดบนหุ่นไว้
- ตรวจอายุจากก่อน camera read ไม่ได้ตรวจภาพเก่าที่ driver ส่งซ้ำด้วยเวลาใหม่
- UNKNOWN training ต้องมีมือที่ตรวจพบ กรณีไม่พบมือถูกจัดการนอก classifier
- รอบถ่ายใหม่ไม่ได้เท่ากับผู้ใช้ใหม่ ต้องประเมินผู้ควบคุม/สภาพจริงที่ต้องการรองรับ

## Validation and current status

```powershell
.\.venv-gesture\Scripts\python.exe -m unittest discover -s tests -p "test_gesture*.py" -v
```

29 tests ผ่านใน environment ที่พัฒนา รวม UDP loopback, timeout/rearm,
collector save/undo/resume ผ่านกล้องและหน้าจอจำลอง, session split,
MLP train/export/load บนข้อมูลสังเคราะห์ และการรับสถานะหลัง poll
MediaPipe hand model เคยโหลดและประมวลผลภาพว่างผ่านแล้ว

ยังไม่มีชุดข้อมูลมือจริงของผู้ใช้หรือโมเดล MLP สำหรับใช้งานที่ฝึกจากข้อมูลนั้น
ยังไม่ได้ยืนยันความแม่นยำกล้องจริง latency ระยะหยุด หรือการควบคุมหุ่นจริง
ผลสังเคราะห์ตรวจความถูกต้องของซอฟต์แวร์ ไม่ใช่หลักฐานว่าแม่นกว่า Teachable Machine

## Next steps

1. ติดตั้งบนเครื่องปลายทางและเลือกกล้องผู้ควบคุม
2. เก็บข้อมูลจริง 5 sessions ให้ครบทุกคลาส
3. เทรน อ่าน validation report และทดลอง MLP ใน preview
4. ตรวจท่าที่สับสนและท่าช่วงเปลี่ยน เก็บข้อมูลเพิ่มตามที่พบ
5. ทดสอบบนหุ่นที่ตั้งค่าพร้อมแล้ว เริ่มล้อยกพ้นพื้นก่อนลงสนาม

## References

- https://developers.google.com/edge/mediapipe/solutions/vision/hand_landmarker/python
- https://scikit-learn.org/stable/modules/generated/sklearn.neural_network.MLPClassifier.html

แพ็กเกจภายนอกและโมเดลที่ดาวน์โหลดมีเงื่อนไขสิทธิ์ของเจ้าของแต่ละราย
