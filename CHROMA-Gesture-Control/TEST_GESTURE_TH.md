# เช็กลิสต์ทดสอบ gesture control

> **เลิกใช้แล้ว (1 ต.ค. 2026):** หน้านี้เป็นของระบบสองมือเดิม ระบบหลักตอนนี้คือคันโยกมือเดียว `lever_control.py` ดู [README_LEVER_TH.md](README_LEVER_TH.md) หน้านี้เก็บไว้สำรอง

ทำตามลำดับ ผ่านขั้นหนึ่งก่อนค่อยไปขั้นถัดไป ทุกคำสั่งรันในโฟลเดอร์ `CHROMA-Gesture-Control`
(บน Windows ใช้ `.\.venv-gesture\Scripts\python.exe` แทน `python`)

## 1. ซอฟต์แวร์ (ไม่ต้องมีกล้องหรือหุ่น)

```bash
python -m unittest discover -s tests -p "test_gesture*.py" -v
```

- [ ] ขึ้น `OK` ครบ 37 tests

## 2. กล้องจริง ไม่ต่อหุ่น

```bash
python gesture_control.py            # ถ้ากล้องผิดตัว: --camera 1
```

- [ ] หัวจอขึ้น `PREVIEW | MEDIAPIPE`
- [ ] บรรทัด `Hands:` อ่านทุกท่าถูก **ทั้งสองมือ**: ✋ OPEN, ✊ FIST, ✌ V, ☝ ONE, 🤟 ILOVEYOU, 👍 THUMB_UP, 👎 THUMB_DOWN
- [ ] ทำท่าครั้งละ 5 วินาที ไม่กระพริบเป็นท่าอื่น (UNKNOWN บ้างได้)
- [ ] มือซ้ายขึ้น `L:` มือขวาขึ้น `R:`
- [ ] ✌ ซ้าย → `ENABLED`, ✋ ขวาในกรอบ → `READY`, เลื่อน 4 ทิศ → FORWARD / BACK / LEFT / RIGHT
- [ ] 👍 / 👎 เปลี่ยน `GEAR`; ✊ ซ้าย → `PAUSED`
- [ ] เอามือขวาลง → `NO FRESH DRIVE HAND`
- [ ] ทดสอบในห้อง/แสงเดียวกับวันแข่ง

ถ้า UNKNOWN เยอะไป ลอง `--min-score 0.6`; ถ้าคำสั่งหลุดเอง ลอง `--min-score 0.8`
ถ้าท่าไหนอ่านไม่ได้จริง ใช้ `--rules` หรือเทรนเองตาม TRAIN_GESTURES_TH.md

## 3. หุ่นจำลอง (ไม่ต้องมีหุ่นจริง)

```bash
# เทอร์มินัล 1 ที่ root ของ repo
python fake_robot.py
# เทอร์มินัล 2 ใน CHROMA-Gesture-Control
python gesture_control.py --live --robot 127.0.0.1
```

- [ ] ✌ ซ้าย → เทอร์มินัล 1 ขึ้น `IDLE -> RUNNING (start)`
- [ ] ขับเดินหน้า → `wheels +0.15 +0.15` (เกียร์ 1), 👍 → `+0.35`
- [ ] 🤟 → `servo [40]`, ☝ → `servo [0]`
- [ ] ✊ ซ้าย → `RUNNING -> IDLE (remote stop)`
- [ ] ปิด fake_robot ระหว่างขับ → จอขึ้น `Robot status lost` ภายใน ~0.6 วินาที

## 4. หุ่นจริง ยกล้อพ้นพื้น

ก่อนเริ่ม: flash `firmware/robot_ctrl` แล้ว, หุ่นกับ PC อยู่ hotspot ทีมเดียวกัน, ปิด teleop/autonomy

```bash
python gesture_control.py --live     # IP ค่าเริ่มต้น 10.178.188.50
```

- [ ] หัวจอขึ้น `LIVE` และ `robot IDLE servo=...` (ถ้า `no status` = ต่อไม่ติด ตรวจ IP/Wi-Fi)
- [ ] ✌ ซ้าย → `robot RUNNING`, ไฟ LED บนบอร์ดติด
- [ ] ขับ 4 ทิศ ล้อหมุนถูกทาง (ผิดทาง: แก้ `L_INVERT/R_INVERT` ใน `config.h`)
- [ ] เอามือขวาลงระหว่างล้อหมุน → ล้อหยุดทันที
- [ ] ☝ / 🤟 ปากคีบเปิด / ปิดจริง
- [ ] ✊ ซ้าย → `robot IDLE` ล้อหยุด, ✌ เริ่มใหม่ได้
- [ ] ปิด Wi-Fi ของ PC ระหว่างขับ → ล้อหยุดภายใน 0.3 วินาที
- [ ] กด Esc ระหว่างขับ → ล้อหยุด หุ่นเป็น IDLE

## 5. หุ่นจริง บนสนาม

- [ ] เริ่มเกียร์ 1 ทั้งหมด มีคนพร้อมยกหุ่น (ไม่มีปุ่มหยุดฉุกเฉินบนหุ่น)
- [ ] เก็บหิน 1 ก้อนตามคู่มือผู้เล่น (PLAYER_MANUAL_TH.md หัวข้อ 6) ได้ครบ
- [ ] จดระยะเบรกของแต่ละเกียร์ และ latency ที่รู้สึกได้ ถ้าเกียร์ไหนเร็วไปปรับ `--gears`
