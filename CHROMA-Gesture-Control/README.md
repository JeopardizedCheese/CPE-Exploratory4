# CHROMA Gesture Control

**ระบบหลัก (ตั้งแต่ 1 ต.ค. 2026): คันโยกเสมือนมือเดียว `lever_control.py`** กำมือโยกเพื่อขับ
ยกนิ้วโป้งแล้วโยกขึ้น/ลงเพื่อหนีบ/ปล่อย ใช้โมเดลท่ามือสำเร็จรูปของ MediaPipe ไม่ต้องเทรน
ใช้กับ `firmware/robot_ctrl` ปัจจุบัน (IDLE/RUNNING, UDP 4211) คู่มือเต็ม: [README_LEVER_TH.md](README_LEVER_TH.md)

1. ติดตั้ง Python 3.12
2. เปิด PowerShell ในโฟลเดอร์นี้ รัน `.\setup_lever.ps1`
3. ทดลองตามลำดับด้านล่าง

```powershell
# ไม่ใช้กล้องและหุ่น: เมาส์แทนมือ
.\.venv-gesture\Scripts\python.exe lever_control.py --demo

# กล้องจริง ไม่ต่อหุ่น
.\.venv-gesture\Scripts\python.exe lever_control.py --camera 0

# ต่อหุ่นจริง (ความเร็วเริ่ม 0.15, floor 0.65 แทน MIN_DUTY 0.71 ของ firmware)
.\.venv-gesture\Scripts\python.exe lever_control.py --camera 0 --live --robot 10.178.188.50 --min-duty .65
```

บน Linux ใช้ `./.venv-gesture/bin/python` แทน `.\.venv-gesture\Scripts\python.exe`
ระหว่างใช้งาน กด `+` / `-` เพื่อเพิ่ม/ลดความเร็วทีละ 0.05 (ช่วง 0.02–1.00) มีผลทันที

## ระบบเดิม: สองมือ (เลิกใช้แล้ว เก็บไว้สำรอง)

`gesture_control.py` (ขวาขับ ซ้ายสั่งงาน), `gesture_collect.py` / `gesture_train.py` (เก็บข้อมูลและเทรน MLP),
`gesture_data_v1/` และ `models/gesture_mlp.npz` ยังอยู่และยังรันได้ แต่ไม่ใช่ระบบที่ใช้แข่งแล้ว
**ห้ามลบ `gesture_control.py`, `gesture_logic.py`, `gesture_model.py` และ `models/`**: คันโยกใช้ตัวอ่านกล้อง
โปรโตคอล UDP และโมเดล MediaPipe จากไฟล์เหล่านี้

- [ท่าควบคุมสองมือและการต่อหุ่น](README_GESTURE_TH.md)
- [คู่มือผู้เล่นสองมือ](PLAYER_MANUAL_TH.md)
- [เช็กลิสต์ทดสอบสองมือ](TEST_GESTURE_TH.md)
- [วิธีเก็บข้อมูลและเทรน](TRAIN_GESTURES_TH.md)
- [project.md: บริบท โครงสร้าง และโปรโตคอล (เขียนสำหรับระบบสองมือ)](project.md)

แพ็กเกจนี้ไม่รวม firmware การติดตั้งครั้งแรกต้องใช้อินเทอร์เน็ตเพื่อโหลด dependencies

ทดสอบซอฟต์แวร์ทั้งหมด (คันโยก + ระบบเดิม): `.\.venv-gesture\Scripts\python.exe -m unittest discover -s tests -v`
