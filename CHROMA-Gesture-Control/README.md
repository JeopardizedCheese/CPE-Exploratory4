# CHROMA Gesture Control

แพ็กเกจควบคุมหุ่นด้วยสองมือ (ขวาขับ ซ้ายสั่งงาน) พร้อมเก็บข้อมูลและเทรน MLP ของคุณเอง
ใช้กับ `firmware/robot_ctrl` ปัจจุบัน (IDLE/RUNNING, UDP 4211)

1. ติดตั้ง Python 3.12 และแตก ZIP
2. เปิด PowerShell ในโฟลเดอร์นี้ รัน `.\setup_gesture.ps1`
3. ทดลองกล้อง หรือเริ่มเก็บข้อมูลตามคำสั่งด้านล่าง

```powershell
# ใช้โมเดลท่ามือสำเร็จรูปของ MediaPipe (ไม่ต้องเทรน) ไม่มีการเชื่อมต่อหุ่น
.\.venv-gesture\Scripts\python.exe gesture_control.py --camera 0

# ต่อหุ่นจริง (IP ค่าเริ่มต้น 10.178.188.50)
.\.venv-gesture\Scripts\python.exe gesture_control.py --camera 0 --live

# ---- ไม่บังคับ: เทรนโมเดลเอง ----

# เก็บข้อมูล: เลข 1–8 เลือกคลาส, H เลือกมือซ้าย/ขวา, R บันทึก
.\.venv-gesture\Scripts\python.exe gesture_collect.py --camera 0

# หลังเก็บครบอย่างน้อย 5 sessions แต่ละ session มีครบ 8 คลาส × 2 มือ
.\.venv-gesture\Scripts\python.exe gesture_train.py

# ทดลองโมเดลที่คุณเทรน ไม่มีการเชื่อมต่อหุ่น
.\.venv-gesture\Scripts\python.exe gesture_control.py --camera 0 --classifier models/gesture_v2.npz
```

เริ่มเก็บ 60 ตัวอย่างต่อคลาสต่อมือต่อ session เปลี่ยนมุม ระยะ หรือแสงระหว่าง sessions
ข้อมูลมือเดียวชุดเดิมอยู่ที่ `gesture_data_v1/` และโมเดลเดิม `models/gesture_mlp.npz` (สำรอง)
แพ็กเกจนี้ไม่รวม firmware
การติดตั้งครั้งแรกต้องใช้อินเทอร์เน็ตเพื่อโหลด dependencies และโมเดล MediaPipe

- [คู่มือผู้เล่น: ท่ามือบังคับหุ่น](PLAYER_MANUAL_TH.md)
- [เช็กลิสต์ทดสอบ](TEST_GESTURE_TH.md)
- [project.md — บริบท โครงสร้าง สถานะ และโปรโตคอล](project.md)
- [วิธีเก็บข้อมูลและเทรน](TRAIN_GESTURES_TH.md)
- [ท่าควบคุมและการต่อหุ่น](README_GESTURE_TH.md)
