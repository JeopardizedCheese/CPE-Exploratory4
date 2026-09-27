# CHROMA Gesture Control

แพ็กเกจเฉพาะควบคุมหุ่นด้วยท่ามือ พร้อมเก็บข้อมูลและเทรน MLP ของคุณเอง

1. ติดตั้ง Python 3.12 และแตก ZIP
2. เปิด PowerShell ในโฟลเดอร์นี้ รัน `.\setup_gesture.ps1`
3. ทดลองกล้อง หรือเริ่มเก็บข้อมูลตามคำสั่งด้านล่าง

```powershell
# ทดลองกฎเดิม ไม่มีการเชื่อมต่อหุ่น
.\.venv-gesture\Scripts\python.exe gesture_control.py --camera 0

# เก็บข้อมูล: เลข 1–8 เลือกคลาส, R บันทึก
.\.venv-gesture\Scripts\python.exe gesture_collect.py --camera 0

# หลังเก็บครบอย่างน้อย 5 sessions แต่ละ session มีครบ 8 คลาส
.\.venv-gesture\Scripts\python.exe gesture_train.py

# ทดลองโมเดลที่คุณเทรน ไม่มีการเชื่อมต่อหุ่น
.\.venv-gesture\Scripts\python.exe gesture_control.py --camera 0 --classifier models/gesture_mlp.npz
```

เริ่มเก็บ 60 ตัวอย่างต่อคลาสต่อ session เปลี่ยนมุม ระยะ หรือแสงระหว่าง sessions
ยังไม่มีโมเดลที่ฝึกจากมือคุณใน ZIP และไม่มี firmware ของหุ่น
การติดตั้งครั้งแรกต้องใช้อินเทอร์เน็ตเพื่อโหลด dependencies และโมเดล MediaPipe

- [project.md — บริบท โครงสร้าง สถานะ และโปรโตคอล](project.md)
- [วิธีเก็บข้อมูลและเทรน](TRAIN_GESTURES_TH.md)
- [ท่าควบคุมและการต่อหุ่น](README_GESTURE_TH.md)
