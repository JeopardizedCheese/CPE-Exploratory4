# Software validation — Virtual Lever v1

Validated on Windows, Python 3.12.14, OpenCV 4.13.0, MediaPipe 0.10.32, NumPy 2.5.3.

- 68 automated tests passed across the existing gesture/training suites and the new lever suite (27 state/transport tests and 4 application smoke tests).
- The tests cover neutral arming, movement in four directions, gesture and frame loss, duplicate/stale/reordered observations, two-hand rejection, servo entry, neutral rearming, no repeated grip events, mode cancellation, fresh status/session gating, start timeout, immediate zero commands, bounded grip retries, shutdown, and preview without a camera/socket.
- The actual bundled MediaPipe Gesture Recognizer loaded and processed a blank RGB frame successfully, returning no hands. This validates the model/runtime path, not recognition accuracy.
- The real OpenCV HighGUI window opened, rendered a generated preview, and closed successfully.
- Actual localhost UDP integration exercised START, neutral arming, forward drive, servo entry, three idempotent CLOSE transmissions preceded by zero-wheel commands, and final STOP. 48 JSON v3 packets were captured; sequence numbers were unique and increasing.
- `pip check` reported no broken requirements.
- Generated DRIVE/SERVO screen images were visually inspected for readable text and unclipped controls.

`evidence/` contains generated screenshots; the localhost packet trace was captured during validation but is not included. Hand positions in these screenshots are simulated.

Not validated: real webcam recognition, physical motors/gripper, Wi-Fi latency/loss, camera buffering, lighting/occlusion, or end-to-end emergency stopping distance. No real robot was contacted and no firmware was flashed.

Re-run the tests from the package directory:

```powershell
.\.venv-gesture\Scripts\python.exe -m unittest discover -s tests -v
```

The new entry point is `lever_control.py`; it reuses the existing camera worker, protocol, and MediaPipe model. Existing firmware and the `gesture_control.py` two-hand entry point are unchanged.
