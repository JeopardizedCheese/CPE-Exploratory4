# Smoother motion and measured robot turns

This update is for **changing the direction of the whole robot**, using the
existing overhead camera and AprilTag. It does not measure individual wheel
rotation. Start with this file for the motion changes added on 2026-09-29.

## What is causing the problem?

The supplied firmware applies this mapping to every nonzero legacy drive command:

```text
actual PWM fraction = 0.71 + abs(command) * (1.00 - 0.71)
```

| Requested command | Actual PWM in the supplied firmware |
| --- | --- |
| 0 | 0% |
| 0.01 | about 71.3% |
| 0.10 | about 73.9% |
| 0.20 | about 76.8% |
| 0.50 | about 85.5% |
| 1.00 | 100% |

The ramp occurs **before** this mapping, so it still jumps from stopped to about
71% power. `L_GAIN` and `R_GAIN` are also applied before that floor, which makes
small speed differences between wheels hard to command. The settings are power
commands; they have never been measured speeds in mm/s.

There is no 90-degree restriction in the wheel library or the planner. The
autonomy defaults instead accept alignment errors up to 6 degrees, exit a travel
turn within 15 degrees, and use a minimum 70 ms turn pulse. Its stored pulse table
estimates about 6 degrees for that shortest pulse. Those settings cannot establish
1-degree accuracy. The gripper's servo angle commands do not turn the robot.

The archive's notes report about 0.2 seconds of command-to-camera delay and
150–300 degrees/second in some charged-battery turns. These are prior recorded
claims in the project, not measurements made in this update. At those rates,
the robot moves another 30–60 degrees during 0.2 seconds, before considering
coasting. Simply shortening a timed 90-degree turn is unreliable.

## Can we choose the voltage?

You can choose **PWM duty**, which rapidly switches the motor drive on and off.
For an idealized drive, average applied voltage is approximately duty times battery
voltage; actual motor voltage/current depend on the switching circuit and motor.
It is not a regulated DC voltage, and 25% duty does not guarantee 25% speed.
The ESP32 LEDC hardware supports adjustable duty, and the course InEngMotor library
uses 20 kHz, 8-bit PWM (signed commands from -255 to 255).

A lower battery, friction, wheel slip, unequal motors, and payload change the
motion produced by a duty value. The useful control loop is:

```text
requested heading or speed -> compare with camera measurement
     -> adjust each wheel's PWM -> measure again -> slow/stop at the target
```

No encoder or gyro is assumed. Camera feedback measures the robot body, so it can
correct direction and ground speed, but it cannot regulate the two wheel RPMs
individually or detect hidden camera buffering automatically.

## What was added

- Firmware command `duty`: true signed PWM fraction, without the 71% minimum or
  hidden wheel gains. `l=0.10` becomes about 26/255, within 8-bit rounding.
- Ramp limits applied to actual duty: 0.6/second rising and 1.2/second falling.
  A zero command, STOP, invalid drive, lost Wi-Fi or watchdog expiry removes drive
  immediately. Direction reversal reaches zero before changing sign.
- Status reports `direct_pwm:1`, `mode`, `pwm_l`, `pwm_r`, and controller session.
- `motion_control.py`: a bounded power measurement, a relative heading turn,
  or a forward speed trial specified in mm/s. It waits for the **g** key before
  enabling movement. It requires fresh camera measurements and matching firmware.
- `motion_feedback.py`: predictive proportional/derivative heading control,
  proportional/integral speed control with integral limits, acceleration limiting
  for the requested speed, and heading correction while driving forward.
- Logs contain requested power, reported PWM, measured angle/speed, stop reasons,
  and final error. Loss of feedback latches a stop; it does not restart by itself.

The motor library's zero command **coasts**; it is not a mechanical instant stop.
The controller waits for measured stillness and uses the configured camera delay
to stop applying turn power early. No unmeasured braking distance is promised.

**Compatibility:** existing `drive` packets retain the old mapping, so existing
`autonomy.py`, gesture control and `teleop.py` do not silently use the wrong pulse
calibration. Use **motion_control.py** for the new mode. This is a separate motion
test/controller, not a claim that the autonomous sorting route is now calibrated.
The existing simulator/fake_robot does not advertise the new firmware feature.

## Set up and measure before tuning

1. Use this updated project folder and the project's Python environment with
   `requirements.txt` installed. In Arduino IDE, use ESP32 Arduino core 3.x,
   ArduinoJson 7 and the course InEngMotor library. Copy `secrets.example.h` to
   `secrets.h`, enter your Wi-Fi details, check board/IP settings in `config.h`,
   then compile and upload `firmware/robot_ctrl/robot_ctrl.ino`. Keep
   `motor_output.h` in the same sketch folder. This update was not flashed remotely.
2. Check wheel direction with the wheels raised. Forward must make both wheels
   drive the robot forward; right turns must rotate clockwise in the camera.
   If wrong, correct `L_INVERT` / `R_INVERT` before floor tests.
3. Fix the camera in its calibrated location. Check tag orientation, camera
   resolution, arena corners, tag height, axle offset and full body footprint in
   `calib.json`. Motion testing does not require the color/background calibration.
4. Put the robot on clear floor near the middle of the arena. The tool checks a
   circle covering the full body against arena edges; it does not avoid stones
   or other obstacles. Measure with the normal payload and an adequately charged
   battery; record battery voltage if a meter is available.
5. Preview the camera without connecting to the robot:

   ```powershell
   python motion_control.py --preview --camera 1 --turn-deg 15
   ```

   Watch the reported heading while stationary. If it jumps by a degree or more,
   a 1-degree specification is already below what this setup can verify. Improve
   tag visibility, lighting, exposure or camera geometry before tighter control.

6. Run a **single short power trial**, starting low:

   ```powershell
   python motion_control.py 10.178.188.50 --camera 1 --test-duty 0.20 --duration 2
   ```

   Press **g** to begin. SPACE, q, x or ESC stops. Examine the reported actual PWM,
   speed and `runs/motion/<run>/trace.jsonl`. Repeat in small duty increments only
   as needed; `--max-duty` must be explicitly raised to test above its default 0.4.
   Repeat with `--test-motion left` and `--test-motion right`. Keep stalled trials
   brief; a stalled motor's current does not become zero merely because it is still.

   Find the lowest **repeatable, smooth moving** power on the floor in each
   direction. A short test from rest measures starting behavior, not a separate
   minimum running duty; do not assume the two are equal. Wheel-lifted motion also
   does not establish the loaded threshold. No automatic full-power kick is used.

If the robot genuinely cannot move below 71% and immediately runs too fast above
that, removing a software floor alone cannot deliver smooth slow motion. Check
mechanical binding, motor/driver/power compatibility and gearing. A slower geared
drivetrain may be needed. Faster local feedback from encoders/gyro is a further
hardware path if camera timing cannot meet the required accuracy.

## Request an angle or a speed

After finding workable power limits, examples are:

```powershell
# Turn right by 15 degrees; left is -15. Completion requires measured stillness.
python motion_control.py 10.178.188.50 --camera 1 --turn-deg 15 --max-duty 0.40

# Attempt a 1-degree right turn with a tighter measured tolerance.
# This is a test request, NOT a verified physical capability.
python motion_control.py 10.178.188.50 --camera 1 --turn-deg 1 --tolerance-deg 0.25 --max-duty 0.40

# Regulate forward ground speed to 100 mm/s for a 5-second trial.
python motion_control.py 10.178.188.50 --camera 1 --speed-mm-s 100 --duration 5 --max-duty 0.40
```

These numbers are starting examples, not fitted values for your motors. A low
power limit may result in no motion; the tool reports this instead of claiming
success. `--base-duty` optionally adds a **measured** running-power feedforward
term to turn/speed control. Leave it zero until measured; a large value recreates
the very jump we are trying to remove. `--turn-kp`, `--turn-kd`, `--speed-kp`,
`--speed-ki` and `--camera-delay` are explicit tuning settings.

The tool aborts above its measured angular/linear speed limits (90 deg/s and
300 mm/s by default), on stale pose/status, significant heading deviation during
straight driving, arena edge proximity, timeout, or lack of progress. Defaults
are conservative starting limits and still need real testing. If the stop reason
is a speed limit, first reduce motor power rather than simply raising the limit.

Speed trial completion means **the trial ended**. Check `mean_speed_mm_s`,
`mean_speed_error_mm_s`, saturation in the trace, and
`mean_speed_within_10_percent` in `summary.json`. The mean uses the second half of
the powered interval. A short, saturated or stalled trial may not achieve the
requested speed. No voltage sensing or battery feedforward has been invented.

Turn completion means the **camera** stayed within the requested angular
tolerance with low measured movement for at least 0.5 seconds plus the configured
camera delay. It is not an independently verified absolute heading measurement.

## How to establish accuracy

Repeat left/right turns of 1, 5, 15, 45 and 90 degrees at least ten times, using
an independent marked angle reference as well as the camera. Record requested
angle, settled measured angle, independent error, battery state and payload.
Check the mean and worst error, and repeat at a partly discharged battery.
For speeds, measure distance/time over a marked path at several targets, such as
50, 100 and 150 mm/s, only where the drivetrain can sustain those speeds.

At 0.2 seconds of delay, even 5 degrees/second accounts for 1 degree of motion
before feedback catches up; coasting and image noise add error. A 1-degree goal
therefore needs a sufficiently slow controllable drivetrain and much better than
1-degree camera repeatability. If tests do not demonstrate that, use the achieved
tolerance or improve hardware. Lowering a tolerance number in code is not proof.

## Validation performed here

- 127 main-project tests passed, including 17 new motion/PWM tests.
- Tests exercise small-angle commands, signed/wrapped angles, delayed idealized
  turn dynamics, stall/overspeed/stale-pose stops, finite input checks, speed
  saturation, axle-based velocity, edge limits, and firmware capability checking.
- Mocked runner tests verify that preview opens no network link, old firmware
  cannot be armed, and tag loss after arming sends zero power and STOP.
- The actual `motor_output.h` was compiled and executed with host g++, including
  10%-to-26/255 mapping, bounded acceleration, stop and reversal behavior.
- Python modules compiled and CLI argument validation was checked.
- No ESP32 toolchain was available locally. The full sketch has **not** been
  compiled for the board or flashed. No real camera/robot/battery accuracy tests
  were performed. Idealized controller tests omit real stiction and wheel slip.

## References checked

- [ESP32 LEDC duty control](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/ledc.html)
- [Course InEngMotor library](https://github.com/zerotwobook/Embedded_System_Expo_2026/blob/main/InEngMotor.zip)
- [Toshiba TB6612FNG datasheet](https://toshiba.semicon-storage.com/info/TB6612FNG_datasheet_en_20141001.pdf?did=10660&prodName=TB6612FNG)

Project findings above come from `firmware/robot_ctrl/config.h`,
`firmware/robot_ctrl/robot_ctrl.ino`, `autonomy.py` and the supplied `HANDOFF.md`.
Historical handoff tasks were treated as background information, not as new user
instructions.
