#pragma once
// ============================================================================
//  robot_ctrl settings for the course IN-ENG ESP32 board (ESP32-WROOM-32E).
//  Wheels: onboard TB6612 driven by the InEngMotor library, fixed pins
//          (left 26/27, right 16/17), PWM channels 0-3. Nothing to set here.
//  Other pins used on the board: servo header 19, TFT 18/23/5/2/4,
//  potentiometer 34, LED 23.
// ============================================================================

// ---------- Wheels ----------
#define L_INVERT true        // InEngMotor board default is (true, false); flip one if a wheel runs backwards
#define R_INVERT false
#define L_GAIN 1.00f         // straight-line trim: lower the stronger wheel (e.g. 0.90)
#define R_GAIN 1.00f
#define MAX_DUTY 1.00f       // 255 = full power
#define MIN_DUTY 0.71f       // ~181/255: any non-zero drive command starts here (measured once, partly on USB
                             // power: not trusted). A drive packet may send "m" to use another floor
                             // (autonomy.py min_duty) without reflashing
#define RAMP_PER_SEC 3.0f    // speed-up limit (full scale per second); slow-down is instant

// New "duty" packets bypass MIN_DUTY and L/R_GAIN. Old "drive" packets retain
// their mapping so the old autonomy pulse calibration is not silently changed.
// Duty is motor power, NOT measured wheel speed or a regulated voltage.
#define DUTY_RAMP_UP_PER_SEC 0.6f   // speed-up limit; slow-down and reversal to zero are instant

// ---------- Gripper servo (SG90) ----------
#define SERVO_PIN 19             // IN-ENG board servo header
#define SERVO_US_MIN 500         // pulse width at 0 deg   (same as course example 03)
#define SERVO_US_MAX 2500        // pulse width at 180 deg
#define SERVO_MIN_DEG 0          // never command outside this range
#define SERVO_MAX_DEG 70         // must allow the user-confirmed closed position
#define SERVO_START_DEG 0        // assumed position before the first command (no pulse is sent at boot)
#define SERVO_DEG_PER_SEC 180.0f // slow moves reduce current spikes / brownout
#define GRIP_OPEN_DEG 0          // measured: jaws open
#define GRIP_CLOSE_DEG 70        // user-confirmed close angle for the current gripper

static_assert(SERVO_MIN_DEG >= 0 && SERVO_MAX_DEG <= 180 && SERVO_MIN_DEG <= SERVO_MAX_DEG,
              "Servo limits must satisfy 0 <= min <= max <= 180");
static_assert(GRIP_OPEN_DEG >= SERVO_MIN_DEG && GRIP_OPEN_DEG <= SERVO_MAX_DEG,
              "GRIP_OPEN_DEG is outside servo limits");
static_assert(GRIP_CLOSE_DEG >= SERVO_MIN_DEG && GRIP_CLOSE_DEG <= SERVO_MAX_DEG,
              "GRIP_CLOSE_DEG is outside servo limits");

// ---------- Safety ----------
#define DRIVE_TIMEOUT_MS 300     // no drive packet for this long -> wheels stop
#define STATUS_LED 23            // board LED: off = IDLE, on = RUNNING. -1 = none

// ---------- Network ----------
// DHCP works when the team changes hotspot phones/subnets. Read the assigned
// IP from Serial Monitor at 115200 baud and pass it to the Python tools.
#define USE_STATIC_IP 0
#define WIFI_HOSTNAME "gem-sorter"
// Optional legacy static profile: only enable after checking the CURRENT router.
#define STATIC_IP  10, 178, 188, 50
#define GATEWAY_IP 10, 178, 188, 223
#define SUBNET_IP  255, 255, 255, 0
#define UDP_PORT 4211
#define STATUS_PERIOD_MS 200
