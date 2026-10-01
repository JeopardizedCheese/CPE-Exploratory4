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
#define SERVO_PIN 33             // IN-ENG board servo header
#define SERVO_US_MIN 2500         // pulse width at 0 deg   (same as course example 03)
#define SERVO_US_MAX 500        // pulse width at 180 deg
#define SERVO_MIN_DEG 0          // never command outside this range
#define SERVO_MAX_DEG 360         // a little past closed
#define SERVO_START_DEG 0        // assumed position before the first command (no pulse is sent at boot)
#define SERVO_DEG_PER_SEC 180.0f // slow moves reduce current spikes / brownout
#define GRIP_OPEN_DEG 0          // measured: jaws open
#define GRIP_CLOSE_DEG 250        // measured: holds every stone size with less strain

// ---------- 360 deg (continuous rotation) servo ----------
// The gripper servo is a 360 servo: no position sensor, the pulse sets speed and direction.
// Each grip/servo command is a timed spin; the angle (GRIP_OPEN/CLOSE_DEG, status "servo")
// is estimated from the time spun. Same convention as before: start = open = 0 deg at the
// counter-clockwise end, closing (larger angles) turns clockwise. Between moves: no pulse
// (servo at rest, cannot creep). 0 = normal 180 deg servo.
#define SERVO_CONTINUOUS 1
#define SERVO_CLOSE_PULSE_SIGN -1     // closing = clockwise = pulse below SERVO_STOP_US (most 360
                                      // servos); +1 if it closes counter-clockwise
#define SERVO_STOP_US 1500            // pulse at which it stands still
#define SERVO_REST_NO_PULSE 1         // 1: at rest send no pulse; 0: send SERVO_STOP_US (trim it)
#define SERVO_SPIN_US 250             // spin pulse = SERVO_STOP_US +/- this: larger = faster
#define SERVO_SPIN_DEG_PER_SEC 200.0f // CALIBRATE: degrees turned per second at that pulse
#define SERVO_MAX_MOVE_MS 1500        // safety: one move never spins longer than this
#define SERVO_OPEN_EXTRA_DEG 0        // >0: opening spins this much further into the open stop (cancels drift)

// ---------- Safety ----------
#define DRIVE_TIMEOUT_MS 300     // no drive packet for this long -> wheels stop
#define STATUS_LED 23            // board LED: off = IDLE, on = RUNNING. -1 = none

// ---------- Network ----------
// Fixed IP on the team hotspot (10.178.188.0/24, gateway .223).
// Set USE_STATIC_IP 0 to go back to an automatic (DHCP) address.

/*Mick's wifi*/
#define USE_STATIC_IP 1
#define STATIC_IP  172, 20, 10, 2
#define GATEWAY_IP 172, 20, 10, 1
#define SUBNET_IP  255, 255, 255, 0

/* P's Wifi */
// #define STATIC_IP  10, 178, 188, 50
// #define GATEWAY_IP 10, 178, 188, 223
// #define SUBNET_IP  255, 255, 255, 0

#define UDP_PORT 4211
#define STATUS_PERIOD_MS 200
