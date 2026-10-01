// Robot controller: wheels + gripper servo + all safety logic, driven by UDP protocol v3.
// Board: course IN-ENG ESP32 board. Requires ESP32 Arduino core 3.x, ArduinoJson 7
// and the course InEngMotor library (https://github.com/zerotwobook/Embedded_System_Expo_2026).
//
// States:  IDLE --start--> RUNNING --stop--> IDLE
// Wheels move only in RUNNING and only while drive packets keep arriving (300 ms watchdog).
// The gripper camera (HuskyLens, V3) answers "look" commands; see config.h.
// The gripper obeys grip/servo commands in both states.
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include <InEngMotor.h>
#include <math.h>
#include <esp_system.h>
#include "secrets.h"   // copy secrets.example.h -> secrets.h
#include "config.h"
#include "motor_output.h"

#if !defined(ESP_ARDUINO_VERSION_MAJOR) || ESP_ARDUINO_VERSION_MAJOR < 3
#error "Use ESP32 Arduino core 3.x (Boards Manager: esp32 by Espressif >= 3.0)"
#endif

enum State { IDLE, RUNNING };
const char *STATE_NAME[] = {"IDLE", "RUNNING"};
State state = IDLE;
const char *reason = "boot";

WiFiUDP udp;
String activeSession;
uint32_t lastSeq = 0;
unsigned long lastRx = 0, lastDrive = 0, lastStatus = 0, lastLoop = 0;
IPAddress peerIp;
uint16_t peerPort = 0;

float cmdL = 0, cmdR = 0, outL = 0, outR = 0;   // -1..1: commands/outputs, NOT measured speeds
bool directDuty = false;
float driveFloor = MIN_DUTY;  // legacy drive floor: MIN_DUTY, or "m" of the last drive packet
int pwmL = 0, pwmR = 0;  // actual signed PWM written to the library, before pin inversion
float servoPos = SERVO_START_DEG, servoTarget = SERVO_START_DEG;
bool servoActive = false;                       // no pulses until the first grip/servo command

// ---------------------------------------------------------------- wheels
// -1..1 -> signed speed for InEngMotor::drive. Any non-zero command gets at least
// driveFloor (MIN_DUTY unless the drive packet sends "m") so the motors never sit
// stalled; 0 means coast.
int toSpeed(float v) {
  return motion::floorToPwm(v, driveFloor, MAX_DUTY);
}

void applyMotors() {
  pwmL = directDuty ? motion::dutyToPwm(outL) : toSpeed(outL * L_GAIN);
  pwmR = directDuty ? motion::dutyToPwm(outR) : toSpeed(outR * R_GAIN);
  inengmotor.drive(pwmL, pwmR);
}

void motorsOff() {
  cmdL = cmdR = outL = outR = 0;
  applyMotors();
}

void setupMotors() {
  inengmotor.begin();                   // claims PWM channels 0-3: must run before setupServo()
  inengmotor.setInvert(L_INVERT, R_INVERT);
  motorsOff();
}

// Speed up gradually, but slow down / reverse-to-stop instantly.
float ramp(float out, float cmd, float dt) {
  if (fabsf(cmd) < fabsf(out) || (cmd * out) < 0) return (cmd * out < 0) ? 0 : cmd;
  float step = RAMP_PER_SEC * dt;
  if (cmd > out) return fminf(cmd, out + step);
  return fmaxf(cmd, out - step);
}

// ---------------------------------------------------------------- gripper servo
void servoWrite(float deg) {
  float us = SERVO_US_MIN + (SERVO_US_MAX - SERVO_US_MIN) * deg / 180.0f;
  ledcWrite(SERVO_PIN, (uint32_t)(us * 65535.0f / 20000.0f));   // 50 Hz, 16-bit
}

void setupServo() {
  ledcAttach(SERVO_PIN, 50, 16);
  ledcWrite(SERVO_PIN, 0);            // no pulse = servo idle: nothing moves at power-on
  servoPos = servoTarget = constrain(SERVO_START_DEG, SERVO_MIN_DEG, SERVO_MAX_DEG);
}

#if SERVO_CONTINUOUS
// 360 servo: timed spins, at rest between moves (see config.h).
unsigned long servoMoveStart = 0;
bool servoMoving = false;
float servoGoal = SERVO_START_DEG;     // where the spin stops (target, or past it into the open stop)

void servoRest() {
  servoMoving = false;
#if SERVO_REST_NO_PULSE
  ledcWrite(SERVO_PIN, 0);
#else
  ledcWrite(SERVO_PIN, (uint32_t)(SERVO_STOP_US * 65535.0f / 20000.0f));
#endif
}

void servoSpin(float dir) {          // dir +1 = closing (clockwise), -1 = opening (counter-clockwise)
  float us = SERVO_STOP_US + dir * SERVO_CLOSE_PULSE_SIGN * SERVO_SPIN_US;
  ledcWrite(SERVO_PIN, (uint32_t)(us * 65535.0f / 20000.0f));
}

bool setServo(float deg) {
  if (!isfinite(deg)) return false;
  servoTarget = constrain(deg, SERVO_MIN_DEG, SERVO_MAX_DEG);
  servoActive = true;                 // position: assumed SERVO_START_DEG at boot, then estimated
  servoGoal = servoTarget;
  if (servoTarget <= SERVO_MIN_DEG && SERVO_OPEN_EXTRA_DEG > 0) servoGoal = SERVO_MIN_DEG - SERVO_OPEN_EXTRA_DEG;
  servoMoveStart = millis();
  servoMoving = fabsf(servoGoal - servoPos) >= 0.5f;
  if (!servoMoving) servoRest();
  return true;
}

void updateServo(float dt) {
  if (!servoActive || !servoMoving) return;
  float d = servoGoal - servoPos;
  if (fabsf(d) < 0.5f || millis() - servoMoveStart > SERVO_MAX_MOVE_MS) {
    servoPos = servoTarget;           // arrived (or gave up): the estimate is the target
    servoRest();
    return;
  }
  float step = SERVO_SPIN_DEG_PER_SEC * dt;
  servoPos += constrain(d, -step, step);
  servoSpin(d > 0 ? 1.0f : -1.0f);
}

#else
bool setServo(float deg) {
  if (!isfinite(deg)) return false;
  servoTarget = constrain(deg, SERVO_MIN_DEG, SERVO_MAX_DEG);
  if (!servoActive) {                 // first command: go straight there (position unknown before)
    servoActive = true;
    servoPos = servoTarget;
    servoWrite(servoPos);
  }
  return true;
}

void updateServo(float dt) {
  if (!servoActive) return;
  float d = servoTarget - servoPos;
  if (fabsf(d) < 0.01f) return;
  float step = SERVO_DEG_PER_SEC * dt;
  servoPos += constrain(d, -step, step);
  servoWrite(servoPos);
}
#endif

// ---------------------------------------------------------------- gripper camera
// HuskyLens 1 in object classification mode on Serial2 (pins in config.h). A "look" command
// collects GRIPCAM_READINGS class IDs, one per camera frame; the PC turns them into the grip
// check verdict. Never blocks the loop. Protocol: HuskyLens/HUSKYLENSArduino "HUSKYLENS
// Protocol.md": 55 AA 11 <len> <cmd> <data...> <sum of all bytes, low byte>.
bool statusNow = false;                         // send the next status packet at once
#if GRIPCAM_ENABLE
HardwareSerial &cam = Serial2;
enum { HL_REQUEST = 0x20, HL_RETURN_INFO = 0x29, HL_RETURN_BLOCK = 0x2A, HL_RETURN_ARROW = 0x2B,
       HL_KNOCK = 0x2C, HL_ALGORITHM = 0x2D, HL_OBJECT_CLASSIFICATION = 6 };
uint8_t camBuf[32];
int camLen = 0;
unsigned long camLastReply = 0, camNextKnock = 0, camAskedAt = 0;
bool camWasAlive = false;

int32_t lookN = -1;                             // the PC's look number; -1 = none this session
bool lookDone = false, lookAsking = false;      // asking: a request is out, reply not complete
unsigned long lookStart = 0;
int lookIds[GRIPCAM_READINGS];
int lookCount = 0, lookBlocksLeft = 0, lookId = 0;
int32_t lookFrame = -1, lastFrame = -1;

bool camAlive(unsigned long now) {
  return camLastReply && now - camLastReply < GRIPCAM_ALIVE_MS;
}

void camSend(uint8_t cmd, int arg = -1) {       // arg >= 0: one 16-bit data word
  uint8_t p[8];
  int n = 0;
  p[n++] = 0x55; p[n++] = 0xAA; p[n++] = 0x11; p[n++] = arg >= 0 ? 2 : 0; p[n++] = cmd;
  if (arg >= 0) { p[n++] = arg & 0xFF; p[n++] = (arg >> 8) & 0xFF; }
  uint8_t sum = 0;
  for (int i = 0; i < n; i++) sum += p[i];
  p[n++] = sum;
  cam.write(p, n);
}

void camReading() {                             // one complete reply to a request
  lookAsking = false;
  if (lookFrame != lastFrame && lookCount < GRIPCAM_READINGS) {   // same frame twice counts once
    lastFrame = lookFrame;
    lookIds[lookCount++] = lookId;
  }
}

void camFrame(uint8_t cmd, const uint8_t *d, int len, unsigned long now) {
  camLastReply = now;                           // any valid frame: the camera is there
  if (!lookAsking) return;
  if (cmd == HL_RETURN_INFO && len >= 6) {      // count, learned IDs, frame number, reserved
    lookBlocksLeft = d[0] | (d[1] << 8);
    lookFrame = d[4] | (d[5] << 8);
    lookId = 0;                                 // nothing recognised = ID 0
    if (lookBlocksLeft == 0) camReading();
  } else if ((cmd == HL_RETURN_BLOCK || cmd == HL_RETURN_ARROW) && len >= 10 && lookBlocksLeft > 0) {
    if (lookId == 0) lookId = d[8] | (d[9] << 8);   // classification: the first result's ID
    if (--lookBlocksLeft == 0) camReading();
  }
}

void camPoll(unsigned long now) {
  while (cam.available()) {
    uint8_t b = cam.read();
    if (camLen == 0) { if (b == 0x55) camBuf[camLen++] = b; continue; }
    if (camLen == 1) { if (b == 0xAA) camBuf[camLen++] = b; else camLen = (b == 0x55); continue; }
    camBuf[camLen++] = b;
    if (camLen == 4 && camBuf[3] > sizeof(camBuf) - 6) { camLen = 0; continue; }   // impossible length
    if (camLen > 4 && camLen == 6 + camBuf[3]) {          // header, address, length, command, data, sum
      uint8_t sum = 0;
      for (int i = 0; i < camLen - 1; i++) sum += camBuf[i];
      if (sum == camBuf[camLen - 1]) camFrame(camBuf[4], camBuf + 5, camBuf[3], now);
      camLen = 0;
    }
  }
}

void camLook(int32_t n, unsigned long now) {
  lookN = n;
  lookDone = lookAsking = false;
  lookStart = now;
  lookCount = 0;
  lastFrame = -1;
}

void camUpdate(unsigned long now) {
  camPoll(now);
  bool alive = camAlive(now);
  if (alive && !camWasAlive) camSend(HL_ALGORITHM, HL_OBJECT_CLASSIFICATION);   // (re)connected
  camWasAlive = alive;
  if (lookN >= 0 && !lookDone) {
    if (now - lookStart < GRIPCAM_SETTLE_MS) return;
    if (lookCount >= GRIPCAM_READINGS || now - lookStart > GRIPCAM_SETTLE_MS + GRIPCAM_LOOK_MS) {
      lookDone = true;
      lookAsking = false;
      statusNow = true;
      return;
    }
    // next request 20 ms after the last reply; resend one that got no reply in 100 ms
    if (now - camAskedAt >= (lookAsking ? 100UL : 20UL)) {
      lookAsking = true;
      lookBlocksLeft = 0;
      camAskedAt = now;
      camSend(HL_REQUEST);
    }
  } else if ((long)(now - camNextKnock) >= 0) {
    camNextKnock = now + GRIPCAM_KNOCK_MS;
    camSend(HL_KNOCK);
  }
}

void camStatus(JsonDocument &doc, unsigned long now) {
  doc["gripcam"] = camAlive(now) ? "ok" : "none";
  if (lookN < 0) return;
  JsonObject look = doc["look"].to<JsonObject>();
  look["n"] = lookN;
  look["done"] = lookDone;
  JsonArray ids = look["ids"].to<JsonArray>();
  for (int i = 0; i < lookCount; i++) ids.add(lookIds[i]);
}
#endif

// ---------------------------------------------------------------- state
void enter(State s, const char *why) {
  if (state == s) return;
  state = s;
  reason = why;
  if (s != RUNNING) motorsOff();
  if (s == RUNNING) { lastDrive = 0; cmdL = cmdR = 0; }
  Serial.printf("[%lu] -> %s (%s)\n", millis(), STATE_NAME[s], why);
}

// ---------------------------------------------------------------- network
void handlePacket(char *buf, unsigned long now) {
  JsonDocument doc;
  if (deserializeJson(doc, buf) || doc["v"] != 3 || !doc["s"].is<const char *>() ||
      !doc["q"].is<uint32_t>() || !doc["c"].is<const char *>()) {
    motorsOff();
    return;
  }
  String session = doc["s"].as<String>();
  uint32_t seq = doc["q"].as<uint32_t>();
  if (session.length() != 12) return;
  if (session != activeSession) {
    // A new controller may take over only after the old one has been silent.
    if (activeSession.length() && now - lastRx <= DRIVE_TIMEOUT_MS) return;
    activeSession = session;
    lastSeq = 0;
#if GRIPCAM_ENABLE
    lookN = -1;                           // a new controller never sees the old one's look
#endif
  }
  if (seq <= lastSeq) return;             // duplicate / reordered: ignore
  lastSeq = seq;
  lastRx = now;
  peerIp = udp.remoteIP();
  peerPort = udp.remotePort();

  const char *c = doc["c"];
  if (!strcmp(c, "drive") || !strcmp(c, "duty")) {
    float l = doc["l"] | NAN, r = doc["r"] | NAN;
    if (state != RUNNING || !isfinite(l) || !isfinite(r) || fabsf(l) > 1 || fabsf(r) > 1) {
      motorsOff();
      return;
    }
    bool requestedDuty = !strcmp(c, "duty");
    // Optional "m" on drive packets: floor duty in place of MIN_DUTY for this packet.
    float m = MIN_DUTY;
    if (!requestedDuty && !doc["m"].isNull()) {
      m = doc["m"] | NAN;
      if (!isfinite(m) || m < 0 || m > MAX_DUTY) {
        motorsOff();
        return;
      }
    }
    if (requestedDuty != directDuty) motorsOff();  // do not reinterpret a moving output
    directDuty = requestedDuty;
    if (!requestedDuty) driveFloor = m;
    cmdL = l; cmdR = r; lastDrive = now;
  } else if (!strcmp(c, "start")) {
    if (state == IDLE) enter(RUNNING, "start");
  } else if (!strcmp(c, "stop")) {
    enter(IDLE, "remote stop");
  } else if (!strcmp(c, "servo")) {             // raw angle, for calibration (servo index 0 only)
    if ((doc["i"] | 0) == 0) setServo(doc["deg"] | NAN);
  } else if (!strcmp(c, "grip")) {
    const char *p = doc["p"] | "";
    if (!strcmp(p, "open")) setServo(GRIP_OPEN_DEG);
    if (!strcmp(p, "close")) setServo(GRIP_CLOSE_DEG);
  } else if (!strcmp(c, "look")) {              // grip check: classify what the jaws hold (V3)
#if GRIPCAM_ENABLE
    if (doc["n"].is<int32_t>() && doc["n"].as<int32_t>() >= 0) camLook(doc["n"].as<int32_t>(), now);
#endif
  }
  // "ping" and unknown commands only refresh the link.
}

void pollUdp(unsigned long now) {
  for (int k = 0; k < 8; k++) {                 // drain a few packets per loop
    int len = udp.parsePacket();
    if (!len) return;
    if (len >= 512) { while (udp.available()) udp.read(); motorsOff(); continue; }
    char buf[512];
    int n = udp.read(buf, sizeof(buf) - 1);
    if (n <= 0) continue;
    buf[n] = 0;
    handlePacket(buf, now);
  }
}

void sendStatus(unsigned long now) {
  if (!peerPort || (!statusNow && now - lastStatus < STATUS_PERIOD_MS)) return;
  lastStatus = now;
  statusNow = false;
  JsonDocument doc;
  doc["state"] = STATE_NAME[state];
  doc["why"] = reason;
  doc["l"] = outL;
  doc["r"] = outR;
  doc["direct_pwm"] = 1;  // feature version; old firmware has no such field
  doc["mode"] = directDuty ? "duty" : "legacy";
  doc["min_duty"] = driveFloor;  // legacy floor in use; old firmware has no such field
  doc["pwm_l"] = pwmL;
  doc["pwm_r"] = pwmR;
  doc["session"] = activeSession;
  doc["rx_age_ms"] = now - lastRx;
  doc["rssi"] = WiFi.RSSI();
  JsonArray s = doc["servo"].to<JsonArray>();   // kept as a list: the PC side reads servo[0]
  s.add(roundf(servoPos));
#if GRIPCAM_ENABLE
  camStatus(doc, now);
#else
  doc["gripcam"] = "off";
#endif
  char out[512];
  size_t n = serializeJson(doc, out, sizeof(out));
  udp.beginPacket(peerIp, peerPort);
  udp.write((const uint8_t *)out, n);
  udp.endPacket();
}

// ---------------------------------------------------------------- main
void setup() {
  setupMotors();                       // first: make sure wheels are off
  Serial.begin(115200);
  // Why did we (re)start? 1 = power-on, 9 = BROWNOUT (servo/motors pulled the voltage down),
  // 4 = software crash, 3/12 = watchdog/panic.
  Serial.printf("\nreset reason: %d\n", (int)esp_reset_reason());
  if (STATUS_LED >= 0) pinMode(STATUS_LED, OUTPUT);
  setupServo();                        // after setupMotors: the servo gets its own PWM channel
#if GRIPCAM_ENABLE
  cam.begin(GRIPCAM_BAUD, SERIAL_8N1, GRIPCAM_RX_PIN, GRIPCAM_TX_PIN);   // explicit pins: never 16/17
#endif
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);                // lower latency
#if USE_STATIC_IP
  WiFi.config(IPAddress(STATIC_IP), IPAddress(GATEWAY_IP), IPAddress(SUBNET_IP), IPAddress(GATEWAY_IP));
#endif
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) delay(100);
  WiFi.setAutoReconnect(true);
  udp.begin(UDP_PORT);
  Serial.printf("robot_ctrl ready. IP %s udp/%d\n", WiFi.localIP().toString().c_str(), UDP_PORT);
  lastLoop = millis();
}

void loop() {
  unsigned long now = millis();
  float dt = (now - lastLoop) / 1000.0f;
  lastLoop = now;

  pollUdp(now);

  if (state == RUNNING) {
    if (now - lastDrive > DRIVE_TIMEOUT_MS || WiFi.status() != WL_CONNECTED) cmdL = cmdR = 0;
  } else {
    cmdL = cmdR = 0;
  }
  if (directDuty) {
    outL = motion::rampDuty(outL, cmdL, dt, DUTY_RAMP_UP_PER_SEC);
    outR = motion::rampDuty(outR, cmdR, dt, DUTY_RAMP_UP_PER_SEC);
  } else {
    outL = ramp(outL, cmdL, dt);
    outR = ramp(outR, cmdR, dt);
  }
  applyMotors();
  updateServo(dt);
#if GRIPCAM_ENABLE
  camUpdate(now);
#endif
  sendStatus(now);

  if (STATUS_LED >= 0) digitalWrite(STATUS_LED, state == RUNNING ? HIGH : LOW);
  delay(2);
}
