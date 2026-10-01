// Robot controller: wheels + gripper servo + all safety logic, driven by UDP protocol v3.
// Board: course IN-ENG ESP32 board. Requires ESP32 Arduino core 3.x, ArduinoJson 7
// and the course InEngMotor library (https://github.com/zerotwobook/Embedded_System_Expo_2026).
//
// States:  IDLE --start--> RUNNING --stop--> IDLE
// Wheels move only in RUNNING and only while drive packets keep arriving (300 ms watchdog).
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
IPAddress announcedIp;
bool udpReady = false;
unsigned long lastBindAttempt = 0;

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
void updateNetwork(unsigned long now) {
  IPAddress ip = WiFi.localIP();
  if (WiFi.status() != WL_CONNECTED || ip == IPAddress(0, 0, 0, 0)) {
    if (udpReady) {
      Serial.println("Wi-Fi disconnected; wheels stop. Waiting for a new IP.");
      motorsOff();
      udp.stop();
      udpReady = false;
      peerPort = 0;
    }
    return;
  }
  if (udpReady && ip == announcedIp) return;
  if (udpReady) {
    motorsOff();
    udp.stop();
    udpReady = false;
    peerPort = 0;
  }
  if (now - lastBindAttempt < 1000) return;
  lastBindAttempt = now;
  if (!udp.begin(UDP_PORT)) {
    Serial.println("UDP bind failed; retrying.");
    return;
  }
  udpReady = true;
  announcedIp = ip;
  Serial.printf("robot_ctrl network ready. IP %s gateway %s subnet %s udp/%d (%s)\n",
                ip.toString().c_str(), WiFi.gatewayIP().toString().c_str(),
                WiFi.subnetMask().toString().c_str(), UDP_PORT,
                USE_STATIC_IP ? "static" : "DHCP");
}

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
  if (!peerPort || now - lastStatus < STATUS_PERIOD_MS) return;
  lastStatus = now;
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
  doc["grip_open_deg"] = GRIP_OPEN_DEG;
  doc["grip_close_deg"] = GRIP_CLOSE_DEG;
  doc["servo_min_deg"] = SERVO_MIN_DEG;
  doc["servo_max_deg"] = SERVO_MAX_DEG;
  JsonArray s = doc["servo"].to<JsonArray>();   // kept as a list: the PC side reads servo[0]
  s.add(roundf(servoPos));
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
  WiFi.setHostname(WIFI_HOSTNAME);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);                // lower latency
#if USE_STATIC_IP
  if (!WiFi.config(IPAddress(STATIC_IP), IPAddress(GATEWAY_IP), IPAddress(SUBNET_IP), IPAddress(GATEWAY_IP)))
    Serial.println("Static IP configuration failed; check config.h.");
#endif
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) delay(100);
  if (WiFi.status() != WL_CONNECTED)
    Serial.println("Wi-Fi not connected. Check hotspot name/password and 2.4 GHz compatibility.");
  updateNetwork(millis());
  lastLoop = millis();
}

void loop() {
  unsigned long now = millis();
  float dt = (now - lastLoop) / 1000.0f;
  lastLoop = now;

  updateNetwork(now);
  if (udpReady) pollUdp(now);

  if (state == RUNNING) {
    if (now - lastDrive > DRIVE_TIMEOUT_MS || !udpReady) cmdL = cmdR = 0;
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
  if (udpReady) sendStatus(now);

  if (STATUS_LED >= 0) digitalWrite(STATUS_LED, state == RUNNING ? HIGH : LOW);
  delay(2);
}
