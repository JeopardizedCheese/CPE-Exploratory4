#pragma once
#include <math.h>

// Hardware-independent helpers, also compiled by tests/test_motor_output.py.
namespace motion {
inline float clamp(float value, float low, float high) {
  return fmaxf(low, fminf(high, value));
}

// A duty request is already a fraction of the supply; never add a minimum floor.
inline int dutyToPwm(float duty) {
  if (!isfinite(duty)) return 0;
  return (int)roundf(clamp(duty, -1.0f, 1.0f) * 255.0f);
}

// Legacy drive: any non-zero command -1..1 starts at `floor` duty and scales linearly up
// to maxDuty; below 0.01 the wheel coasts.
inline int floorToPwm(float v, float floor, float maxDuty) {
  if (!isfinite(v)) return 0;
  v = clamp(v, -1.0f, 1.0f);
  float mag = fabsf(v);
  if (mag < 0.01f) return 0;
  int s = (int)roundf((floor + mag * (maxDuty - floor)) * 255.0f);
  return v > 0 ? s : -s;
}

// Limit increases in actual duty, not in the input to a nonlinear minimum-power map.
// Slowing down is immediate, like the legacy drive ramp: a zero request is an immediate
// electrical stop (coast in InEngMotor), and a lower request applies at once.
inline float rampDuty(float output, float target, float dt, float upPerSecond) {
  if (!isfinite(target) || !isfinite(dt) || dt <= 0) return 0;
  target = clamp(target, -1.0f, 1.0f);
  // Never reverse directly. Reach zero on this update; reverse on a later update.
  if (target == 0 || output * target < 0) return 0;
  if (fabsf(target) < fabsf(output)) return target;
  float step = upPerSecond * fminf(dt, 0.05f);  // a delayed loop must not jump to full power
  return output + clamp(target - output, -step, step);
}
}
