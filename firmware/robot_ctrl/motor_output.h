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

// Limit changes in actual duty, not in the input to a nonlinear minimum-power map.
// A zero request is an immediate electrical stop (coast in InEngMotor).
inline float rampDuty(float output, float target, float dt,
                      float upPerSecond, float downPerSecond) {
  if (!isfinite(target) || !isfinite(dt) || dt <= 0) return 0;
  target = clamp(target, -1.0f, 1.0f);
  if (target == 0) return 0;
  // Never reverse directly. Reach zero on this update; reverse on a later update.
  if (output * target < 0) target = 0;
  float rate = fabsf(target) > fabsf(output) ? upPerSecond : downPerSecond;
  float step = rate * fminf(dt, 0.05f);  // a delayed loop must not jump to full power
  return output + clamp(target - output, -step, step);
}
}
