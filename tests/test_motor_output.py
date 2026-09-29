"""Compile and exercise the actual firmware math on the host (not an ESP32 build)."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


class MotorOutputTests(unittest.TestCase):
    def test_actual_cpp_helpers(self):
        compiler = shutil.which('g++') or shutil.which('clang++')
        if not compiler:
            self.skipTest('Host C++ compiler unavailable')
        folder = Path(__file__).resolve().parents[1] / 'firmware' / 'robot_ctrl'
        source = r'''
#include "motor_output.h"
#include <cassert>
#include <limits>
int main() {
  assert(motion::dutyToPwm(0) == 0);
  assert(motion::dutyToPwm(.1f) == 26);
  assert(motion::dutyToPwm(-.1f) == -26);
  assert(motion::dutyToPwm(1.5f) == 255);
  assert(motion::dutyToPwm(std::numeric_limits<float>::quiet_NaN()) == 0);
  float out = 0;
  for (int k=0; k<20; ++k) {
    float next = motion::rampDuty(out, .5f, .01f, .6f, 1.2f);
    assert(next >= out && next-out <= .00601f);
    out = next;
  }
  assert(out < .121f); // no 71% jump after start
  assert(motion::rampDuty(.7f, 0, .01f, .6f, 1.2f) == 0);
  assert(motion::rampDuty(0, .8f, 2.0f, .6f, 1.2f) < .031f);
  out = .1f;
  while (out > 0) {
    float next = motion::rampDuty(out, -.5f, .01f, .6f, 1.2f);
    assert(next >= 0); // reversal reaches zero first
    out = next;
  }
  assert(motion::rampDuty(out, -.5f, .01f, .6f, 1.2f) < 0);
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            cpp, exe = Path(tmp) / 'check.cpp', Path(tmp) / 'check.exe'
            cpp.write_text(source, encoding='utf-8')
            subprocess.run([compiler, '-std=c++11', '-Wall', '-Wextra', '-Werror',
                            '-I', str(folder), str(cpp), '-o', str(exe)], check=True, capture_output=True)
            subprocess.run([str(exe)], check=True, capture_output=True)


if __name__ == '__main__':
    unittest.main()
