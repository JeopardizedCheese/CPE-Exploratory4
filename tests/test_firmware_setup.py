"""Host checks of the actual setup helpers; not an ESP32/Wi-Fi hardware build."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


FOLDER = Path(__file__).resolve().parents[1]/'firmware/robot_ctrl'
SKETCH = (FOLDER/'robot_ctrl.ino').read_text(encoding='utf-8')


class FirmwareSetupTests(unittest.TestCase):
    def compile_and_run(self, source, bad_limit=False):
        compiler = shutil.which('g++') or shutil.which('clang++')
        if not compiler:
            self.skipTest('Host C++ compiler unavailable')
        with tempfile.TemporaryDirectory() as folder:
            cpp, exe = Path(folder)/'check.cpp', Path(folder)/'check.exe'
            cpp.write_text(source, encoding='utf-8')
            if bad_limit:
                cfg = (FOLDER/'config.h').read_text(encoding='utf-8').replace('#define SERVO_MAX_DEG 70', '#define SERVO_MAX_DEG 55')
                (Path(folder)/'config.h').write_text(cfg, encoding='utf-8')
            result = subprocess.run([compiler, '-std=c++11', '-Wall', '-Wextra', '-Werror',
                                     '-I', str(FOLDER), str(cpp), '-o', str(exe)], capture_output=True, text=True)
            if bad_limit:
                self.assertNotEqual(result.returncode, 0)
                self.assertIn('GRIP_CLOSE_DEG is outside servo limits', result.stderr)
            else:
                self.assertEqual(result.returncode, 0, result.stderr)
                subprocess.run([str(exe)], check=True, capture_output=True)

    def test_actual_set_servo_reaches_70_instead_of_clamping_to_55(self):
        function = 'bool setServo('+SKETCH.split('bool setServo(', 1)[1].split('\nvoid updateServo(', 1)[0]
        self.compile_and_run(r'''
#include "config.h"
#include <cassert>
#include <cmath>
using std::isfinite;
#define constrain(x, lo, hi) ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))
float servoTarget = 0, servoPos = 0, written = -1;
bool servoActive = false;
void servoWrite(float value) { written = value; }
'''+function+r'''
int main() {
  assert(setServo(GRIP_CLOSE_DEG));
  assert(servoTarget == 70 && servoPos == 70 && written == 70);
  assert(setServo(999) && servoTarget == 70);
  assert(setServo(-1) && servoTarget == 0);
  assert(!setServo(NAN));
}
''')

    def test_incompatible_close_limit_cannot_compile(self):
        self.compile_and_run('#include "config.h"\nint main() {}\n', bad_limit=True)

    def test_network_rebinds_after_disconnect_ip_change_and_failed_bind(self):
        function = 'void updateNetwork('+SKETCH.split('void updateNetwork(', 1)[1].split('\nvoid handlePacket(', 1)[0]
        self.compile_and_run(r'''
#include "config.h"
#include <cassert>
#include <cstdint>
#include <string>
struct IPAddress {
  uint32_t value;
  IPAddress(int a=0,int b=0,int c=0,int d=0):value((uint32_t(a)<<24)|(b<<16)|(c<<8)|d) {}
  bool operator==(const IPAddress &other) const { return value==other.value; }
  std::string toString() const { return std::to_string(value); }
};
const int WL_CONNECTED = 3;
struct WifiStub {
  int state=0;
  IPAddress ip;
  IPAddress localIP() { return ip; }
  IPAddress gatewayIP() { return IPAddress(172,20,10,1); }
  IPAddress subnetMask() { return IPAddress(255,255,255,240); }
  int status() { return state; }
} WiFi;
struct UdpStub {
  int attempts=0, stops=0;
  bool bindOK=true;
  bool begin(int port) { assert(port==4211); ++attempts; return bindOK; }
  void stop() { ++stops; }
} udp;
struct SerialStub {
  void println(const char*) {}
  template<class... Args> void printf(const char*, Args...) {}
} Serial;
IPAddress announcedIp;
bool udpReady=false;
unsigned long lastBindAttempt=0;
uint16_t peerPort=1234;
int motorStops=0;
void motorsOff() { ++motorStops; }
'''+function+r'''
int main() {
  assert(USE_STATIC_IP == 0);
  updateNetwork(15000);  // phone absent at startup
  assert(!udpReady && udp.attempts==0);
  WiFi.state=WL_CONNECTED;
  updateNetwork(15100);  // connected but DHCP still pending
  assert(!udpReady && udp.attempts==0);
  WiFi.ip=IPAddress(172,20,10,2);
  updateNetwork(15200);
  assert(udpReady && udp.attempts==1 && announcedIp==WiFi.ip);
  updateNetwork(15210);
  assert(udp.attempts==1);
  WiFi.ip=IPAddress(172,20,10,3);
  updateNetwork(15300);
  assert(!udpReady && motorStops==1 && peerPort==0);
  updateNetwork(16200);
  assert(udpReady && udp.attempts==2 && announcedIp==WiFi.ip);
  peerPort=4321;
  WiFi.state=0;
  updateNetwork(16300);
  assert(!udpReady && motorStops==2 && peerPort==0);
  WiFi.state=WL_CONNECTED;
  udp.bindOK=false;
  updateNetwork(17300);
  assert(!udpReady && udp.attempts==3);
  updateNetwork(17400);
  assert(udp.attempts==3);  // no busy retry every firmware loop
  udp.bindOK=true;
  updateNetwork(18300);
  assert(udpReady && udp.attempts==4);
}
''')


if __name__ == '__main__':
    unittest.main()
