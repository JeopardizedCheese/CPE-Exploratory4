"""Automatic check of robot_ctrl over Wi-Fi, no camera or vision needed.

    python firmware_check.py <ESP_IP>          # real robot: WHEELS OFF THE GROUND
    python firmware_check.py 127.0.0.1         # against fake_robot.py (checks this script)

It sends real protocol v3 packets and verifies each promise from the ESP32's own
status replies: link, IDLE refuses to drive, start, driving, the 300 ms watchdog,
gripper open/close angles, stop, bad packets, and control takeover.
Ends with a PASS/FAIL table. Takes about 20 seconds.
"""
import argparse
import time

from teleop import Link


class Check:
    def __init__(self, ip, port, grip_open, grip_close):
        self.link = Link(ip, port)
        self.grip_open, self.grip_close = grip_open, grip_close
        self.results = []

    def wait(self, seconds, drive=None):
        """Let time pass while polling status; optionally keep sending a drive command."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if drive is not None:
                self.link.send('drive', l=drive[0], r=drive[1])
            self.link.poll()
            time.sleep(0.05)
        return self.link.status

    def record(self, name, ok, detail=''):
        self.results.append((name, ok, detail))
        print(f"{'PASS' if ok else 'FAIL'}  {name}  {detail}")

    def run(self):
        self.link.send('ping')
        s = self.wait(0.8)
        self.record('link: status replies arrive', bool(s), '' if s else 'no reply: check IP, Wi-Fi, firmware')
        if not s:
            return
        if s.get('state') != 'IDLE':
            self.link.send('stop')
            s = self.wait(0.5)
        self.record('starts in IDLE', s.get('state') == 'IDLE', f"state={s.get('state')}")

        s = self.wait(0.8, drive=(0.5, 0.5))
        self.record('IDLE refuses to drive', s.get('l') == 0 and s.get('r') == 0, f"l={s.get('l')} r={s.get('r')}")

        self.link.send('grip', p='close')
        s = self.wait(1.0)
        servo = (s.get('servo') or [None])[0]
        self.record(f'grip close -> {self.grip_close} deg (works in IDLE)', servo == self.grip_close, f'servo={servo}')
        self.link.send('grip', p='open')
        s = self.wait(1.0)
        servo = (s.get('servo') or [None])[0]
        self.record(f'grip open -> {self.grip_open} deg', servo == self.grip_open, f'servo={servo}')

        self.link.send('start')
        s = self.wait(0.5)
        self.record('start -> RUNNING', s.get('state') == 'RUNNING', f"state={s.get('state')}")

        s = self.wait(1.0, drive=(0.5, 0.5))
        self.record('drive forward', s.get('l', 0) > 0 and s.get('r', 0) > 0, f"l={s.get('l')} r={s.get('r')}")
        s = self.wait(0.6, drive=(-0.5, 0.5))
        self.record('drive spin left (l<0, r>0)', s.get('l', 0) < 0 < s.get('r', 0), f"l={s.get('l')} r={s.get('r')}")

        self.wait(0.3, drive=(0.5, 0.5))
        s = self.wait(0.8)                                   # silence > 300 ms
        self.record('watchdog: silence stops wheels', s.get('l') == 0 and s.get('r') == 0 and s.get('state') == 'RUNNING',
                    f"l={s.get('l')} r={s.get('r')} state={s.get('state')}")

        self.link.sock.sendto(b'{"v":3,"s":"garbage"', self.link.addr)      # malformed packet
        s = self.wait(0.5, drive=None)
        self.record('bad packet ignored (no crash)', s.get('state') == 'RUNNING' and self.link.status_age() < 0.6)

        s = self.wait(0.8, drive=(1.5, 0.5))                 # out of range -> must not drive
        self.record('out-of-range drive rejected', s.get('l') == 0 and s.get('r') == 0, f"l={s.get('l')} r={s.get('r')}")

        other = Link(self.link.addr[0], self.link.addr[1])   # second controller while first is active
        for _ in range(10):
            self.link.send('drive', l=0.4, r=0.4)
            other.send('stop')
            time.sleep(0.03)
        s = self.wait(0.3, drive=(0.4, 0.4))
        self.record('no takeover while controller active', s.get('state') == 'RUNNING', f"state={s.get('state')}")

        self.link.send('stop')
        s = self.wait(0.5, drive=(0.5, 0.5))
        self.record('stop -> IDLE, wheels off', s.get('state') == 'IDLE' and s.get('l') == 0, f"state={s.get('state')}")
        self.link.send('start')
        s = self.wait(0.5)
        self.record('start again after stop', s.get('state') == 'RUNNING', f"state={s.get('state')}")
        self.link.send('stop')
        self.wait(0.3)

        time.sleep(0.4)                                      # first controller silent -> takeover allowed
        other.send('ping')
        for _ in range(5):
            other.send('ping')
            other.poll()
            time.sleep(0.1)
        self.record('takeover after silence', other.status is not None and other.status_age() < 1)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('esp_ip')
    p.add_argument('--port', type=int, default=4211)
    p.add_argument('--grip-open', type=int, default=0, help='= GRIP_OPEN_DEG in config.h')
    p.add_argument('--grip-close', type=int, default=100, help='= GRIP_CLOSE_DEG in config.h')
    args = p.parse_args()
    print('WHEELS OFF THE GROUND. Starting in 3 s...')
    time.sleep(3)
    c = Check(args.esp_ip, args.port, args.grip_open, args.grip_close)
    c.run()
    passed = sum(ok for _, ok, _ in c.results)
    print(f'\n{passed}/{len(c.results)} passed')
    c.link.send('stop')


if __name__ == '__main__':
    main()
