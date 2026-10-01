"""Automatic check of robot_ctrl over Wi-Fi, no camera or vision needed.

    python firmware_check.py <ESP_IP>          # real robot: WHEELS OFF THE GROUND
    python firmware_check.py 127.0.0.1         # against fake_robot.py (checks this script)
    python firmware_check.py <ESP_IP> --look 10   # gripper camera (V3): grip check go/no-go

It sends real protocol v3 packets and verifies each promise from the ESP32's own
status replies: link, IDLE refuses to drive, start, driving, the 300 ms watchdog,
gripper open/close angles, stop, bad packets, and control takeover.
Ends with a PASS/FAIL table. Takes about 20 seconds.

--look N checks only the gripper camera (wheels stay off), N grip checks in each of three
setups: a stone gripped in the jaws; a stone touching the tips of the closed jaws (they push it
along, so it is trained as Single); nothing. Each look prints the camera's 5 class IDs and the
verdict (IDs mapped by autonomy.grip_check_ids from --config). FAIL if any look with a stone
comes back Empty: then run V2 (autonomy2.py).
"""
import argparse
import json
import time
from pathlib import Path

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
        s = self.wait(1.8)                                   # 360 servo: 250 deg at ~200 deg/s
        servo = (s.get('servo') or [None])[0]
        self.record(f'grip close -> {self.grip_close} deg (works in IDLE)', servo == self.grip_close, f'servo={servo}')
        self.link.send('grip', p='open')
        s = self.wait(1.8)
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


def close_jaws(link):
    link.send('grip', p='close')
    end = time.monotonic() + 1.8                              # 360 servo: 250 deg at ~200 deg/s
    while time.monotonic() < end:
        link.send('ping')
        link.poll()
        time.sleep(0.05)


def look_check(link, looks, options, held, label):
    """Ask for `looks` grip checks with the jaws closed. Returns the verdicts."""
    from autonomy import grip_verdict
    verdicts = []
    for k in range(looks):
        n = int(time.time() * 1000) % 1_000_000_000 + k          # unique per look
        link.send('look', n=n)
        end, ids = time.monotonic() + 1.5, None
        while time.monotonic() < end and ids is None:
            link.poll()
            look = (link.status or {}).get('look')
            if isinstance(look, dict) and look.get('n') == n and look.get('done'):
                ids = look.get('ids') or []
            time.sleep(0.02)
        v = 'unsure (no answer)' if ids is None else grip_verdict(
            ids, options['grip_check_ids'], options['grip_check_empty_votes'], options['grip_check_votes'])
        verdicts.append(v)
        bad = held and v == 'empty'
        print(f"{'FAIL' if bad else '    '}  {label} look {k + 1:2}: ids={ids} -> {v}")
        link.send('ping')
    return verdicts


def run_look(args):
    from autonomy import DEFAULTS
    cfg = json.loads(args.config.read_text(encoding='utf-8')) if args.config.is_file() else {}
    options = dict(DEFAULTS, **cfg.get('autonomy', {}))
    print('Grip check IDs:', options['grip_check_ids'], f"(from {args.config})")
    link = Link(args.esp_ip, args.port)
    link.send('ping')
    end = time.monotonic() + 3
    while time.monotonic() < end and not (link.status and link.status.get('gripcam') == 'ok'):
        link.send('ping')
        link.poll()
        time.sleep(0.1)
    gripcam = (link.status or {}).get('gripcam', 'no status' if not link.status else 'missing (old firmware)')
    print(f"{'PASS' if gripcam == 'ok' else 'FAIL'}  gripcam={gripcam}")
    if gripcam != 'ok':
        print('Check the wiring (T->GPIO25, R->GPIO32, 5 V, GND) and HuskyLens Protocol Type "Serial 115200".')
        return False
    setups = [('in jaws', True, 'Put ONE stone between the OPEN jaws (they will close on it)', True),
               ('at tips', True, 'Put ONE stone touching the tips of the CLOSED jaws', False),
               ('empty  ', False, 'Take the stone away (closed jaws, nothing in or at them)', False)]
    results = {}
    for label, held, prompt, open_first in setups:
        if open_first:
            link.send('grip', p='open')
            time.sleep(1.8)
        input(f"\n{prompt}, then press Enter... ")
        close_jaws(link)
        results[label] = (held, look_check(link, args.look, options, held, label))
    link.send('grip', p='open')
    print()
    for label, (held, verdicts) in results.items():
        print(f"{label}: {({v: verdicts.count(v) for v in sorted(set(verdicts))})}")
    ok = not any(held and 'empty' in verdicts for held, verdicts in results.values())
    print(f"\n{'PASS' if ok else 'FAIL'}  no stone read as Empty"
          + ('' if ok else '  -> do not use V3: run autonomy2.py'))
    empty_ok = results['empty  '][1].count('empty')
    print(f"info  empty jaws read as Empty {empty_ok}/{args.look} (the rest only cost what they did before V3)")
    return ok


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('esp_ip')
    p.add_argument('--port', type=int, default=4211)
    p.add_argument('--grip-open', type=int, default=0, help='= GRIP_OPEN_DEG in config.h')
    p.add_argument('--grip-close', type=int, default=250, help='= GRIP_CLOSE_DEG in config.h')
    p.add_argument('--look', type=int, metavar='N', help='only check the gripper camera: N grip checks '
                   'with a stone in the jaws, then N without (V3 go/no-go)')
    p.add_argument('--config', type=Path, default=Path(__file__).with_name('calib.json'),
                   help='--look: where autonomy.grip_check_ids is read from')
    args = p.parse_args()
    if args.look:
        run_look(args)
        return
    print('WHEELS OFF THE GROUND. Starting in 3 s...')
    time.sleep(3)
    c = Check(args.esp_ip, args.port, args.grip_open, args.grip_close)
    c.run()
    passed = sum(ok for _, ok, _ in c.results)
    print(f'\n{passed}/{len(c.results)} passed')
    c.link.send('stop')


if __name__ == '__main__':
    main()
