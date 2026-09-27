"""Stand-in for firmware/robot_ctrl on the PC: same UDP v3 protocol, states, 300 ms
drive watchdog, 5-minute timer, rate-limited servos and status packets.

    python fake_robot.py                          # terminal 1
    python teleop.py 127.0.0.1                    # terminal 2 (or autonomy.py 127.0.0.1 ...)

It simulates only the firmware, not the camera: autonomy.py against it will wait for
a pose. For a full closed loop without hardware use `python autonomy.py --sim --show`.
"""
import argparse
import json
import socket
import time
from pathlib import Path

from sim import SimRobot


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--port', type=int, default=4211)
    p.add_argument('--run-seconds', type=float, default=0, help='0 = no run timer (like the firmware)')
    p.add_argument('--config', type=Path, default=Path(__file__).with_name('calib.json'))
    args = p.parse_args()
    cfg = json.loads(args.config.read_text())
    w, h = cfg['arena']['size_mm']
    robot = SimRobot(cfg, [], w / 2, h / 2, 0, params={'run_time_s': args.run_seconds or None})
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', args.port))
    sock.setblocking(False)
    session, last_seq, last_rx, peer = None, 0, 0.0, None
    t0 = time.monotonic()
    last_status = last_print = 0.0
    print(f'fake robot on udp/{args.port}')
    while True:
        now = time.monotonic() - t0
        while True:
            try:
                data, addr = sock.recvfrom(2048)
            except BlockingIOError:
                break
            try:
                pkt = json.loads(data)
            except ValueError:
                continue
            if pkt.get('v') != 3 or not isinstance(pkt.get('s'), str) or len(pkt['s']) != 12:
                continue
            if pkt['s'] != session:                       # takeover only after silence, like the firmware
                if session and now - last_rx <= 0.3:
                    continue
                session, last_seq = pkt['s'], 0
            if pkt.get('q', 0) <= last_seq:
                continue
            last_seq, last_rx, peer = pkt['q'], now, addr
            fields = {k: v for k, v in pkt.items() if k not in ('v', 's', 'q', 'c')}
            before = robot.state
            robot.command(pkt.get('c', ''), now, **fields)
            if robot.state != before:
                print(f'{now:7.2f}  {before} -> {robot.state} ({robot.why})')
        prev = robot.state
        robot.update(now)
        if robot.state != prev:
            print(f'{now:7.2f}  {prev} -> {robot.state} ({robot.why})')
        if peer and now - last_status >= 0.2:
            last_status = now
            status = dict(robot.status(now), rx_age_ms=int((now - last_rx) * 1000))
            sock.sendto(json.dumps(status).encode(), peer)
        if now - last_print >= 1.0:
            last_print = now
            print(f'{now:7.2f}  {robot.state:7s} wheels {robot.out[0]:+.2f} {robot.out[1]:+.2f}  '
                  f'servo {[round(v) for v in robot.servo]}  x={robot.x:.0f} y={robot.y:.0f}')
        time.sleep(0.005)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
