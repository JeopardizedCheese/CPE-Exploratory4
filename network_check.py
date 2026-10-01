"""Check robot UDP status without commanding motors or the gripper.

    python network_check.py <IP_FROM_SERIAL_MONITOR>

Close other control programs first. The v3 ping establishes a controller session;
an existing controller can reject it until that controller has stopped sending.
This tool sends only ping. firmware_check.py, by contrast, actuates the robot.
"""
import argparse
import ipaddress
import json
import math
import socket
import time
import uuid


def probe(ip, port=4211, seconds=3.0):
    address = ipaddress.IPv4Address(ip)
    if address.is_multicast or address.is_unspecified or int(address) == 0xffffffff:
        raise ValueError('Use the individual robot IPv4 address shown in Serial Monitor')
    if not 1 <= port <= 65535 or not math.isfinite(seconds) or not 0 < seconds <= 60:
        raise ValueError('Use port 1..65535 and a duration above 0 and at most 60 seconds')
    session, seq = uuid.uuid4().hex[:12], 0
    statuses, times = [], []
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(('', 0))
        started = time.monotonic()
        deadline, next_ping = started+seconds, started
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_ping:
                seq += 1
                sock.sendto(json.dumps({'v': 3, 's': session, 'q': seq, 'c': 'ping'}).encode(), (str(address), port))
                next_ping = now+.1
            sock.settimeout(max(.001, min(.05, deadline-time.monotonic())))
            try:
                data, sender = sock.recvfrom(2048)
            except (socket.timeout, ConnectionResetError):
                continue
            if sender != (str(address), port):
                continue
            try:
                status = json.loads(data)
            except (ValueError, UnicodeError):
                continue
            if not isinstance(status, dict) or status.get('state') not in ('IDLE', 'RUNNING'):
                continue
            if status.get('session', session) != session:
                continue
            statuses.append(status)
            times.append(time.monotonic())
    return {'ip': str(address), 'port': port, 'replies': len(statuses),
            'status': statuses[-1] if statuses else None,
            'last_status_age_s': time.monotonic()-times[-1] if times else None,
            'max_status_gap_s': max((b-a for a, b in zip(times, times[1:])), default=None)}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('esp_ip')
    parser.add_argument('--port', type=int, default=4211)
    parser.add_argument('--seconds', type=float, default=3.0)
    parser.add_argument('--grip-close', type=float, default=70)
    args = parser.parse_args()
    if not math.isfinite(args.grip_close) or not 0 <= args.grip_close <= 180:
        parser.error('--grip-close must be between 0 and 180 degrees')
    try:
        result = probe(args.esp_ip, args.port, args.seconds)
    except (ValueError, OSError) as e:
        raise SystemExit('Network check failed: '+str(e)) from None
    if not result['replies']:
        raise SystemExit('No robot status. Check the current Serial IP, same hotspot, '
                         'iPhone Maximize Compatibility, firewall, and other running controllers.')
    s = result['status']
    print(f"Replies: {result['replies']}; robot {result['ip']}:{result['port']}; state {s['state']}; RSSI {s.get('rssi', 'unknown')}")
    print(f"Last status age: {result['last_status_age_s']:.3f}s; max status gap: {result['max_status_gap_s']}")
    print('Status gaps are not ping round-trip latency or a camera-delay measurement.')
    if result['last_status_age_s'] > .6:
        raise SystemExit('Status stopped arriving; inspect the link before starting autonomy.')
    if s['state'] != 'IDLE':
        raise SystemExit('Robot reports RUNNING. Stop the active run first; this checker sends no stop command.')
    values = [s.get(k) for k in ('grip_close_deg', 'servo_min_deg', 'servo_max_deg')]
    if all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in values):
        close, minimum, maximum = values
        print(f'Firmware close target: {close} deg; servo limits: {minimum}..{maximum} deg')
        if abs(close-args.grip_close) > .01 or not minimum <= close <= maximum:
            raise SystemExit('Gripper configuration mismatch. Check config.h and upload the corrected firmware.')
    else:
        print('Firmware does not report gripper limits; update firmware to verify its close setting here.')
    print('UDP link check complete. No motor or gripper commands were sent; physical motion is untested.')


if __name__ == '__main__':
    main()
