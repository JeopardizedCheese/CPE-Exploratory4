"""Automatic stress test for robot_ctrl: drives and grips nonstop and watches for trouble.

    python stress_test.py <ESP_IP>                 # 5 minutes, wheels OFF the ground
    python stress_test.py <ESP_IP> --minutes 15    # longer
    python stress_test.py 127.0.0.1 --minutes 1    # against fake_robot.py

It repeats a pattern like an autonomous run (forward, spin left/right, reverse,
close/open the gripper under load) with drive packets 20x per second, and records:
  - status replies that stopped arriving (Wi-Fi drops)
  - ESP32 reboots (state falls back to IDLE with a full timer, or 'boot' reason)
  - how long the gripper takes to reach its angle
Ctrl+C stops it early (sends stop). A summary prints at the end.
"""
import argparse
import time

from teleop import Link

PATTERN = [                       # (seconds, left, right, grip)
    (1.5, 0.6, 0.6, None),
    (1.0, -0.6, 0.6, None),
    (1.0, 0.6, -0.6, None),
    (1.5, -0.6, -0.6, None),
    (0.8, 0.0, 0.0, 'close'),
    (1.5, 0.5, 0.5, None),        # drive while gripping (servo + motors together)
    (1.0, 0.6, -0.6, None),
    (0.8, 0.0, 0.0, 'open'),
    (1.0, 1.0, 1.0, None),        # full power
]


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('esp_ip')
    p.add_argument('--port', type=int, default=4211)
    p.add_argument('--minutes', type=float, default=5)
    p.add_argument('--grip-open', type=int, default=0)
    p.add_argument('--grip-close', type=int, default=45)
    args = p.parse_args()

    link = Link(args.esp_ip, args.port)
    print('WHEELS OFF THE GROUND (or plenty of room). Starting in 3 s... Ctrl+C to stop.')
    time.sleep(3)
    link.send('start')

    t0 = time.monotonic()
    end = t0 + args.minutes * 60
    stats = {'cycles': 0, 'status_gaps': 0, 'longest_gap_s': 0.0, 'reboots': 0,
             'unexpected_states': [], 'restarts': 0, 'grip_times': []}
    last_status_t = time.monotonic()
    grip_wait = None                       # (target_angle, sent_at)
    prev_state = None

    try:
        while time.monotonic() < end:
            for seconds, l, r, grip in PATTERN:
                if grip:
                    link.send('grip', p=grip)
                    grip_wait = (args.grip_close if grip == 'close' else args.grip_open, time.monotonic())
                step_end = time.monotonic() + seconds
                while time.monotonic() < step_end:
                    link.send('drive', l=l, r=r)
                    link.poll()
                    s = link.status
                    now = time.monotonic()
                    if s and link.status_t > last_status_t:
                        gap = link.status_t - last_status_t
                        if gap > 1.0:
                            stats['status_gaps'] += 1
                        stats['longest_gap_s'] = max(stats['longest_gap_s'], gap)
                        last_status_t = link.status_t
                        state = s.get('state')
                        if state != prev_state:
                            if prev_state == 'RUNNING' and state == 'IDLE':
                                if s.get('why') == 'boot':
                                    stats['reboots'] += 1
                                    print(f'[{now - t0:6.1f}s] REBOOT: robot restarted (check power / brownout)')
                                else:
                                    stats['unexpected_states'].append((round(now - t0, 1), state, s.get('why')))
                                    print(f'[{now - t0:6.1f}s] unexpected IDLE ({s.get("why")})')
                            prev_state = state
                        if state == 'IDLE':                          # keep the test going
                            link.send('start')
                            stats['restarts'] += 1
                        if grip_wait and s.get('servo') and s['servo'][0] == grip_wait[0]:
                            stats['grip_times'].append(now - grip_wait[1])
                            grip_wait = None
                    elif now - last_status_t > 1.0 and stats['status_gaps'] == 0:
                        print(f'[{now - t0:6.1f}s] no status for {now - last_status_t:.1f} s')
                    time.sleep(0.05)
            stats['cycles'] += 1
            print(f'[{time.monotonic() - t0:6.1f}s] cycle {stats["cycles"]} ok  '
                  f'rssi={(link.status or {}).get("rssi")}  state={(link.status or {}).get("state")}')
    except KeyboardInterrupt:
        print('\nstopped early')
    finally:
        link.send('drive', l=0, r=0)
        link.send('stop')

    g = stats['grip_times']
    print('\n=== STRESS TEST SUMMARY ===')
    print(f"ran {(time.monotonic() - t0) / 60:.1f} min, {stats['cycles']} pattern cycles")
    print(f"status gaps > 1 s: {stats['status_gaps']}   longest gap: {stats['longest_gap_s']:.2f} s")
    print(f"reboots suspected: {stats['reboots']}")
    print(f"unexpected stops: {stats['unexpected_states'] or 'none'}")
    print(f"re-starts sent: {stats['restarts']}")
    if g:
        print(f"gripper reached its angle in {min(g):.2f}-{max(g):.2f} s ({len(g)} moves)")
    ok = stats['reboots'] == 0 and stats['status_gaps'] == 0 and not stats['unexpected_states']
    print('RESULT:', 'PASS' if ok else 'CHECK THE LINES ABOVE')


if __name__ == '__main__':
    main()
