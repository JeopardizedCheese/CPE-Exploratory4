"""Compare planner settings in the simulator, on the field-fitted robot model.

    python sim_bench.py                                  # current calib.json settings
    python sim_bench.py --set turn_mode=fixed            # override autonomy values
    python sim_bench.py --physics low-battery --seeds 8

Per scenario it prints stones placed, turn hunting (turn direction reversed within 1 s),
the largest single-turn overshoot, and time with the tag lost. Simulation only: it shows
whether a change helps against the measured robot behaviour, not that the robot will do it.
"""
import argparse
import json
import math
from pathlib import Path

import autonomy
import sim
from autonomy import Planner


def run(cfg, scenario, seeds, seconds, physics):
    placed = hunting = frames = lost = 0
    overshoot = []
    original = Planner.step
    for seed in seeds:
        log = []

        def step(self, now, pose, *a, **k):
            result = original(self, now, pose, *a, **k)
            log.append((now, result[0], result[1], self.debug.get('reason', ''),
                        self.debug.get('turn_moved_deg'), self.debug.get('turn_planned_deg')))
            return result
        Planner.step = step
        try:
            r = autonomy.run_sim(cfg, autonomy.scenario(cfg, scenario, seed), seconds=seconds,
                                 params=physics, seed=seed)
        finally:
            Planner.step = original
        placed += r['correct']
        last = None
        for t, l, rr, reason, _, _ in log:
            frames += 1
            lost += reason == 'tag_missing'
            if l * rr < 0:
                d = math.copysign(1, l)
                if last and last[1] != d and t - last[0] < 1.0:
                    hunting += 1
                last = (t, d)
    return placed, hunting, 100 * lost / max(frames, 1)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--config', type=Path, default=Path(__file__).with_name('calib.json'))
    ap.add_argument('--physics', choices=['charged', 'low-battery', 'ideal'], default='charged')
    ap.add_argument('--seeds', type=int, default=6)
    ap.add_argument('--seconds', type=float, default=180)
    ap.add_argument('--set', action='append', default=[], metavar='KEY=VALUE',
                    help='override calib.json "autonomy" values, e.g. turn_mode=fixed or cruise=0.25')
    args = ap.parse_args()
    cfg = json.loads(args.config.read_text(encoding='utf-8'))
    for item in args.set:
        key, value = item.split('=', 1)
        try:
            value = json.loads(value)
        except ValueError:
            pass                                  # plain string such as fixed
        cfg.setdefault('autonomy', {})[key] = value
    physics = {'charged': sim.FIELD_PARAMS, 'low-battery': sim.LOW_BATTERY_PARAMS, 'ideal': {}}[args.physics]
    print(f"physics={args.physics} seeds={args.seeds} seconds={args.seconds:.0f} overrides={args.set or 'none'}")
    for scenario in ('scattered', 'pile'):
        placed, hunting, lost = run(cfg, scenario, range(args.seeds), args.seconds, physics)
        print(f'  {scenario:9}  placed {placed:3}  turn hunting {hunting:4}  tag lost {lost:4.1f}% of frames')


if __name__ == '__main__':
    main()
