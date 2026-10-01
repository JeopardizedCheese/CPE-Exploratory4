"""Replay recorded runs through V1 and V2 vision side by side; crop sheets of the new V2 targets.

    .venv/bin/python evidence/v2-replay/compare_v1_v2.py \
        20260930-091711-b90153,20260930-092017-7dd201,20260930-092247-df6aea evidence/v2-replay/new
    (optional: vision_key=json ... to change a V2 switch, e.g. own_reach_mm=12)

Run from the repo root. Needs runs/autonomy/<run>/ (video.avi + config.json; gitignored, original
laptop only) and the 09:15 empty-field reference background_0915.png (repo root, untracked).
Prints counts (targets, frames without a target, V1 targets lost in V2, new V2 targets by kind:
edge / outermost / other) and writes <out>_edge.png, <out>_outermost.png, <out>_other.png:
one 60 x 60 px crop per distinct new stone, arrow = approach (green edge, magenta outermost).
The reference guard's clock is driven from frame time so replays repeat exactly.
"""
import sys, json, math, collections, types
sys.path.insert(0, '.')
import cv2, numpy as np
import reference_guard, profiles
from perception import Perception
runs, out = sys.argv[1].split(','), sys.argv[2]
extra = [a.split('=', 1) for a in sys.argv[3:]]
st = collections.Counter(); per_stone = collections.Counter(); tiles = {'edge': [], 'outermost': [], 'other': []}; seen = []
def match(t, ts):
    return any(u['color'] == t['color'] and math.hypot(u['x'] - t['x'], u['y'] - t['y']) < 20 for u in ts)
for run in runs:
    cfg = json.load(open(f'runs/autonomy/{run}/config.json')); bg = cv2.imread('background_0915.png')
    old = json.loads(json.dumps(cfg)); profiles.apply_profile(old, 'v1')
    new = json.loads(json.dumps(cfg)); profiles.apply_profile(new, 'v2')
    for k, v in extra:
        new['vision'][k] = json.loads(v)
    clock = {'t': 0.0}
    reference_guard.time = types.SimpleNamespace(monotonic=lambda: clock['t'])
    a, b = Perception(old, bg), Perception(new, bg)
    v = cv2.VideoCapture(f'runs/autonomy/{run}/video.avi'); i = -1
    while True:
        ok, raw = v.read(); i += 1
        if not ok: break
        clock['t'] = i / 30
        sa, sb = a.step(raw, i / 30), b.step(raw, i / 30)
        st['frames'] += 1; st['v1_targets'] += len(sa.targets); st['v2_targets'] += len(sb.targets)
        st['v1_frames_without'] += not sa.targets; st['v2_frames_without'] += not sb.targets
        st['v1_coloured_obs'] += sum(o['color'] != 0 for o in sa.observations)
        st['v2_coloured_obs'] += sum(o['color'] != 0 for o in sb.observations)
        st['v2_buried_obs'] += sum(o.reason == 'pile_buried' for o in sb.raw_observations)
        st['lost'] += sum(not match(t, sb.targets) for t in sa.targets)
        raw_t = [o for o in sb.raw_observations if o.stable and o.isolated] if sb.status == 'ok' else []
        for t, o in zip(sb.targets, raw_t):
            if match(t, sa.targets): continue
            kind = {'pile_outermost': 'outermost', 'pile_edge': 'edge'}.get(o.reason, 'other')
            st['new_' + kind] += 1
            per_stone[(kind, run, round(t['x'] / 50), round(t['y'] / 50), t['color'])] += 1
            if not any(math.hypot(t['x'] - x, t['y'] - y) < 50 and k == kind for x, y, k in seen):
                seen.append((t['x'], t['y'], kind))
                cx, cy = int(round(o.x)), int(round(o.y)); img = sb.frame.copy()
                h = math.radians(t.get('approach_deg') or 0)
                col = {'edge': (0, 255, 0), 'outermost': (255, 0, 255), 'other': (0, 200, 255)}[kind]
                cv2.arrowedLine(img, (int(cx - math.cos(h) * 25), int(cy - math.sin(h) * 25)), (cx, cy), col, 1, tipLength=.3)
                crop = cv2.copyMakeBorder(img, 30, 30, 30, 30, cv2.BORDER_CONSTANT)[cy:cy + 60, cx:cx + 60]
                crop = cv2.resize(crop, None, fx=4, fy=4, interpolation=cv2.INTER_NEAREST)
                cv2.putText(crop, f'{run[9:15]} f{i} c{t["color"]}', (3, 12), 0, .4, (0, 0, 255), 1)
                tiles[kind].append(crop)
print(dict(st))
for k, n in sorted(per_stone.items()): print('  ', k, n)
for kind, ts in tiles.items():
    print(kind, 'distinct', len(ts))
    if not ts: continue
    while len(ts) % 6: ts.append(np.zeros_like(ts[0]))
    cv2.imwrite(f'{out}_{kind}.png', np.vstack([np.hstack(ts[k:k + 6]) for k in range(0, len(ts), 6)]))
