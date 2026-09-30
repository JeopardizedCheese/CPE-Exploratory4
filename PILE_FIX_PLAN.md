# Plan: pile-mode fix (touching stones shown as "?")

Status 2026-10-01: **designed and decided, not built.** A prototype exists (code in the appendix). The next session
builds it, following this file. Build test-first; the decisions below are settled, so don't re-open them. Readiness
was checked on 2026-10-01 (plan matches the code, 163 tests pass, replay runs and the 09:15 reference are on disk);
the user will say when to build.

## The problem (evidence)

The robot drives back to the park point ("the start") although stones are on the field. Over the 09-29 night and
09-30 runs, PARK took 311 s: 227 s with no target from vision, 84 s with a target the planner refused (skip list,
wall margin: see "Related, not in scope").

Stones that touch (the start pile, pairs, a stone against the jaws) merge into one colour-0 blob ("?" in
`detect_live.py`). Pile mode (`vision.directional_candidates`) is meant to pick the pile's edge stones: it splits the
blob into single-colour regions and accepts one if a gripper-wide corridor (60 x 80 mm) away from the pile is free.
It never succeeds, because only a stone's **saturated core** counts as the stone (`own` = colour region + a 25 mm
circle around the region's centroid). The stone's own pale highlight and blurred rim (e.g. HSV 84/74/208 on a cyan
stone whose range needs S ≥ 230) lie outside both, count as "blocked", and block the stone's own corridor. In the
0920 pile (`runs/autonomy/20260930-092017-7dd201`, frame 20) not one of 16 candidate regions had a free corridor in
any direction, not even the outermost teal stone with bare floor beside it. Same failure: 0929-234618 and -234639
(start pile, 10 s PARK), 0930-062814 (a touching cyan and orange pair).

## The fix

In pile mode, a blob pixel that votes for no colour belongs to the **nearest** single-colour region, if it is within
`own_reach_mm` (15 mm) of it. `own` = the region + those pixels. Everything else (size check, fraction, dominance,
corridor test) stays as it is, but works on this `own`.

Prototype result on the 3 replayable runs (0917, 0920, 0922; 3090 frames): targets 2057 → 3137, frames with no
target 1747 → 1350, **0 of today's targets lost**, and every sampled new target was a real stone with an approach
pointing away from its neighbours. On the 0920 pile, 2–3 edge stones become targets.

## Decisions (user, 2026-10-01; don't re-open)

| # | Decision |
|---|---|
| Q1 | Build now, not together with the crash code. Clusters remain after the crash, and the crash code needs this anyway. |
| Q2 | The robot may take edge stones from the intact start pile before any crash code exists. |
| Q3 | `vision.gripper_width_mm` stays 60 until the new arm exists (then: measured jaw opening + ~10 mm). |
| Q4 | Config switch `vision.pile_edge_pixels`: `"nearest"` (fix) or `"legacy"` (current code, byte-identical). |
| Q5/Q11 | No field testing time is left. The bar to push is in "Acceptance bar" below. |
| Q6 | Default `"nearest"`. |
| Q7 | `vision.own_reach_mm` = 15. |
| Q8 | Confidence stays `fraction × dominance`, as today. The fix only changes which pixels belong to a stone. |
| Q9 | One commit per logical change. The pile fix **stays local** until the user has seen the acceptance evidence. |
| Q10 | `--set` accepts named vision keys: `--set vision.pile_edge_pixels=legacy`, `--set vision.own_reach_mm=12`; typos rejected. |
| Q12 | A pile stone's centre (where the gripper aims) is the centroid of `own` (core + pale rim), in `"nearest"` mode. In `"legacy"` mode it stays the core centroid. |

User context: at match start all gems are clumped into one big pile, and the team will make the robot crash into
it. **Do not write crash code; the user will provide it.** Keep the big pile's colour-0 observation in the output
(the fix only adds edge-stone targets beside it; it already does).

Why Q12: legacy uses the centroid of the saturated core, which leans toward the stone's shadowed side; the centroid
of `own` (core + rim) is closer to the true centre.

Build detail (not a user decision): count votes as today, on the pixels that really voted (`region & region_all` in
legacy terms), not on the morphologically closed region the appendix uses, so confidence stays exactly as Q8 says.

## Build steps

1. **Tests first** (`tests/test_pile.py`, alongside the existing pile tests):
   - A synthetic pile: stones of different colours touching each other, each with a 1–2 px pale rim (low saturation,
     same hue) and bare floor on the outside. `"legacy"` finds no edge stones; `"nearest"` finds the outer ones, each
     with `approach_deg` pointing away from the pile. Check this is a real regression test: it must fail on today's code.
   - Label mapping: two regions of different colours with pale pixels between them; each pale pixel goes to the
     nearer region. This guards the `DIST_LABEL_PIXEL` ordering assumption in the appendix code.
   - An isolated single stone (no pile): `Detector.process` output is identical in both modes.
   - Stones inside the intact pile (neighbours all around) are **not** targets.
   - `tests/test_autonomy.py`: `apply_overrides` accepts `vision.pile_edge_pixels=legacy|nearest` and
     `vision.own_reach_mm=<positive number>`; it rejects other `vision.*` keys and bad values.
2. **`vision.py` `directional_candidates`** (~line 327): add the `"nearest"` branch from the appendix, selected by
   `options.get('pile_edge_pixels', 'nearest')`. Keep the legacy code path byte-identical. Keep the returned tuple
   and the confidence `fraction × dominance`, with dominance computed as today (other colours' votes within
   `own_radius_mm` of the centre).
3. **`autonomy.py` `apply_overrides`** (~line 738): accept `vision.<key>` for `pile_edge_pixels` and `own_reach_mm`
   only. They go to `cfg['vision']`. Update the `--set` help text. Overrides are applied before `Perception` is built
   and before the run's `config.json` is saved (checked), so the switch is also recorded per run.
4. **`calib.json`**: no change needed; the defaults are in code. The user's working copy has uncommitted edits
   (grip 205 etc.), so **don't stage `calib.json`**.
5. **Docs**: CHANGELOG (EN + TH), HANDOFF.md (remove this item from open work, list the switch), README_ROBOT.md if
   it lists run flags.

## Acceptance bar (Q11), before anything is pushed

1. All tests pass (163 today + the new ones).
2. Replay of 0917, 0920 and 0922 with the **final** code (appendix script): 0 of today's targets lost; report the
   target count and the frames with no target.
3. A crop sheet of every distinct new target: a real stone, with an approach away from its neighbours. **The user
   looks at it and decides.** If they dislike any crop, it stays local.
4. `--set vision.pile_edge_pixels=legacy` gives today's output on a replay (hash-identical).

The simulator can't judge this: `sim.py` never calls `vision.py` (it makes its own detections), so `sim_bench` can
neither show the gain nor any harm. Don't cite a sim number for this change.

Replays need the 09:15 empty-field reference. `background.png` is gitignored and was never committed, so git can't
restore it. A copy is `background_0915.png` in the repo root (untracked on purpose, original author's laptop only;
don't commit it). The team recalibrates every run, which overwrites `background.png`, so replays read the copy.

## Related, not in scope (found in the same study; the user has not decided)

- Planner: after a failure a stone is skipped for 25 s (`skip_s`). When it is the only stone, the robot parks
  (0917: lime stone, 33.7–55.7 s). The wall keep-out tests only the one `approach_deg` vision gives, and that can jump
  after BACKOFF (0917 cyan: −117° → −0.5°, then rejected). Sim-testable.
- Stones on a zone's edge are mostly inside `exclude_polygons` → a "?" sliver (by design).
- Vision (study 2026-10-01, replay numbers in the session notes): a robot mask with parallax (convex hull of the
  floor footprint and the footprint raised to tag height) cuts colour-0 self-blobs near the robot 22.0k → 13.3k
  mm²/frame. A shadow rule (darker than the reference, same chromaticity) raises targets/frame 0.67 → 0.87 and lets a
  stone in the open jaws keep its colour. Both are replay-only; the hull also needs `Planner._under_robot` and
  `sim.py` updated.

## Appendix A: prototype `"nearest"` branch

Tested only by replay; clean it up and cover it with the tests above. In the prototype, fraction and dominance were
replaced by fraction alone; restore dominance (Q8).

```python
# inside directional_candidates, after component/votable/blocked/color_masks are windowed
own_reach = options.get('own_reach_mm', 15) / scale
regions = np.zeros(component.shape, np.int32)       # every single-colour region, numbered 1..n
info = []
for cid, m in color_masks.items():
    closed = cv2.morphologyEx((component & votable & (m > 0)).astype(np.uint8), cv2.MORPH_CLOSE, kernel)
    n, lab, stats, _ = cv2.connectedComponentsWithStats(closed)
    for r in range(1, n):
        info.append(cid)
        regions[(lab == r) & (regions == 0)] = len(info)
if not info:
    return []
# nearest region for every pixel: DIST_LABEL_PIXEL numbers the zero pixels of the input
# (= region pixels) 1.. in row-major order
dist, near_idx = cv2.distanceTransformWithLabels(np.uint8(regions == 0), cv2.DIST_L2, 5,
                                                 labelType=cv2.DIST_LABEL_PIXEL)
lut = np.zeros(near_idx.max() + 1, np.int32)
order = np.flatnonzero((regions != 0).ravel())
lut[1:len(order) + 1] = regions.ravel()[order]
nearest = lut[near_idx]
for idx, cid in enumerate(info, start=1):
    region = regions == idx
    own = component & (region | ((nearest == idx) & (dist <= own_reach) & (regions == 0)))
    oy, ox = np.nonzero(own)
    if not len(ox):
        continue
    area_mm2 = len(ox) * scale * scale
    extent = max(ox.max() - ox.min() + 1, oy.max() - oy.min() + 1) * scale
    if not (min_area <= area_mm2 <= max_area) or extent > max_extent:
        continue
    votes_here = np.count_nonzero(region)
    fraction = votes_here / len(ox)
    if fraction < options.get('min_color_fraction', .3):
        continue
    cx, cy = ox.mean(), oy.mean()                   # Q12: own centroid
    # restore: dominance over other colours' votes within own_radius of (cx, cy), as legacy
    outward = np.arctan2(cy - by, cx - bx)
    if np.hypot(cx - bx, cy - by) < 1:
        outward = _away_from_nearest(blocked, own, cx, cy)
    approach = _find_approach(blocked, own, cx, cy, outward, width_px, length_px, max_turn, start_px)
    if approach is not None:
        found.append((float(cx + x0), float(cy + y0), cid, round(fraction * dominance, 3), area_mm2, approach))
```

## Appendix B: replay comparison (acceptance bar items 2–3)

Run from the repo root: `.venv/bin/python <script> 20260930-091711-b90153,20260930-092017-7dd201,20260930-092247-df6aea out.png`.
It needs `runs/` (on the original author's laptop only, gitignored) and the 09:15 reference `background_0915.png`.

```python
import sys, json, math, cv2, numpy as np, collections
sys.path.insert(0, '.')
from perception import Perception
st = collections.Counter(); tiles = []; seen = []
for run in sys.argv[1].split(','):
    cfg = json.load(open(f'runs/autonomy/{run}/config.json')); bg = cv2.imread('background_0915.png')
    cfg_old = json.loads(json.dumps(cfg)); cfg_old.setdefault('vision', {})['pile_edge_pixels'] = 'legacy'
    cfg_new = json.loads(json.dumps(cfg)); cfg_new.setdefault('vision', {})['pile_edge_pixels'] = 'nearest'
    a, b = Perception(cfg_old, bg), Perception(cfg_new, bg)
    v = cv2.VideoCapture(f'runs/autonomy/{run}/video.avi'); i = -1
    while True:
        ok, raw = v.read(); i += 1
        if not ok: break
        sa, sb = a.step(raw, i / 30), b.step(raw, i / 30)
        st['frames'] += 1; st['legacy_targets'] += len(sa.targets); st['nearest_targets'] += len(sb.targets)
        st['legacy_frames_without'] += not sa.targets; st['nearest_frames_without'] += not sb.targets
        match = lambda t, ts: any(u['color'] == t['color'] and math.hypot(u['x']-t['x'], u['y']-t['y']) < 20 for u in ts)
        st['lost'] += sum(not match(t, sb.targets) for t in sa.targets)
        for t in sb.targets:
            if match(t, sa.targets): continue
            st['new'] += 1
            if not any(math.hypot(t['x']-x, t['y']-y) < 50 for x, y in seen):   # one crop per distinct stone
                seen.append((t['x'], t['y']))
                cx, cy = int(t['x']/3), int(t['y']/3); img = sb.frame.copy()
                h = math.radians(t.get('approach_deg') or 0)
                cv2.arrowedLine(img, (int(cx - math.cos(h)*25), int(cy - math.sin(h)*25)), (cx, cy), (0, 255, 0), 1, tipLength=.3)
                crop = cv2.copyMakeBorder(img, 30, 30, 30, 30, cv2.BORDER_CONSTANT)[cy:cy+60, cx:cx+60]
                crop = cv2.resize(crop, None, fx=4, fy=4, interpolation=cv2.INTER_NEAREST)
                cv2.putText(crop, f'{run[9:15]} f{i} c{t["color"]}', (3, 12), 0, .4, (0, 0, 255), 1)
                tiles.append(crop)
print(dict(st))
while len(tiles) % 4: tiles.append(np.zeros_like(tiles[0]))
cv2.imwrite(sys.argv[2], np.vstack([np.hstack(tiles[k:k+4]) for k in range(0, len(tiles), 4)]))
```

Replays that hash `Perception` diagnostics must drive the reference guard's clock from frame time
(`reference_guard.time = SimpleNamespace(monotonic=lambda: frame_t)`): it reads `time.monotonic()` for its 2 s hold,
so otherwise two identical replays hash differently.

## Valid flags to remind the user of (they asked for this with every run command)

```bash
.venv/bin/python autonomy.py <ESP_IP> --camera 1 --record                                    # real run
.venv/bin/python autonomy.py <ESP_IP> --camera 1 --record --set vision.pile_edge_pixels=legacy   # old pile behaviour (after this is built)
  --set min_duty=X                 drive floor (firmware must report min_duty)
  --set vision.own_reach_mm=N      pale-rim reach for pile stones (default 15, after this is built)
  --set <any autonomy key>=VALUE   typos are rejected; saved in runs/autonomy/<time>/config.json
  --dry-run  --check-config  --show-full-frame  --headless  --record-fps N
```
