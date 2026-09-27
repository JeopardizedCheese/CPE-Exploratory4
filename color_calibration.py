"""Measured HSV calibration, ambiguity diagnostics, and persistent sampling sessions.

Calibration sample coverage is a fit diagnostic, not test accuracy. Evaluation
sessions are never used to construct ranges. No guessed boundaries resolve overlaps.
"""
from itertools import combinations
import json
from pathlib import Path

import cv2
import numpy as np

NAMES = {1: 'violet', 2: 'cyan', 3: 'crimson', 4: 'orange', 5: 'skyblue', 6: 'lime'}


def mask_for(hsv, ranges):
    mask = np.zeros(hsv.shape[:2], np.uint8)
    for r in ranges:
        lo, hi = np.asarray(r['lo']), np.asarray(r['hi'])
        if (lo.shape != (3,) or hi.shape != (3,) or not np.isfinite(lo).all()
                or not np.isfinite(hi).all() or np.any(lo < 0)
                or np.any(hi > [179, 255, 255]) or np.any(lo > hi)):
            raise ValueError('Invalid HSV range; hue must be 0..179, S/V 0..255, lo <= hi')
        mask |= cv2.inRange(hsv, lo.astype(np.uint8), hi.astype(np.uint8))
    return mask


def color_masks(hsv, cfg):
    return {int(k.split('_')[0]): mask_for(hsv, ranges)
            for k, ranges in cfg.get('hsv', {}).items() if ranges}


def exclusive_labels(hsv, cfg):
    """0=unmatched/low color evidence; -1=ambiguous; 1..6=one matching class."""
    masks = color_masks(hsv, cfg)
    ownership = np.zeros(hsv.shape[:2], np.uint8)
    labels = np.zeros(hsv.shape[:2], np.int16)
    for cid, mask in masks.items():
        ownership += (mask > 0).astype(np.uint8)
        labels[mask > 0] = cid
    labels[ownership > 1] = -1
    reliable = ((hsv[..., 1] >= cfg.get('min_saturation', 60))
                & (hsv[..., 2] >= cfg.get('min_value', 45)))
    labels[~reliable] = 0
    return labels


def range_overlaps(cfg):
    """Exact intersections of configured HSV boxes, including red wrap ranges."""
    result = []
    items = sorted((int(k.split('_')[0]), v) for k, v in cfg.get('hsv', {}).items() if v)
    for (a, ar), (b, br) in combinations(items, 2):
        boxes = []
        for x in ar:
            for y in br:
                lo, hi = np.maximum(x['lo'], y['lo']), np.minimum(x['hi'], y['hi'])
                if np.all(lo <= hi):
                    boxes.append({'lo': lo.tolist(), 'hi': hi.tolist()})
        if boxes:
            result.append({'classes': [a, b], 'names': [NAMES[a], NAMES[b]], 'ranges': boxes})
    return result


def ranges_from_samples(px, min_saturation=60, min_value=45):
    a = np.asarray(px, dtype=float).reshape(-1, 3)
    a = a[(a[:, 1] >= min_saturation) & (a[:, 2] >= min_value)]
    if len(a) < 8:
        return []
    angle = a[:, 0] * np.pi / 90
    center = np.arctan2(np.sin(angle).mean(), np.cos(angle).mean()) * 90 / np.pi % 180
    unwrapped = (a[:, 0] - center + 90) % 180 - 90 + center
    lo, hi = np.percentile(unwrapped, [5, 95]) + np.array([-4, 4])
    if hi - lo > 45:
        return []
    # Both ends come from measurements. Unconditional S/V=255 made the old
    # cyan and sky-blue boxes almost identical even when their samples differed.
    sv_lo = np.maximum([min_saturation, min_value],
                       np.floor(np.percentile(a[:, 1:], 5, axis=0)) - [25, 30]).astype(int)
    sv_hi = np.minimum(255, np.ceil(np.percentile(a[:, 1:], 95, axis=0)) + [25, 30]).astype(int)
    low, high = int(np.floor(lo)) % 180, int(np.ceil(hi)) % 180
    spans = [(low, high)] if low <= high else [(0, high), (low, 179)]
    return [{'lo': [l, *sv_lo.tolist()], 'hi': [h, *sv_hi.tolist()]} for l, h in spans]


def sample_signature(cfg, source_size):
    arena = cfg.get('arena', {})
    return {'source_size_px': list(source_size), 'camera_index': cfg.get('camera_index', 0),
            'camera_properties': cfg.get('camera_properties', {}),
            'corners_px': arena.get('corners_px', []),
            'size_mm': arena.get('size_mm'), 'mm_per_px': arena.get('mm_per_px', 2),
            'min_saturation': cfg.get('min_saturation', 60), 'min_value': cfg.get('min_value', 45)}


def load_samples(path, signature=None):
    path = Path(path)
    if not path.exists():
        return {'version': 1, 'signature': signature, 'sessions': {}}
    data = json.loads(path.read_text(encoding='utf-8'))
    if data.get('version') != 1 or not isinstance(data.get('sessions'), dict):
        raise ValueError('Unsupported color sample file')
    if signature is not None and data.get('signature') != signature:
        raise ValueError('Camera settings/geometry differ from saved samples. Use a new --samples file.')
    return data


def save_json(path, data):
    """Replace a complete JSON file atomically, so an interrupted save keeps the old file."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_name(path.name + '.tmp')
    temp.write_text(json.dumps(data, indent=2, allow_nan=False), encoding='utf-8')
    temp.replace(path)


def patches_by_color(data, purpose):
    result = {cid: [] for cid in NAMES}
    for session in data['sessions'].values():
        if session['purpose'] == purpose:
            for key, patches in session['samples'].items():
                result[int(key)].extend(patches)
    return result


def sample_report(data, cfg, purpose):
    classes = {}
    for cid, patches in patches_by_color(data, purpose).items():
        pixels = [p for patch in patches for p in patch['hsv']]
        if not pixels:
            continue
        labels = exclusive_labels(np.array(pixels, np.uint8).reshape(-1, 1, 3), cfg).ravel()
        wrong = (labels > 0) & (labels != cid)
        classes[NAMES[cid]] = {
            'patches': len(patches), 'pixels': len(pixels),
            'correct_fraction': float(np.mean(labels == cid)),
            'wrong_fraction': float(np.mean(wrong)),
            'ambiguous_fraction': float(np.mean(labels == -1)),
            'unmatched_fraction': float(np.mean(labels == 0)),
            'predicted_counts': {NAMES[k]: int(np.count_nonzero(labels == k)) for k in NAMES}}
    return {'purpose': purpose, 'sessions': sum(s['purpose'] == purpose for s in data['sessions'].values()),
            'classes': classes,
            'note': 'Labeled patch pixel coverage; correlated pixels are not independent stone detections.'}


def propose_ranges(data, cfg, min_patches=5):
    """Build from calibration sessions only; block obviously unusable sample fits."""
    import copy
    candidate = copy.deepcopy(cfg)
    errors = []
    changed = False
    for cid, patches in patches_by_color(data, 'calibration').items():
        if not patches:
            continue
        if len(patches) < min_patches:
            errors.append(f'{NAMES[cid]}: collect at least {min_patches} patches (have {len(patches)})')
            continue
        pixels = [p for patch in patches for p in patch['hsv']]
        ranges = ranges_from_samples(pixels, cfg.get('min_saturation', 60), cfg.get('min_value', 45))
        if not ranges:
            errors.append(f'{NAMES[cid]}: mixed or insufficient color samples')
            continue
        candidate.setdefault('hsv', {})[f'{cid}_{NAMES[cid]}'] = ranges
        changed = True
    report = sample_report(data, candidate, 'calibration')
    for name, row in report['classes'].items():
        if row['patches'] >= min_patches and (row['correct_fraction'] < .6 or row['wrong_fraction'] > .05):
            errors.append(f"{name}: only {row['correct_fraction']:.0%} uniquely matches its label; "
                          f"{row['ambiguous_fraction']:.0%} ambiguous, {row['wrong_fraction']:.0%} wrong")
    if not changed:
        errors.append('No usable calibration samples yet')
    return candidate, report, errors
