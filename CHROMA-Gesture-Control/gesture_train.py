"""Train the command classifier from gesture_collect.py sessions.

TRAIN sessions fit the model, VALIDATION sessions choose the confidence gate (threshold/margin),
TEST sessions are scored once at the end. A session is never split across sets.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import time
import warnings

import numpy as np

from gesture_collect import MIN_SAMPLES, read_session
from gesture_logic import GRIP, MOTION
from gesture_model import (FEATURE_COUNT, FEATURE_VERSION, HANDS, LABEL_SET, LABELS, REJECT, SPLITS,
                           GestureModel, export_model, gated_labels)

ROOT = Path(__file__).resolve().parent
DANGEROUS = set(MOTION) | set(GRIP)        # a wrong one of these moves the robot or the gripper
THRESHOLDS = (.5, .6, .7, .8, .85, .9, .95)
MARGINS = (0., .1, .2, .3)
DANGER_TOLERANCE = .002                    # accept gates within 0.2 % of the safest one, then max recall
SHORT = {'NONE': 'NONE', 'STOP': 'STOP', 'FORWARD': 'FWD', 'BACK': 'BACK', 'LEFT': 'LEFT',
         'RIGHT': 'RIGHT', 'GRIP_OPEN': 'OPEN', 'GRIP_CLOSE': 'CLOSE'}


def load_recordings(directory):
    """-> dict of arrays x, y, hand, session, split + inventory, from every session folder."""
    metas = sorted(Path(directory).glob('*/session.json'))
    if not metas:
        raise ValueError(f'No sessions in {directory}. Record with gesture_collect.py first.')
    xs, rows, inventory, seen = [], [], [], set()
    for meta_path in metas:
        meta = read_session(meta_path.parent)
        for path in sorted(meta_path.parent.glob('*.npz')):
            digest = hashlib.sha256(path.read_bytes()).hexdigest()
            if digest in seen:
                raise ValueError(f'Duplicate recording: {path}; do not copy takes between sessions')
            seen.add(digest)
            with np.load(path, allow_pickle=False) as data:
                x = np.asarray(data['X'], dtype=float)
                label, hand = str(data['label'].item()), str(data['hand'].item())
                session, split = str(data['session'].item()), str(data['split'].item())
                if (str(data['feature_version'].item()) != FEATURE_VERSION
                        or str(data['label_set'].item()) != LABEL_SET):
                    raise ValueError(f'Recording from another gesture system: {path}')
            if (x.ndim != 2 or x.shape[1] != FEATURE_COUNT or len(x) == 0
                    or not np.isfinite(x).all() or np.max(np.abs(x)) > 15
                    or label not in LABELS or hand not in HANDS
                    or session != meta['session'] or split != meta['split']):
                raise ValueError(f'Invalid recording: {path}')
            xs.append(x)
            rows += [(label, hand, session, split)] * len(x)
            inventory.append({'path': str(path.relative_to(directory)), 'sha256': digest, 'label': label,
                              'hand': hand, 'session': session, 'split': split, 'samples': len(x)})
    if not xs:
        raise ValueError('Sessions exist but hold no takes yet')
    y, hand, session, split = (np.array(c) for c in zip(*rows))
    return {'x': np.concatenate(xs), 'y': y, 'hand': hand, 'session': session, 'split': split,
            'inventory': inventory}


def counts_table(d):
    return {split: {'sessions': sorted(set(d['session'][d['split'] == split].tolist())),
                    'samples': {f'{label}/{hand}': int(np.sum((d['split'] == split) & (d['y'] == label)
                                                              & (d['hand'] == hand)))
                                for label in LABELS for hand in HANDS}}
            for split in SPLITS}


def check_data(d):
    problems = []
    for split, row in counts_table(d).items():
        if not row['sessions']:
            problems.append(f'{split}: no session (gesture_collect.py --split {split})')
            continue
        short = [k for k, n in row['samples'].items() if n < MIN_SAMPLES]
        if short:
            problems.append(f'{split}: fewer than {MIN_SAMPLES} samples of ' + ', '.join(short))
    if problems:
        raise ValueError('Not enough data to train:\n  ' + '\n  '.join(problems))


def metrics(y, predictions):
    y, predictions = np.asarray(y), np.asarray(predictions)
    dangerous = np.isin(predictions, list(DANGEROUS)) & (predictions != y)
    command = y != REJECT
    return {'samples': int(len(y)), 'accuracy': float(np.mean(predictions == y)),
            'dangerous_wrong': int(dangerous.sum()), 'dangerous_rate': float(dangerous.mean()),
            'command_recall': float(np.sum((predictions == y) & command) / max(1, command.sum())),
            'none_rate': float(np.mean(predictions == REJECT)),
            'per_class_recall': {label: float(np.sum((y == label) & (predictions == label))
                                              / max(1, np.sum(y == label))) for label in LABELS},
            'confusion_rows_truth_columns_prediction':
                [[int(np.sum((y == t) & (predictions == p))) for p in LABELS] for t in LABELS]}


def choose_gate(probabilities, classes, y):
    """Safest gate on validation (within DANGER_TOLERANCE), then the most commands recognised."""
    table = []
    for threshold in THRESHOLDS:
        for margin in MARGINS:
            m = metrics(y, gated_labels(probabilities, classes, threshold, margin))
            table.append({'threshold': threshold, 'margin': margin, 'dangerous_rate': m['dangerous_rate'],
                          'command_recall': m['command_recall'], 'accuracy': m['accuracy']})
    safest = min(row['dangerous_rate'] for row in table)
    ok = [row for row in table if row['dangerous_rate'] <= safest + DANGER_TOLERANCE]
    best = max(ok, key=lambda row: (row['command_recall'], -row['dangerous_rate'], -row['threshold']))
    return best['threshold'], best['margin'], table


def train(directory, output=None, seed=42, iterations=500):
    if iterations < 1:
        raise ValueError('iterations must be >= 1')
    stamp = time.strftime('%Y%m%d-%H%M%S')
    output = Path(output) if output else ROOT/'models'/f'gesture_commands_{stamp}.npz'
    report_path = output.with_suffix('.report.json')
    if output.exists() or report_path.exists():
        raise ValueError(f'{output} already exists; choose another --output')
    d = load_recordings(directory)
    check_data(d)
    from sklearn.neural_network import MLPClassifier
    from sklearn.preprocessing import StandardScaler
    import sklearn
    part = {split: np.flatnonzero(d['split'] == split) for split in SPLITS}
    x_train, y_train = d['x'][part['train']], d['y'][part['train']]
    scaler = StandardScaler().fit(x_train)
    classifier = MLPClassifier(hidden_layer_sizes=(64, 32), activation='relu', solver='adam', alpha=.01,
                               max_iter=iterations, early_stopping=False, random_state=seed)
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter('always')
        classifier.fit(scaler.transform(x_train), y_train)
    proba = {split: classifier.predict_proba(scaler.transform(d['x'][idx])) for split, idx in part.items()}
    threshold, margin, gate_table = choose_gate(proba['validation'], classifier.classes_,
                                                d['y'][part['validation']])
    report = {'created_utc': datetime.now(timezone.utc).isoformat(), 'model': output.name,
              'feature_version': FEATURE_VERSION, 'label_set': LABEL_SET, 'sklearn_version': sklearn.__version__,
              'hidden_layers': [64, 32], 'seed': seed, 'threshold': threshold, 'margin': margin,
              'gate_chosen_on': 'validation', 'gate_table_validation': gate_table,
              'training_iterations': int(classifier.n_iter_), 'training_loss': float(classifier.loss_),
              'warnings': [str(w.message) for w in caught], 'data': counts_table(d),
              'recordings': d['inventory'], 'splits': {}}
    for split, idx in part.items():
        predicted = gated_labels(proba[split], classifier.classes_, threshold, margin)
        y, hands = d['y'][idx], d['hand'][idx]
        report['splits'][split] = {
            'with_gate': metrics(y, predicted),
            'raw_argmax': metrics(y, classifier.classes_[proba[split].argmax(axis=1)]),
            'per_hand': {hand: metrics(y[hands == hand], predicted[hands == hand]) for hand in HANDS}}
    report['interpretation'] = (
        'Frame-level results on held-out sessions, not robot success. The controller also needs a '
        'command held 0.15 s (motion) / 0.4 s (grip) and stops on NONE, so single wrong frames rarely move '
        'the robot. If you change anything after looking at TEST, record a new test session.')
    output.parent.mkdir(parents=True, exist_ok=True)
    export_model(output, scaler, classifier,
                 {'threshold': threshold, 'margin': margin, 'created_utc': report['created_utc'], 'seed': seed,
                  'training_sessions': report['data']['train']['sessions']})
    portable = GestureModel(output)        # numpy inference must equal scikit-learn
    np.testing.assert_allclose(portable.probabilities(d['x'][part['test']]), proba['test'], rtol=1e-6, atol=1e-8)
    with report_path.open('x', encoding='utf-8') as f:
        json.dump(report, f, ensure_ascii=False, indent=2, allow_nan=False)
    return report, output


def format_counts(table):
    lines = []
    for split, row in table.items():
        lines.append(f'{split.upper():<11} {len(row["sessions"])} session(s)')
        lines.append(f'  {"command":<11}' + ''.join(f'{HANDS[h]:>8}' for h in HANDS))
        for label in LABELS:
            cells = ''.join(f'{row["samples"][f"{label}/{h}"]:>7}{"*" if row["samples"][f"{label}/{h}"] < MIN_SAMPLES else " "}'
                            for h in HANDS)
            lines.append(f'  {label:<11}{cells}')
    lines.append(f'  * = below {MIN_SAMPLES}: record more before training')
    return '\n'.join(lines)


def format_split(name, result):
    m = result['with_gate']
    lines = [f'== {name.upper()}  ({m["samples"]} frames)',
             f'  accuracy {m["accuracy"]:.1%}   commands recognised {m["command_recall"]:.1%}   '
             f'read as NONE {m["none_rate"]:.1%}   DANGEROUS wrong {m["dangerous_wrong"]} ({m["dangerous_rate"]:.2%})',
             '  per hand: ' + '   '.join(f'{HANDS[h]} {r["accuracy"]:.1%} (dangerous {r["dangerous_wrong"]})'
                                        for h, r in result['per_hand'].items()),
             '  recall:   ' + '  '.join(f'{SHORT[k]} {v:.0%}' for k, v in m['per_class_recall'].items()),
             '  confusion (rows = shown, columns = read as):',
             '  ' + ' ' * 7 + ''.join(f'{SHORT[k]:>7}' for k in LABELS)]
    for label, row in zip(LABELS, m['confusion_rows_truth_columns_prediction']):
        lines.append(f'  {SHORT[label]:>6} ' + ''.join(f'{n:>7}' for n in row))
    return '\n'.join(lines)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data', type=Path, default=ROOT/'gesture_data')
    parser.add_argument('--output', type=Path, help='Default: models/gesture_commands_<time>.npz')
    parser.add_argument('--inspect', action='store_true', help='Show the data per split; do not train')
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--iterations', type=int, default=500)
    args = parser.parse_args(argv)
    try:
        if args.inspect:
            d = load_recordings(args.data)
            print(format_counts(counts_table(d)))
            check_data(d)
            print('Ready to train.')
            return 0
        report, output = train(args.data, args.output, args.seed, args.iterations)
    except (ValueError, OSError, KeyError) as exc:
        print(exc)
        return 1
    print(format_counts(report['data']))
    print(f'\nGate chosen on VALIDATION: threshold {report["threshold"]}, margin {report["margin"]}')
    for name in SPLITS:
        print(format_split(name, report['splits'][name]))
    for warning in report['warnings']:
        print(f'WARNING: {warning}')
    print(f'\nModel:  {output}\nReport: {output.with_suffix(".report.json")}')
    print('gesture_control.py and the trainer\'s model check (M) use the newest model automatically.')
    print('Frame-level numbers only: check it live in preview before driving the robot.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
