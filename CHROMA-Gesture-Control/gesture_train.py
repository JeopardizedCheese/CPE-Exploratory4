"""Train an MLP on your recordings, holding out entire recording sessions."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import warnings

import numpy as np

from gesture_logic import classify_landmarks
from gesture_model import (FEATURE_COUNT, FEATURE_VERSION, LABELS, GestureModel,
                           export_model, gated_labels)

ACTIVE = {'OPEN', 'V', 'ONE', 'THREE', 'THUMB_UP', 'THUMB_DOWN'}


def load_recordings(directory):
    files = sorted(Path(directory).glob('*/*.npz'))
    if not files:
        raise ValueError('No recordings found. Run gesture_collect.py first.')
    xs, ys, groups, inventory = [], [], [], []
    seen = set()
    for path in files:
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        if digest in seen:
            raise ValueError(f'Duplicate recording: {path}; do not copy takes between sessions')
        seen.add(digest)
        with np.load(path, allow_pickle=False) as data:
            if str(data['feature_version'].item()) != FEATURE_VERSION:
                raise ValueError(f'Wrong feature version: {path}')
            x = np.asarray(data['X'], dtype=float)
            label, session = str(data['label'].item()), str(data['session'].item())
            if (x.ndim != 2 or x.shape[1] != FEATURE_COUNT or len(x) == 0
                    or not np.isfinite(x).all() or np.max(np.abs(x)) > 15
                    or label not in LABELS or not session):
                raise ValueError(f'Invalid recording: {path}')
        xs.append(x)
        ys.extend([label]*len(x))
        groups.extend([session]*len(x))
        inventory.append({'path': str(path.resolve()), 'sha256': digest,
                          'label': label, 'session': session, 'samples': len(x)})
    return np.concatenate(xs), np.array(ys), np.array(groups), inventory


def session_counts(y, groups):
    return {str(g): {label: int(np.sum((groups == g) & (y == label))) for label in LABELS}
            for g in np.unique(groups)}


def split_sessions(y, groups, seed=42):
    """Strictly complete sessions: 60/20/20 with five sessions, by group."""
    counts = session_counts(y, groups)
    if len(counts) < 5:
        raise ValueError(f'Need at least 5 independent recording sessions; found {len(counts)}. '
                         'Record all 8 labels each time, then restart collector for a new session.')
    incomplete = {g: {label: n for label, n in row.items() if n < 30}
                  for g, row in counts.items() if min(row.values()) < 30}
    if incomplete:
        raise ValueError('Each session needs at least 30 samples of every label. Missing/short: '
                         + json.dumps(incomplete))
    ordered = np.array(sorted(counts))
    np.random.default_rng(seed).shuffle(ordered)
    holdout = max(1, int(len(ordered)*.2))
    selected = {'train': ordered[2*holdout:], 'validation': ordered[:holdout],
                'test': ordered[holdout:2*holdout]}
    return {name: np.flatnonzero(np.isin(groups, values)) for name, values in selected.items()}


def metrics(y, predictions):
    y, predictions = np.asarray(y), np.asarray(predictions)
    confusion = [[int(np.sum((y == truth) & (predictions == pred))) for pred in LABELS]
                 for truth in LABELS]
    active = np.isin(predictions, list(ACTIVE))
    actual_active = np.isin(y, list(ACTIVE))
    unknown = y == 'UNKNOWN'
    wrong_commands = (predictions != y) & active
    return {'samples': len(y), 'accuracy': float(np.mean(predictions == y)),
            'unknown_output_rate': float(np.mean(predictions == 'UNKNOWN')),
            'wrong_active_commands': int(wrong_commands.sum()),
            'wrong_active_rate': float(wrong_commands.mean()),
            'correct_active_recall': float(np.sum((predictions == y) & actual_active)/max(1, actual_active.sum())),
            'unknown_to_active_rate': float(np.sum(unknown & active)/max(1, unknown.sum())),
            'confusion_labels': list(LABELS), 'confusion_rows_truth_columns_prediction': confusion,
            'per_class_recall': {label: float(np.sum((y == label) & (predictions == label))/max(1, np.sum(y == label)))
                                 for label in LABELS}}


def train(directory, output, seed=42, threshold=.9, margin=.2, iterations=500):
    if not .5 <= threshold <= 1 or not 0 <= margin <= 1 or iterations < 1:
        raise ValueError('Invalid threshold, margin, or training iterations')
    output = Path(output)
    report_path = output.with_suffix('.report.json')
    if output.exists() or report_path.exists():
        raise ValueError('Output already exists. Choose a new --output filename to preserve your previous model.')
    x, y, groups, inventory = load_recordings(directory)
    split = split_sessions(y, groups, seed)
    from sklearn.neural_network import MLPClassifier
    from sklearn.preprocessing import StandardScaler
    import sklearn
    scaler = StandardScaler().fit(x[split['train']])
    # No internal random-frame validation; no test data in normalization or fit.
    classifier = MLPClassifier(hidden_layer_sizes=(64, 32), activation='relu',
                               solver='adam', alpha=.01, max_iter=iterations,
                               early_stopping=False, random_state=seed)
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter('always')
        classifier.fit(scaler.transform(x[split['train']]), y[split['train']])
    report = {'created_utc': datetime.now(timezone.utc).isoformat(),
              'feature_version': FEATURE_VERSION, 'sklearn_version': sklearn.__version__,
              'hidden_layers': [64, 32], 'seed': seed, 'threshold': threshold, 'margin': margin,
              'scores_are_calibrated_probabilities': False, 'hardware_validated': False,
              'training_iterations': int(classifier.n_iter_), 'training_loss': float(classifier.loss_),
              'warnings': [str(w.message) for w in caught], 'sessions': session_counts(y, groups),
              'recordings': inventory, 'splits': {}}
    for name, indices in split.items():
        p = classifier.predict_proba(scaler.transform(x[indices]))
        predicted = gated_labels(p, classifier.classes_, threshold, margin)
        baseline = [classify_landmarks(row.reshape(21, 3)[:, :2].tolist()) for row in x[indices]]
        report['splits'][name] = {
            'sessions': sorted(set(groups[indices].tolist())),
            'raw_classifier': metrics(y[indices], classifier.classes_[p.argmax(axis=1)]),
            'with_rejection': metrics(y[indices], predicted),
            'original_rules': metrics(y[indices], baseline)}
    report['interpretation'] = (
        'Frame-level results on held-out sessions, not robot success rates. '
        'UNKNOWN includes modeled other gestures and rejected predictions. '
        'No-hand/multiple-hand loss is handled outside the classifier. '
        'Choose thresholds on validation only; if you tune after viewing test results, '
        'collect another untouched test session. No claim of superiority until evaluated on real data.')
    output.parent.mkdir(parents=True, exist_ok=True)
    export_model(output, scaler, classifier,
                 {'threshold': threshold, 'margin': margin, 'created_utc': report['created_utc'],
                  'seed': seed, 'training_sessions': report['splits']['train']['sessions']})
    # Check portable inference against the actual trained implementation.
    portable = GestureModel(output)
    np.testing.assert_allclose(portable.probabilities(x[split['test']]),
                               classifier.predict_proba(scaler.transform(x[split['test']])),
                               rtol=1e-6, atol=1e-8)
    with report_path.open('x', encoding='utf-8') as f:
        json.dump(report, f, ensure_ascii=False, indent=2, allow_nan=False)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data', type=Path, default=Path(__file__).parent/'gesture_data')
    parser.add_argument('--output', type=Path, default=Path(__file__).parent/'models/gesture_mlp.npz')
    parser.add_argument('--inspect', action='store_true', help='Show data counts without training')
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--threshold', type=float, default=.9)
    parser.add_argument('--margin', type=float, default=.2)
    parser.add_argument('--iterations', type=int, default=500)
    args = parser.parse_args()
    try:
        if args.inspect:
            _, y, groups, _ = load_recordings(args.data)
            print(json.dumps(session_counts(y, groups), indent=2))
            return
        report = train(args.data, args.output, args.seed, args.threshold, args.margin, args.iterations)
    except (ValueError, OSError, KeyError) as exc:
        parser.exit(1, f'{exc}\n')
    print(f'Model: {args.output}\nReport: {args.output.with_suffix(".report.json")}')
    for name in ('validation', 'test'):
        result = report['splits'][name]['with_rejection']
        print(f'{name}: accuracy={result["accuracy"]:.3f}, '
              f'wrong active={result["wrong_active_commands"]}/{result["samples"]}, '
              f'UNKNOWN outputs={result["unknown_output_rate"]:.1%}')
    for warning in report['warnings']:
        print(f'WARNING: {warning}')
    print('Preview the model before connecting a robot. These are frame-level metrics, not physical validation.')


if __name__ == '__main__':
    main()
