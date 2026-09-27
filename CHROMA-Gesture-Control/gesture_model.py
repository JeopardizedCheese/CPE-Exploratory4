"""Shared landmark features and a portable, pickle-free MLP classifier."""
import json
from pathlib import Path

import numpy as np

LABELS = ('OPEN', 'FIST', 'V', 'ONE', 'THREE', 'THUMB_UP', 'THUMB_DOWN', 'UNKNOWN')
FEATURE_VERSION = 'wrist-relative-palm-scale-mirrored-xyz-v1'
FEATURE_COUNT = 63


def landmark_features(landmarks, width, height):
    """Remove translation/size, preserve orientation (thumb up != down).

    Input is MediaPipe image-normalized xyz from a MIRRORED frame. z is scaled
    like x per MediaPipe's image landmark convention. No handedness mirroring.
    """
    p = np.asarray(landmarks, dtype=np.float64)
    if p.shape != (21, 3) or width <= 0 or height <= 0 or not np.isfinite(p).all():
        return None
    p = p * np.array([width/height, 1., width/height])
    p -= p[0].copy()
    scale = np.linalg.norm(p[9, :2])  # wrist -> middle MCP
    if scale < .025:
        return None  # tiny/degenerate hand geometry
    features = (p/scale).reshape(-1)
    if np.max(np.abs(features)) > 15:
        return None
    return features


def gated_labels(probabilities, classes, threshold, margin):
    p = np.asarray(probabilities)
    if p.ndim != 2 or p.shape[1] != len(classes) or not np.isfinite(p).all():
        raise ValueError('Invalid classifier output')
    best = p.argmax(axis=1)
    top = np.sort(p, axis=1)
    labels = np.asarray(classes, dtype=str)[best].copy()
    labels[(top[:, -1] < threshold) | (top[:, -1]-top[:, -2] < margin)] = 'UNKNOWN'
    return labels


class GestureModel:
    """ReLU MLP + StandardScaler exported from scikit-learn; numpy inference."""
    def __init__(self, path):
        self.path = Path(path)
        with np.load(self.path, allow_pickle=False) as data:
            self.meta = json.loads(str(data['metadata'].item()))
            if self.meta.get('feature_version') != FEATURE_VERSION:
                raise ValueError('Classifier feature version does not match this program')
            if self.meta.get('activation') != 'relu' or self.meta.get('format') != 1:
                raise ValueError('Unsupported classifier format')
            self.classes = np.asarray(data['classes'], dtype=str)
            if self.classes.ndim != 1 or len(self.classes) != len(LABELS) or set(self.classes) != set(LABELS):
                raise ValueError('Classifier must include all eight supported labels')
            self.mean = np.asarray(data['mean'], dtype=float)
            self.scale = np.asarray(data['scale'], dtype=float)
            if self.mean.shape != (FEATURE_COUNT,) or self.scale.shape != (FEATURE_COUNT,):
                raise ValueError('Invalid feature scaler shape')
            if not np.isfinite(self.mean).all() or not np.isfinite(self.scale).all() or np.any(self.scale <= 0):
                raise ValueError('Invalid feature scaler values')
            n_layers = self.meta.get('layers')
            if not isinstance(n_layers, int) or not 1 <= n_layers <= 5:
                raise ValueError('Invalid MLP layer count')
            self.weights, self.biases = [], []
            previous = FEATURE_COUNT
            for i in range(n_layers):
                w, b = np.asarray(data[f'w{i}'], dtype=float), np.asarray(data[f'b{i}'], dtype=float)
                if (w.ndim != 2 or w.shape[0] != previous or not 1 <= w.shape[1] <= 1024
                        or b.shape != (w.shape[1],) or not np.isfinite(w).all() or not np.isfinite(b).all()):
                    raise ValueError('Invalid MLP weights')
                previous = w.shape[1]
                self.weights.append(w)
                self.biases.append(b)
            if previous != len(LABELS):
                raise ValueError('Invalid MLP output shape')
        self.threshold = float(self.meta['threshold'])
        self.margin = float(self.meta['margin'])
        if not .5 <= self.threshold <= 1 or not 0 <= self.margin <= 1:
            raise ValueError('Invalid rejection thresholds')

    def probabilities(self, features):
        x = np.asarray(features, dtype=float)
        if x.ndim == 1:
            x = x[None, :]
        if x.ndim != 2 or x.shape[1] != FEATURE_COUNT or not np.isfinite(x).all():
            raise ValueError('Invalid hand features')
        with np.errstate(over='raise', invalid='raise'):
            x = (x-self.mean)/self.scale
            for i, (w, b) in enumerate(zip(self.weights, self.biases)):
                x = x @ w + b
                if i < len(self.weights)-1:
                    x = np.maximum(x, 0.)
            x -= x.max(axis=1, keepdims=True)
            p = np.exp(x)
            return p/p.sum(axis=1, keepdims=True)

    def predict(self, features):
        p = self.probabilities(features)[0]
        index = int(p.argmax())
        label = gated_labels(p[None, :], self.classes, self.threshold, self.margin)[0]
        return str(label), float(p[index]), str(self.classes[index])


def export_model(path, scaler, classifier, metadata):
    if classifier.activation != 'relu' or classifier.out_activation_ != 'softmax':
        raise ValueError('Only multiclass ReLU/softmax MLP is supported')
    metadata = dict(metadata, format=1, feature_version=FEATURE_VERSION,
                    activation='relu', layers=len(classifier.coefs_))
    arrays = {'mean': scaler.mean_, 'scale': scaler.scale_, 'classes': classifier.classes_,
              'metadata': np.array(json.dumps(metadata, allow_nan=False))}
    for i, (w, b) in enumerate(zip(classifier.coefs_, classifier.intercepts_)):
        arrays[f'w{i}'], arrays[f'b{i}'] = w, b
    # Refuse to silently replace an already trained model.
    with Path(path).open('xb') as f:
        np.savez_compressed(f, **arrays)
