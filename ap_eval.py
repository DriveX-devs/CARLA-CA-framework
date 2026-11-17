# -*- coding: utf-8 -*-
"""
Average precision of 3D detections against CARLA ground truth.

The rule is the SimBEV2X benchmark's centre-distance AP (the nuScenes
detection rule), the one the SiMO weights are reported with:

  * per frame and class, the predictions are sorted by score and capped
    (``max_predictions``);
  * each prediction, best score first, takes the nearest still unassigned
    ground-truth object of its class by 3D distance of the box centres, and
    is a true positive if that distance is within the threshold;
  * the true/false positives of all frames are pooled per class and sorted by
    score; AP is the trapezoid area of the precision-recall curve over the
    points with precision >= 0.1, after prepending (recall 0, precision 1);
  * AP is computed at 0.5, 1, 2 and 4 m and averaged.

A ground-truth object counts when its box centre is inside the evaluation
range around the receiver and enough LiDAR points of the current sweep of
any contributing agent fall inside its box (``min_points``, 1 as in the
benchmark's valid flag).

Two label sets are scored: the SiMO classes themselves (pedestrian, bicycle,
motorcycle, car, truck, bus) and the three CA categories the collision
avoidance works with (vehicle, bike, pedestrian), in which a truck detected
as a car is not an error.
"""

import json

import numpy as np
from scipy.spatial import cKDTree

THRESHOLDS_M = (0.5, 1.0, 2.0, 4.0)

CA_CATEGORIES = ('vehicle', 'bike', 'pedestrian')
SIMO_DYNAMIC = ('pedestrian', 'bicycle', 'motorcycle', 'car', 'truck', 'bus')
CLASS_TO_CATEGORY = {'pedestrian': 'pedestrian', 'bicycle': 'bike',
                     'motorcycle': 'bike', 'car': 'vehicle',
                     'truck': 'vehicle', 'bus': 'vehicle'}

# CARLA blueprint base_type -> SimBEV2X class (vans are trucks there)
_BASE_TYPE_CLASS = {'car': 'car', 'truck': 'truck', 'van': 'truck',
                    'bus': 'bus', 'motorcycle': 'motorcycle',
                    'bicycle': 'bicycle'}
_TRUCKS = ('carlacola', 'european_hgv', 'firetruck', 'ambulance', 'sprinter',
           'volkswagen.t2')
_MOTORCYCLES = ('harley', 'kawasaki', 'vespa', 'yamaha')
_BICYCLES = ('crossbike', 'century', 'omafiets')


def simbev_class_of(actor):
    """SimBEV2X class of a CARLA actor (None for anything else)."""
    type_id = actor.type_id
    if type_id.startswith('walker.'):
        return 'pedestrian'
    if not type_id.startswith('vehicle.'):
        return None
    base = str(actor.attributes.get('base_type', '')).lower()
    if base in _BASE_TYPE_CLASS:
        return _BASE_TYPE_CLASS[base]
    # blueprints without a base_type: fall back on the model name
    if 'fusorosa' in type_id:
        return 'bus'
    if any(k in type_id for k in _TRUCKS):
        return 'truck'
    if any(k in type_id for k in _MOTORCYCLES):
        return 'motorcycle'
    if any(k in type_id for k in _BICYCLES):
        return 'bicycle'
    return 'car'


def points_in_boxes(points_world, boxes):
    """Number of points inside each oriented box.

    points_world: (N, 3); boxes: list of dicts with 'inv' (4x4 world -> box
    frame) and 'extent' (half sizes x, y, z). Returns an int array."""
    counts = np.zeros(len(boxes), dtype=np.int64)
    if not len(points_world) or not boxes:
        return counts
    tree = cKDTree(points_world[:, :2])
    for i, box in enumerate(boxes):
        radius = float(np.linalg.norm(box['extent'][:2]))
        near = tree.query_ball_point(box['centre'][:2], radius)
        if not near:
            continue
        local = points_world[near] @ box['inv'][:3, :3].T + box['inv'][:3, 3]
        counts[i] = int(np.all(np.abs(local) <= box['extent'], axis=1).sum())
    return counts


def average_precision(scores, hits, n_gt):
    """AP of one class at one threshold from pooled (score, hit) pairs."""
    if n_gt == 0:
        return float('nan')
    if len(scores) == 0:
        return 0.0
    order = np.argsort(-np.asarray(scores), kind='stable')
    hit = np.asarray(hits, dtype=np.float64)[order]
    tp = np.cumsum(hit)
    fp = np.cumsum(1.0 - hit)
    recall = np.r_[0.0, tp / n_gt]
    precision = np.r_[1.0, tp / np.maximum(tp + fp, 1e-12)]
    keep = precision >= 0.1
    if keep.sum() < 2:
        return 0.0
    area = np.trapz(precision[keep], recall[keep])
    return float(np.clip(area, 0.0, 1.0))


class ApAccumulator(object):
    """Pools the detections of a run for one label set."""

    def __init__(self, classes, max_predictions=None):
        self.classes = tuple(classes)
        caps = max_predictions or {}
        self.caps = {c: int(caps.get(c, 100)) for c in self.classes}
        self.n_gt = {c: 0 for c in self.classes}
        self.n_frames = 0
        self._scores = {(c, t): [] for c in self.classes
                        for t in THRESHOLDS_M}
        self._hits = {(c, t): [] for c in self.classes for t in THRESHOLDS_M}
        # true-positive centre errors at the 2 m threshold (the benchmark's
        # translation error), per class
        self._ate = {c: [] for c in self.classes}

    def add_frame(self, pred_centres, pred_scores, pred_classes,
                  gt_centres, gt_classes):
        """One frame of one receiver. Classes are names; predictions and
        ground truth of classes outside this label set are ignored."""
        self.n_frames += 1
        pred_centres = np.asarray(pred_centres, dtype=np.float64) \
            .reshape(-1, 3)
        pred_scores = np.asarray(pred_scores, dtype=np.float64).reshape(-1)
        pred_classes = np.asarray(pred_classes, dtype=object).reshape(-1)
        gt_centres = np.asarray(gt_centres, dtype=np.float64).reshape(-1, 3)
        gt_classes = np.asarray(gt_classes, dtype=object).reshape(-1)
        for cls in self.classes:
            pm = np.where(pred_classes == cls)[0]
            pm = pm[np.argsort(-pred_scores[pm], kind='stable')]
            pm = pm[:self.caps[cls]]
            gt = gt_centres[gt_classes == cls]
            self.n_gt[cls] += len(gt)
            if len(gt) and len(pm):
                dist = np.linalg.norm(pred_centres[pm][:, None] - gt[None],
                                      axis=2)
            else:
                dist = np.zeros((len(pm), len(gt)))
            for thr in THRESHOLDS_M:
                used = np.zeros(len(gt), dtype=bool)
                for i in range(len(pm)):
                    hit = 0.0
                    if len(gt):
                        d = np.where(used, np.inf, dist[i])
                        j = int(np.argmin(d))
                        if d[j] <= thr:
                            used[j] = True
                            hit = 1.0
                            if thr == 2.0:
                                self._ate[cls].append(float(d[j]))
                    self._scores[(cls, thr)].append(pred_scores[pm[i]])
                    self._hits[(cls, thr)].append(hit)

    def result(self):
        """{class: {'ap': {thr: value}, 'ap_mean': value, 'n_gt': n,
        'ate_m': mean true-positive centre error}} plus the mean AP over
        the classes that have ground truth."""
        out = {'classes': {}, 'frames': self.n_frames}
        means = []
        for cls in self.classes:
            aps = {}
            for thr in THRESHOLDS_M:
                aps['%g' % thr] = average_precision(
                    self._scores[(cls, thr)], self._hits[(cls, thr)],
                    self.n_gt[cls])
            values = [v for v in aps.values() if not np.isnan(v)]
            mean = float(np.mean(values)) if values else float('nan')
            if self.n_gt[cls]:
                means.append(mean)
            out['classes'][cls] = {
                'ap': aps, 'ap_mean': mean, 'n_gt': int(self.n_gt[cls]),
                'n_pred': len(self._scores[(cls, THRESHOLDS_M[0])]),
                'ate_m': float(np.mean(self._ate[cls]))
                if self._ate[cls] else float('nan')}
        out['map'] = float(np.mean(means)) if means else float('nan')
        return out


class ApEvaluation(object):
    """The AP bookkeeping of a run: one pair of accumulators (SiMO classes,
    CA categories) per evaluated stream, e.g. each CAV on its own and the
    cooperative fusion."""

    def __init__(self, eval_range_m=54.4, min_points=1, max_predictions=None):
        self.eval_range = float(eval_range_m)
        self.min_points = int(min_points)
        self.max_predictions = max_predictions
        self._streams = {}

    def _stream(self, name):
        if name not in self._streams:
            caps = self.max_predictions or {}
            self._streams[name] = {
                'class': ApAccumulator(SIMO_DYNAMIC, caps),
                'category': ApAccumulator(CA_CATEGORIES, caps)}
        return self._streams[name]

    def in_range(self, centres_world, world_to_receiver):
        """Mask of the world-frame centres inside the evaluation square
        around the receiver (in its own frame)."""
        centres_world = np.asarray(centres_world).reshape(-1, 3)
        if not len(centres_world):
            return np.zeros(0, dtype=bool)
        local = centres_world @ world_to_receiver[:3, :3].T + \
            world_to_receiver[:3, 3]
        return (np.abs(local[:, 0]) <= self.eval_range) & \
               (np.abs(local[:, 1]) <= self.eval_range)

    def add_frame(self, stream, world_to_receiver, pred_centres, pred_scores,
                  pred_class_names, gt_objects, gt_point_counts,
                  pred_categories=None):
        """gt_objects: dicts with 'centre' (world) and 'simbev_class';
        gt_point_counts: LiDAR points inside each (current sweeps of the
        contributing agents). A detector that only knows the CA categories
        passes them as pred_categories with pred_class_names None: only the
        category label set is then scored."""
        acc = self._stream(stream)
        gt_centres = np.array([g['centre'] for g in gt_objects]) \
            .reshape(-1, 3)
        valid = self.in_range(gt_centres, world_to_receiver) & \
            (np.asarray(gt_point_counts).reshape(-1) >= self.min_points)
        gt_centres = gt_centres[valid]
        gt_classes = [g['simbev_class'] for g, v in zip(gt_objects, valid)
                      if v]
        pred_centres = np.asarray(pred_centres).reshape(-1, 3)
        keep = self.in_range(pred_centres, world_to_receiver)
        pred_centres = pred_centres[keep]
        pred_scores = np.asarray(pred_scores).reshape(-1)[keep]
        if pred_class_names is not None:
            pred_names = [n for n, k in zip(pred_class_names, keep) if k]
            acc['class'].add_frame(pred_centres, pred_scores, pred_names,
                                   gt_centres, gt_classes)
            categories = [CLASS_TO_CATEGORY.get(n) for n in pred_names]
        else:
            acc['class_scored'] = False
            categories = [c for c, k in zip(pred_categories, keep) if k]
        acc['category'].add_frame(
            pred_centres, pred_scores, categories, gt_centres,
            [CLASS_TO_CATEGORY.get(n) for n in gt_classes])

    def results(self):
        out = {}
        for name, acc in self._streams.items():
            out[name] = {'category': acc['category'].result()}
            if acc.get('class_scored', True):
                out[name]['class'] = acc['class'].result()
        return out

    def summary_lines(self, results=None):
        results = results or self.results()
        lines = ['Detection AP (centre distance, SimBEV2X rule: 0.5 / 1 / 2 '
                 '/ 4 m and their mean, x100); ground truth within +-%.1f m '
                 'with >= %d LiDAR point(s)'
                 % (self.eval_range, self.min_points)]
        for name in sorted(results):
            for label_set in ('category', 'class'):
                if label_set not in results[name]:
                    continue
                res = results[name][label_set]
                lines.append('  %s, %s (mAP %s over %d frames)'
                             % (name, 'CA categories' if label_set ==
                                'category' else 'SiMO classes',
                                _fmt(res['map']), res['frames']))
                for cls, r in res['classes'].items():
                    if not r['n_gt']:
                        continue
                    lines.append(
                        '    %-11s %s  mean %s | GT %6d  pred %7d  '
                        'centre error %s m'
                        % (cls, ' '.join(_fmt(r['ap'][k]) for k in
                                         ('0.5', '1', '2', '4')),
                           _fmt(r['ap_mean']), r['n_gt'], r['n_pred'],
                           '%.2f' % r['ate_m'] if not np.isnan(r['ate_m'])
                           else ' n/a'))
        return lines

    def write(self, path_json, path_txt=None):
        results = self.results()
        with open(path_json, 'w') as handle:
            json.dump({'thresholds_m': THRESHOLDS_M,
                       'eval_range_m': self.eval_range,
                       'min_lidar_points': self.min_points,
                       'streams': results}, handle, indent=2, sort_keys=True)
            handle.write('\n')
        lines = self.summary_lines(results)
        if path_txt:
            with open(path_txt, 'w') as handle:
                handle.write('\n'.join(lines) + '\n')
        return lines


def _fmt(value):
    return '  n/a' if value is None or np.isnan(value) \
        else '%5.1f' % (100.0 * value)
