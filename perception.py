# -*- coding: utf-8 -*-
"""
Perception backends of the CA pipeline.

The application loop does not know which model produces the detections: it
asks a backend to (1) run every CAV's *local stage* on that CAV's sensors
and then (2) produce each CAV's *detections* in the CARLA world frame. The
two-step split is what lets a cooperative model use the other CAVs' local
results: every local stage has run before the first detection is asked for.

    simo    SiMO (LiDAR, cameras or both), the base model. Each CAV encodes
            its own sensors; a CAV detects from its own message alone
            (cooperation: none) or from the messages of the CAVs in range
            too, fused in-process with no network in between
            (cooperation: ideal).
    yolo    YOLOv8 on four cameras + LiDAR frustum fusion, the original
            pipeline, kept as an optional backend.
    oracle  ground truth as detections (objects the CAV's LiDAR actually
            hits), to exercise everything downstream of perception.

Every detection carries ``position`` (world frame, metres), ``category``
(vehicle / bike / pedestrian), ``conf`` and ``cls_name``: that is all the
tracker and the rest of the pipeline read.
"""

import os
import time

import numpy as np

import ap_eval
from carla_utils import transform_matrix
from sensors import SensorRig

BACKENDS = ('simo', 'yolo', 'oracle')
CATEGORIES = ('vehicle', 'bike', 'pedestrian')


class OracleDetection(object):
    """A ground-truth object handed to the pipeline as a detection."""

    __slots__ = ('position', 'conf', 'cls_name', 'category', 'n_points',
                 'camera_idx', 'bbox', 'uv_points', 'gt_id')

    def __init__(self, gt, n_points):
        self.position = np.array(gt['centre'], dtype=np.float64)
        self.conf = 1.0
        self.cls_name = gt['simbev_class']
        self.category = gt['category']
        self.n_points = int(n_points)
        self.camera_idx = -1
        self.bbox = None
        self.uv_points = None
        self.gt_id = gt['id']


class PerceptionResult(object):
    """What a backend returns for one CAV and one step."""

    __slots__ = ('detections', 'raw_centres', 'raw_scores', 'raw_classes',
                 't_fuse_ms', 'reference')

    def __init__(self, detections, raw_centres, raw_scores, raw_classes,
                 t_fuse_ms, reference=None):
        self.detections = detections      # thresholded, for the tracker
        self.raw_centres = raw_centres    # every candidate, for the AP
        self.raw_scores = raw_scores
        self.raw_classes = raw_classes    # SimBEV2X class names (or None)
        self.t_fuse_ms = t_fuse_ms
        # single-agent candidates of the same CAV when the detections are
        # cooperative: (centres, scores, classes) or None
        self.reference = reference


def lidar_world_points(points, lidar_transform):
    """(N, 3+) sensor-frame returns -> (N, 3) world-frame points."""
    pose = transform_matrix(lidar_transform)
    xyz = np.asarray(points, dtype=np.float64)[:, :3]
    return xyz @ pose[:3, :3].T + pose[:3, 3]


class _Backend(object):
    name = None
    cooperative = False

    def __init__(self, cfg, repo_root, dt, opt):
        self.cfg = cfg
        self.repo_root = repo_root
        self.dt = dt

    # -- rig -----------------------------------------------------------
    def sensing_cfg(self):
        return self.cfg['sensing']

    def with_cameras(self):
        return True

    def make_rig(self, world, vehicle):
        return SensorRig(world, vehicle, self.sensing_cfg(),
                         with_cameras=self.with_cameras())

    def attach(self, index, cav):
        """Per-CAV set-up, called once the CAV and its rig exist."""

    # -- per step ------------------------------------------------------
    def local_stage(self, index, cav, step, t_sim, images, points):
        """Run the CAV's own part; returns its wall-clock cost in ms."""
        raise NotImplementedError

    def detect(self, index, cav, cavs, gt_objects, point_counts):
        raise NotImplementedError

    def describe(self):
        return {'backend': self.name}

    def label(self):
        return self.name


# ---------------------------------------------------------------------------
# SiMO
# ---------------------------------------------------------------------------
class SimoBackend(_Backend):
    name = 'simo'

    def __init__(self, cfg, repo_root, dt, opt):
        super().__init__(cfg, repo_root, dt, opt)
        from simo_perception import SimoPerception
        simo_cfg = dict(cfg['simo'])
        if getattr(opt, 'simo_sensors', None):
            simo_cfg['sensors'] = opt.simo_sensors
        if getattr(opt, 'simo_sweeps', None) is not None:
            simo_cfg['sweeps'] = int(opt.simo_sweeps)
        if getattr(opt, 'simo_cooperation', None):
            simo_cfg['cooperation'] = opt.simo_cooperation
        devices = getattr(opt, 'simo_devices', None) or \
            simo_cfg.get('devices', None)
        if devices is not None and not isinstance(devices, str):
            devices = list(devices)
        self.simo_cfg = simo_cfg
        self.cooperation = str(simo_cfg.get('cooperation', 'none'))
        if self.cooperation not in ('none', 'ideal'):
            raise ValueError("simo.cooperation must be none or ideal, got %r"
                             % self.cooperation)
        self.cooperative = self.cooperation == 'ideal'
        self.comm_range = float(simo_cfg.get('comm_range_m', 80.0))
        self.single_reference = bool(
            simo_cfg.get('single_agent_reference', True))
        self.engine = SimoPerception(simo_cfg, repo_root, dt,
                                     devices=devices)
        self._calibration = {}

    def sensing_cfg(self):
        return self.simo_cfg['sensing']

    def with_cameras(self):
        return self.engine.use_camera

    def attach(self, index, cav):
        cav['simo_device'] = self.engine.device_of(index)

    def local_stage(self, index, cav, step, t_sim, images, points):
        engine = self.engine
        rig = cav['rig']
        t0 = time.perf_counter()
        lidar_tf = rig.lidar.sensor.get_transform()
        kwargs = {}
        if engine.use_camera:
            if not rig.cameras:
                raise RuntimeError('simo.sensors=%s but the SiMO rig has no '
                                   'cameras' % engine.sensors)
            # the mounts are rigid: calibrate once per CAV
            if cav['name'] not in self._calibration:
                self._calibration[cav['name']] = \
                    engine.camera_calibration(rig.cameras, lidar_tf)
            intrinsics, extrinsics = self._calibration[cav['name']]
            kwargs = {'images_rgb': [np.ascontiguousarray(img[:, :, ::-1])
                                     for img in images],
                      'intrinsics': intrinsics, 'cam_to_lidar': extrinsics}
        message = engine.encode(
            cav['name'], index, step, t_sim, lidar_tf,
            points=points if engine.use_lidar else None,
            body_box=rig.body_box(engine.ego_margin), **kwargs)
        if message.device.startswith('cuda'):
            engine.torch.cuda.synchronize(message.device)
        cav['simo_message'] = message
        cav['lidar_transform'] = lidar_tf
        return (time.perf_counter() - t0) * 1000.0

    def _drop_self(self, cav, det):
        """Remove the candidates on the receiver's own body: the model was
        trained with every vehicle of the scene as ground truth, the ego
        included, and proposes a car where its own sensor sits."""
        if not len(det.corners):
            return det
        lo, hi = cav['rig'].body_box(0.5)
        inv = np.linalg.inv(transform_matrix(cav['lidar_transform']))
        centres = det.centres() @ inv[:3, :3].T + inv[:3, 3]
        own = np.all((centres[:, :2] >= lo[:2]) & (centres[:, :2] <= hi[:2]),
                     axis=1)
        det.corners, det.scores, det.labels = \
            det.corners[~own], det.scores[~own], det.labels[~own]
        return det

    def _raw(self, det):
        return (det.centres(), det.scores,
                [det.class_names[int(i)] for i in det.labels])

    def detect(self, index, cav, cavs, gt_objects, point_counts):
        engine = self.engine
        t0 = time.perf_counter()
        own = cav['simo_message']
        others = []
        if self.cooperative:
            here = own.pose[:3, 3]
            for other in cavs:
                if other is cav:
                    continue
                msg = other['simo_message']
                if np.linalg.norm(msg.pose[:3, 3] - here) <= self.comm_range:
                    others.append(msg)
        det = self._drop_self(cav, engine.detect(own, others))
        cav['simo_senders'] = [m.name for m in others]
        t_fuse = (time.perf_counter() - t0) * 1000.0
        reference = None
        if self.cooperative and self.single_reference:
            # not timed: it exists only to score the cooperation gain
            reference = self._raw(self._drop_self(cav, engine.detect(own)))
        centres, scores, classes = self._raw(det)
        return PerceptionResult(det.boxes(engine.thresholds), centres,
                                scores, classes, t_fuse, reference)

    def describe(self):
        engine = self.engine
        return {'backend': self.name,
                'weights': os.path.basename(engine.weights_path),
                'model_config': os.path.basename(engine.config_path),
                'sensors': engine.sensors,
                'cooperation': self.cooperation,
                'comm_range_m': self.comm_range,
                'sweeps': engine.n_sweeps,
                'sweep_interval_s': engine.sweep_interval_s,
                'score_threshold': engine.thresholds,
                'devices': engine.devices,
                'class_names': engine.class_names}

    def label(self):
        return 'SiMO (%s%s)' % (self.engine.sensors,
                                ', ideal cooperation' if self.cooperative
                                else '')


# ---------------------------------------------------------------------------
# YOLOv8 + LiDAR frustum fusion (optional)
# ---------------------------------------------------------------------------
class YoloBackend(_Backend):
    name = 'yolo'

    def __init__(self, cfg, repo_root, dt, opt):
        super().__init__(cfg, repo_root, dt, opt)
        import importlib.util
        if importlib.util.find_spec('ultralytics') is None:
            raise RuntimeError(
                'the yolo perception backend needs the ultralytics package '
                '(pip install ultralytics); SiMO is the default backend')
        self.capacity = getattr(opt, 'model', None) or \
            str(cfg['detection']['model_capacity'])
        weights_dir = str(cfg['detection']['weights_dir'])
        self.weights_dir = weights_dir if os.path.isabs(weights_dir) \
            else os.path.join(repo_root, weights_dir)

    def attach(self, index, cav):
        from detector import YoloDetector
        from fusion import CameraLidarFusion
        # one YOLO model per CAV (only the first announces itself)
        cav['detector'] = YoloDetector(
            self.capacity, self.weights_dir,
            self.cfg['detection']['confidence'],
            list(self.cfg['detection']['classes']), quiet=index > 0)
        cav['fusion'] = CameraLidarFusion(self.cfg['fusion'])

    def local_stage(self, index, cav, step, t_sim, images, points):
        t0 = time.perf_counter()
        cav['cam_dets'] = cav['detector'].detect_batch(images)
        cav['points'] = points
        return (time.perf_counter() - t0) * 1000.0

    def detect(self, index, cav, cavs, gt_objects, point_counts):
        from fusion import suppress_contained
        t0 = time.perf_counter()
        rig, fusion = cav['rig'], cav['fusion']
        world_pts = fusion.lidar_to_world(cav['points'], rig.lidar.sensor)
        fused = []
        for i, cam in enumerate(rig.cameras):
            if not cav['cam_dets'][i]:
                continue
            dets_2d = suppress_contained(cav['cam_dets'][i])
            uv, depth, wxyz = fusion.project_to_camera(world_pts, cam.sensor)
            fused += fusion.fuse_camera(dets_2d, uv, depth, wxyz, i,
                                        cam.sensor)
        merged = fusion.merge_detections(fused)
        t_fuse = (time.perf_counter() - t0) * 1000.0
        centres = np.array([d.position for d in merged]).reshape(-1, 3)
        scores = np.array([d.conf for d in merged])
        return PerceptionResult(merged, centres, scores, None, t_fuse)

    def describe(self):
        return {'backend': self.name, 'capacity': self.capacity,
                'confidence': float(self.cfg['detection']['confidence'])}

    def label(self):
        return 'YOLOv8-%s + LiDAR fusion' % self.capacity


# ---------------------------------------------------------------------------
# oracle
# ---------------------------------------------------------------------------
class OracleBackend(_Backend):
    name = 'oracle'

    def __init__(self, cfg, repo_root, dt, opt):
        super().__init__(cfg, repo_root, dt, opt)
        oracle_cfg = cfg.get('oracle', None) or {}
        self.min_points = int(oracle_cfg.get('min_points', 5))
        self.max_range = float(oracle_cfg.get('range_m', 54.4))

    def sensing_cfg(self):
        return (self.cfg.get('simo', None) or {}).get('sensing', None) \
            or self.cfg['sensing']

    def with_cameras(self):
        return False

    def local_stage(self, index, cav, step, t_sim, images, points):
        cav['lidar_transform'] = cav['rig'].lidar.sensor.get_transform()
        return 0.0

    def detect(self, index, cav, cavs, gt_objects, point_counts):
        t0 = time.perf_counter()
        here = transform_matrix(cav['lidar_transform'])[:3, 3]
        detections = []
        for gt, n in zip(gt_objects, point_counts):
            if gt.get('static') or n < self.min_points:
                continue
            if np.linalg.norm(gt['centre'][:2] - here[:2]) > self.max_range:
                continue
            detections.append(OracleDetection(gt, n))
        centres = np.array([d.position for d in detections]).reshape(-1, 3)
        return PerceptionResult(
            detections, centres, np.ones(len(detections)),
            [d.cls_name for d in detections],
            (time.perf_counter() - t0) * 1000.0)

    def describe(self):
        return {'backend': self.name, 'min_points': self.min_points,
                'range_m': self.max_range}

    def label(self):
        return 'oracle (ground truth with >= %d LiDAR points)' \
            % self.min_points


def create_backend(name, cfg, repo_root, dt, opt):
    if name == 'simo':
        return SimoBackend(cfg, repo_root, dt, opt)
    if name == 'yolo':
        return YoloBackend(cfg, repo_root, dt, opt)
    if name == 'oracle':
        return OracleBackend(cfg, repo_root, dt, opt)
    raise ValueError('unknown perception backend %r (choose from %s)'
                     % (name, ', '.join(BACKENDS)))


# ---------------------------------------------------------------------------
# ground truth for the AP
# ---------------------------------------------------------------------------
def box_fields(centre_location, rotation, extent, min_extent=0.15):
    """The box description points_in_boxes() and the AP need."""
    import carla
    matrix = transform_matrix(carla.Transform(centre_location, rotation))
    return {'centre': np.array([centre_location.x, centre_location.y,
                                centre_location.z]),
            'inv': np.linalg.inv(matrix),
            'extent': np.array([max(extent.x, min_extent),
                                max(extent.y, min_extent),
                                max(extent.z, min_extent)])}


def static_vehicle_ground_truth(world):
    """Parked cars, trucks and buses of the map (level meshes, not actors)
    as ground-truth boxes: a detector is right to find them, so they must
    not count as false positives.

    Parked two-wheelers are left out on purpose. They stand without a
    rider, and the benchmark the model was trained on (SimBEV2X) labels
    only the static cars and trucks of a map: its bicycles and motorcycles
    are always ridden actors."""
    import carla
    labels = (('Car', 'car'), ('Truck', 'truck'), ('Bus', 'bus'))
    objects = []
    for label_name, cls in labels:
        label = getattr(carla.CityObjectLabel, label_name, None)
        if label is None:
            continue
        for i, bb in enumerate(world.get_level_bbs(label)):
            fields = box_fields(bb.location, bb.rotation, bb.extent)
            fields.update({'id': 'static_%s_%d' % (cls, i),
                           'simbev_class': cls, 'static': True,
                           'category': ap_eval.CLASS_TO_CATEGORY[cls]})
            objects.append(fields)
    return objects
