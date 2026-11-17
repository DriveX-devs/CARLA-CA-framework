# -*- coding: utf-8 -*-
"""
SiMO perception for the CA pipeline.

SiMO (cooperative LiDAR + camera 3D detection, the ``simo_proto`` inference
package) replaces the YOLOv8 + LiDAR frustum fusion as the base model. The
package is used unmodified; this module only feeds it and converts its
output.

The model is always run through its *split* interface, because that is the
shape every use of it in this framework has:

    local stage   one per CAV, on that CAV's own sensors and (optionally) on
                  its own GPU:  SimoPerception.encode() -> SimoMessage
    fusion stage  at a receiver, on its own message plus any subset of the
                  others':      SimoPerception.detect() -> SimoDetections

A CAV that fuses nothing but its own message is the single-agent detector
the basic policies use. Handing detect() the messages of the other CAVs is
SiMO's cooperative (intermediate) fusion; ``cooperation: ideal`` does that
in-process, with no network in between. The receiver need not be a vehicle:
``sensorless_message()`` gives the empty local message of a station without
sensors (a roadside unit / edge server), which is how an edge-side fusion of
the CAVs' feature maps is expressed. A message can also be restricted to a
subset of its BEV cells (``SimoMessage.select_cells``), the per-cell hook
request-driven schemes need.

Frames. CARLA's LiDAR points are already in the convention the model
expects (sensor frame, x forward, y right, z up), and the 4x4 pose of a
sensor is ``carla.Transform.get_matrix()``. Detections come back in the
receiver's LiDAR frame and are moved to the CARLA world frame here.
"""

import math
import os
import sys
from collections import deque

import numpy as np

from carla_utils import camera_intrinsic, transform_matrix

# SiMO class -> CA category (the three categories the tracker, the LDMs and
# the collision avoidance work with). Static classes are detected by the
# model but are not road users: they never become CA tracks.
SIMO_CATEGORY = {
    'pedestrian': 'pedestrian',
    'bicycle': 'bike', 'motorcycle': 'bike',
    'car': 'vehicle', 'truck': 'vehicle', 'bus': 'vehicle',
}
SIMO_STATIC = ('traffic_light', 'traffic_sign', 'traffic_cone', 'barrier')

# OpenCV camera axes (x right, y down, z forward) expressed in the camera's
# CARLA frame (x forward, y right, z up)
_CARLA_FROM_OPENCV = np.array([[0, 0, 1, 0], [1, 0, 0, 0],
                               [0, -1, 0, 0], [0, 0, 0, 1]], dtype=np.float64)

# The tensors of a message that travel to a receiver ('head_skip' stays with
# its owner), and the BEV grid each one lives on
TRANSMITTED = ('bev', 'head_coop', 'pillars')


def resolve_path(path, repo_root):
    path = os.path.expanduser(str(path))
    return path if os.path.isabs(path) else os.path.join(repo_root, path)


def import_simo(package_dir):
    """Import the simo_proto package from its directory (it is not
    pip-installed: it ships as a folder with code, config and weights)."""
    if not os.path.isdir(os.path.join(package_dir, 'simo_proto')):
        raise RuntimeError(
            'SiMO package not found: %s has no simo_proto/ folder. Set '
            'simo.package_dir in the scenario yaml (or SIMO_PACKAGE_DIR) to '
            'the directory that contains simo_proto/, configs/ and weights/.'
            % package_dir)
    if package_dir not in sys.path:
        sys.path.insert(0, package_dir)
    import simo_proto
    return simo_proto


class SweepBuffer(object):
    """LiDAR history of one agent.

    The model was trained on the current sweep plus up to nine previous ones
    (SimBEV2X key frames, about 0.43 s apart), every sweep moved into the
    current LiDAR frame and tagged with its time lag. The buffer keeps the
    recent sweeps of the simulation and picks, for each of the wanted lags,
    the stored sweep closest to it.
    """

    def __init__(self, n_sweeps, interval_s, dt):
        self.n_sweeps = max(1, int(n_sweeps))
        self.interval_s = float(interval_s)
        self.dt = float(dt)
        # ticks back of every previous sweep (rounded to the simulation step)
        self.offsets = sorted({max(1, int(round(k * self.interval_s / dt)))
                               for k in range(1, self.n_sweeps)})
        keep = (self.offsets[-1] if self.offsets else 0) + 1
        self._history = deque(maxlen=keep)   # (tick, t_sim, pose, xyz)

    def assemble(self, tick, t_sim, pose, xyz):
        """Store the current sweep and return (N, 4) float32 points
        [x, y, z, time lag s] in the current LiDAR frame."""
        xyz = np.asarray(xyz, dtype=np.float32)
        self._history.append((tick, t_sim, pose, xyz))
        parts = [np.c_[xyz, np.zeros(len(xyz), np.float32)]]
        if self.offsets:
            by_tick = {h[0]: h for h in self._history}
            inv_now = np.linalg.inv(pose)
            for off in self.offsets:
                past = by_tick.get(tick - off)
                if past is None:
                    continue
                _, t_past, pose_past, xyz_past = past
                rel = (inv_now @ pose_past).astype(np.float32)
                moved = xyz_past @ rel[:3, :3].T + rel[:3, 3]
                lag = np.full(len(moved), t_sim - t_past, np.float32)
                parts.append(np.c_[moved, lag])
        return np.concatenate(parts).astype(np.float32)


class SimoMessage(object):
    """What one agent's local stage produces: the tensors of the model's
    message plus the bookkeeping the receivers need."""

    __slots__ = ('name', 'tensors', 'pose', 'device', 'n_points', 't_sim')

    def __init__(self, name, tensors, pose, device, n_points, t_sim):
        self.name = name
        self.tensors = tensors      # dict: bev, head_skip, head_coop, pillars
        self.pose = pose            # 4x4 T_world_lidar when it was sensed
        self.device = device
        self.n_points = int(n_points)
        self.t_sim = t_sim

    def transmitted_bytes(self, dtype_bytes=4):
        """Size of the transmitted part, uncompressed (float32 by default)."""
        return int(sum(self.tensors[k].numel() for k in TRANSMITTED)
                   * dtype_bytes)

    def occupied_cells(self):
        """(H, W) bool mask of the 0.8 m BEV cells that hold any feature."""
        return (self.tensors['bev'][0].abs().amax(dim=0) > 0).cpu().numpy()

    def select_cells(self, mask):
        """A copy of the message that carries only the BEV cells where
        ``mask`` (H, W, on the 'bev' grid) is true; the finer maps are
        masked with the mask upsampled to their grid. The other cells are
        zeroed, which is what a receiver sees of a cell nobody sent."""
        import torch
        import torch.nn.functional as F
        bev = self.tensors['bev']
        m = torch.as_tensor(np.asarray(mask), dtype=bev.dtype,
                            device=bev.device)[None, None]
        out = {}
        for key, value in self.tensors.items():
            if value.shape[-2:] == m.shape[-2:]:
                out[key] = value * m
            else:
                out[key] = value * F.interpolate(m, size=value.shape[-2:],
                                                 mode='nearest')
        return SimoMessage(self.name, out, self.pose, self.device,
                           self.n_points, self.t_sim)


class SimoBox(object):
    """One SiMO detection in the CARLA world frame."""

    __slots__ = ('position', 'conf', 'cls_name', 'category', 'corners',
                 'yaw', 'extent', 'n_points', 'camera_idx', 'bbox',
                 'uv_points')

    def __init__(self, corners_world, score, cls_name):
        self.corners = corners_world                 # (8, 3)
        self.position = corners_world.mean(axis=0)   # box centre, world
        self.conf = float(score)
        self.cls_name = cls_name
        self.category = SIMO_CATEGORY.get(cls_name)
        front = corners_world[[0, 1]].mean(axis=0) - \
            corners_world[[2, 3]].mean(axis=0)
        self.yaw = math.degrees(math.atan2(front[1], front[0]))
        self.extent = np.array([
            np.linalg.norm(corners_world[0] - corners_world[3]),
            np.linalg.norm(corners_world[0] - corners_world[1]),
            np.linalg.norm(corners_world[0] - corners_world[4])])
        # fields of the camera+LiDAR Detection3D the downstream code reads
        self.n_points = 0
        self.camera_idx = -1
        self.bbox = None
        self.uv_points = None


class SimoDetections(object):
    """All the boxes of one fusion, kept both raw (down to the model's own
    score floor, for the AP metric) and thresholded (for tracking)."""

    def __init__(self, corners_world, scores, labels, class_names):
        self.corners = corners_world      # (N, 8, 3) world frame
        self.scores = scores              # (N,)
        self.labels = labels              # (N,) indices into class_names
        self.class_names = class_names

    def centres(self):
        return self.corners.mean(axis=1) if len(self.corners) \
            else np.zeros((0, 3))

    def boxes(self, thresholds):
        """Road-user boxes with a score at or above their category's
        threshold, as SimoBox objects."""
        out = []
        for corners, score, label in zip(self.corners, self.scores,
                                         self.labels):
            name = self.class_names[int(label)]
            category = SIMO_CATEGORY.get(name)
            if category is None or score < thresholds[category]:
                continue
            out.append(SimoBox(corners, score, name))
        return out


class SimoPerception(object):
    """The SiMO models of a run and the two stages built on them.

    One model instance is loaded per device and shared by the agents placed
    on it: inference is stateless, so a model per CAV would only cost
    memory. ``devices`` spreads the local stages over the GPUs (agent i on
    devices[i % len(devices)]); the fusion stage runs on the receiver's
    device.
    """

    def __init__(self, simo_cfg, repo_root, dt, devices=None, quiet=False):
        import torch
        self.torch = torch
        self.cfg = simo_cfg
        package_dir = resolve_path(
            os.environ.get('SIMO_PACKAGE_DIR') or simo_cfg['package_dir'],
            repo_root)
        self.package_dir = package_dir
        self.simo = import_simo(package_dir)
        config_path = resolve_path(simo_cfg.get('config',
                                                'configs/simo_mm10.yaml'),
                                   package_dir)
        weights_path = resolve_path(simo_cfg['weights'], package_dir)
        for path in (config_path, weights_path):
            if not os.path.isfile(path):
                raise RuntimeError('SiMO file not found: %s' % path)
        self.config_path, self.weights_path = config_path, weights_path

        self.sensors = str(simo_cfg.get('sensors', 'lidar'))
        if self.sensors not in ('lidar', 'camera', 'both'):
            raise ValueError("simo.sensors must be lidar, camera or both, "
                             "got %r" % self.sensors)
        self.use_lidar = self.sensors in ('lidar', 'both')
        self.use_camera = self.sensors in ('camera', 'both')

        if devices is None:
            devices = simo_cfg.get('devices', None)
        if isinstance(devices, str):
            devices = [d for d in devices.replace(' ', '').split(',') if d]
        if not devices:
            devices = ['cuda:0' if torch.cuda.is_available() else 'cpu']
        self.devices = [str(d) for d in devices]
        for dev in self.devices:
            if dev.startswith('cuda'):
                idx = int(dev.split(':')[1]) if ':' in dev else 0
                if not torch.cuda.is_available() or \
                        idx >= torch.cuda.device_count():
                    raise RuntimeError(
                        'SiMO device %s is not available (%d CUDA device(s) '
                        'visible)' % (dev, torch.cuda.device_count()
                                      if torch.cuda.is_available() else 0))

        model_cfg = self.simo.load_config(config_path)
        state = torch.load(weights_path, map_location='cpu',
                           weights_only=True)
        self.models = {}
        for dev in dict.fromkeys(self.devices):
            model = self.simo.SiMOModel(model_cfg).to(dev).eval()
            model.load_state_dict(state, strict=True)
            self.models[dev] = model
        first = self.models[self.devices[0]]
        self.class_names = list(first.post.class_names)
        self.lidar_range = [float(v) for v in first.lidar_range]

        self.dt = float(dt)
        self.n_sweeps = int(simo_cfg.get('sweeps', 10))
        self.sweep_interval_s = float(simo_cfg.get('sweep_interval_s', 0.43))
        thr = simo_cfg.get('score_threshold', None) or {}
        self.thresholds = {c: float(thr.get(c, 0.3))
                           for c in ('vehicle', 'bike', 'pedestrian')}
        # ego-body returns are removed here with the ego's own bounding box
        # (the package removes a fixed box sized for another sensor mount)
        self.ego_margin = float(simo_cfg.get('ego_mask_margin', 0.3))
        self._buffers = {}
        self._agent_device = {}

        # warm up: the first CUDA inference is much slower than steady state
        for dev, model in self.models.items():
            dummy = np.zeros((2000, 4), np.float32)
            dummy[:, :2] = np.random.RandomState(0).uniform(-20, 20,
                                                            (2000, 2))
            dummy[:, 2] = -1.8
            msg = model.encode_agent(model.prepare_agent(dummy, None))
            model.postprocess(model.fuse(msg, None, [np.identity(4)]))
            self._message_shapes = {k: tuple(v.shape)
                                    for k, v in msg.items()}
        if not quiet:
            print('SiMO ready: %s | sensors %s | %d sweep(s) %.2f s apart | '
                  'devices %s | thresholds %s'
                  % (os.path.basename(weights_path), self.sensors,
                     self.n_sweeps, self.sweep_interval_s,
                     ', '.join(dict.fromkeys(self.devices)),
                     self.thresholds))

    # ------------------------------------------------------------------
    def device_of(self, agent_index):
        return self.devices[agent_index % len(self.devices)]

    def _buffer(self, name):
        if name not in self._buffers:
            self._buffers[name] = SweepBuffer(self.n_sweeps,
                                              self.sweep_interval_s, self.dt)
        return self._buffers[name]

    @staticmethod
    def mask_body(xyz, lo, hi):
        """Drop the points inside the axis-aligned box [lo, hi] (the ego's
        own body, in the LiDAR frame)."""
        inside = np.all((xyz >= lo) & (xyz <= hi), axis=1)
        return xyz[~inside]

    def camera_calibration(self, cameras, lidar_transform):
        """Per-camera intrinsics and camera-to-LiDAR extrinsics (OpenCV
        camera axes) from the spawned sensors."""
        inv_lidar = np.linalg.inv(transform_matrix(lidar_transform))
        intrinsics, extrinsics = [], []
        for cam in cameras:
            intrinsics.append(camera_intrinsic(cam.width, cam.height,
                                               cam.fov).astype(np.float32))
            cam_to_lidar = inv_lidar @ \
                transform_matrix(cam.sensor.get_transform())
            extrinsics.append((cam_to_lidar @ _CARLA_FROM_OPENCV)
                              .astype(np.float32))
        return intrinsics, extrinsics

    # ---- local stage --------------------------------------------------
    def encode(self, name, agent_index, tick, t_sim, lidar_transform,
               points=None, body_box=None, images_rgb=None, intrinsics=None,
               cam_to_lidar=None):
        """Local stage of one agent on its own sensors.

        points: (N, 3+) LiDAR returns of this tick in the sensor frame;
        body_box: (lo, hi) corners of the ego's body in that frame;
        images_rgb: list of HxWx3 uint8 RGB arrays (with intrinsics and
        cam_to_lidar) when the cameras are used.
        """
        dev = self.device_of(agent_index)
        model = self.models[dev]
        pose = transform_matrix(lidar_transform)
        pts4 = None
        n_points = 0
        if self.use_lidar:
            if points is None:
                raise ValueError('SiMO sensors=%s needs LiDAR points'
                                 % self.sensors)
            xyz = np.asarray(points, dtype=np.float32)[:, :3]
            if body_box is not None:
                xyz = self.mask_body(xyz, body_box[0], body_box[1])
            pts4 = self._buffer(name).assemble(tick, t_sim, pose, xyz)
            n_points = len(pts4)
        images = None
        if self.use_camera:
            if not images_rgb:
                raise ValueError('SiMO sensors=%s needs camera images'
                                 % self.sensors)
            images = images_rgb
        with self.torch.no_grad():
            agent = model.prepare_agent(pts4, images, intrinsics,
                                        cam_to_lidar)
            tensors = model.encode_agent(agent)
        return SimoMessage(name, tensors, pose, dev, n_points, t_sim)

    def sensorless_message(self, name, pose, device=None):
        """The local message of a station without sensors (a roadside unit
        or an edge server acting as the fusion point).

        Its BEV map is empty, as the model makes it for an agent with no
        sensor; its head inputs are what the detection head computes for an
        agent with no LiDAR (not zeros: the head's layers have biases)."""
        dev = device or self.devices[0]
        model = self.models[dev]
        torch = self.torch
        enc = model.lidar_encoder
        shape = (1, enc.num_features, enc.ny, enc.nx)
        with torch.no_grad():
            head = model.head.agent_message(
                None, None, torch.zeros(1, device=dev), shape)
            bev = torch.zeros(self._message_shapes['bev'], device=dev)
        tensors = {'bev': bev, 'head_skip': head['skip'],
                   'head_coop': head['coop'], 'pillars': head['pillars']}
        return SimoMessage(name, tensors, np.asarray(pose, np.float64), dev,
                           0, None)

    # ---- fusion stage -------------------------------------------------
    def detect(self, ego, others=()):
        """Fuse a receiver's own message with the received ones and decode
        the boxes. ``others`` empty = single-agent detection."""
        torch = self.torch
        dev = ego.device
        model = self.models[dev]
        with torch.no_grad():
            received = None
            poses = [ego.pose]
            if others:
                received = {k: torch.cat([m.tensors[k].to(dev)
                                          for m in others])
                            for k in TRANSMITTED}
                poses += [m.pose for m in others]
            out = model.fuse(ego.tensors, received, poses)
            corners, scores, labels = model.postprocess(out)
        corners = np.asarray(corners, dtype=np.float64).reshape(-1, 8, 3)
        scores = np.asarray(scores, dtype=np.float64).reshape(-1)
        labels = np.asarray(labels, dtype=np.int64).reshape(-1)
        if len(corners):
            homog = np.concatenate([corners, np.ones((len(corners), 8, 1))],
                                   axis=2)
            corners = (homog @ ego.pose.T)[..., :3]
        return SimoDetections(corners, scores, labels, self.class_names)
