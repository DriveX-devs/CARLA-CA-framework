# -*- coding: utf-8 -*-
"""
Sensor rig for the CA pipeline: RGB cameras + one LiDAR per CAV.

Sensors are built directly with the CARLA API. Data retrieval is queue-based
so that, in synchronous mode, every world.tick() can be matched with exactly
the sensor frames it produced.

The rig is described by a ``sensing`` configuration block: a ``lidar`` entry
and an optional ``camera`` entry (a rig without cameras is a LiDAR-only CAV,
which is how SiMO runs by default). Two rigs ship with the framework:

  * the SiMO rig (``simo.sensing``), which reproduces the sensors the model
    was trained on (SimBEV2X vehicles: roof LiDAR at 1.8 m keeping 32 of its
    128 rings, six 1600x900 cameras with an 80 degree field of view);
  * the YOLOv8 rig (``sensing``): four Full-HD cameras and a 64-ring LiDAR.

With CARLA's multi-GPU mode the server spreads these sensors over its
secondary (rendering) servers by itself: nothing changes on this side.
"""

import queue
import weakref

import carla
import numpy as np

# LiDAR blueprint attributes a configuration may set
_LIDAR_KEYS = ('channels', 'range', 'points_per_second', 'rotation_frequency',
               'upper_fov', 'lower_fov', 'horizontal_fov',
               'dropoff_general_rate', 'dropoff_intensity_limit',
               'dropoff_zero_intensity', 'noise_stddev',
               'atmosphere_attenuation_rate')


class RgbCamera(object):
    """One RGB camera attached to a vehicle at a relative (x, y, z, yaw)."""

    def __init__(self, world, vehicle, rel_pos, cam_cfg, name):
        self.name = name
        bp = world.get_blueprint_library().find('sensor.camera.rgb')
        bp.set_attribute('image_size_x', str(int(cam_cfg['image_size_x'])))
        bp.set_attribute('image_size_y', str(int(cam_cfg['image_size_y'])))
        bp.set_attribute('fov', str(float(cam_cfg['fov'])))

        x, y, z, yaw = [float(v) for v in rel_pos]
        transform = carla.Transform(carla.Location(x=x, y=y, z=z),
                                    carla.Rotation(yaw=yaw))
        self.sensor = world.spawn_actor(bp, transform, attach_to=vehicle)

        self.width = int(cam_cfg['image_size_x'])
        self.height = int(cam_cfg['image_size_y'])
        self.fov = float(cam_cfg['fov'])
        self._queue = queue.Queue()

        weak_self = weakref.ref(self)
        self.sensor.listen(lambda data: RgbCamera._on_data(weak_self, data))

    @staticmethod
    def _on_data(weak_self, data):
        self = weak_self()
        if self is None:
            return
        arr = np.frombuffer(data.raw_data, dtype=np.uint8)
        arr = np.reshape(arr, (data.height, data.width, 4))[:, :, :3]
        self._queue.put((data.frame, np.ascontiguousarray(arr)))  # BGR

    def fetch(self, frame_id, timeout=10.0):
        """Return the BGR image produced at world frame >= frame_id."""
        while True:
            frame, img = self._queue.get(timeout=timeout)
            if frame >= frame_id:
                return img

    def destroy(self):
        try:
            self.sensor.stop()
            self.sensor.destroy()
        except Exception:
            pass


class Lidar(object):
    """Spinning LiDAR attached to the vehicle roof."""

    def __init__(self, world, vehicle, lidar_cfg):
        bp = world.get_blueprint_library().find('sensor.lidar.ray_cast')
        for key in _LIDAR_KEYS:
            if key in lidar_cfg and lidar_cfg[key] is not None:
                bp.set_attribute(key, str(lidar_cfg[key]))

        transform = carla.Transform(
            carla.Location(x=float(lidar_cfg.get('x', -0.5)),
                           z=float(lidar_cfg['z'])))
        self.sensor = world.spawn_actor(bp, transform, attach_to=vehicle)
        self.channels = int(bp.get_attribute('channels'))
        self._queue = queue.Queue()

        weak_self = weakref.ref(self)
        self.sensor.listen(lambda data: Lidar._on_data(weak_self, data))

    @staticmethod
    def _on_data(weak_self, data):
        self = weak_self()
        if self is None:
            return
        pts = np.copy(np.frombuffer(data.raw_data, dtype=np.float32))
        pts = np.reshape(pts, (pts.shape[0] // 4, 4))  # x, y, z, intensity
        self._queue.put((data.frame, pts))

    def fetch(self, frame_id, timeout=10.0):
        """Return the (N, 4) sweep produced at world frame >= frame_id, in
        the sensor frame (x forward, y right, z up)."""
        while True:
            frame, pts = self._queue.get(timeout=timeout)
            if frame >= frame_id:
                return pts

    def destroy(self):
        try:
            self.sensor.stop()
            self.sensor.destroy()
        except Exception:
            pass


class SensorRig(object):
    """All sensors of one CAV plus synced retrieval after each tick."""

    CAM_NAMES = ('front', 'right', 'left', 'rear')

    def __init__(self, world, vehicle, sensing_cfg, with_cameras=True):
        self.vehicle = vehicle
        cam_cfg = sensing_cfg.get('camera', None) if with_cameras else None
        self.cameras = []
        if cam_cfg and cam_cfg.get('positions', None):
            names = list(cam_cfg.get('names', None) or self.CAM_NAMES)
            self.cameras = [
                RgbCamera(world, vehicle, rel_pos, cam_cfg,
                          names[i] if i < len(names) else 'cam%d' % i)
                for i, rel_pos in enumerate(cam_cfg['positions'])]
        self.lidar = Lidar(world, vehicle, sensing_cfg['lidar'])

    def body_box(self, margin=0.3):
        """(lo, hi) corners, in the LiDAR frame, of the box that holds the
        vehicle's own body: LiDAR returns inside it are the ego itself.

        The LiDAR is mounted without rotation, so its frame is the vehicle
        frame shifted by the mount position."""
        bb = self.vehicle.bounding_box
        mount = self.lidar.sensor.get_transform().location - \
            self.vehicle.get_transform().location
        yaw = np.radians(self.vehicle.get_transform().rotation.yaw)
        # mount offset expressed in the vehicle frame (yaw only: the pitch
        # and roll of a car on the road are negligible for a body mask)
        mx = np.cos(yaw) * mount.x + np.sin(yaw) * mount.y
        my = -np.sin(yaw) * mount.x + np.cos(yaw) * mount.y
        centre = np.array([bb.location.x - mx, bb.location.y - my,
                           bb.location.z - mount.z])
        half = np.array([bb.extent.x, bb.extent.y, bb.extent.z]) + margin
        return centre - half, centre + half

    def fetch(self, frame_id, timeout=10.0):
        """Return ([BGR image per camera], lidar points) for this tick.
        Raises queue.Empty when a sensor does not answer within timeout."""
        images = [cam.fetch(frame_id, timeout) for cam in self.cameras]
        points = self.lidar.fetch(frame_id, timeout)
        return images, points

    def destroy(self):
        for cam in self.cameras:
            cam.destroy()
        self.lidar.destroy()
