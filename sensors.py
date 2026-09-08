# -*- coding: utf-8 -*-
"""
Sensor rig for the CA pipeline: 4 RGB cameras + 1 LiDAR per CAV.

Sensors are built directly with the CARLA API (this OpenCDA fork's
create_vehicle_manager is wired to the ms-van3t stack and unusable
standalone). Data retrieval is queue-based so that, in synchronous mode,
every world.tick() can be matched with exactly the sensor frames it produced.
"""

import queue
import weakref

import carla
import numpy as np


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

    def fetch(self, frame_id, timeout=2.0):
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
        for key in ('channels', 'range', 'points_per_second',
                    'rotation_frequency', 'upper_fov', 'lower_fov',
                    'dropoff_general_rate', 'dropoff_intensity_limit',
                    'dropoff_zero_intensity', 'noise_stddev'):
            bp.set_attribute(key, str(lidar_cfg[key]))

        transform = carla.Transform(
            carla.Location(x=-0.5, z=float(lidar_cfg['z'])))
        self.sensor = world.spawn_actor(bp, transform, attach_to=vehicle)
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

    def fetch(self, frame_id, timeout=2.0):
        """Return the (N, 4) sweep produced at world frame >= frame_id."""
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

    def __init__(self, world, vehicle, sensing_cfg):
        self.vehicle = vehicle
        cam_cfg = sensing_cfg['camera']
        self.cameras = [
            RgbCamera(world, vehicle, rel_pos, cam_cfg,
                      self.CAM_NAMES[i] if i < len(self.CAM_NAMES)
                      else 'cam%d' % i)
            for i, rel_pos in enumerate(cam_cfg['positions'])]
        self.lidar = Lidar(world, vehicle, sensing_cfg['lidar'])

    def fetch(self, frame_id):
        """Return ([BGR image per camera], lidar points) for this tick."""
        images = [cam.fetch(frame_id) for cam in self.cameras]
        points = self.lidar.fetch(frame_id)
        return images, points

    def destroy(self):
        for cam in self.cameras:
            cam.destroy()
        self.lidar.destroy()
