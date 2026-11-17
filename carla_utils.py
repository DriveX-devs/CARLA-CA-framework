# -*- coding: utf-8 -*-
"""
CARLA helpers of the CA framework: world set-up and the frame conversions
the perception pipeline needs.

Everything here talks to the CARLA Python API directly (tested with CARLA
0.9.16), so the framework runs on its own: no co-simulation framework has to
be installed around it.

Frames
------
CARLA uses a left-handed world: x forward/east, y right/south, z up, and a
yaw that grows clockwise seen from above. ``transform_matrix()`` returns the
4x4 matrix of a ``carla.Transform`` in that convention, i.e. the matrix that
takes a point expressed in the frame of the transform into the world frame.
"""

import math
import time
from datetime import datetime

import carla
import numpy as np

# Weather parameters a scenario yaml may set (world.weather.*)
_WEATHER_KEYS = ('sun_altitude_angle', 'sun_azimuth_angle', 'cloudiness',
                 'precipitation', 'precipitation_deposits', 'wind_intensity',
                 'fog_density', 'fog_distance', 'fog_falloff', 'wetness')


def add_current_time(cfg):
    """Stamp a configuration with the wall-clock time the run started at."""
    cfg['current_time'] = datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
    return cfg


# ---------------------------------------------------------------------------
# world
# ---------------------------------------------------------------------------
class CarlaSession(object):
    """A connection to a CARLA server with one map loaded in synchronous mode.

    The settings found on the server are restored by ``close()``: a server
    left in synchronous mode with nobody ticking it hangs the next client.

    ``reload_world=False`` keeps the world the server already has, after
    removing whatever actors a previous client left in it. CARLA's multi-GPU
    mode needs it: there a map loaded from a client reaches the primary
    server only, the rendering servers stop delivering camera frames, and a
    second load hangs the primary. Servers of a multi-GPU set-up are started
    on the wanted map instead (``CarlaUE4.sh /Game/Carla/Maps/<town> ...``).
    """

    def __init__(self, world_cfg, town, host='localhost', port=2000,
                 timeout_s=120.0, reload_world=True):
        self.client = carla.Client(host, int(port))
        self.client.set_timeout(float(timeout_s))
        self.town = str(town)

        available = [m.split('/')[-1] for m in
                     self.client.get_available_maps()]
        if self.town not in available:
            raise RuntimeError(
                'map %r is not installed on the CARLA server at %s:%d '
                '(available: %s). The additional maps (Town06, Town07, '
                'Town11, Town12, Town13, Town15) come from the '
                'AdditionalMaps package.'
                % (self.town, host, int(port), ', '.join(sorted(available))))

        current = self.client.get_world().get_map().name.split('/')[-1]
        if reload_world:
            self.world = self.client.load_world(self.town)
        elif current != self.town:
            raise RuntimeError(
                'the CARLA server at %s:%d runs %s and the world is not to '
                'be reloaded: start it on %s (CarlaUE4.sh '
                '/Game/Carla/Maps/%s ...), or drop --no-reload-world'
                % (host, int(port), current, self.town, self.town))
        else:
            self.world = self.client.get_world()
            removed = self._remove_leftover_actors()
            if removed:
                print('  removed %d actor(s) left in the world by a previous '
                      'client' % removed)

        self._original_settings = self.world.get_settings()
        settings = self.world.get_settings()
        settings.synchronous_mode = bool(world_cfg.get('sync_mode', True))
        settings.fixed_delta_seconds = float(
            world_cfg['fixed_delta_seconds'])
        # Sub-stepping keeps the physics step at 10 ms or less whatever the
        # frame period (CARLA requires fixed_delta <= substep * max_substeps)
        settings.substepping = True
        settings.max_substep_delta_time = 0.01
        settings.max_substeps = max(
            1, int(math.ceil(float(world_cfg['fixed_delta_seconds']) / 0.01)))
        self.world.apply_settings(settings)

        weather_cfg = world_cfg.get('weather', None)
        if weather_cfg:
            weather = self.world.get_weather()
            for key in _WEATHER_KEYS:
                if key in weather_cfg:
                    setattr(weather, key, float(weather_cfg[key]))
            self.world.set_weather(weather)

        self.map = self.world.get_map()

    def _remove_leftover_actors(self):
        actors = self.world.get_actors()
        leftovers = []
        for pattern in ('sensor.*', 'controller.*', 'vehicle.*', 'walker.*'):
            leftovers += list(actors.filter(pattern))
        for actor in leftovers:
            try:
                if actor.type_id.startswith(('sensor.', 'controller.')):
                    actor.stop()
                actor.destroy()
            except RuntimeError:
                pass
        return len(leftovers)

    def tick(self):
        return self.world.tick()

    def close(self):
        try:
            self.world.apply_settings(self._original_settings)
        except RuntimeError:
            pass


def wait_for_server(host, port, timeout_s=120.0):
    """Block until a CARLA server answers on host:port (or raise)."""
    deadline = time.time() + timeout_s
    last = None
    while time.time() < deadline:
        try:
            client = carla.Client(host, int(port))
            client.set_timeout(5.0)
            return client.get_server_version()
        except RuntimeError as exc:
            last = exc
            time.sleep(2.0)
    raise RuntimeError('no CARLA server on %s:%d after %.0f s (%s)'
                       % (host, int(port), timeout_s, last))


# ---------------------------------------------------------------------------
# frames
# ---------------------------------------------------------------------------
def pose_list(transform):
    """carla.Transform -> [x, y, z, roll, yaw, pitch] (m, degrees), the pose
    layout of OPV2V and SimBEV2X."""
    loc, rot = transform.location, transform.rotation
    return [loc.x, loc.y, loc.z, rot.roll, rot.yaw, rot.pitch]


def transform_matrix(transform):
    """4x4 matrix of a carla.Transform: its frame -> the world frame."""
    return np.array(transform.get_matrix(), dtype=np.float64)


def inverse_transform_matrix(transform):
    """4x4 matrix world frame -> the frame of a carla.Transform."""
    return np.array(transform.get_inverse_matrix(), dtype=np.float64)


def world_to_sensor(world_points, sensor_transform):
    """(4, N) homogeneous world points -> (4, N) points in a sensor frame."""
    return inverse_transform_matrix(sensor_transform) @ world_points


def camera_intrinsic(width, height, fov_deg):
    """3x3 pinhole intrinsic matrix of a CARLA camera (horizontal FOV)."""
    focal = width / (2.0 * math.tan(math.radians(fov_deg) / 2.0))
    k = np.identity(3)
    k[0, 0] = k[1, 1] = focal
    k[0, 2] = width / 2.0
    k[1, 2] = height / 2.0
    return k


def camera_intrinsic_of(camera_sensor):
    """Intrinsic matrix of a spawned camera, from its blueprint attributes."""
    return camera_intrinsic(int(camera_sensor.attributes['image_size_x']),
                            int(camera_sensor.attributes['image_size_y']),
                            float(camera_sensor.attributes['fov']))


def box_corners_world(actor):
    """(8, 3) world-frame corners of an actor's bounding box (bottom face
    first, then the top face, same order on both)."""
    bb = actor.bounding_box
    e = bb.extent
    local = np.array([[e.x, e.y, -e.z], [e.x, -e.y, -e.z],
                      [-e.x, -e.y, -e.z], [-e.x, e.y, -e.z],
                      [e.x, e.y, e.z], [e.x, -e.y, e.z],
                      [-e.x, -e.y, e.z], [-e.x, e.y, e.z]])
    local = local + np.array([bb.location.x, bb.location.y, bb.location.z])
    homog = np.c_[local, np.ones(8)].T
    return (transform_matrix(actor.get_transform()) @ homog)[:3].T
