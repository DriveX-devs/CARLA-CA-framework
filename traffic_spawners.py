# -*- coding: utf-8 -*-
"""
Background-actor spawners for the CA scenarios.

Every CA scenario populates the Town10HD_Opt intersection with three kinds of
non-CAV actors, all driven by CARLA itself (Traffic Manager autopilot for the
wheeled ones, `controller.ai.walker` for the walkers):

  * background vehicles — forced red, so they are visually distinct from the
    green CAVs in the annotated videos;
  * bikes — two-wheelers with a rider, preferentially spawned around the
    intersection;
  * pedestrians — walkers on the sidewalk navigation mesh.

These functions were previously imported from
`dataset_generator/generate_dataset.py`; they live here so that `CA/` depends
only on `opencda.*` and not on the dataset generator. They are pure CARLA
code (no OpenCDA, no dataset-dumping logic).

`experiment_support.py` reuses the two blueprint pools to replay a recorded
traffic manifest deterministically.
"""

import math
import random

import carla


# Only these vehicles render a single, faithful body paint, so forcing their
# `color` to red actually makes them look red. Everything else in CARLA's
# vehicle library is excluded on purpose:
#   * liveried / multi-material: police chargers, carlacola, VW t2 (two-tone),
#     ambulance / firetruck / sprinter / cybertruck (no `color` attribute);
#   * two-wheelers (bikes, motorbikes): tiny bodies with riders that never
#     read as a clean colour from above.
# This is the fix for background vehicles showing up non-red (or green/blue).
_RED_CAPABLE_CARS = [
    'vehicle.audi.a2',
    'vehicle.audi.etron',
    'vehicle.audi.tt',
    'vehicle.bmw.grandtourer',
    'vehicle.chevrolet.impala',
    'vehicle.citroen.c3',
    'vehicle.dodge.charger_2020',
    'vehicle.ford.mustang',
    'vehicle.jeep.wrangler_rubicon',
    'vehicle.lincoln.mkz_2017',
    'vehicle.lincoln.mkz_2020',
    'vehicle.mercedes.coupe',
    'vehicle.mercedes.coupe_2020',
    'vehicle.mini.cooper_s',
    'vehicle.mini.cooper_s_2021',
    'vehicle.nissan.micra',
    'vehicle.nissan.patrol',
    'vehicle.nissan.patrol_2021',
    'vehicle.seat.leon',
    'vehicle.tesla.model3',
    'vehicle.toyota.prius',
]


def _red_background_blueprints(world):
    """Blueprint pool for background traffic: only reliably red-paintable cars.

    Intersects the curated allowlist with what this CARLA build actually
    provides (and that still exposes a modifiable `color`), so it stays safe
    across versions. Falls back to any recolorable vehicle if the allowlist
    somehow matches nothing.
    """
    lib = world.get_blueprint_library()
    blueprints = []
    for vid in _RED_CAPABLE_CARS:
        found = lib.filter(vid)
        for bp in found:
            if bp.has_attribute('color') and \
                    bp.get_attribute('color').is_modifiable:
                blueprints.append(bp)
    if not blueprints:                       # defensive fallback
        blueprints = [bp for bp in lib.filter('vehicle.*')
                      if bp.has_attribute('color')
                      and bp.get_attribute('color').is_modifiable]
    return blueprints


def spawn_background_traffic(world, client, tm_port, number, exclude_indices):
    """Spawn `number` autopilot vehicles (all forced red) at free spawn points."""
    tm = client.get_trafficmanager(tm_port)
    tm.set_synchronous_mode(True)

    blueprints = _red_background_blueprints(world)
    print('Background pool: %d red-paintable car models.' % len(blueprints))
    spawn_points = world.get_map().get_spawn_points()
    candidate_indices = [i for i in range(len(spawn_points))
                         if i not in exclude_indices]
    random.shuffle(candidate_indices)

    bg_list = []
    for idx in candidate_indices:
        if len(bg_list) >= number:
            break
        bp = random.choice(blueprints)
        # Force every background vehicle red (CAVs stay green).
        bp.set_attribute('color', '255, 0, 0')
        bp.set_attribute('role_name', 'autopilot')
        vehicle = world.try_spawn_actor(bp, spawn_points[idx])
        if vehicle is not None:
            vehicle.set_autopilot(True, tm_port)
            # Never run red lights (0% ignore = always stop at reds).
            tm.ignore_lights_percentage(vehicle, 0.0)
            bg_list.append(vehicle)

    print('Spawned %d background vehicles.' % len(bg_list))
    return tm, bg_list


# CARLA's bicycle blueprints (two-wheelers with a rider).
_BIKE_BLUEPRINTS = [
    'vehicle.bh.crossbike',
    'vehicle.diamondback.century',
    'vehicle.gazelle.omafiets',
]


def spawn_bikes(world, client, tm_port, bike_cfg, exclude_indices):
    """Spawn `bike_cfg.num` bicycles on autopilot at free road spawn points.

    Spawn points are taken, in order of preference, from
    ``bike_cfg.spawn_point_indices``, then free points within
    ``spawn_near_radius`` meters of ``spawn_near``, then any free point.
    Every index actually used is added to `exclude_indices`.
    """
    number = int(bike_cfg.get('num', 0) or 0)
    if number <= 0:
        return []

    tm = client.get_trafficmanager(tm_port)
    tm.set_synchronous_mode(True)

    lib = world.get_blueprint_library()
    blueprints = []
    for bike_id in _BIKE_BLUEPRINTS:
        blueprints.extend(lib.filter(bike_id))
    if not blueprints:
        print('WARNING: no bicycle blueprints in this CARLA build, '
              'skipping bikes.')
        return []

    spawn_points = world.get_map().get_spawn_points()
    n_sp = len(spawn_points)

    wanted = [int(i) % n_sp
              for i in (bike_cfg.get('spawn_point_indices', None) or [])]
    candidates = [i for i in wanted if i not in exclude_indices]

    near = bike_cfg.get('spawn_near', None)
    near_radius = float(bike_cfg.get('spawn_near_radius', 0) or 0)
    if near is not None and near_radius > 0:
        near_loc = carla.Location(x=float(near[0]), y=float(near[1]), z=0.0)
        ring = [i for i in range(n_sp)
                if i not in exclude_indices and i not in candidates
                and spawn_points[i].location.distance(near_loc) <= near_radius]
        random.shuffle(ring)
        candidates += ring

    rest = [i for i in range(n_sp)
            if i not in exclude_indices and i not in candidates]
    random.shuffle(rest)
    candidates += rest

    bikes = []
    for idx in candidates:
        if len(bikes) >= number:
            break
        bp = random.choice(blueprints)
        bp.set_attribute('role_name', 'bike')
        bike = world.try_spawn_actor(bp, spawn_points[idx])
        if bike is not None:
            bike.set_autopilot(True, tm_port)
            tm.ignore_lights_percentage(bike, 0.0)
            exclude_indices.add(idx)
            bikes.append(bike)

    print('Spawned %d bike(s) (%d requested).' % (len(bikes), number))
    return bikes


def spawn_pedestrians(world, client, ped_cfg, seed=None):
    """Spawn `ped_cfg.num` AI-driven pedestrians on the sidewalk nav mesh.

    Each walker gets a `controller.ai.walker` steering it toward a random
    navigation target at `ped_cfg.speed` m/s. When `spawn_center` /
    `spawn_radius` are set, spawn locations are restricted to that circle.

    Returns (walkers, controllers); the controllers must be stop()ped before
    the walkers are destroyed.
    """
    number = int(ped_cfg.get('num', 0) or 0)
    if number <= 0:
        return [], []

    # must be set before spawning for the crossing behaviour to apply
    world.set_pedestrians_cross_factor(
        float(ped_cfg.get('cross_factor', 0.1)))
    if seed is not None and hasattr(world, 'set_pedestrians_seed'):
        world.set_pedestrians_seed(int(seed))   # only exists in CARLA >= 0.9.13

    center = ped_cfg.get('spawn_center', None)
    radius = float(ped_cfg.get('spawn_radius', 0) or 0)
    speed_cfg = float(ped_cfg.get('speed', 0) or 0)

    lib = world.get_blueprint_library()
    walker_bps = lib.filter('walker.pedestrian.*')

    spawn_transforms = []
    tries = 0
    while len(spawn_transforms) < number and tries < number * 60:
        tries += 1
        loc = world.get_random_location_from_navigation()
        if loc is None:
            continue
        if center is not None and radius > 0 and \
                math.hypot(loc.x - float(center[0]),
                           loc.y - float(center[1])) > radius:
            continue
        spawn_transforms.append(carla.Transform(loc))

    batch = []
    speeds = []
    for transform in spawn_transforms:
        bp = random.choice(walker_bps)
        if bp.has_attribute('is_invincible'):
            bp.set_attribute('is_invincible', 'false')
        if speed_cfg > 0:
            speeds.append(speed_cfg)
        elif bp.has_attribute('speed'):
            # recommended_values = [walk, run]
            speeds.append(float(bp.get_attribute('speed').recommended_values[1]))
        else:
            speeds.append(1.4)
        batch.append(carla.command.SpawnActor(bp, transform))

    results = client.apply_batch_sync(batch, True)
    walker_ids, walker_speeds = [], []
    for res, spd in zip(results, speeds):
        if not res.error:
            walker_ids.append(res.actor_id)
            walker_speeds.append(spd)

    controller_bp = lib.find('controller.ai.walker')
    batch = [carla.command.SpawnActor(controller_bp, carla.Transform(), wid)
             for wid in walker_ids]
    results = client.apply_batch_sync(batch, True)
    controller_ids = [res.actor_id for res in results if not res.error]

    world.tick()   # let the server register the new actors before start()

    walkers = list(world.get_actors(walker_ids))
    controllers = list(world.get_actors(controller_ids))
    for controller, speed in zip(controllers, walker_speeds):
        controller.start()
        target = world.get_random_location_from_navigation()
        if target is not None:
            controller.go_to_location(target)
        controller.set_max_speed(speed)

    print('Spawned %d pedestrian(s) (%d requested).' % (len(walkers), number))
    return walkers, controllers
