# -*- coding: utf-8 -*-
"""Deterministic traffic manifests for CA comparison campaigns.

CARLA 0.9.12 cannot seed navigation-mesh sampling for pedestrians.  A
campaign therefore samples the traffic layout once, records every actor and
walker destination, and strictly replays that manifest in subsequent runs.
"""

import json
import math
import os
import random

import carla

from dataset_generator.generate_dataset import (
    _BIKE_BLUEPRINTS, _red_background_blueprints)


MANIFEST_VERSION = 1


def _location_dict(location):
    return {"x": float(location.x), "y": float(location.y),
            "z": float(location.z)}


def _transform_dict(transform):
    return {
        "location": _location_dict(transform.location),
        "rotation": {"pitch": float(transform.rotation.pitch),
                     "yaw": float(transform.rotation.yaw),
                     "roll": float(transform.rotation.roll)},
    }


def _location(data):
    return carla.Location(x=float(data["x"]), y=float(data["y"]),
                          z=float(data.get("z", 0.0)))


def _transform(data):
    rotation = data.get("rotation", {})
    return carla.Transform(
        _location(data["location"]),
        carla.Rotation(pitch=float(rotation.get("pitch", 0.0)),
                       yaw=float(rotation.get("yaw", 0.0)),
                       roll=float(rotation.get("roll", 0.0))))


def _destroy_created(controllers, walkers, vehicles, tm_port):
    for controller in controllers:
        try:
            controller.stop()
            controller.destroy()
        except Exception:
            pass
    for walker in walkers:
        try:
            walker.destroy()
        except Exception:
            pass
    for vehicle in vehicles:
        try:
            vehicle.set_autopilot(False, tm_port)
            vehicle.destroy()
        except Exception:
            pass


def _write_json_atomic(path, value):
    parent = os.path.dirname(os.path.abspath(path))
    os.makedirs(parent, exist_ok=True)
    temporary = path + ".tmp"
    with open(temporary, "w") as handle:
        json.dump(value, handle, indent=2, sort_keys=True)
        handle.write("\n")
    os.replace(temporary, path)


def _sample_near_navigation(world, number, center, radius):
    transforms = []
    tries = 0
    while len(transforms) < number and tries < number * 60:
        tries += 1
        location = world.get_random_location_from_navigation()
        if location is None:
            continue
        if center is not None and radius > 0 and math.hypot(
                location.x - float(center[0]),
                location.y - float(center[1])) > radius:
            continue
        transforms.append(carla.Transform(location))
    if len(transforms) != number:
        raise RuntimeError("could only sample %d/%d pedestrian spawn points"
                           % (len(transforms), number))
    return transforms


def _create_manifest(world, seed, town, n_background, bike_cfg, ped_cfg,
                     exclude_indices):
    """Create the actor plan without depending on global Python RNG state."""
    rng = random.Random(int(seed))
    spawn_points = world.get_map().get_spawn_points()
    n_sp = len(spawn_points)
    used = set(int(i) for i in exclude_indices)
    manifest = {"version": MANIFEST_VERSION, "town": str(town),
                "seed": int(seed), "background": [], "bikes": [],
                "pedestrians": []}

    lib = world.get_blueprint_library()
    bike_blueprints = []
    for blueprint_id in _BIKE_BLUEPRINTS:
        bike_blueprints.extend(lib.filter(blueprint_id))
    bike_count = int(bike_cfg.get("num", 0) or 0)
    wanted = [int(i) % n_sp for i in
              (bike_cfg.get("spawn_point_indices", None) or [])]
    candidates = [i for i in wanted if i not in used]
    near = bike_cfg.get("spawn_near", None)
    radius = float(bike_cfg.get("spawn_near_radius", 0) or 0)
    if near is not None and radius > 0:
        near_location = carla.Location(x=float(near[0]), y=float(near[1]))
        ring = [i for i in range(n_sp) if i not in used and i not in candidates
                and spawn_points[i].location.distance(near_location) <= radius]
        rng.shuffle(ring)
        candidates += ring
    rest = [i for i in range(n_sp) if i not in used and i not in candidates]
    rng.shuffle(rest)
    candidates += rest
    if bike_count and not bike_blueprints:
        raise RuntimeError("no bicycle blueprints are available")
    for index in candidates[:bike_count]:
        blueprint = rng.choice(bike_blueprints)
        manifest["bikes"].append(
            {"blueprint": blueprint.id, "spawn_point_index": int(index)})
        used.add(index)
    if len(manifest["bikes"]) != bike_count:
        raise RuntimeError("could only plan %d/%d bikes"
                           % (len(manifest["bikes"]), bike_count))

    background_blueprints = _red_background_blueprints(world)
    candidates = [i for i in range(n_sp) if i not in used]
    rng.shuffle(candidates)
    for index in candidates[:int(n_background)]:
        blueprint = rng.choice(background_blueprints)
        manifest["background"].append(
            {"blueprint": blueprint.id, "spawn_point_index": int(index)})
        used.add(index)
    if len(manifest["background"]) != int(n_background):
        raise RuntimeError("could only plan %d/%d background vehicles"
                           % (len(manifest["background"]), n_background))

    ped_count = int(ped_cfg.get("num", 0) or 0)
    spawn_transforms = _sample_near_navigation(
        world, ped_count, ped_cfg.get("spawn_center", None),
        float(ped_cfg.get("spawn_radius", 0) or 0))
    walker_blueprints = list(lib.filter("walker.pedestrian.*"))
    speed_cfg = float(ped_cfg.get("speed", 0) or 0)
    for transform in spawn_transforms:
        blueprint = rng.choice(walker_blueprints)
        if speed_cfg > 0:
            speed = speed_cfg
        elif blueprint.has_attribute("speed"):
            speed = float(blueprint.get_attribute(
                "speed").recommended_values[1])
        else:
            speed = 1.4
        target = world.get_random_location_from_navigation()
        if target is None:
            raise RuntimeError("CARLA returned no pedestrian destination")
        manifest["pedestrians"].append(
            {"blueprint": blueprint.id,
             "spawn": _transform_dict(transform),
             "target": _location_dict(target), "speed": float(speed)})
    return manifest


def load_traffic_manifest(path):
    with open(path) as handle:
        manifest = json.load(handle)
    if int(manifest.get("version", -1)) != MANIFEST_VERSION:
        raise RuntimeError("unsupported traffic manifest version in %s" % path)
    return manifest


def spawn_manifest_traffic(world, client, tm, tm_port, n_background,
                           bike_cfg, ped_cfg, exclude_indices, manifest_path,
                           create_manifest, seed, town):
    """Create/replay a manifest and return actors plus the manifest object.

    Returns ``(background, bikes, walkers, controllers, manifest)``. Replay is
    deliberately strict: any failed spawn invalidates the comparison run.
    """
    if create_manifest:
        if os.path.exists(manifest_path):
            raise RuntimeError("refusing to overwrite traffic manifest %s"
                               % manifest_path)
        manifest = _create_manifest(world, seed, town, n_background, bike_cfg,
                                    ped_cfg, exclude_indices)
    else:
        if not os.path.isfile(manifest_path):
            raise RuntimeError("traffic manifest does not exist: %s"
                               % manifest_path)
        manifest = load_traffic_manifest(manifest_path)
    if str(manifest.get("town")) != str(town):
        raise RuntimeError("traffic manifest town %r does not match %r"
                           % (manifest.get("town"), town))
    if int(manifest.get("seed")) != int(seed):
        raise RuntimeError("traffic manifest seed does not match run seed")
    if len(manifest.get("background", [])) != int(n_background):
        raise RuntimeError("traffic manifest background count mismatch")
    if len(manifest.get("bikes", [])) != int(
            bike_cfg.get("num", 0) or 0):
        raise RuntimeError("traffic manifest bike count mismatch")
    if len(manifest.get("pedestrians", [])) != int(
            ped_cfg.get("num", 0) or 0):
        raise RuntimeError("traffic manifest pedestrian count mismatch")

    spawn_points = world.get_map().get_spawn_points()
    lib = world.get_blueprint_library()
    vehicles, bikes, walkers, controllers = [], [], [], []
    try:
        for entry in manifest["bikes"]:
            blueprint = lib.find(entry["blueprint"])
            blueprint.set_attribute("role_name", "bike")
            actor = world.try_spawn_actor(
                blueprint, spawn_points[int(entry["spawn_point_index"])])
            if actor is None:
                raise RuntimeError("failed to replay bike at spawn point %s"
                                   % entry["spawn_point_index"])
            actor.set_autopilot(True, tm_port)
            tm.ignore_lights_percentage(actor, 0.0)
            bikes.append(actor)

        for entry in manifest["background"]:
            blueprint = lib.find(entry["blueprint"])
            blueprint.set_attribute("color", "255, 0, 0")
            blueprint.set_attribute("role_name", "autopilot")
            actor = world.try_spawn_actor(
                blueprint, spawn_points[int(entry["spawn_point_index"])])
            if actor is None:
                raise RuntimeError(
                    "failed to replay background vehicle at spawn point %s"
                    % entry["spawn_point_index"])
            actor.set_autopilot(True, tm_port)
            tm.ignore_lights_percentage(actor, 0.0)
            vehicles.append(actor)

        world.set_pedestrians_cross_factor(
            float(ped_cfg.get("cross_factor", 0.1)))
        for entry in manifest["pedestrians"]:
            blueprint = lib.find(entry["blueprint"])
            if blueprint.has_attribute("is_invincible"):
                blueprint.set_attribute("is_invincible", "false")
            walker = world.try_spawn_actor(blueprint, _transform(entry["spawn"]))
            if walker is None:
                raise RuntimeError("failed to replay pedestrian at %r"
                                   % entry["spawn"]["location"])
            walkers.append(walker)
        controller_blueprint = lib.find("controller.ai.walker")
        for walker in walkers:
            controllers.append(world.spawn_actor(
                controller_blueprint, carla.Transform(), attach_to=walker))
        world.tick()
        for controller, entry in zip(controllers, manifest["pedestrians"]):
            controller.start()
            controller.go_to_location(_location(entry["target"]))
            controller.set_max_speed(float(entry["speed"]))
        if create_manifest:
            _write_json_atomic(manifest_path, manifest)
        print("Replayed deterministic traffic manifest: %d vehicles, %d "
              "bikes, %d pedestrians" %
              (len(vehicles), len(bikes), len(walkers)))
        return vehicles, bikes, walkers, controllers, manifest
    except Exception:
        _destroy_created(controllers, walkers, bikes + vehicles, tm_port)
        raise
