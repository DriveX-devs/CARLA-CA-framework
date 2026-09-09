# -*- coding: utf-8 -*-
"""
CA (Cooperative Awareness) demo: 4 free-moving CAVs (Traffic Manager
autopilot) plus background traffic, bikes and pedestrians. Every CAV runs
its OWN YOLOv8 + LiDAR fusion pipeline that estimates the position, velocity
and heading of every object it sees (vehicles, bikes, pedestrians).
Estimates are validated live against CARLA ground truth and summarized at
the end of the run.

Usage:
    conda activate msvan3t_carla
    cd OpenCDA
    python CA/run_ca.py --model n            # yolov8n
    python CA/run_ca.py --model x --seconds 20

Outputs (CA/output/run_<timestamp>/):
    metrics.csv        one line per step per CAV per identified object
                       (vehicles, bikes and pedestrians; ground-truth fields
                       empty when the track matches no GT object), including
                       the matched GT actor's readable name (Cav1..4,
                       veh_1..N, bike_1..B, ped_1..P) and the per-step
                       pipeline timing split (detection / fusion / tracking)
    summary.txt        final error + timing statistics per CAV and category
    <Cav>/<camera>_<step>.png   annotated frames (bbox + fused LiDAR cluster)

See CA/PIPELINE.md for a detailed description of the whole pipeline.
"""

import argparse
import csv
import math
import os
import sys
import time
from datetime import datetime

import cv2
import numpy as np
from omegaconf import OmegaConf

sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

import opencda.scenario_testing.utils.sim_api as sim_api
from opencda.core.common.cav_world import CavWorld
from opencda.scenario_testing.utils.yaml_utils import add_current_time

# background actor spawners (bundled with this repository)
from CA.traffic_spawners import (
    spawn_background_traffic, spawn_bikes, spawn_pedestrians)

from CA.sensors import SensorRig
from CA.detector import YoloDetector
from CA.fusion import CameraLidarFusion, CATEGORIES, suppress_contained
from CA.tracker import VehicleTracker, wrap_deg
from CA.PyCA.t2c import t2c as T2CEngine, NO_COLLISION
# Reuse the single, one-to-one track<->GT association (mutual exclusion +
# category compatibility) so this standalone demo links exactly like the
# campaign runner. Importing the module has no side effects (its main() is
# guarded by __main__).
from CA.run_ca_extended import _associate_tracks_to_gt

# annotation colours per category (BGR)
CATEGORY_COLORS = {'vehicle': (0, 255, 255),      # yellow
                   'bike': (255, 255, 0),         # cyan
                   'pedestrian': (255, 0, 255)}   # magenta


def arg_parse():
    parser = argparse.ArgumentParser(
        description='4-CAV YOLOv8 + LiDAR cooperative awareness demo.')
    parser.add_argument('--config', type=str, default='CA/ca_convoy.yaml')
    parser.add_argument('--model', type=str, default=None,
                        choices=['n', 'm', 'x'],
                        help='YOLOv8 capacity (overrides the yaml).')
    parser.add_argument('--seconds', type=int, default=None,
                        help='Override scenario.duration_seconds.')
    parser.add_argument('-s', '--host', type=str, default='localhost')
    parser.add_argument('-p', '--port', type=int, default=None,
                        help='CARLA RPC port (overrides the yaml).')
    parser.add_argument('-tm', '--tm_port', type=int, default=8000)
    parser.add_argument('-v', '--version', type=str, default='0.9.12')
    return parser.parse_args()


# ---------------------------------------------------------------------------
def collect_gt(world):
    """Ground truth of every dynamic object in the scene.

    Returns a list of dicts {id, category, xy, speed, yaw}; category
    follows the same taxonomy as the detections: 'vehicle' (4+ wheels),
    'bike' (2-wheelers, rider included) and 'pedestrian' (walkers).
    """
    objs = []
    for v in world.get_actors().filter('vehicle.*'):
        wheels = int(v.attributes.get('number_of_wheels', 4))
        category = 'bike' if wheels == 2 else 'vehicle'
        t = v.get_transform()
        center = t.transform(v.bounding_box.location)
        vel = v.get_velocity()
        objs.append({'id': v.id, 'category': category,
                     'xy': np.array([center.x, center.y]),
                     'speed': math.sqrt(vel.x ** 2 + vel.y ** 2 +
                                        vel.z ** 2),
                     'yaw': t.rotation.yaw})
    for w in world.get_actors().filter('walker.pedestrian.*'):
        t = w.get_transform()
        vel = w.get_velocity()
        objs.append({'id': w.id, 'category': 'pedestrian',
                     'xy': np.array([t.location.x, t.location.y]),
                     'speed': math.sqrt(vel.x ** 2 + vel.y ** 2 +
                                        vel.z ** 2),
                     'yaw': t.rotation.yaw})
    return objs


def annotate(image, det, track, gt, ca=None):
    """Draw one fused detection + its track state on a camera frame.

    `ca` is the (t2c, s2c, warning, type) tuple from the PyCA collision
    check: t2c/s2c are printed with the estimates, and a firing warning
    turns the whole box red.
    """
    warning = ca is not None and ca[2]
    color = (0, 0, 255) if warning \
        else CATEGORY_COLORS.get(det.category, (0, 255, 255))
    x1, y1, x2, y2 = [int(v) for v in det.bbox]
    cv2.rectangle(image, (x1, y1), (x2, y2), color, 3 if warning else 2)
    for u, v in det.uv_points.astype(int):
        cv2.circle(image, (u, v), 2, (0, 255, 0), -1)

    lines = ['%s %.2f  (%d lidar pts)' % (det.cls_name, det.conf,
                                          det.n_points)]
    if warning:
        lines.insert(0, 'CA WARNING (%s)' % ca[3])
    if track is not None:
        lines.append('pos  (%.1f, %.1f)' % (track.position[0],
                                            track.position[1]))
        lines.append('spd  %.1f m/s' % track.speed)
        if track.heading is not None:
            lines.append('hdg  %.1f deg' % track.heading)
        if ca is not None and ca[0] >= 0:
            lines.append('t2c  %.1f s   s2c %.1f m' % (ca[0], ca[1]))
        if gt is not None:
            err = np.linalg.norm(track.position[:2] - gt['xy'])
            lines.append('GT   spd %.1f  hdg %.1f  |dpos| %.2fm'
                         % (gt['speed'], gt['yaw'], err))
    y_text = max(20, y1 - 8 - 18 * len(lines))
    for line in lines:
        cv2.putText(image, line, (x1, y_text), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(image, line, (x1, y_text), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, color, 1, cv2.LINE_AA)
        y_text += 18


# ---------------------------------------------------------------------------
def main():
    opt = arg_parse()

    opencda_root = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    cfg_path = opt.config if os.path.isabs(opt.config) \
        else os.path.join(opencda_root, opt.config)
    cfg = OmegaConf.load(cfg_path)
    cfg = add_current_time(cfg)

    if opt.port is not None:
        cfg['world']['client_port'] = opt.port
    capacity = opt.model or str(cfg['detection']['model_capacity'])

    sc = cfg['scenario']
    duration = opt.seconds if opt.seconds is not None \
        else int(sc['duration_seconds'])
    dt = float(cfg['world']['fixed_delta_seconds'])
    total_steps = int(round(duration / dt))

    out_root = cfg['output']['root']
    out_root = out_root if os.path.isabs(out_root) \
        else os.path.join(opencda_root, out_root)
    run_dir = os.path.join(out_root,
                           'run_%s' % datetime.now().strftime('%Y%m%d_%H%M%S'))
    os.makedirs(run_dir, exist_ok=True)
    save_stride = int(cfg['output']['save_stride'])
    print_stride = int(cfg['output']['print_stride'])
    eval_dist = {c: float(cfg['output']['eval_match_dist'][c])
                 for c in CATEGORIES}

    bike_cfg = sc.get('bikes', None) or {}
    ped_cfg = sc.get('pedestrians', None) or {}
    n_background = int(sc.get('background_traffic_num', 0) or 0)

    ca_cfg = cfg.get('collision', None) or {}
    ca_t2c_th = float(ca_cfg.get('t2c_th', 4.8))
    ca_s2c_th = float(ca_cfg.get('s2c_th', 4.2))
    ca_alpha_th = float(ca_cfg.get('alpha_th_deg', 17.0))
    if bool(ca_cfg.get('check_rear_end', False)):
        raise ValueError(
            'Rear-end collision checks are disabled for CA experiments; '
            'set collision.check_rear_end to false')

    print('=== CA: YOLOv8-%s + LiDAR fusion, %d CAVs ==='
          % (capacity, len(sc['cavs'])))
    print('  collisions : cross-road only (rear-end disabled)')
    print('  town        : %s' % sc['town'])
    print('  duration    : %ds (%d steps @ %.2fs)'
          % (duration, total_steps, dt))
    print('  background  : %d vehicles, %d bikes, %d pedestrians'
          % (n_background, int(bike_cfg.get('num', 0) or 0),
             int(ped_cfg.get('num', 0) or 0)))
    print('  output      : %s' % run_dir)

    weights_dir = os.path.join(opencda_root,
                               str(cfg['detection']['weights_dir']))

    scenario_manager = None
    cavs = []
    bg_list, bike_list = [], []
    walker_list, walker_controllers = [], []
    try:
        # ---- world -------------------------------------------------------
        scenario_manager = sim_api.ScenarioManager(
            cfg, apply_ml=False, carla_version=opt.version,
            town=str(sc['town']), cav_world=CavWorld(apply_ml=False),
            carla_host=opt.host, carla_port=int(cfg['world']['client_port']))
        world = scenario_manager.world
        client = scenario_manager.client
        spawn_points = world.get_map().get_spawn_points()

        tm = client.get_trafficmanager(opt.tm_port)
        tm.set_synchronous_mode(True)

        # ---- CAVs (independent, free to move on TM autopilot) --------------
        bp_lib = world.get_blueprint_library()
        used_indices = set()
        for cav_def in sc['cavs']:
            bp = bp_lib.find('vehicle.lincoln.mkz_2017')
            bp.set_attribute('color', str(cav_def['color']))
            bp.set_attribute('role_name', str(cav_def['name']))
            sp_idx = int(cav_def['spawn_point_index'])
            used_indices.add(sp_idx)
            sp = spawn_points[sp_idx]

            vehicle = world.spawn_actor(bp, sp)
            world.tick()

            # one YOLO model per CAV (only the first announces itself)
            detector = YoloDetector(
                capacity, weights_dir,
                cfg['detection']['confidence'],
                list(cfg['detection']['classes']),
                quiet=bool(cavs))
            rig = SensorRig(world, vehicle, cfg['sensing'])
            fusion = CameraLidarFusion(cfg['fusion'])
            tracker = VehicleTracker(dt, cfg['tracking'])

            vehicle.set_autopilot(True, opt.tm_port)
            tm.ignore_lights_percentage(
                vehicle, 100.0 if sc['ignore_traffic_lights'] else 0.0)
            tm.auto_lane_change(vehicle, False)
            speed_perc = float(sc.get('target_speed_perc', 0) or 0)
            if speed_perc:
                tm.vehicle_percentage_speed_difference(vehicle, speed_perc)

            cavs.append({'name': str(cav_def['name']), 'vehicle': vehicle,
                         'detector': detector, 'rig': rig, 'fusion': fusion,
                         'tracker': tracker,
                         'ca_engine': T2CEngine(node_id=str(cav_def['name']),
                                                T2C_th=ca_t2c_th),
                         'errors': {c: [] for c in CATEGORIES},
                         'timings': {op: [] for op in
                                     ('detect', 'fuse', 'track', 'ca')},
                         'ca_warnings': 0, 'ca_min_t2c': float('inf'),
                         'unmatched': 0})
            print('  spawned %s at spawn point %d (%.1f, %.1f, yaw %.0f)'
                  % (cav_def['name'], sp_idx, sp.location.x, sp.location.y,
                     sp.rotation.yaw))
            os.makedirs(os.path.join(run_dir, str(cav_def['name'])),
                        exist_ok=True)

        # ---- background actors (CA/traffic_spawners.py) --------------------
        bike_list = spawn_bikes(world, client, opt.tm_port, bike_cfg,
                                used_indices)
        _, bg_list = spawn_background_traffic(
            world, client, opt.tm_port, n_background,
            exclude_indices=used_indices)
        walker_list, walker_controllers = spawn_pedestrians(
            world, client, ped_cfg,
            seed=cfg['world'].get('seed', None))

        # readable name of every spawned actor (used in the gt_name /
        # name CSV columns): Cav1..4, veh_1..N, bike_1..B, ped_1..P
        actor_names = {}
        for cav in cavs:
            actor_names[cav['vehicle'].id] = cav['name']
        for i, v in enumerate(bg_list, 1):
            actor_names[v.id] = 'veh_%d' % i
        for i, b in enumerate(bike_list, 1):
            actor_names[b.id] = 'bike_%d' % i
        for i, w in enumerate(walker_list, 1):
            actor_names[w.id] = 'ped_%d' % i

        # ---- warmup --------------------------------------------------------
        for _ in range(int(sc['warmup_steps'])):
            world.tick()
        # drain stale sensor frames so the loop starts aligned
        frame = world.tick()
        for cav in cavs:
            cav['rig'].fetch(frame)

        # ---- metrics csv ---------------------------------------------------
        # one line per step per CAV per identified (confirmed) object;
        # ground-truth columns are empty when no GT object matches. The
        # t_*_ms columns repeat, on every row of the same step+CAV, the
        # wall-clock time that CAV's pipeline spent on that step, split
        # per operation (YOLO detection / camera-LiDAR fusion / tracking).
        csv_file = open(os.path.join(run_dir, 'metrics.csv'), 'w', newline='')
        writer = csv.writer(csv_file)
        writer.writerow(['step', 'cav', 'track_id', 'category',
                         'est_x', 'est_y', 'est_speed', 'est_heading',
                         'gt_id', 'gt_name', 'gt_category', 'gt_x', 'gt_y',
                         'gt_speed', 'gt_heading',
                         'err_pos', 'err_speed', 'err_heading',
                         't2c', 's2c', 'ca_warning', 'ca_type',
                         't_detect_ms', 't_fuse_ms', 't_track_ms',
                         't_ca_ms', 't_total_ms'])

        # per-step ground truth of every dynamic object (offline analysis)
        gt_file = open(os.path.join(run_dir, 'gt.csv'), 'w', newline='')
        gt_writer = csv.writer(gt_file)
        gt_writer.writerow(['step', 'id', 'name', 'category', 'x', 'y',
                            'speed', 'heading'])

        # ---- main loop -----------------------------------------------------
        print('Running %d steps...' % total_steps)
        start = time.time()
        for step in range(total_steps):
            frame = world.tick()

            gt_all = collect_gt(world)
            for g in gt_all:
                gt_writer.writerow([step, g['id'],
                                    actor_names.get(g['id'], ''),
                                    g['category'],
                                    '%.2f' % g['xy'][0], '%.2f' % g['xy'][1],
                                    '%.2f' % g['speed'], '%.1f' % g['yaw']])

            for cav in cavs:
                rig, fusion = cav['rig'], cav['fusion']
                images, points = rig.fetch(frame)

                t0 = time.perf_counter()
                cam_dets = cav['detector'].detect_batch(images)
                t1 = time.perf_counter()

                world_pts = fusion.lidar_to_world(points, rig.lidar.sensor)
                fused = []
                for i, cam in enumerate(rig.cameras):
                    if not cam_dets[i]:
                        continue
                    dets_2d = suppress_contained(cam_dets[i])
                    uv, depth, wxyz = fusion.project_to_camera(world_pts,
                                                               cam.sensor)
                    fused += fusion.fuse_camera(dets_2d, uv, depth, wxyz,
                                                i, cam.sensor)
                merged = fusion.merge_detections(fused)
                t2 = time.perf_counter()

                tracks = cav['tracker'].step(merged)
                t3 = time.perf_counter()

                # ---- collision avoidance (PyCA t2c/s2c) -------------------
                # ego state from the simulator (a vehicle knows its own
                # kinematics); tracked objects use the fused position and
                # the Kalman velocity, constant velocity assumed (ax=ay=0).
                # Constant-velocity model for ego and objects alike (ax=ay=0),
                # identical to run_ca_extended.py's GT/on-board/edge checks.
                ego = cav['vehicle']
                ego_tf = ego.get_transform()
                ego_c = ego_tf.transform(ego.bounding_box.location)
                ego_v = ego.get_velocity()
                engine = cav['ca_engine']
                ca_results = {}
                for track in tracks:
                    px, py = float(track.x[0]), float(track.x[1])
                    tvx, tvy = float(track.x[2]), float(track.x[3])
                    raw_t2c = engine.TimeToCollision(
                        ego_c.x, ego_c.y, ego_v.x, ego_v.y, 0.0, 0.0,
                        px, py, tvx, tvy, 0.0, 0.0)
                    raw_s2c = engine.computes2c(
                        ego_c.x, ego_c.y, ego_v.x, ego_v.y, 0.0, 0.0,
                        px, py, tvx, tvy, 0.0, 0.0, raw_t2c) \
                        if raw_t2c >= 0 else NO_COLLISION
                    # Rear-end pairs retain their diagnostic label and raw
                    # t2c/s2c values, but never enter the warning policy.
                    # (CollisionAvoidanceService.classify); the track's motion
                    # direction stands in for its heading
                    trk_heading = track.heading if track.heading is not None \
                        else math.degrees(math.atan2(tvy, tvx))
                    ca_type = 'rear-end' if abs(wrap_deg(
                        ego_tf.rotation.yaw - trk_heading)) < ca_alpha_th \
                        else 'cross-road'
                    if ca_type == 'cross-road':
                        warn_t2c, _ = engine.CheckCollision(
                            ego_c.x, ego_c.y, ego_v.x, ego_v.y,
                            0.0, 0.0,
                            px, py, tvx, tvy, 0.0, 0.0,
                            s2c_threshold=ca_s2c_th)
                    else:
                        warn_t2c = NO_COLLISION
                    warning = warn_t2c != NO_COLLISION
                    if warning:
                        cav['ca_warnings'] += 1
                        cav['ca_min_t2c'] = min(cav['ca_min_t2c'], raw_t2c)
                    ca_results[track.track_id] = (raw_t2c, raw_s2c,
                                                  warning, ca_type)
                t4 = time.perf_counter()

                t_detect = (t1 - t0) * 1000.0
                t_fuse = (t2 - t1) * 1000.0
                t_track = (t3 - t2) * 1000.0
                t_ca = (t4 - t3) * 1000.0
                cav['timings']['detect'].append(t_detect)
                cav['timings']['fuse'].append(t_fuse)
                cav['timings']['track'].append(t_track)
                cav['timings']['ca'].append(t_ca)
                timing_cols = ['%.1f' % t_detect, '%.2f' % t_fuse,
                               '%.2f' % t_track, '%.2f' % t_ca,
                               '%.1f' % (t_detect + t_fuse + t_track + t_ca)]

                # ---- evaluate against ground truth ------------------------
                ego_id = cav['vehicle'].id
                gt_states = [g for g in gt_all if g['id'] != ego_id]
                det2track = {id(t.last_detection): t for t in tracks}
                # One-to-one track<->GT association for this CAV/step: each GT
                # actor is claimed by at most one track and a vehicle track is
                # never bound to a lone pedestrian (see _associate_tracks_to_gt).
                track2gt = _associate_tracks_to_gt(tracks, gt_states, eval_dist)

                for track in tracks:
                    raw_t2c, raw_s2c, ca_warn, ca_type = \
                        ca_results[track.track_id]
                    ca_cols = [('%.2f' % raw_t2c) if raw_t2c >= 0 else '-1',
                               ('%.2f' % raw_s2c) if raw_s2c >= 0 else '',
                               '1' if ca_warn else '0', ca_type]
                    # GT actor this track was associated to (or None); the
                    # one-to-one assignment was computed once for the step above.
                    best_gt = track2gt[track.track_id]

                    est_heading = '%.1f' % track.heading \
                        if track.heading is not None else ''
                    if best_gt is None:
                        cav['unmatched'] += 1
                        writer.writerow(
                            [step, cav['name'], track.track_id,
                             track.category,
                             '%.2f' % track.position[0],
                             '%.2f' % track.position[1],
                             '%.2f' % track.speed, est_heading,
                             '', '', '', '', '', '', '', '', '', ''] +
                            ca_cols + timing_cols)
                        continue

                    err_pos = float(np.linalg.norm(track.position[:2] -
                                                   best_gt['xy']))
                    err_speed = track.speed - best_gt['speed']
                    # heading is only comparable when the track's velocity
                    # state is mature AND the GT object actually moves (the
                    # motion direction of a stationary object is undefined)
                    err_head = wrap_deg(track.heading - best_gt['yaw']) \
                        if track.heading_valid and best_gt['speed'] >= 1.0 \
                        else float('nan')
                    cav['errors'][track.category].append(
                        (err_pos, err_speed, err_head))
                    writer.writerow(
                        [step, cav['name'], track.track_id, track.category,
                         '%.2f' % track.position[0],
                         '%.2f' % track.position[1],
                         '%.2f' % track.speed, est_heading,
                         best_gt['id'],
                         actor_names.get(best_gt['id'], ''),
                         best_gt['category'],
                         '%.2f' % best_gt['xy'][0],
                         '%.2f' % best_gt['xy'][1],
                         '%.2f' % best_gt['speed'],
                         '%.1f' % best_gt['yaw'],
                         '%.2f' % err_pos, '%.2f' % err_speed,
                         '%.1f' % err_head if not math.isnan(err_head)
                         else ''] + ca_cols + timing_cols)

                # ---- console status ----------------------------------------
                if step % print_stride == 0 and tracks:
                    counts = {c: 0 for c in CATEGORIES}
                    errs = []
                    for track in tracks:
                        counts[track.category] += 1
                        gt = track2gt.get(track.track_id)
                        if gt is not None:
                            errs.append(np.linalg.norm(
                                track.position[:2] - gt['xy']))
                    n_matched = len(errs)
                    print('  [%4d] %s: %d veh, %d bike, %d ped | '
                          'matched %d/%d | mean pos err %s'
                          % (step, cav['name'], counts['vehicle'],
                             counts['bike'], counts['pedestrian'],
                             n_matched, len(tracks),
                             '%.2fm' % np.mean(errs) if errs else 'n/a'))

                # ---- save annotated frames --------------------------------
                if step % save_stride == 0:
                    by_cam = {}
                    for det in merged:
                        by_cam.setdefault(det.camera_idx, []).append(det)
                    for cam_idx, dets in by_cam.items():
                        img = images[cam_idx].copy()
                        for det in dets:
                            track = det2track.get(id(det))
                            gt = track2gt.get(track.track_id) \
                                if track is not None else None
                            ca = ca_results.get(track.track_id) \
                                if track is not None else None
                            annotate(img, det, track, gt, ca)
                        cam_name = rig.cameras[cam_idx].name
                        cv2.imwrite(os.path.join(
                            run_dir, cav['name'],
                            '%s_%05d.png' % (cam_name, step)), img)

        elapsed = time.time() - start
        csv_file.close()
        gt_file.close()
        print('Done: %d steps in %.1fs wall clock (%.1f steps/s).'
              % (total_steps, elapsed, total_steps / elapsed))

        # ---- summary -------------------------------------------------------
        lines = ['CA run summary  (YOLOv8-%s per CAV, %d CAVs, %d steps '
                 '@ %.2fs)' % (capacity, len(cavs), total_steps, dt)]
        for cav in cavs:
            total = sum(len(v) for v in cav['errors'].values())
            lines.append('')
            lines.append('%s: %d matched object detections '
                         '(+%d unmatched track steps, mostly static '
                         'scenery vehicles with no GT actor)'
                         % (cav['name'], total, cav['unmatched']))
            t = {op: np.array(v) for op, v in cav['timings'].items()}
            lines.append('  timing/step : detect %.1f ms | fusion %.1f ms '
                         '| tracking %.2f ms | collision %.2f ms | '
                         'total %.1f ms (mean)'
                         % (t['detect'].mean(), t['fuse'].mean(),
                            t['track'].mean(), t['ca'].mean(),
                            t['detect'].mean() + t['fuse'].mean() +
                            t['track'].mean() + t['ca'].mean()))
            if cav['ca_warnings']:
                lines.append('  CA          : %d warning rows (min t2c '
                             '%.2f s)' % (cav['ca_warnings'],
                                          cav['ca_min_t2c']))
            else:
                lines.append('  CA          : no collision warnings')
            for cat in CATEGORIES:
                errs = cav['errors'][cat]
                if not errs:
                    lines.append('  %-10s: none detected' % cat)
                    continue
                pos = np.array([e[0] for e in errs])
                spd = np.array([e[1] for e in errs])
                hdg = np.array([e[2] for e in errs
                                if not math.isnan(e[2])])
                msg = ('  %-10s: %5d dets | pos mean %.2fm RMSE %.2fm | '
                       'speed RMSE %.2fm/s'
                       % (cat, len(errs), pos.mean(),
                          np.sqrt((pos ** 2).mean()),
                          np.sqrt((spd ** 2).mean())))
                if len(hdg):
                    msg += ' | heading RMSE %.1fdeg' % \
                           np.sqrt((hdg ** 2).mean())
                lines.append(msg)
        summary = '\n'.join(lines)
        print('\n' + summary)
        with open(os.path.join(run_dir, 'summary.txt'), 'w') as f:
            f.write(summary + '\n')

    finally:
        print('Cleaning up...')
        for controller in walker_controllers:
            try:
                controller.stop()
                controller.destroy()
            except Exception:
                pass
        for walker in walker_list:
            try:
                walker.destroy()
            except Exception:
                pass
        for v in bike_list + bg_list:
            try:
                v.set_autopilot(False, opt.tm_port)
                v.destroy()
            except Exception:
                pass
        for cav in cavs:
            try:
                cav['rig'].destroy()
            except Exception:
                pass
            try:
                cav['vehicle'].set_autopilot(False, opt.tm_port)
                cav['vehicle'].destroy()
            except Exception:
                pass
        if scenario_manager is not None:
            scenario_manager.close()
        print('Simulation finished.')


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print(' - Exited by user.')
    except Exception:
        import traceback
        traceback.print_exc()
    finally:
        # CARLA's Python binding can segfault destroying sensors at
        # interpreter shutdown; exit hard once everything is written.
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(0)
