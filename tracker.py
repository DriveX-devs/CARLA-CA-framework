# -*- coding: utf-8 -*-
"""
Multi-object tracker for the CA pipeline.

Each fused detection (world-frame position) feeds a per-object
constant-velocity Kalman filter. Velocity comes straight from the filter
state; heading is the direction of the velocity vector (CARLA yaw
convention: atan2(vy, vx), degrees). Below `min_speed_for_heading` the last
reliable heading is held, since the velocity direction of a near-stationary
object is dominated by noise.
"""

import math

import numpy as np


def wrap_deg(angle):
    """Wrap an angle difference to (-180, 180]."""
    return (angle + 180.0) % 360.0 - 180.0


class Track(object):
    _next_id = 1

    def __init__(self, detection, dt, accel_noise, meas_noise):
        self.track_id = Track._next_id
        Track._next_id += 1

        self.dt = dt
        # state: [x, y, vx, vy]
        self.x = np.array([detection.position[0], detection.position[1],
                           0.0, 0.0])
        self.P = np.diag([meas_noise ** 2, meas_noise ** 2, 25.0, 25.0])

        self.F = np.eye(4)
        self.F[0, 2] = self.F[1, 3] = dt
        q = accel_noise ** 2
        g = np.array([[0.5 * dt * dt, 0], [0, 0.5 * dt * dt],
                      [dt, 0], [0, dt]])
        self.Q = g @ g.T * q
        self.H = np.zeros((2, 4))
        self.H[0, 0] = self.H[1, 1] = 1.0
        self.R = np.eye(2) * meas_noise ** 2

        self.z = float(detection.position[2])
        self.category = detection.category
        self.hits = 1
        self.misses = 0
        self.heading = None            # deg, CARLA yaw convention
        self.heading_valid = False     # mature track, currently moving
        self.last_detection = detection

    # ------------------------------------------------------------------
    def predict(self):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, detection, min_speed_for_heading):
        z = np.array(detection.position[:2])
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ self.H) @ self.P

        self.z = 0.8 * self.z + 0.2 * float(detection.position[2])
        self.hits += 1
        self.misses = 0
        self.last_detection = detection

        moving = self.speed >= min_speed_for_heading
        if moving:
            new_heading = math.degrees(math.atan2(self.x[3], self.x[2]))
            if self.heading is None:
                self.heading = new_heading
            else:
                # circular exponential smoothing to damp velocity noise
                self.heading = wrap_deg(
                    self.heading + 0.35 * wrap_deg(new_heading -
                                                   self.heading))
        # heading is only trustworthy once the velocity state has converged
        # (young tracks report the direction of measurement noise)
        self.heading_valid = moving and self.hits >= 8

    # ------------------------------------------------------------------
    @property
    def position(self):
        return np.array([self.x[0], self.x[1], self.z])

    @property
    def velocity(self):
        return np.array([self.x[2], self.x[3], 0.0])

    @property
    def speed(self):
        return float(np.hypot(self.x[2], self.x[3]))


class VehicleTracker(object):

    def __init__(self, dt, tracking_cfg):
        self.dt = dt
        # per-category association gate: pedestrians walk close to each
        # other, so their gate must be tight to avoid identity swaps
        gate = tracking_cfg['gate_dist']
        try:
            self.gate_dist = {c: float(gate[c])
                              for c in ('vehicle', 'bike', 'pedestrian')}
        except (TypeError, KeyError):        # scalar fallback
            self.gate_dist = {c: float(gate)
                              for c in ('vehicle', 'bike', 'pedestrian')}
        self.accel_noise = float(tracking_cfg['accel_noise'])
        self.meas_noise = float(tracking_cfg['meas_noise'])
        self.confirm_hits = int(tracking_cfg['confirm_hits'])
        self.max_misses = int(tracking_cfg['max_misses'])
        self.min_speed_for_heading = \
            float(tracking_cfg['min_speed_for_heading'])
        self.tracks = []

    def step(self, detections):
        """Advance one tick with the fused detections; returns live tracks."""
        for track in self.tracks:
            track.predict()

        # greedy nearest-neighbour association (same category only)
        unmatched = list(range(len(detections)))
        pairs = []
        for track in self.tracks:
            best_j, best_d = None, self.gate_dist[track.category]
            for j in unmatched:
                if detections[j].category != track.category:
                    continue
                d = np.linalg.norm(track.x[:2] -
                                   np.array(detections[j].position[:2]))
                if d < best_d:
                    best_j, best_d = j, d
            if best_j is not None:
                pairs.append((track, best_j))
                unmatched.remove(best_j)

        for track, j in pairs:
            track.update(detections[j], self.min_speed_for_heading)
        matched_tracks = {id(t) for t, _ in pairs}
        for track in self.tracks:
            if id(track) not in matched_tracks:
                track.misses += 1

        for j in unmatched:
            self.tracks.append(Track(detections[j], self.dt,
                                     self.accel_noise, self.meas_noise))

        self.tracks = [t for t in self.tracks if t.misses <= self.max_misses]

        # suppress duplicate tracks on the same object: keep the most mature
        # track, drop any other of the same category within the gate of it
        kept = []
        for track in sorted(self.tracks, key=lambda t: -t.hits):
            if all(track.category != k.category or
                   np.linalg.norm(track.x[:2] - k.x[:2]) >
                   self.gate_dist[track.category]
                   for k in kept):
                kept.append(track)
        self.tracks = kept
        return self.confirmed()

    def confirmed(self):
        return [t for t in self.tracks if t.hits >= self.confirm_hits]
