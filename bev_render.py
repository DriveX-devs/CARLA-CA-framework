# -*- coding: utf-8 -*-
"""
Bird's-eye view of one CAV's perception at one step, for the backends that
have no camera boxes to draw on (SiMO on LiDAR, oracle): the LiDAR sweep,
the ground-truth objects the sweep reaches, the detections and the tracks.

The view is the CAV's own LiDAR frame, forward up: x grows upwards and y
(right of the vehicle) grows to the right.
"""

import cv2
import numpy as np

_COLORS = {'vehicle': (0, 255, 255), 'bike': (255, 255, 0),
           'pedestrian': (255, 0, 255)}          # BGR


def render_bev(points, lidar_pose, gt_objects, gt_point_counts, detections,
               tracks, ca_results=None, range_m=54.4, size=900, title=''):
    """Return a BGR image (size x size)."""
    img = np.full((size, size, 3), 24, np.uint8)
    scale = size / (2.0 * range_m)
    inv = np.linalg.inv(lidar_pose)

    def px(local):
        local = np.atleast_2d(local)
        u = size / 2.0 + local[:, 1] * scale
        v = size / 2.0 - local[:, 0] * scale
        return np.stack([u, v], axis=1)

    def to_local(world):
        world = np.atleast_2d(np.asarray(world, dtype=np.float64))
        return world @ inv[:3, :3].T + inv[:3, 3]

    for r in range(10, int(range_m) + 1, 10):
        cv2.circle(img, (size // 2, size // 2), int(r * scale), (48, 48, 48),
                   1, cv2.LINE_AA)

    xyz = np.asarray(points)[:, :3]
    uv = px(xyz).astype(np.int32)
    ok = (uv[:, 0] >= 0) & (uv[:, 0] < size) & (uv[:, 1] >= 0) & \
        (uv[:, 1] < size)
    uv, z = uv[ok], xyz[ok, 2]
    shade = np.clip((z + 2.0) / 4.0, 0.0, 1.0)
    img[uv[:, 1], uv[:, 0]] = np.stack(
        [90 + 80 * shade, 90 + 120 * shade, 90 + 60 * shade],
        axis=1).astype(np.uint8)

    # ground truth: hollow circles, thicker when the sweep reaches the object
    counts = gt_point_counts if gt_point_counts is not None \
        else [0] * len(gt_objects)
    for gt, n in zip(gt_objects, counts):
        centre = px(to_local(gt['centre']))[0]
        if not (0 <= centre[0] < size and 0 <= centre[1] < size):
            continue
        color = _COLORS.get(gt.get('category'), (160, 160, 160))
        radius = max(4, int(np.linalg.norm(gt['extent'][:2]) * scale))
        cv2.circle(img, (int(centre[0]), int(centre[1])), radius, color,
                   2 if n > 0 else 1, cv2.LINE_AA)

    # detections: boxes when the backend gives them, crosses otherwise
    for det in detections:
        color = _COLORS.get(det.category, (200, 200, 200))
        corners = getattr(det, 'corners', None)
        if corners is not None:
            poly = px(to_local(corners[:4])).astype(np.int32)
            cv2.polylines(img, [poly], True, color, 2, cv2.LINE_AA)
        else:
            c = px(to_local(det.position))[0].astype(int)
            cv2.drawMarker(img, (int(c[0]), int(c[1])), color,
                           cv2.MARKER_TILTED_CROSS, 10, 2, cv2.LINE_AA)

    # tracks: velocity arrows, red when the collision check fires
    for track in tracks:
        pos = np.array([track.position[0], track.position[1],
                        getattr(track, 'z', 0.0)])
        tip = pos + np.array([track.x[2], track.x[3], 0.0])
        a, b = px(to_local(np.stack([pos, tip]))).astype(int)
        ca = ca_results.get(track.track_id) if ca_results else None
        warning = ca is not None and ca[3]
        color = (0, 0, 255) if warning else (255, 255, 255)
        cv2.arrowedLine(img, (int(a[0]), int(a[1])), (int(b[0]), int(b[1])),
                        color, 2 if warning else 1, cv2.LINE_AA)
        cv2.putText(img, '%d' % track.track_id, (int(a[0]) + 6, int(a[1]) - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv2.LINE_AA)

    cv2.drawMarker(img, (size // 2, size // 2), (0, 255, 0),
                   cv2.MARKER_TRIANGLE_UP, 14, 2, cv2.LINE_AA)
    legend = [title,
              'circles: ground truth (thick = hit by this sweep)',
              'boxes: detections   arrows: tracks (red = CA warning)',
              'yellow vehicle / cyan bike / magenta pedestrian']
    for i, line in enumerate(legend):
        cv2.putText(img, line, (10, 20 + 18 * i), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (230, 230, 230), 1, cv2.LINE_AA)
    return img
