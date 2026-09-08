# -*- coding: utf-8 -*-
"""
Camera + LiDAR late fusion for the CA pipeline.

Strategy (frustum association):
  1. The LiDAR sweep is transformed to world coordinates once per tick
     (OpenCDA's sensor_transformation math).
  2. For every camera the world points are projected onto the image plane.
  3. For every YOLO box, the projected points falling inside a slightly
     shrunk box are collected and clustered by camera depth; the cluster
     closest to the camera (the box owner) is kept, which rejects the
     background points that leak into the box.
  4. The cluster's world-frame centroid is pushed a small, fixed distance
     away from the sensor to convert the *visible-surface* centroid into an
     *object-center* estimate.
  5. Detections of the same physical object seen by several cameras are
     merged in world space (weighted by LiDAR point count).

The output feeds the tracker, which derives velocity and heading.
"""

import numpy as np

from opencda.core.sensing.perception.sensor_transformation import (
    get_camera_intrinsic, world_to_sensor, x_to_world_transformation)


# COCO class -> evaluation category. CARLA's bicycles carry a rider, so the
# person detected on top of a bike is merged into the bike (see
# merge_detections); standalone persons are pedestrians.
CLS_CATEGORY = {
    'car': 'vehicle', 'bus': 'vehicle', 'truck': 'vehicle',
    'bicycle': 'bike', 'motorcycle': 'bike',
    'person': 'pedestrian',
}
CATEGORIES = ('vehicle', 'bike', 'pedestrian')


class Detection3D(object):
    """A fused (camera + LiDAR) detection in world coordinates."""

    __slots__ = ('position', 'n_points', 'conf', 'cls_name', 'category',
                 'camera_idx', 'bbox', 'uv_points')

    def __init__(self, position, n_points, conf, cls_name, camera_idx,
                 bbox, uv_points):
        self.position = position        # np.array [x, y, z] world frame
        self.n_points = int(n_points)
        self.conf = float(conf)
        self.cls_name = cls_name
        self.category = CLS_CATEGORY.get(cls_name, 'vehicle')
        self.camera_idx = camera_idx
        self.bbox = bbox                # 2D box in the source camera
        self.uv_points = uv_points      # (M, 2) pixel coords of cluster pts


def suppress_contained(detections_2d):
    """Drop partial duplicate boxes of the same class in one camera.

    YOLO occasionally emits a second small box on a fragment of an already
    detected object (e.g. the tail of a car). A box whose area is >70 %
    inside a larger box of the same class is that kind of fragment; its
    LiDAR cluster would spawn a phantom object, so it is removed here.
    """
    keep = []
    order = sorted(detections_2d,
                   key=lambda d: -(d.bbox[2] - d.bbox[0]) *
                                  (d.bbox[3] - d.bbox[1]))
    for det in order:                          # big boxes first
        x1, y1, x2, y2 = det.bbox
        area = max(1e-6, (x2 - x1) * (y2 - y1))
        contained = False
        for big in keep:
            if big.cls_name != det.cls_name:
                continue
            bx1, by1, bx2, by2 = big.bbox
            ix = max(0.0, min(x2, bx2) - max(x1, bx1))
            iy = max(0.0, min(y2, by2) - max(y1, by1))
            if ix * iy / area > 0.7:
                contained = True
                break
        if not contained:
            keep.append(det)
    return keep


class CameraLidarFusion(object):

    def __init__(self, fusion_cfg):
        self.bbox_shrink = float(fusion_cfg['bbox_shrink'])
        self.depth_gap = float(fusion_cfg['depth_gap'])
        self.min_ego_dist = float(fusion_cfg['min_ego_dist'])
        # per-category parameters (dicts keyed by CATEGORIES)
        self.min_points = {c: int(fusion_cfg['min_points'][c])
                           for c in CATEGORIES}
        self.center_offset = {c: float(fusion_cfg['center_offset'][c])
                              for c in CATEGORIES}
        self.merge_dist = {c: float(fusion_cfg['merge_dist'][c])
                           for c in CATEGORIES}

    # ------------------------------------------------------------------
    def lidar_to_world(self, lidar_points, lidar_sensor):
        """(N, 4) sweep in LiDAR frame -> (4, M) homogeneous world points.

        Returns on the ego body are dropped (they would otherwise fall into
        boxes of nearby vehicles).
        """
        xyz = lidar_points[:, :3]
        keep = np.linalg.norm(xyz[:, :2], axis=1) > self.min_ego_dist
        xyz = xyz[keep]
        homog = np.r_[xyz.T, [np.ones(xyz.shape[0])]]
        lidar_2_world = x_to_world_transformation(lidar_sensor.get_transform())
        return lidar_2_world @ homog

    def project_to_camera(self, world_points, camera_sensor):
        """Project world points into one camera.

        Returns (uv, depth, world_xyz): pixel coords (M, 2), camera depth
        (M,), and the matching world coordinates (M, 3) — only points inside
        the image and in front of the camera.
        """
        sensor_points = world_to_sensor(world_points,
                                        camera_sensor.get_transform())
        # UE4 -> standard camera axes: (x, y, z) -> (y, -z, x)
        cam = np.array([sensor_points[1],
                        -sensor_points[2],
                        sensor_points[0]])
        K = get_camera_intrinsic(camera_sensor)
        uvw = K @ cam
        depth = uvw[2]
        with np.errstate(divide='ignore', invalid='ignore'):
            uv = (uvw[:2] / depth).T                       # (M, 2)

        w = int(camera_sensor.attributes['image_size_x'])
        h = int(camera_sensor.attributes['image_size_y'])
        mask = (depth > 0.5) & \
               (uv[:, 0] >= 0) & (uv[:, 0] < w) & \
               (uv[:, 1] >= 0) & (uv[:, 1] < h)
        return uv[mask], depth[mask], world_points[:3, mask].T

    # ------------------------------------------------------------------
    def _foreground_cluster(self, depths, min_points):
        """Indices of the depth cluster that owns the bounding box.

        Points are sorted by depth and split wherever consecutive depths gap
        by more than `depth_gap`; the nearest cluster with enough points is
        the foreground object, everything behind it is background leakage.
        """
        order = np.argsort(depths)
        sorted_d = depths[order]
        gaps = np.where(np.diff(sorted_d) > self.depth_gap)[0]
        start = 0
        for end in list(gaps) + [len(sorted_d) - 1]:
            cluster = order[start:end + 1]
            if len(cluster) >= min_points:
                return cluster
            start = end + 1
        return None

    def fuse_camera(self, detections_2d, uv, depth, world_xyz,
                    camera_idx, camera_sensor):
        """Lift one camera's 2D detections to 3D using the projected sweep."""
        out = []
        cam_loc = camera_sensor.get_transform().location
        cam_pos = np.array([cam_loc.x, cam_loc.y, cam_loc.z])

        for det in detections_2d:
            category = CLS_CATEGORY.get(det.cls_name, 'vehicle')
            min_points = self.min_points[category]

            x1, y1, x2, y2 = det.bbox
            dx = (x2 - x1) * self.bbox_shrink
            dy = (y2 - y1) * self.bbox_shrink
            inside = (uv[:, 0] >= x1 + dx) & (uv[:, 0] <= x2 - dx) & \
                     (uv[:, 1] >= y1 + dy) & (uv[:, 1] <= y2 - dy)
            if inside.sum() < min_points:
                continue

            idx = self._foreground_cluster(depth[inside], min_points)
            if idx is None:
                continue

            pts_world = world_xyz[inside][idx]
            centroid = pts_world.mean(axis=0)

            # visible-surface centroid -> object center: push away from the
            # sensor along the horizontal viewing ray
            ray = centroid - cam_pos
            ray[2] = 0.0
            norm = np.linalg.norm(ray)
            if norm > 1e-3:
                centroid = centroid + \
                    ray / norm * self.center_offset[category]

            out.append(Detection3D(centroid, len(idx), det.conf,
                                   det.cls_name, camera_idx, det.bbox,
                                   uv[inside][idx]))
        return out

    def _mergeable(self, a, b):
        """Whether two detections are duplicates of the same object.

        Same category merges within the category's radius. A person and a
        bike also merge (CARLA bicycles carry a rider, and YOLO sees both a
        'bicycle' and a 'person' on the same LiDAR cluster) — the pair is
        reported as a bike.
        """
        d = np.linalg.norm(a.position[:2] - b.position[:2])
        if a.category == b.category:
            return d < self.merge_dist[a.category]
        if {a.category, b.category} == {'pedestrian', 'bike'}:
            return d < 1.2
        return False

    def merge_detections(self, detections):
        """Merge multi-camera / rider duplicates of the same object."""
        merged = []
        used = [False] * len(detections)
        order = sorted(range(len(detections)),
                       key=lambda i: -detections[i].n_points)
        for i in order:
            if used[i]:
                continue
            group = [detections[i]]
            used[i] = True
            for j in order:
                if used[j]:
                    continue
                if self._mergeable(detections[i], detections[j]):
                    group.append(detections[j])
                    used[j] = True
            weights = np.array([g.n_points for g in group], dtype=float)
            pos = np.average([g.position for g in group], axis=0,
                             weights=weights)
            # rider+bicycle pairs are reported as the bike
            bikes = [g for g in group if g.category == 'bike']
            best = bikes[0] if bikes else group[0]
            out = Detection3D(pos, int(weights.sum()), best.conf,
                              best.cls_name, best.camera_idx,
                              best.bbox, best.uv_points)
            merged.append(out)
        return merged
