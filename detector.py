# -*- coding: utf-8 -*-
"""
YOLOv8 vehicle detector for the CA pipeline.

Uses ultralytics YOLOv8 with selectable capacity (n / m / x). This is the
most recent YOLO family installable in the msvan3t_carla env (Python 3.7
caps ultralytics at 8.0.x; YOLO11 needs Python >= 3.8). Every CAV
instantiates its OWN detector (one YOLO model per CAV); the 4 camera images
of one CAV at one simulation step are run as a single batch.
"""

import os

import numpy as np
import torch
from ultralytics import YOLO

CAPACITIES = ('n', 'm', 'x')


class Detection2D(object):
    """One 2D detection (vehicle / bike / person) in one camera image."""

    __slots__ = ('bbox', 'conf', 'cls_id', 'cls_name')

    def __init__(self, bbox, conf, cls_id, cls_name):
        self.bbox = bbox            # (x1, y1, x2, y2) float pixels
        self.conf = float(conf)
        self.cls_id = int(cls_id)
        self.cls_name = cls_name


class YoloDetector(object):
    """Batched YOLOv8 inference restricted to the configured classes."""

    def __init__(self, capacity, weights_dir, confidence, classes,
                 quiet=False):
        if capacity not in CAPACITIES:
            raise ValueError('capacity must be one of %s, got %r'
                             % (CAPACITIES, capacity))
        weights = os.path.join(weights_dir, 'yolov8%s.pt' % capacity)
        # YOLO() downloads to cwd if the file is missing; pin it to weights_dir
        if not os.path.isfile(weights):
            os.makedirs(weights_dir, exist_ok=True)
            cwd = os.getcwd()
            os.chdir(weights_dir)
            try:
                YOLO('yolov8%s.pt' % capacity)
            finally:
                os.chdir(cwd)

        self.model = YOLO(weights)
        self.device = 0 if torch.cuda.is_available() else 'cpu'
        self.confidence = float(confidence)
        self.classes = [int(c) for c in classes]

        # warm up: the first CUDA inference is much slower than steady state
        dummy = np.zeros((640, 640, 3), dtype=np.uint8)
        self.model.predict(dummy, device=self.device, verbose=False)
        if not quiet:
            print('YOLOv8-%s ready on %s (classes %s, conf >= %.2f)'
                  % (capacity, 'cuda:0' if self.device == 0 else 'cpu',
                     self.classes, self.confidence))

    def detect_batch(self, images_bgr):
        """Run one batched inference.

        Parameters
        ----------
        images_bgr : list of np.ndarray
            BGR images (ultralytics expects BGR for numpy input).

        Returns
        -------
        list of list of Detection2D, aligned with images_bgr.
        """
        results = self.model.predict(
            images_bgr, device=self.device, verbose=False,
            conf=self.confidence, classes=self.classes)

        all_dets = []
        for res in results:
            dets = []
            boxes = res.boxes
            if boxes is not None and len(boxes) > 0:
                xyxy = boxes.xyxy.cpu().numpy()
                confs = boxes.conf.cpu().numpy()
                clses = boxes.cls.cpu().numpy().astype(int)
                for bb, cf, cl in zip(xyxy, confs, clses):
                    dets.append(Detection2D(tuple(bb), cf, cl,
                                            res.names[int(cl)]))
            all_dets.append(dets)
        return all_dets
