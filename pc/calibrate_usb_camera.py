#!/usr/bin/env python3
"""Calibrate a fixed USB camera mode from original chessboard photographs."""
from __future__ import annotations

import argparse
import glob
import json
import math
from pathlib import Path


def calibrate(paths, columns, rows, square_mm):
    import cv2
    import numpy as np
    if columns < 3 or rows < 3 or not math.isfinite(square_mm) or square_mm <= 0:
        raise ValueError('Supply INNER corner columns/rows >=3 and a positive measured square size')
    pattern = np.zeros((rows * columns, 3), np.float32)
    pattern[:, :2] = np.mgrid[0:columns, 0:rows].T.reshape(-1, 2) * square_mm
    objects, corners, accepted, rejected = [], [], [], []
    size = None
    for path in paths:
        image = cv2.imread(str(path))
        if image is None:
            rejected.append(dict(path=str(path), reason='cannot read'))
            continue
        current = image.shape[1], image.shape[0]
        if size is not None and current != size:
            raise ValueError('Calibration photographs must use one resolution and camera mode')
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        ok, points = cv2.findChessboardCorners(gray, (columns, rows),
                                              cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not ok:
            rejected.append(dict(path=str(path), reason='inner corners not found'))
            continue
        size = current
        points = cv2.cornerSubPix(gray, points, (11, 11), (-1, -1),
                                 (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, .001))
        objects.append(pattern.copy())
        corners.append(points)
        accepted.append(str(path))
    if len(accepted) < 12:
        raise ValueError(f'Only {len(accepted)} usable views; collect >=12 varied tilts/positions, including image edges')
    rms, matrix, distortion, rotations, translations = cv2.calibrateCamera(objects, corners, size, None, None)
    if not np.isfinite(matrix).all() or not np.isfinite(distortion).all() or not math.isfinite(rms):
        raise ValueError('Non-finite calibration result')
    if matrix[0, 0] <= 0 or matrix[1, 1] <= 0:
        raise ValueError('Invalid focal length')
    errors = []
    for object_points, image_points, rotation, translation in zip(objects, corners, rotations, translations):
        projected, _ = cv2.projectPoints(object_points, rotation, translation, matrix, distortion)
        errors.append(float(np.sqrt(np.mean(np.sum((projected - image_points) ** 2, axis=2)))))
    return dict(width=size[0], height=size[1], fx=float(matrix[0, 0]), fy=float(matrix[1, 1]),
                cx=float(matrix[0, 2]), cy=float(matrix[1, 2]),
                distortion_coefficients=distortion.ravel().tolist(),
                rms_pixels=float(rms), per_view_rms_pixels=errors,
                accepted_images=accepted, rejected_images=rejected,
                calibration_method='opencv_chessboard', square_mm=square_mm,
                pattern_inner_corners=[columns, rows],
                note='Use unchanged focus/zoom/resolution. This calibrates camera rays, not target distance.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--images', required=True, help='Quoted glob of RAW images, e.g. dataset/usb_captures/calibration/*.jpg')
    parser.add_argument('--columns', type=int, default=9)
    parser.add_argument('--rows', type=int, default=6)
    parser.add_argument('--square-mm', type=float, required=True)
    parser.add_argument('--output', type=Path, default=Path('config/usb_camera.json'))
    args = parser.parse_args()
    try:
        paths = sorted(Path(p) for p in glob.glob(args.images))
        if not paths:
            raise ValueError('No images matched')
        result = calibrate(paths, args.columns, args.rows, args.square_mm)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        temporary = args.output.with_suffix(args.output.suffix + '.tmp')
        temporary.write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
        temporary.replace(args.output)
        print(f'Saved {args.output}; {len(result["accepted_images"])} views, RMS={result["rms_pixels"]:.3f}px')
        print('Inspect per-view errors and coverage. Successful fitting alone does not prove calibration accuracy.')
        return 0
    except Exception as exc:
        print(f'Camera calibration failed: {exc}')
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
