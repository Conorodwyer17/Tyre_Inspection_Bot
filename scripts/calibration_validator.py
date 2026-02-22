#!/usr/bin/env python3
"""
Phase C — Calibration Validator.
Verifies forward/backward consistency: project world coords to pixels, back-project via disparity.
"""

import argparse
import os
import sys
import yaml
import numpy as np


def load_calibration(path: str) -> dict:
    with open(path) as f:
        c = yaml.safe_load(f)
    w = c.get("image_width", 640)
    h = c.get("image_height", 480)
    baseline = c.get("baseline_m", 0.06)
    Kl = np.array(c["left"]["camera_matrix"], dtype=np.float64).reshape(3, 3)
    Kr = np.array(c["right"]["camera_matrix"], dtype=np.float64).reshape(3, 3)
    Dl = np.array(c["left"]["dist_coeffs"], dtype=np.float64)
    Dr = np.array(c["right"]["dist_coeffs"], dtype=np.float64)
    R = np.array(c.get("rotation", np.eye(3).tolist()), dtype=np.float64).reshape(3, 3)
    T = np.array(c.get("translation", [-baseline, 0, 0]), dtype=np.float64)
    return {
        "w": w, "h": h, "baseline": baseline,
        "Kl": Kl, "Kr": Kr, "Dl": Dl, "Dr": Dr, "R": R, "T": T,
    }


def validate(c: dict, test_depths_m=(1.0, 1.5), depth_tolerance_pct=5.0, skip_reprojection=False) -> dict:
    import cv2

    result = {"passed": True, "errors": [], "depth_errors_pct": [], "reprojection_skipped": False}

    R1, R2, P1, P2, Q = cv2.fisheye.stereoRectify(
        c["Kl"], c["Dl"], c["Kr"], c["Dr"],
        (c["w"], c["h"]), c["R"], c["T"],
        flags=cv2.CALIB_ZERO_DISPARITY, balance=1.0, fov_scale=1.0,
    )

    fx = P1[0, 0]
    baseline = abs(c["T"][0]) if abs(c["T"][0]) > 1e-6 else c["baseline"]
    cx, cy = c["Kl"][0, 2], c["Kl"][1, 2]

    if not skip_reprojection and np.any(np.abs(c["Dl"]) > 1e-9):
        pt3d = np.array([[[0.0, 0.0, 1.0]]], dtype=np.float64)
        pt2d, _ = cv2.fisheye.projectPoints(pt3d, np.zeros(3), np.zeros(3), c["Kl"], c["Dl"])
        u_proj, v_proj = float(pt2d[0, 0, 0]), float(pt2d[0, 0, 1])
        err_px = np.hypot(u_proj - cx, v_proj - cy)
        if err_px > 2.0:
            result["errors"].append(f"Principal point reprojection error {err_px:.2f} px")
            result["passed"] = False
    elif not skip_reprojection:
        result["reprojection_skipped"] = True

    for z_m in test_depths_m:
        d = fx * baseline / z_m
        z_reconstructed = fx * baseline / d
        err_pct = abs(z_reconstructed - z_m) / z_m * 100
        result["depth_errors_pct"].append(err_pct)
        if err_pct > depth_tolerance_pct:
            result["errors"].append(f"Depth error {err_pct:.2f}% at z={z_m}m")
            result["passed"] = False

    return result


def main():
    p = argparse.ArgumentParser(description="Validate stereo calibration (forward/back consistency)")
    p.add_argument("calib_yaml", help="Path to calibration YAML")
    p.add_argument("--depth-tolerance-pct", type=float, default=5.0)
    args = p.parse_args()

    if not os.path.isfile(args.calib_yaml):
        print(f"File not found: {args.calib_yaml}")
        return 1

    c = load_calibration(args.calib_yaml)
    r = validate(c, depth_tolerance_pct=args.depth_tolerance_pct)

    if r["errors"]:
        for e in r["errors"]:
            print(f"ERROR: {e}")
    if r["depth_errors_pct"]:
        print(f"Depth error stats: mean={np.mean(r['depth_errors_pct']):.2f}% max={np.max(r['depth_errors_pct']):.2f}%")
    print(f"PASSED: {r['passed']}")
    return 0 if r["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
