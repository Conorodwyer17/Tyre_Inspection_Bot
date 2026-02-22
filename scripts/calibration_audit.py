#!/usr/bin/env python3
"""
Phase C - Calibration Audit.
Validates YAML calibration files: intrinsics, baseline, resolution.
"""
import argparse
import os
import sys
import yaml
import json

def audit_yaml(path):
    result = {"path": path, "exists": os.path.isfile(path), "valid": False, "errors": [], "warnings": [], "metrics": {}}
    if not result["exists"]:
        result["errors"].append("File not found: " + path)
        return result
    with open(path) as f:
        try:
            c = yaml.safe_load(f)
        except Exception as e:
            result["errors"].append("YAML parse: " + str(e))
            return result
    if not isinstance(c, dict):
        result["errors"].append("Root must be mapping")
        return result
    w, h = c.get("image_width"), c.get("image_height")
    if w is None or h is None:
        result["errors"].append("Missing image_width or image_height")
    else:
        result["metrics"]["image_width"] = w
        result["metrics"]["image_height"] = h
        if w != 640 or h != 480:
            result["warnings"].append("Resolution differs from Aurora 640x480")
    b = c.get("baseline_m")
    if b is not None:
        result["metrics"]["baseline_m"] = b
        if abs(b - 0.06) > 0.01:
            result["warnings"].append("Baseline differs from 6cm")
    for cam in ("left", "right"):
        if cam not in c:
            result["errors"].append("Missing section: " + cam)
            continue
        sec = c[cam]
        if sec.get("camera_matrix") is None:
            result["errors"].append(cam + ": missing camera_matrix")
        if sec.get("dist_coeffs") is None:
            result["warnings"].append(cam + ": missing dist_coeffs")
    rms = c.get("rms_error") or c.get("rms_reprojection_error")
    if rms is None:
        result["warnings"].append("rms_error undocumented")
    else:
        result["metrics"]["rms_error"] = float(rms)
        if float(rms) > 0.6:
            result["errors"].append("rms_error exceeds 0.6 px")
    result["valid"] = len(result["errors"]) == 0
    return result

def main():
    p = argparse.ArgumentParser()
    p.add_argument("files", nargs="*")
    p.add_argument("--json", "-j", action="store_true")
    args = p.parse_args()
    ws = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    defaults = [
        os.path.join(ws, "src", "aurora_sdk_bridge", "config", "equidistant_calibration.yaml"),
        os.path.join(ws, "src", "Tyre_Inspection_Bot", "src", "amr_hardware", "src", "segment_3d", "segmentation_3d", "config", "stereo_calibration_verified.yaml"),
    ]
    paths = args.files if args.files else [x for x in defaults if os.path.isfile(x) or os.path.exists(os.path.dirname(x))]
    paths = list(dict.fromkeys(p for p in paths if p))
    results = [audit_yaml(p) for p in paths]
    if args.json:
        print(json.dumps(results, indent=2))
    else:
        for r in results:
            print("\n=== " + r["path"] + " ===")
            print("  Valid:", r["valid"])
            for e in r["errors"]: print("  ERROR:", e)
            for w in r["warnings"]: print("  WARN:", w)
    return 0 if all(r["valid"] for r in results) else 1

if __name__ == "__main__":
    sys.exit(main())
