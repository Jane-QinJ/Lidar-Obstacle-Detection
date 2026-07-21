#!/usr/bin/env python3
"""Quantify obstacle-detection performance against SUSTechPOINTS-style JSON labels.

Both ground-truth and predicted boxes use the same schema:
  [{"obj_id": ..., "obj_type": ..., "psr": {"position": {x,y,z}, "rotation": {x,y,z}, "scale": {x,y,z}}}, ...]

Predicted files are produced by environment_ros's `~output_dir` param (one
file per frame, named "<sec>.<nsec>.json" to match the frame's ROS header
stamp). Ground-truth files use the same "<sec>.<nsec>.json" naming, one per
SUSTechPOINTS-labelled frame.

Matching is nearest-center (predicted rotation is always 0 - BoundingBox()
only produces axis-aligned boxes - so IoU-with-rotation isn't meaningful
here); a GT/prediction pair counts as a match if within `--match-dist`
meters of each other, after both are restricted to `--range` meters of the
sensor origin. Timestamps are matched by nearest neighbor within
`--time-tol` seconds, since predicted and GT frames are rarely captured by
the exact same node.
"""
import argparse
import glob
import json
import math
import os


def load_boxes(path, range_limit):
    with open(path) as f:
        data = json.load(f)
    boxes = []
    for obj in data:
        p = obj["psr"]["position"]
        s = obj["psr"]["scale"]
        dist = math.hypot(p["x"], p["y"])
        if dist > range_limit:
            continue
        boxes.append({
            "obj_id": obj.get("obj_id", ""),
            "obj_type": obj.get("obj_type", "Unknown"),
            "x": p["x"], "y": p["y"], "z": p["z"],
            "sx": s["x"], "sy": s["y"], "sz": s["z"],
            "dist": dist,
        })
    return boxes


def parse_stamp(filename):
    base = os.path.basename(filename)
    base = base[:-len(".json")] if base.endswith(".json") else base
    return float(base)


def match_frames(pred_files, gt_files, time_tol):
    gt_by_time = sorted(((parse_stamp(f), f) for f in gt_files), key=lambda t: t[0])
    pairs = []
    for pf in pred_files:
        pt = parse_stamp(pf)
        best = min(gt_by_time, key=lambda gt: abs(gt[0] - pt), default=None)
        if best is not None and abs(best[0] - pt) <= time_tol:
            pairs.append((pf, best[1]))
    return pairs


def center_dist(a, b):
    return math.sqrt((a["x"] - b["x"]) ** 2 + (a["y"] - b["y"]) ** 2 + (a["z"] - b["z"]) ** 2)


def match_boxes(preds, gts, match_dist):
    """Greedy nearest-neighbor matching, closest pairs first."""
    candidates = []
    for pi, p in enumerate(preds):
        for gi, g in enumerate(gts):
            d = center_dist(p, g)
            if d <= match_dist:
                candidates.append((d, pi, gi))
    candidates.sort(key=lambda c: c[0])

    matched_p, matched_g = set(), set()
    tp_dists = []
    for d, pi, gi in candidates:
        if pi in matched_p or gi in matched_g:
            continue
        matched_p.add(pi)
        matched_g.add(gi)
        tp_dists.append(d)

    tp = len(matched_p)
    fp = len(preds) - tp
    fn = len(gts) - tp
    return tp, fp, fn, tp_dists


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("pred_dir", help="directory of predicted per-frame JSON files")
    ap.add_argument("gt_dir", help="directory of ground-truth per-frame JSON files")
    ap.add_argument("--range", type=float, default=2.0, help="max sensor-origin distance (m) to evaluate, default 2.0")
    ap.add_argument("--match-dist", type=float, default=0.5, help="max center distance (m) to count as a match, default 0.5")
    ap.add_argument("--time-tol", type=float, default=0.1, help="max timestamp gap (s) to pair a predicted frame with a GT frame, default 0.1")
    args = ap.parse_args()

    pred_files = sorted(glob.glob(os.path.join(args.pred_dir, "*.json")))
    gt_files = sorted(glob.glob(os.path.join(args.gt_dir, "*.json")))
    if not pred_files:
        raise SystemExit(f"no JSON files found in {args.pred_dir}")
    if not gt_files:
        raise SystemExit(f"no JSON files found in {args.gt_dir}")

    pairs = match_frames(pred_files, gt_files, args.time_tol)
    if not pairs:
        raise SystemExit(
            f"no predicted/GT frame pairs within {args.time_tol}s of each other - "
            "check that pred_dir and gt_dir cover the same time range"
        )

    total_tp = total_fp = total_fn = 0
    all_dists = []
    for pf, gf in pairs:
        preds = load_boxes(pf, args.range)
        gts = load_boxes(gf, args.range)
        tp, fp, fn, dists = match_boxes(preds, gts, args.match_dist)
        total_tp += tp
        total_fp += fp
        total_fn += fn
        all_dists.extend(dists)

    precision = total_tp / (total_tp + total_fp) if (total_tp + total_fp) else float("nan")
    recall = total_tp / (total_tp + total_fn) if (total_tp + total_fn) else float("nan")
    f1 = 2 * precision * recall / (precision + recall) if (precision + recall) else float("nan")
    mean_err = sum(all_dists) / len(all_dists) if all_dists else float("nan")

    print(f"frame pairs matched:   {len(pairs)} / {len(pred_files)} predicted frames")
    print(f"range filter:          <= {args.range} m from sensor origin")
    print(f"match threshold:       <= {args.match_dist} m center distance")
    print(f"true positives:        {total_tp}")
    print(f"false positives:       {total_fp}")
    print(f"false negatives:       {total_fn}")
    print(f"precision:             {precision:.3f}")
    print(f"recall:                {recall:.3f}")
    print(f"f1:                    {f1:.3f}")
    print(f"mean center error (m): {mean_err:.3f}")


if __name__ == "__main__":
    main()
