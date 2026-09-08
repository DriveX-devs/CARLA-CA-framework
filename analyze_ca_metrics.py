# -*- coding: utf-8 -*-
"""
Post-run metric analysis for the (extended) CA application.

Takes the log files of one run directory (CA/output/run_<timestamp>/,
produced by run_ca.py or run_ca_extended.py) and computes, with 95%
confidence intervals:

1. DETECTED VRUs: average number of VRUs (pedestrians + bikes) detected per
   CAV among those within --vru-distance metres of the CAV. One sample per
   (step, CAV); a VRU counts as detected when a confirmed track of that CAV
   is matched to it in metrics.csv at that step.

2. MISDETECTION PROBABILITY: probability that a VRU within --vru-distance of
   a CAV is NOT detected by it. One Bernoulli sample per (step, CAV, VRU in
   range), built from gt.csv (who is in range) vs metrics.csv (who was
   matched); 95% Wilson score interval.

3. TOTAL WARNING LATENCY (centralized CA, modes 2/3): for every warning
   delivered to a CAV (edge_warnings.csv), the end-to-end chain
       sensing period                      (--sensing-ms, default 50 = one
                                            20 Hz sensor frame period)
     + perception inference               (YOLO + fusion + tracking wall
                                            time of the uplink step, from
                                            timing.csv; 0 for VAM-triggered
                                            warnings: the VRU state is
                                            self-knowledge, not perception)
     + detection/VAM uplink latency_ms    (simulated 5G, ns-3)
     + edge CA execution time             (wall clock, edge_ca_ms)
     + warning downlink latency_ms        (simulated 5G, ns-3)
     + actuation time                     (--actuation-ms, default 100 ms:
                                            brake-system pressure build-up
                                            of an automated emergency brake,
                                            ~0.2-0.3 s in the AEB
                                            literature/UN R152 tests; use
                                            ~1200 ms to model a human driver
                                            reacting to an HMI warning)
   For comparison the DECENTRALIZED equivalent is also computed from the
   local CA warnings of metrics.csv (sensing + perception + local CA +
   actuation, no network).

Usage:
    python CA/analyze_ca_metrics.py CA/output/run_<timestamp> \
        [--vru-distance 30] [--sensing-ms 50] [--actuation-ms 100]

The report is printed and saved to <run_dir>/analysis.txt.
"""

import argparse
import csv
import math
import os
import sys

Z95 = 1.959964  # two-sided 95% normal quantile


# ---------------------------------------------------------------------------
# Statistics helpers
# ---------------------------------------------------------------------------
def mean_ci(values):
    """Sample mean and 95% CI half-width (normal approximation)."""
    n = len(values)
    if n == 0:
        return float("nan"), float("nan"), 0
    mean = sum(values) / n
    if n == 1:
        return mean, float("nan"), 1
    var = sum((v - mean) ** 2 for v in values) / (n - 1)
    return mean, Z95 * math.sqrt(var / n), n


def wilson_ci(k, n):
    """Wilson score 95% interval for a proportion k/n -> (p, lo, hi)."""
    if n == 0:
        return float("nan"), float("nan"), float("nan")
    p = k / n
    z2 = Z95 * Z95
    denom = 1.0 + z2 / n
    center = (p + z2 / (2 * n)) / denom
    half = (Z95 * math.sqrt(p * (1 - p) / n + z2 / (4 * n * n))) / denom
    return p, max(0.0, center - half), min(1.0, center + half)


def fmt_mean(label, values, unit=""):
    mean, hw, n = mean_ci(values)
    if n == 0:
        return "  %-26s: no samples" % label
    hw_s = "+/- %.3f" % hw if hw == hw else "+/- n/a"
    return "  %-26s: %.3f %s %s  (n=%d)" % (label, mean, hw_s, unit, n)


# ---------------------------------------------------------------------------
# Log loading
# ---------------------------------------------------------------------------
VRU_CATEGORIES = ("pedestrian", "bike")


def load_gt(run_dir):
    """gt.csv -> {step: {'cavs': {name: (x, y)}, 'vrus': {id: (x, y, cat)}}}"""
    scene = {}
    with open(os.path.join(run_dir, "gt.csv")) as f:
        for row in csv.DictReader(f):
            step = int(row["step"])
            entry = scene.setdefault(step, {"cavs": {}, "vrus": {}})
            x, y = float(row["x"]), float(row["y"])
            if row["name"].startswith("Cav"):
                entry["cavs"][row["name"]] = (x, y)
            elif row["category"] in VRU_CATEGORIES:
                entry["vrus"][int(row["id"])] = (x, y, row["category"])
    return scene


def load_metrics(run_dir):
    """metrics.csv -> (detected VRU triples {(step, cav, gt_id)},
    local-CA warning rows [(step, cav)])."""
    detected = set()
    local_warnings = []
    with open(os.path.join(run_dir, "metrics.csv")) as f:
        for row in csv.DictReader(f):
            step, cav = int(row["step"]), row["cav"]
            if row["gt_id"] and row["gt_category"] in VRU_CATEGORIES:
                detected.add((step, cav, int(row["gt_id"])))
            if row.get("ca_warning") == "1":
                local_warnings.append((step, cav))
    return detected, local_warnings


def load_timing(run_dir):
    """timing.csv -> {(step, cav): (t_detect, t_fuse, t_track, t_ca)} [ms]"""
    path = os.path.join(run_dir, "timing.csv")
    if not os.path.isfile(path):
        return None
    timing = {}
    with open(path) as f:
        for row in csv.DictReader(f):
            timing[(int(row["step"]), row["cav"])] = (
                float(row["t_detect_ms"]), float(row["t_fuse_ms"]),
                float(row["t_track_ms"]), float(row["t_ca_ms"]))
    return timing


def load_edge_warnings(run_dir):
    path = os.path.join(run_dir, "edge_warnings.csv")
    if not os.path.isfile(path):
        return []
    with open(path) as f:
        return list(csv.DictReader(f))


# ---------------------------------------------------------------------------
# Metrics
# ---------------------------------------------------------------------------
def detection_metrics(scene, detected, vru_distance):
    """Metrics 1 and 2: per-(step, CAV) detected-VRU counts and per-VRU
    misdetection Bernoulli samples, plus per-CAV / per-category splits."""
    counts = []                       # detected VRUs in range, per (step, cav)
    in_range_counts = []              # VRUs in range, per (step, cav)
    counts_by_cav = {}
    miss_k = miss_n = 0               # global misdetections / eligible
    miss_by_cat = {c: [0, 0] for c in VRU_CATEGORIES}
    miss_by_cav = {}

    for step, entry in scene.items():
        for cav, (cx, cy) in entry["cavs"].items():
            n_det = n_in = 0
            for vid, (vx, vy, cat) in entry["vrus"].items():
                if math.hypot(vx - cx, vy - cy) > vru_distance:
                    continue
                n_in += 1
                is_detected = (step, cav, vid) in detected
                n_det += is_detected
                miss_n += 1
                miss_k += not is_detected
                miss_by_cat[cat][1] += 1
                miss_by_cat[cat][0] += not is_detected
                cav_kn = miss_by_cav.setdefault(cav, [0, 0])
                cav_kn[1] += 1
                cav_kn[0] += not is_detected
            counts.append(n_det)
            in_range_counts.append(n_in)
            counts_by_cav.setdefault(cav, []).append(n_det)

    return {"counts": counts, "in_range": in_range_counts,
            "counts_by_cav": counts_by_cav, "miss": (miss_k, miss_n),
            "miss_by_cat": miss_by_cat, "miss_by_cav": miss_by_cav}


def warning_latency(edge_rows, timing, sensing_ms, actuation_ms):
    """Metric 3: per delivered warning, the component and total latencies."""
    comp = {"sensing": [], "perception": [], "uplink": [], "edge_ca": [],
            "downlink": [], "actuation": []}
    totals = []
    totals_by_kind = {}
    skipped = 0
    for row in edge_rows:
        try:
            downlink = float(row["latency_ms"])
            uplink = float(row["uplink_latency_ms"])
            edge_ca = float(row["edge_ca_ms"])
            kind = row["uplink_kind"]
        except (KeyError, ValueError):
            skipped += 1          # pre-instrumentation run or failed uplink
            continue
        perception = 0.0
        if kind == "detection":
            if timing is None:
                skipped += 1
                continue
            key = (int(row["uplink_tx_step"]), row["uplink_sender"])
            if key not in timing:
                skipped += 1
                continue
            t_detect, t_fuse, t_track, _ = timing[key]
            perception = t_detect + t_fuse + t_track
        total = (sensing_ms + perception + uplink + edge_ca + downlink +
                 actuation_ms)
        comp["sensing"].append(sensing_ms)
        comp["perception"].append(perception)
        comp["uplink"].append(uplink)
        comp["edge_ca"].append(edge_ca)
        comp["downlink"].append(downlink)
        comp["actuation"].append(actuation_ms)
        totals.append(total)
        totals_by_kind.setdefault(kind, []).append(total)
    return totals, totals_by_kind, comp, skipped


def local_warning_latency(local_warnings, timing, sensing_ms, actuation_ms):
    """Decentralized equivalent: sensing + perception + local CA + actuation
    for every local CA warning row (no network)."""
    totals = []
    if timing is None:
        return totals
    for step, cav in local_warnings:
        if (step, cav) not in timing:
            continue
        t_detect, t_fuse, t_track, t_ca = timing[(step, cav)]
        totals.append(sensing_ms + t_detect + t_fuse + t_track + t_ca +
                      actuation_ms)
    return totals


# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="Post-run CA metrics (95%% CI): detected VRUs, "
                    "misdetection probability, total warning latency.")
    parser.add_argument("run_dir", help="CA/output/run_<timestamp> directory")
    parser.add_argument("--vru-distance", type=float, default=30.0,
                        help="distance threshold [m] under which a VRU "
                             "counts as detectable by a CAV (default 30)")
    parser.add_argument("--sensing-ms", type=float, default=50.0,
                        help="sensing period [ms] added to every latency "
                             "chain (default 50 = one 20 Hz frame period)")
    parser.add_argument("--actuation-ms", type=float, default=100.0,
                        help="modelled actuation time [ms] (default 100 = "
                             "campaign actuation assumption; ~1200 for a "
                             "human driver reacting to an HMI warning)")
    args = parser.parse_args()

    run_dir = args.run_dir
    if not os.path.isfile(os.path.join(run_dir, "gt.csv")):
        sys.exit("error: %s does not look like a run directory (no gt.csv)"
                 % run_dir)

    scene = load_gt(run_dir)
    detected, local_warnings = load_metrics(run_dir)
    timing = load_timing(run_dir)
    edge_rows = load_edge_warnings(run_dir)

    lines = []
    lines.append("CA metric analysis of %s" % os.path.normpath(run_dir))
    lines.append("  VRU distance threshold: %.1f m | sensing period: %.0f ms"
                 " | actuation time: %.0f ms"
                 % (args.vru_distance, args.sensing_ms, args.actuation_ms))

    # ---- 1) detected VRUs -------------------------------------------------
    det = detection_metrics(scene, detected, args.vru_distance)
    lines.append("")
    lines.append("1) DETECTED VRUs per CAV within %.1f m "
                 "(mean +/- 95%% CI, one sample per step per CAV)"
                 % args.vru_distance)
    lines.append(fmt_mean("all CAVs", det["counts"], "VRUs"))
    lines.append(fmt_mean("VRUs in range (context)", det["in_range"], "VRUs"))
    for cav in sorted(det["counts_by_cav"]):
        lines.append(fmt_mean(cav, det["counts_by_cav"][cav], "VRUs"))

    # ---- 2) misdetection probability -------------------------------------
    k, n = det["miss"]
    p, lo, hi = wilson_ci(k, n)
    lines.append("")
    lines.append("2) MISDETECTION PROBABILITY (VRU within %.1f m of a CAV "
                 "not detected by it; 95%% Wilson CI)" % args.vru_distance)
    lines.append("  %-26s: %.4f  [%.4f, %.4f]  (%d misses / %d samples)"
                 % ("all CAVs", p, lo, hi, k, n))
    for cat in VRU_CATEGORIES:
        ck, cn = det["miss_by_cat"][cat]
        cp, clo, chi = wilson_ci(ck, cn)
        if cn:
            lines.append("  %-26s: %.4f  [%.4f, %.4f]  (%d / %d)"
                         % (cat, cp, clo, chi, ck, cn))
    for cav in sorted(det["miss_by_cav"]):
        ck, cn = det["miss_by_cav"][cav]
        cp, clo, chi = wilson_ci(ck, cn)
        lines.append("  %-26s: %.4f  [%.4f, %.4f]  (%d / %d)"
                     % (cav, cp, clo, chi, ck, cn))

    # ---- 3) total warning latency ----------------------------------------
    lines.append("")
    lines.append("3) TOTAL WARNING LATENCY (mean +/- 95% CI, per delivered "
                 "warning): sensing + perception inference + uplink + "
                 "edge CA + warning downlink + actuation")
    totals, by_kind, comp, skipped = warning_latency(
        edge_rows, timing, args.sensing_ms, args.actuation_ms)
    if totals:
        lines.append(fmt_mean("TOTAL (centralized)", totals, "ms"))
        for kind in sorted(by_kind):
            lines.append(fmt_mean("  triggered by %s" % kind,
                                  by_kind[kind], "ms"))
        lines.append("  components (mean per warning):")
        for name in ("sensing", "perception", "uplink", "edge_ca",
                     "downlink", "actuation"):
            mean, hw, _ = mean_ci(comp[name])
            lines.append("    %-12s %8.3f ms" % (name, mean))
        if skipped:
            lines.append("  (%d warning row(s) skipped: missing uplink/"
                         "timing information)" % skipped)
    elif edge_rows:
        lines.append("  edge_warnings.csv present but without the latency-"
                     "chain columns (run produced by an older version?)")
    else:
        lines.append("  no delivered edge warnings in this run "
                     "(mode 1, or none triggered)")

    local_totals = local_warning_latency(local_warnings, timing,
                                         args.sensing_ms, args.actuation_ms)
    if local_totals:
        lines.append(fmt_mean("DECENTRALIZED equivalent", local_totals, "ms"))
        lines.append("    (local CA warnings: sensing + perception + local "
                     "CA + actuation, no network)")
    elif timing is None:
        lines.append("  (timing.csv not found: perception/local components "
                     "unavailable - re-run with the current "
                     "run_ca_extended.py)")

    report = "\n".join(lines)
    print(report)
    out_path = os.path.join(run_dir, "analysis.txt")
    with open(out_path, "w") as f:
        f.write(report + "\n")
    print("\nsaved to %s" % out_path)


if __name__ == "__main__":
    main()
