# -*- coding: utf-8 -*-
"""Structural validation for CA raw-data campaigns (no metric computation)."""

import argparse
import csv
import hashlib
import json
import os
import sys


REQUIRED_GT = {"step", "sim_time", "id", "name", "category", "x", "y",
               "z", "vx", "vy", "vz", "ax", "ay", "az", "speed",
               "heading"}
REQUIRED_METRICS = {"step", "sim_time", "cav", "track_id",
                    "edge_object_id", "est_vx", "est_vy",
                    "ego_gt_distance", "gt_name", "in_range", "ca_warning",
                    "ca_type"}
REQUIRED_GT_CA = {"step", "sim_time", "cav", "cav_actor_id", "other_id",
                  "other_name", "other_category", "distance_m", "t2c",
                  "s2c", "in_range", "gt_ca_warning", "ca_type"}
REQUIRED_ASSOCIATIONS = {"step", "sim_time", "cav", "cav_station_id",
                         "track_id", "edge_object_id", "gt_id", "gt_name",
                         "gt_category", "association_distance_m"}
REQUIRED_TIMING = {"step", "sim_time", "cav", "t_detect_ms", "t_fuse_ms",
                   "t_track_ms", "t_ca_ms"}
REQUIRED_PACKETS = {"packet_id", "kind", "sender", "receiver", "size_bytes",
                    "tx_step", "tx_sim_t", "app_processing_ms", "status",
                    "latency_ms"}
REQUIRED_RISKS = {"risk_id", "step", "sim_t", "station_a", "type_a", "x_a",
                  "y_a", "station_b", "type_b", "x_b", "y_b",
                  "target_station", "target_cav", "other_station",
                  "collision_type", "t2c", "s2c", "uplink_kind",
                  "uplink_packet_id", "uplink_latency_ms", "uplink_tx_step",
                  "uplink_sender", "edge_ca_ms", "warning_packet_id"}
REQUIRED_WARNINGS = {"rx_step", "sim_t", "cav", "other_station",
                     "other_type", "collision_type", "t2c", "s2c",
                     "latency_ms", "packet_id", "uplink_kind",
                     "uplink_packet_id", "uplink_latency_ms",
                     "uplink_tx_step", "uplink_sender", "edge_ca_ms"}
REQUIRED_EDGE_TIMING = {"step", "sim_t", "trigger_kind", "station_id",
                        "ca_ms", "n_warnings"}
REQUIRED_EDGE_UPDATES = {"rx_step", "sim_t", "kind", "tx_step", "sender",
                         "local_object_id", "canonical_object_id", "x", "y",
                         "uplink_packet_id"}
REQUIRED_STATIONS = {"station_id", "name", "actor_id", "category",
                     "connected"}

# (policy index, campaign sub-directory, app mode)
ALL_POLICIES = ((0, "policy_0_onboard", 1),
                (1, "policy_1_collaborative", 2),
                (2, "policy_2_vam", 3),
                (3, "policy_3_vam_only", 4))
# Campaigns collected before policy 3 existed hold only the first three.
DEFAULT_POLICIES = {0, 1, 2}


def fail(message):
    raise RuntimeError(message)


def rows(path):
    if not os.path.isfile(path):
        fail("missing raw-data file: %s" % path)
    with open(path, newline="") as handle:
        reader = csv.DictReader(handle)
        if not reader.fieldnames:
            fail("raw-data file has no CSV header: %s" % path)
        result = list(reader)
    for line_no, row in enumerate(result, 2):
        if None in row or any(value is None for value in row.values()):
            fail("malformed CSV row width in %s at line %d" %
                 (path, line_no))
    return result


def require_headers(path, required):
    with open(path, newline="") as handle:
        header = set(next(csv.reader(handle), []))
    missing = required - header
    if missing:
        fail("%s is missing columns: %s" %
             (path, ", ".join(sorted(missing))))


def validate_run(run_dir, app_mode, expected_steps):
    success = os.path.join(run_dir, "_SUCCESS")
    if not os.path.isfile(success):
        fail("run has no _SUCCESS marker: %s" % run_dir)
    metadata_path = os.path.join(run_dir, "run_metadata.json")
    with open(metadata_path) as handle:
        metadata = json.load(handle)
    if metadata.get("status") != "collected":
        fail("run metadata is not in collected state")
    if int(metadata.get("app_mode", -1)) != int(app_mode):
        fail("run metadata app_mode mismatch")
    if int(metadata.get("total_steps", -1)) != int(expected_steps):
        fail("run metadata step count mismatch")
    if metadata.get("raw_only") is not True:
        fail("campaign run was not collected with --raw-only")
    thresholds = metadata.get("collision_thresholds", {})
    if thresholds.get("check_rear_end") is not False:
        fail("run metadata does not enforce cross-road-only collision checks")
    config_collision = metadata.get("config", {}).get("collision", {})
    if config_collision.get("check_rear_end") is not False:
        fail("run configuration does not disable rear-end collision checks")
    for forbidden in ("summary.txt", "analysis.txt"):
        if os.path.exists(os.path.join(run_dir, forbidden)):
            fail("raw-only run unexpectedly produced %s" % forbidden)

    gt_path = os.path.join(run_dir, "gt.csv")
    metrics_path = os.path.join(run_dir, "metrics.csv")
    require_headers(gt_path, REQUIRED_GT)
    require_headers(metrics_path, REQUIRED_METRICS)
    gt = rows(gt_path)
    metrics = rows(metrics_path)
    timing_path = os.path.join(run_dir, "timing.csv")
    gt_ca_path = os.path.join(run_dir, "gt_ca.csv")
    associations_path = os.path.join(run_dir, "track_associations.csv")
    stations_path = os.path.join(run_dir, "stations.csv")
    require_headers(timing_path, REQUIRED_TIMING)
    require_headers(gt_ca_path, REQUIRED_GT_CA)
    require_headers(associations_path, REQUIRED_ASSOCIATIONS)
    require_headers(stations_path, REQUIRED_STATIONS)
    timing = rows(timing_path)
    gt_ca = rows(gt_ca_path)
    associations = rows(associations_path)
    stations = rows(stations_path)
    if not stations:
        fail("stations.csv contains no actors")

    gt_by_step = {}
    for row in gt:
        gt_by_step.setdefault(int(row["step"]), set()).add(row["name"])
    if set(gt_by_step) != set(range(expected_steps)):
        fail("gt.csv does not contain exactly the expected steps")
    actor_names = gt_by_step[0]
    if not actor_names or "" in actor_names or any(names != actor_names
                              for names in gt_by_step.values()):
        fail("gt.csv actor-name set changes during the run")
    timing_keys = [(int(row["step"]), row["cav"]) for row in timing]
    expected_timing_keys = [(step, "Cav%d" % cav)
                            for step in range(expected_steps)
                            for cav in range(1, 5)]
    if sorted(timing_keys) != sorted(expected_timing_keys):
        fail("timing.csv does not contain exactly four CAV rows per step")
    expected_gt_ca = expected_steps * 4 * (len(actor_names) - 1)
    if len(gt_ca) != expected_gt_ca:
        fail("gt_ca.csv row count %d != expected %d"
             % (len(gt_ca), expected_gt_ca))
    if any(row["gt_ca_warning"] == "1" and
           row["ca_type"] != "cross-road" for row in gt_ca):
        fail("gt_ca.csv contains a rear-end collision warning")
    if thresholds.get("apply_isinrange") is True and any(
            row["gt_ca_warning"] == "1" and row["in_range"] != "1"
            for row in gt_ca):
        fail("gt_ca.csv contains an out-of-range collision warning")
    if not metrics:
        fail("metrics.csv contains no confirmed tracks")
    if any(row["ca_warning"] == "1" and
           row["ca_type"] != "cross-road" for row in metrics):
        fail("metrics.csv contains a rear-end collision warning")
    if thresholds.get("apply_isinrange") is True and any(
            row["ca_warning"] == "1" and row["in_range"] != "1"
            for row in metrics):
        fail("metrics.csv contains an out-of-range collision warning")
    metric_keys = {(row["step"], row["cav"], row["track_id"]): row
                   for row in metrics}
    association_keys = {(row["step"], row["cav"], row["track_id"]): row
                        for row in associations}
    if len(metric_keys) != len(metrics) or len(association_keys) != \
            len(associations) or set(metric_keys) != set(association_keys):
        fail("metrics.csv and track_associations.csv do not form a 1:1 join")
    for key, metric in metric_keys.items():
        if metric["edge_object_id"] != association_keys[key]["edge_object_id"]:
            fail("track association edge-object ID mismatch for %r" % (key,))

    connected_station_ids = {row["station_id"] for row in stations
                             if row["connected"] == "1"}
    station_by_name = {row["name"]: row["station_id"] for row in stations
                       if row["station_id"]}
    for cav in ("Cav1", "Cav2", "Cav3", "Cav4"):
        if not station_by_name.get(cav):
            fail("stations.csv lacks connected %s" % cav)

    if app_mode == 1:
        for name in ("v2x_packets.csv", "edge_warnings.csv",
                     "edge_risks.csv", "edge_ca_timing.csv"):
            if os.path.exists(os.path.join(run_dir, name)):
                fail("mode 1 unexpectedly produced %s" % name)
    else:
        packet_path = os.path.join(run_dir, "v2x_packets.csv")
        warning_path = os.path.join(run_dir, "edge_warnings.csv")
        risk_path = os.path.join(run_dir, "edge_risks.csv")
        edge_timing_path = os.path.join(run_dir, "edge_ca_timing.csv")
        update_path = os.path.join(run_dir, "edge_ldm_updates.csv")
        require_headers(packet_path, REQUIRED_PACKETS)
        require_headers(warning_path, REQUIRED_WARNINGS)
        require_headers(risk_path, REQUIRED_RISKS)
        require_headers(edge_timing_path, REQUIRED_EDGE_TIMING)
        require_headers(update_path, REQUIRED_EDGE_UPDATES)
        packets = rows(packet_path)
        warnings = rows(warning_path)
        risks = rows(risk_path)
        edge_timing = rows(edge_timing_path)
        edge_updates = rows(update_path)
        if any(row["collision_type"] != "Cross-Road-Collision"
               for row in warnings):
            fail("edge_warnings.csv contains a rear-end collision warning")
        if any(row["collision_type"] != "Cross-Road-Collision"
               for row in risks):
            fail("edge_risks.csv contains a rear-end collision risk")
        packet_by_id = {row["packet_id"]: row for row in packets
                        if row["packet_id"]}
        kinds = [row["kind"] for row in packets]
        if "detection" not in kinds:
            fail("network mode contains no detection packets")
        if app_mode == 2 and "vam" in kinds:
            fail("mode 2 unexpectedly contains VAM packets")
        if app_mode in (3, 4) and "vam" not in kinds:
            fail("mode %d contains no VAM packets" % app_mode)
        if app_mode == 4:
            # V2X-only policy: no CAV-perceived object may ever reach the
            # edge LDM, and the edge may only pair a CAV with a VRU.
            if any(row["kind"] != "vam" for row in edge_updates):
                fail("mode 4 edge LDM was updated by a CAV detection")
            cav_station_ids = {station_by_name[cav]
                               for cav in ("Cav1", "Cav2", "Cav3", "Cav4")}
            if any(row["station_a"] in cav_station_ids and
                   row["station_b"] in cav_station_ids for row in risks):
                fail("mode 4 edge_risks.csv contains a CAV-to-CAV risk")
            if any(row["other_station"] in cav_station_ids
                   for row in warnings):
                fail("mode 4 edge_warnings.csv warns about another CAV")
        association_local_keys = {
            (row["step"], row["cav"], row["edge_object_id"])
            for row in associations}
        canonical_ids = set(connected_station_ids)
        for update in edge_updates:
            packet = packet_by_id.get(update["uplink_packet_id"])
            if packet is None or packet["status"] != "delivered":
                fail("edge LDM update does not join to a delivered uplink")
            canonical_ids.add(update["canonical_object_id"])
            if update["kind"] == "detection" and (
                    update["tx_step"], update["sender"],
                    update["local_object_id"]) not in association_local_keys:
                fail("edge detection update does not join to a local track")
        timing_key_set = {(row["step"], row["cav"]) for row in timing}
        edge_timing_keys = {(row["step"], row["station_id"])
                            for row in edge_timing}
        for warning in warnings:
            packet = packet_by_id.get(warning["packet_id"])
            if packet is None or packet["kind"] != "warning" or \
                    packet["status"] != "delivered":
                fail("edge warning does not join to a delivered warning packet")
            uplink = packet_by_id.get(warning["uplink_packet_id"])
            if uplink is None:
                fail("edge warning does not join to its uplink packet")
            if warning["uplink_kind"] == "detection" and (
                    warning["uplink_tx_step"], warning["uplink_sender"]) \
                    not in timing_key_set:
                fail("detection-triggered edge warning lacks processing timing")
            if warning["cav"] not in station_by_name:
                fail("edge warning target does not join to stations.csv")
            if warning["other_station"] not in canonical_ids:
                fail("edge warning object does not join to station/track data")
        for risk in risks:
            packet = packet_by_id.get(risk["warning_packet_id"])
            if packet is None or packet["kind"] != "warning":
                fail("edge risk does not join to its warning packet")
            uplink = packet_by_id.get(risk["uplink_packet_id"])
            if uplink is None or uplink["status"] != "delivered":
                fail("edge risk does not join to a delivered uplink packet")
            if risk["target_station"] != station_by_name.get(
                    risk["target_cav"]):
                fail("edge risk target does not join to stations.csv")
            if risk["station_a"] not in canonical_ids or \
                    risk["station_b"] not in canonical_ids:
                fail("edge risk objects do not join to station/track data")
            if (risk["step"], risk["station_a"]) not in edge_timing_keys and \
                    (risk["step"], risk["station_b"]) not in edge_timing_keys:
                fail("edge risk does not join to edge CA timing")

    required_paths = [gt_path, metrics_path, timing_path, gt_ca_path,
                      associations_path, stations_path, metadata_path]
    if app_mode >= 2:
        required_paths += [packet_path, warning_path, risk_path,
                           edge_timing_path, update_path,
                           os.path.join(run_dir, "bridge_send.csv"),
                           os.path.join(run_dir, "bridge_recv.csv")]
    marker_mtime = os.path.getmtime(success)
    if any(os.path.getmtime(path) > marker_mtime for path in required_paths):
        fail("_SUCCESS predates a required raw-data file")
    return metadata


def canonical_gt_digest(run_dir):
    digest = hashlib.sha256()
    canonical = []
    for row in rows(os.path.join(run_dir, "gt.csv")):
        canonical.append(tuple(row[key] for key in
                               ("step", "name", "category", "x", "y", "z",
                                "vx", "vy", "vz", "ax", "ay", "az",
                                "speed", "heading")))
    for record in sorted(canonical):
        digest.update(("\x1f".join(record) + "\n").encode())
    return digest.hexdigest()


def validate_campaign(campaign_dir, expected_steps, policies=None,
                      models=None):
    """Validate the cells of the selected policies and prove they all share
    the same ground-truth trajectories. `policies` is the set of policy
    indices to check (default: the three original ones, so campaigns
    collected before policy 3 existed keep validating unchanged) and
    `models` the YOLO capacities collected (default: all three)."""
    wanted = DEFAULT_POLICIES if policies is None else set(policies)
    capacities = ("n", "m", "x") if not models else tuple(models)
    unknown_models = set(capacities) - {"n", "m", "x"}
    if unknown_models:
        fail("unknown model capacity: %s" % ", ".join(sorted(unknown_models)))
    selected = tuple(policy for policy in ALL_POLICIES
                     if policy[0] in wanted)
    unknown = wanted - {policy[0] for policy in ALL_POLICIES}
    if unknown:
        fail("unknown policy index: %s" %
             ", ".join(str(index) for index in sorted(unknown)))
    if not selected:
        fail("no known policy selected")
    digests = {}
    for _, policy_dir, app_mode in selected:
        for model in capacities:
            run_dir = os.path.join(campaign_dir, policy_dir, model)
            metadata = validate_run(run_dir, app_mode, expected_steps)
            if metadata.get("model") != model:
                fail("model metadata mismatch in %s" % run_dir)
            digests[os.path.relpath(run_dir, campaign_dir)] = \
                canonical_gt_digest(run_dir)
    unique = set(digests.values())
    if len(unique) != 1:
        detail = "\n".join("  %s %s" % item
                           for item in sorted(digests.items()))
        fail("ground-truth trajectories differ across campaign cells:\n" +
             detail)
    print("raw campaign valid: %d cells (policies %s), identical GT digest %s"
          % (len(digests),
             ", ".join(str(policy[0]) for policy in selected),
             next(iter(unique))))


def main():
    parser = argparse.ArgumentParser(
        description="Validate raw CA files without computing metrics")
    sub = parser.add_subparsers(dest="command")
    run_parser = sub.add_parser("run")
    run_parser.add_argument("run_dir")
    run_parser.add_argument("--app-mode", type=int, required=True)
    run_parser.add_argument("--steps", type=int, required=True)
    campaign_parser = sub.add_parser("campaign")
    campaign_parser.add_argument("campaign_dir")
    campaign_parser.add_argument("--steps", type=int, required=True)
    campaign_parser.add_argument(
        "--models", default=None,
        help="comma-separated YOLO capacities to validate (default: n,m,x)")
    campaign_parser.add_argument(
        "--policies", default=None,
        help="comma-separated policy indices to validate "
             "(default: %s; policy 3 = policy_3_vam_only)"
             % ",".join(str(index) for index in sorted(DEFAULT_POLICIES)))
    args = parser.parse_args()
    try:
        if args.command == "run":
            validate_run(args.run_dir, args.app_mode, args.steps)
            print("raw run valid: %s" % args.run_dir)
        elif args.command == "campaign":
            selected = None if args.policies is None else \
                {int(index) for index in args.policies.split(",") if index}
            capacities = None if args.models is None else \
                [name for name in args.models.split(",") if name]
            validate_campaign(args.campaign_dir, args.steps, selected,
                              capacities)
        else:
            parser.error("a command is required")
    except (OSError, ValueError, RuntimeError) as error:
        print("raw-data validation failed: %s" % error, file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
