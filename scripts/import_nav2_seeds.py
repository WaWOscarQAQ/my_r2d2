#!/usr/bin/env python3
"""Import input samples from the nav2-_fuzz project into my_r2d2's seed corpus.

Converts the previous fuzzer's seed assets into my_r2d2's simple on-disk
formats under config/nav2_seeds/:

  scans/*.txt       two-line payload text (7 scalars + whitespace-separated
                    ranges), the exact format r2d2_scan_bridge consumes
  schedules/*.sched one line per schedule:
                    duration_sec period_ms burst_count burst_gap_ms
                    max_publishes stamp_mode
  provenance.csv    origin path / round / sanitizer kind / scan hash

The original YAML files stay in the nav2-_fuzz checkout; re-run this script
after new interesting seeds are produced there. Content is deduplicated by
hash (scans) or by line (schedules).
"""

import argparse
import csv
import hashlib
import os
import subprocess
import sys

import yaml

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def yaml_env(key):
    helper = os.path.join(REPO_ROOT, "target", "debug", "my_r2d2")
    if os.path.exists(helper) and os.access(helper, os.X_OK):
        command = [helper, "yaml-get", key]
    else:
        command = ["cargo", "run", "--quiet", "--manifest-path", os.path.join(REPO_ROOT, "Cargo.toml"), "--", "yaml-get", key]
    return subprocess.check_output(command, text=True).strip()


DEFAULT_SOURCE = yaml_env("R2D2_FUZZ_SOURCE")

SCAN_DEFAULTS = {
    "angle_min": -3.14159,
    "angle_max": 3.14159,
    "angle_increment": 0.0174533,
    "time_increment": 0.0,
    "scan_time": 0.0,
    "range_min": 0.1,
    "range_max": 12.0,
}

SCHEDULE_DEFAULTS = {
    "duration_sec": 20.0,
    "period_ms": 50,
    "burst_count": 1,
    "burst_gap_ms": 0,
    "max_publishes": 0,
    "stamp_mode": "now",
}


def read_yaml(path):
    with open(path, encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def extract_scan(message_map):
    """Return (scalars, ranges) for the /scan entry, or None."""
    entry = message_map.get("/scan") if message_map else None
    if not entry or entry.get("type") != "sensor_msgs/msg/LaserScan":
        return None
    data = entry.get("data") or {}
    scalars = [
        data.get(key, default)
        for key, default in SCAN_DEFAULTS.items()
    ]
    ranges = list(data.get("ranges") or [])
    return scalars, ranges


def scan_text(scalars, ranges):
    """Format a scan as the two-line payload text consumed by the bridge."""
    line1 = " ".join(format(float(v), ".7g") for v in scalars)
    line2 = " ".join(format(float(v), ".7g") for v in ranges)
    return f"{line1}\n{line2}\n"


def extract_scan_schedule(event):
    """Return the /scan schedule dict (with defaults filled), or None."""
    if not event:
        return None
    topic = (event.get("topics") or {}).get("/scan")
    if topic is None:
        return None
    schedule = dict(SCHEDULE_DEFAULTS)
    schedule["duration_sec"] = float(event.get("duration_sec", 20.0))
    for key in ("period_ms", "burst_count", "burst_gap_ms", "max_publishes"):
        if topic.get(key) is not None:
            schedule[key] = int(topic[key])
    schedule["stamp_mode"] = str(topic.get("stamp_mode", "now"))
    return schedule


def schedule_line(schedule):
    return (
        f"{schedule['duration_sec']:.6g} {schedule['period_ms']} "
        f"{schedule['burst_count']} {schedule['burst_gap_ms']} "
        f"{schedule['max_publishes']} {schedule['stamp_mode']}"
    )


def scan_hash(text):
    return hashlib.sha256(text.encode("utf-8")).hexdigest()[:16]


class Importer:
    def __init__(self, source_root, output_dir):
        self.source = source_root
        self.output = output_dir
        self.scan_seen = {}
        self.schedule_seen = {}
        self.rows = []

    def add_scan(self, name, origin, source_path, scalars, ranges, meta=None):
        text = scan_text(scalars, ranges)
        digest = scan_hash(text)
        if digest in self.scan_seen:
            self.rows.append(
                [self.scan_seen[digest], origin, source_path,
                 meta or {}, "deduped", digest]
            )
            return
        path = os.path.join(self.output, "scans", name)
        with open(path, "w", encoding="utf-8") as stream:
            stream.write(text)
        self.scan_seen[digest] = name
        self.rows.append([name, origin, source_path, meta or {}, "", digest])

    def add_schedule(self, name, origin, source_path, schedule, meta=None):
        line = schedule_line(schedule)
        if line in self.schedule_seen:
            self.rows.append(
                [self.schedule_seen[line], origin, source_path,
                 meta or {}, "deduped", "-"]
            )
            return
        path = os.path.join(self.output, "schedules", name)
        with open(path, "w", encoding="utf-8") as stream:
            stream.write(line + "\n")
        self.schedule_seen[line] = name
        self.rows.append([name, origin, source_path, meta or {}, "", "-"])

    def import_content_seeds(self):
        candidates = [
            ("canonical.txt", "seed_pool/content/initial_content.yaml"),
            ("canonical_local.txt", "local_seed_pool/content/initial_content.yaml"),
        ]
        for name, rel in candidates:
            path = os.path.join(self.source, rel)
            if not os.path.isfile(path):
                continue
            doc = read_yaml(path)
            scan = extract_scan(doc.get("messages"))
            if scan is None:
                continue
            self.add_scan(name, "content", rel, scan[0], scan[1])

    def import_event_seeds(self):
        candidates = [
            ("initial", "seed_pool/event/initial_event.yaml"),
            ("initial_local", "local_seed_pool/event/initial_event.yaml"),
        ]
        for prefix, rel in candidates:
            path = os.path.join(self.source, rel)
            if not os.path.isfile(path):
                continue
            schedule = extract_scan_schedule(read_yaml(path))
            if schedule:
                self.add_schedule(
                    f"{prefix}.sched", "event", rel, schedule
                )
        for pool, prefix in (
            ("seed_pool", "event"),
            ("local_seed_pool", "local_event"),
        ):
            directory = os.path.join(self.source, pool, "interesting_event")
            if not os.path.isdir(directory):
                continue
            for filename in sorted(os.listdir(directory)):
                if not filename.endswith(".yaml"):
                    continue
                rel = f"{pool}/interesting_event/{filename}"
                schedule = extract_scan_schedule(
                    read_yaml(os.path.join(directory, filename))
                )
                if schedule:
                    stem = os.path.splitext(filename)[0].replace("event_round_", "")
                    self.add_schedule(
                        f"{prefix}_round_{stem}.sched", "event", rel, schedule
                    )

    def import_bug_candidates(self):
        for pool, prefix in (("local_seed_pool", "bug"), ("seed_pool", "bug_legacy")):
            directory = os.path.join(self.source, pool, "bug_candidates")
            if not os.path.isdir(directory):
                continue
            for round_dir in sorted(os.listdir(directory)):
                base = os.path.join(directory, round_dir)
                if not os.path.isdir(base):
                    continue
                round_num = round_dir.replace("round_", "")
                meta = {"round": round_num}
                summary_path = os.path.join(base, "bug_summary.json")
                if os.path.isfile(summary_path):
                    with open(summary_path, encoding="utf-8") as stream:
                        # The previous fuzzer writes this file with tab
                        # indentation, which the YAML scanner rejects; tabs
                        # are only layout here, so normalizing them is safe.
                        summary_text = stream.read().replace("\t", "  ")
                    summary = yaml.safe_load(summary_text) or {}
                    meta["sanitizer_kind"] = summary.get("sanitizer_kind", "")
                content_path = os.path.join(base, "content.yaml")
                if os.path.isfile(content_path):
                    content = read_yaml(content_path)
                    scan = extract_scan(content.get("messages"))
                    if scan:
                        self.add_scan(
                            f"{prefix}_round_{round_num}.txt",
                            "bug_candidate",
                            f"{pool}/bug_candidates/{round_dir}/content.yaml",
                            scan[0],
                            scan[1],
                            meta,
                        )
                event = None
                combined_path = os.path.join(base, "combined_input.yaml")
                if os.path.isfile(combined_path):
                    combined = read_yaml(combined_path)
                    event = combined.get("event")
                if event is None:
                    event_path = os.path.join(base, "event.yaml")
                    if os.path.isfile(event_path):
                        event = read_yaml(event_path)
                schedule = extract_scan_schedule(event)
                if schedule:
                    self.add_schedule(
                        f"{prefix}_round_{round_num}.sched",
                        "bug_candidate",
                        f"{pool}/bug_candidates/{round_dir}",
                        schedule,
                        meta,
                    )

    def write_provenance(self):
        path = os.path.join(self.output, "provenance.csv")
        with open(path, "w", encoding="utf-8", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(
                ["file", "origin", "source", "round", "sanitizer_kind",
                 "deduped_to", "scan_sha256"]
            )
            for row in self.rows:
                name, origin, source, meta, deduped, digest = row
                writer.writerow(
                    [name, origin, source, meta.get("round", ""),
                     meta.get("sanitizer_kind", ""), deduped, digest]
                )

    def run(self):
        os.makedirs(os.path.join(self.output, "scans"), exist_ok=True)
        os.makedirs(os.path.join(self.output, "schedules"), exist_ok=True)
        self.import_content_seeds()
        self.import_event_seeds()
        self.import_bug_candidates()
        self.write_provenance()


def main():
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", default=DEFAULT_SOURCE)
    parser.add_argument(
        "--output",
        default=os.path.join(repo_root, "config", "nav2_seeds"),
    )
    args = parser.parse_args()
    if not os.path.isdir(args.source):
        sys.exit(f"source root not found: {args.source}")
    importer = Importer(args.source, args.output)
    importer.run()
    print(
        f"imported {len(importer.scan_seen)} unique scans and "
        f"{len(importer.schedule_seen)} unique schedules into {args.output}"
    )


if __name__ == "__main__":
    main()
