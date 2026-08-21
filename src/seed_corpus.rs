//! Seed corpus loading for the nav2 e2e harness.
//!
//! The previous fuzzer (nav2-_fuzz) stored its input samples as YAML trees.
//! To keep this crate free of YAML dependencies, `scripts/import_nav2_seeds.py`
//! converts them once into two plain formats that this module reads:
//!
//! - `scans/*.txt`: the two-line payload text already consumed by
//!   `r2d2_scan_bridge` (7 scalars on the first line, whitespace-separated
//!   ranges afterwards). Parsed back into the `LaserScan` `ValueTree` shape
//!   here so the pool can mutate it like any generated payload.
//! - `schedules/*.sched`: one line per publish schedule:
//!   `duration_sec period_ms burst_count burst_gap_ms max_publishes stamp_mode`
//!
//! Corpus content deduplication happens in the import script (by content
//! hash / schedule line); loading here is order-stable (sorted file names).

use crate::interface_extractor::{Interface, TypeNode};
use crate::payload::{Payload, Serializer, SimpleSerializer, Value, ValueTree};
use std::collections::HashSet;
use std::fs;
use std::path::{Path, PathBuf};

/// A per-round publish schedule extracted from the previous fuzzer's event
/// seeds. `max_publishes == 0` means "unbounded" (bridge convention).
#[derive(Debug, Clone, PartialEq)]
pub struct Schedule {
    pub duration_sec: f64,
    pub period_ms: u64,
    pub burst_count: u32,
    pub burst_gap_ms: u64,
    pub max_publishes: u64,
    pub stamp_mode: String,
}

/// Parses the bridge payload text (7 scalars + ranges) into the canonical
/// `LaserScan` `ValueTree` shape. The header stamp is synthesized as zero and
/// `frame_id` as `"laser_frame"`, matching what `r2d2_scan_bridge` publishes;
/// `intensities` is an empty array (the text format does not carry it).
pub fn parse_scan_text(text: &str) -> Result<ValueTree, String> {
    let floats: Vec<f32> = text
        .split_whitespace()
        .map(|token| {
            token
                .parse::<f32>()
                .map_err(|_| format!("invalid float in scan seed: {token:?}"))
        })
        .collect::<Result<_, _>>()?;
    if floats.len() < 7 {
        return Err(format!(
            "scan seed needs 7 scalars, found {} floats",
            floats.len()
        ));
    }
    let header = ValueTree::Nested(vec![
        ValueTree::Nested(vec![
            ValueTree::Leaf(Value::I32(0)),
            ValueTree::Leaf(Value::U32(0)),
        ]),
        ValueTree::Leaf(Value::String("laser_frame".to_string())),
    ]);
    let mut fields = Vec::with_capacity(10);
    fields.push(header);
    fields.extend(floats[..7].iter().map(|v| ValueTree::Leaf(Value::F32(*v))));
    fields.push(ValueTree::Array(
        floats[7..]
            .iter()
            .map(|v| ValueTree::Leaf(Value::F32(*v)))
            .collect(),
    ));
    fields.push(ValueTree::Array(Vec::new()));
    Ok(ValueTree::Nested(fields))
}

/// Parses one schedule line (`duration_sec period_ms burst_count burst_gap_ms
/// max_publishes stamp_mode`).
pub fn parse_schedule(line: &str) -> Result<Schedule, String> {
    let parts: Vec<&str> = line.split_whitespace().collect();
    if parts.len() != 6 {
        return Err(format!(
            "schedule line needs 6 fields, found {}: {line:?}",
            parts.len()
        ));
    }
    let number = |token: &str, what: &str| -> Result<u64, String> {
        token
            .parse::<u64>()
            .map_err(|_| format!("invalid {what} in schedule: {token:?}"))
    };
    let duration_sec = parts[0]
        .parse::<f64>()
        .map_err(|_| format!("invalid duration in schedule: {:?}", parts[0]))?;
    let burst_count = number(parts[2], "burst_count")? as u32;
    Ok(Schedule {
        duration_sec,
        period_ms: number(parts[1], "period_ms")?,
        burst_count,
        burst_gap_ms: number(parts[3], "burst_gap_ms")?,
        max_publishes: number(parts[4], "max_publishes")?,
        stamp_mode: parts[5].to_string(),
    })
}

/// Loads all `*.txt` scan seeds from `dir`, deduplicated by serialized
/// content, as pool-ready payloads bound to `interface`. Serialization also
/// validates that every seed matches the interface's type shape.
pub fn load_scan_seeds(dir: &Path, interface: &Interface) -> Result<Vec<Payload>, String> {
    let mut paths = text_files(dir, "txt")?;
    paths.sort();
    let ty = TypeNode::Nested(interface.fields.clone());
    let mut seen = HashSet::new();
    let mut seeds = Vec::new();
    for path in paths {
        let text = fs::read_to_string(&path).map_err(|e| format!("{}: {e}", path.display()))?;
        let value = parse_scan_text(&text)?;
        let bytes = SimpleSerializer
            .serialize(&value, &ty)
            .map_err(|e| format!("{}: {e}", path.display()))?;
        if !seen.insert(bytes.clone()) {
            continue;
        }
        let mut payload = Payload::new(interface.name.clone(), interface.kind, value, 0);
        payload.serialized = bytes;
        seeds.push(payload);
    }
    Ok(seeds)
}

/// Loads all `*.sched` schedules from `dir` as `(file_stem, schedule)` pairs
/// in sorted file-name order.
pub fn load_schedules(dir: &Path) -> Result<Vec<(String, Schedule)>, String> {
    let mut paths = text_files(dir, "sched")?;
    paths.sort();
    let mut schedules = Vec::new();
    for path in paths {
        let text = fs::read_to_string(&path).map_err(|e| format!("{}: {e}", path.display()))?;
        for line in text.lines().map(str::trim).filter(|l| !l.is_empty()) {
            let schedule = parse_schedule(line)?;
            let stem = path
                .file_stem()
                .and_then(|s| s.to_str())
                .unwrap_or("schedule")
                .to_string();
            schedules.push((stem, schedule));
        }
    }
    Ok(schedules)
}

fn text_files(dir: &Path, extension: &str) -> Result<Vec<PathBuf>, String> {
    let entries = fs::read_dir(dir).map_err(|e| format!("{}: {e}", dir.display()))?;
    let mut paths = Vec::new();
    for entry in entries {
        let path = entry.map_err(|e| e.to_string())?.path();
        if path.extension().is_some_and(|ext| ext == extension) {
            paths.push(path);
        }
    }
    Ok(paths)
}
