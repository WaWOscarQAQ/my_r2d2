//! 种子语料加载测试：文本扫描解析、schedule 解析、仓库内 fixture 加载与去重。

use my_r2d2::interface_extractor::{Field, Interface, Kind, Primitive, TypeNode};
use my_r2d2::payload::{Payload, Serializer, SimpleSerializer, Value, ValueTree};
use my_r2d2::seed_corpus::{load_scan_seeds, load_schedules, parse_scan_text, parse_schedule};

const CANONICAL: &str = "\
-3.14159 3.14159 0.0174533 0 0 0.1 12
0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5
";

/// sensor_msgs/LaserScan 的类型形状（与 tests/payload.rs 保持一致）。
fn laser_scan_type() -> TypeNode {
    let header = TypeNode::nested(vec![
        Field::new(
            "stamp",
            TypeNode::nested(vec![
                Field::new("sec", Primitive::I32),
                Field::new("nanosec", Primitive::U32),
            ]),
        ),
        Field::new("frame_id", Primitive::String),
    ]);
    TypeNode::nested(vec![
        Field::new("header", header),
        Field::new("angle_min", Primitive::F32),
        Field::new("angle_max", Primitive::F32),
        Field::new("angle_increment", Primitive::F32),
        Field::new("time_increment", Primitive::F32),
        Field::new("scan_time", Primitive::F32),
        Field::new("range_min", Primitive::F32),
        Field::new("range_max", Primitive::F32),
        Field::new("ranges", TypeNode::array(Primitive::F32.into())),
        Field::new("intensities", TypeNode::array(Primitive::F32.into())),
    ])
}

fn laser_scan_interface() -> Interface {
    match laser_scan_type() {
        TypeNode::Nested(fields) => Interface::new("LaserScan", Kind::Topic, fields),
        other => panic!("expected nested type, got {other:?}"),
    }
}

#[test]
#[allow(clippy::approx_constant)] // -3.14159 是语料数据值，不是 PI
fn parses_canonical_scan_into_laser_scan_shape() {
    let value = parse_scan_text(CANONICAL).expect("parse canonical scan");
    let ValueTree::Nested(fields) = value else {
        panic!("expected nested value");
    };
    assert_eq!(fields.len(), 10);
    let ValueTree::Nested(header) = &fields[0] else {
        panic!("expected nested header");
    };
    let ValueTree::Nested(stamp) = &header[0] else {
        panic!("expected nested stamp");
    };
    assert_eq!(stamp[0], ValueTree::Leaf(Value::I32(0)));
    assert_eq!(stamp[1], ValueTree::Leaf(Value::U32(0)));
    assert_eq!(
        header[1],
        ValueTree::Leaf(Value::String("laser_frame".to_string()))
    );
    assert_eq!(fields[1], ValueTree::Leaf(Value::F32(-3.14159)));
    assert_eq!(fields[7], ValueTree::Leaf(Value::F32(12.0)));
    let ValueTree::Array(ranges) = &fields[8] else {
        panic!("expected ranges array");
    };
    assert_eq!(ranges.len(), 20);
    assert_eq!(ranges[0], ValueTree::Leaf(Value::F32(0.5)));
    let ValueTree::Array(intensities) = &fields[9] else {
        panic!("expected intensities array");
    };
    assert!(intensities.is_empty());
}

#[test]
fn parsed_scan_round_trips_through_serializer() {
    let ty = laser_scan_type();
    let value = parse_scan_text(CANONICAL).expect("parse");
    let bytes = SimpleSerializer.serialize(&value, &ty).expect("serialize");
    let decoded = SimpleSerializer
        .deserialize(&bytes, &ty)
        .expect("deserialize");
    assert_eq!(decoded, value);
}

#[test]
fn parse_scan_text_rejects_short_and_malformed_input() {
    assert!(parse_scan_text("1 2 3").is_err(), "fewer than 7 floats");
    assert!(
        parse_scan_text("1 2 3 4 5 6 nope\n0.5").is_err(),
        "non-float token"
    );
    assert!(parse_scan_text("").is_err(), "empty text");
}

#[test]
fn parse_schedule_reads_all_fields() {
    let schedule = parse_schedule("22 50 2 5 120 now").expect("parse schedule");
    assert_eq!(schedule.duration_sec, 22.0);
    assert_eq!(schedule.period_ms, 50);
    assert_eq!(schedule.burst_count, 2);
    assert_eq!(schedule.burst_gap_ms, 5);
    assert_eq!(schedule.max_publishes, 120);
    assert_eq!(schedule.stamp_mode, "now");
}

#[test]
fn parse_schedule_rejects_malformed_lines() {
    assert!(parse_schedule("22 50 2 5 120").is_err(), "missing field");
    assert!(
        parse_schedule("abc 50 2 5 120 now").is_err(),
        "bad duration"
    );
    assert!(parse_schedule("").is_err(), "empty line");
}

#[test]
fn loads_committed_seed_corpus() {
    let fixture_root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("config/nav2_seeds");
    let interface = laser_scan_interface();
    let seeds =
        load_scan_seeds(&fixture_root.join("scans"), &interface).expect("load committed scans");
    assert!(!seeds.is_empty(), "corpus scans must be committed");
    for seed in &seeds {
        assert_eq!(seed.interface_id, "LaserScan");
        assert!(
            !seed.serialized.is_empty(),
            "seeds are serialized at load time"
        );
    }
    let schedules =
        load_schedules(&fixture_root.join("schedules")).expect("load committed schedules");
    assert!(!schedules.is_empty(), "corpus schedules must be committed");
}

#[test]
fn load_scan_seeds_deduplicates_identical_content() {
    let dir = std::env::temp_dir().join(format!("r2d2_seed_corpus_{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).expect("create temp dir");
    std::fs::write(dir.join("a.txt"), CANONICAL).expect("write a.txt");
    std::fs::write(dir.join("b.txt"), CANONICAL).expect("write b.txt");
    let varied = "-3.14159 3.572294 0.0174533 0 0 0.1 12\n0.5 0.5 0.5\n";
    std::fs::write(dir.join("c.txt"), varied).expect("write c.txt");

    let interface = laser_scan_interface();
    let seeds: Vec<Payload> = load_scan_seeds(&dir, &interface).expect("load seeds");
    assert_eq!(seeds.len(), 2, "identical content must be deduplicated");
    std::fs::remove_dir_all(&dir).expect("cleanup temp dir");
}
