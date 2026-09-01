//! 变异算子测试。
//!
//! 策略：
//! 1. 通过 `operators_per_type` 只放行单个算子 + 固定种子 RNG，实现确定性断言；
//! 2. 用 nav2 fuzzer 的真实目标（LaserScan / Twist 类型树）做类型合规性
//!    集成测试——变异后的值树必须仍能对类型树序列化成功。

use my_r2d2::interface_extractor::{Extractor, Field, FileExtractor, Literal, Primitive, TypeNode};
use my_r2d2::mutation::{Mutator, OpKind, OperatorsPerType, generate_value};
use my_r2d2::payload::{Serializer, SimpleSerializer, Value, ValueTree};
use my_r2d2::payload_generator::{GeneratorConfig, ValueRange, ValueRanges};
use rand::{SeedableRng, rngs::StdRng};
use std::path::{Path, PathBuf};

fn fixtures_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/ros_interfaces")
}

/// 单一算子配置：只放行 `primitive`/`string`/`array`/`nested` 中指定的算子。
fn forced_config(
    primitive: &[OpKind],
    string: &[OpKind],
    array: &[OpKind],
    nested: &[OpKind],
) -> GeneratorConfig {
    GeneratorConfig {
        mutation_energy: 1,
        // 深度 0：只收集根路径，保证单算子命中根节点，断言确定
        max_recursion_depth: 0,
        operators_per_type: OperatorsPerType {
            primitive: primitive.to_vec(),
            string: string.to_vec(),
            array: array.to_vec(),
            nested: nested.to_vec(),
        },
        ..Default::default()
    }
}

fn seeded_rng(seed: u64) -> StdRng {
    StdRng::seed_from_u64(seed)
}

fn leaf(value: Value) -> ValueTree {
    ValueTree::Leaf(value)
}

// ---------------------------------------------------------------------------
// generate_value：全 primitive 生成
// ---------------------------------------------------------------------------

#[test]
fn generate_value_produces_matching_leaf_for_every_primitive() {
    let config = GeneratorConfig::default();
    let mut rng = seeded_rng(42);
    type PrimitiveCheck = fn(&Value) -> bool;
    let cases: Vec<(Primitive, PrimitiveCheck)> = vec![
        (Primitive::Bool, |v| matches!(v, Value::Bool(_))),
        (
            Primitive::I8,
            |v| matches!(v, Value::I8(n) if (-1000..=1000).contains(&(*n as i64))),
        ),
        (Primitive::U8, |v| matches!(v, Value::U8(_))),
        (Primitive::I16, |v| matches!(v, Value::I16(_))),
        (Primitive::U16, |v| matches!(v, Value::U16(_))),
        (Primitive::I32, |v| matches!(v, Value::I32(_))),
        (Primitive::U32, |v| matches!(v, Value::U32(_))),
        (Primitive::I64, |v| matches!(v, Value::I64(_))),
        (Primitive::U64, |v| matches!(v, Value::U64(_))),
        (
            Primitive::F32,
            |v| matches!(v, Value::F32(x) if (-1000.0..=1000.0).contains(&(*x as f64))),
        ),
        (
            Primitive::F64,
            |v| matches!(v, Value::F64(x) if (-1000.0..=1000.0).contains(x)),
        ),
        (
            Primitive::String,
            |v| matches!(v, Value::String(s) if s.len() <= 64 && s.chars().all(|c| (0x20..=0x7e).contains(&(c as u32)))),
        ),
    ];
    for (primitive, check) in cases {
        let value = generate_value(&primitive.into(), &mut rng, &config);
        let ValueTree::Leaf(inner) = &value else {
            panic!("{primitive:?} should generate a leaf, got {value:?}");
        };
        assert!(
            check(inner),
            "{primitive:?} generated out-of-range value {inner:?}"
        );
    }
}

#[test]
fn generate_value_respects_custom_ranges_and_array_bounds() {
    let config = GeneratorConfig {
        per_type_value_ranges: {
            let mut ranges = ValueRanges::default();
            ranges.insert(Primitive::I32, ValueRange::new(-3.0, 3.0));
            ranges.insert(Primitive::String, ValueRange::new(2.0, 2.0));
            ranges
        },
        array_len_range: 5..=5,
        ..Default::default()
    };
    let mut rng = seeded_rng(7);

    let value = generate_value(&Primitive::I32.into(), &mut rng, &config);
    assert!(matches!(value, ValueTree::Leaf(Value::I32(n)) if (-3..=3).contains(&n)));

    let value = generate_value(&Primitive::String.into(), &mut rng, &config);
    assert!(matches!(value, ValueTree::Leaf(Value::String(s)) if s.len() == 2));

    let value = generate_value(&TypeNode::array(Primitive::F64.into()), &mut rng, &config);
    assert!(matches!(value, ValueTree::Array(items) if items.len() == 5));

    let value = generate_value(
        &TypeNode::fixed_array(Primitive::U8.into(), 3),
        &mut rng,
        &config,
    );
    assert!(matches!(value, ValueTree::Array(items) if items.len() == 3));

    let ty = TypeNode::nested(vec![
        Field::new("x", Primitive::F64),
        Field::new("ys", TypeNode::fixed_array(Primitive::U8.into(), 2)),
    ]);
    assert!(
        matches!(generate_value(&ty, &mut rng, &config), ValueTree::Nested(items) if items.len() == 2)
    );
}

#[test]
fn generate_value_uses_field_defaults_before_sampling() {
    let ty = TypeNode::nested(vec![
        Field::new("x", Primitive::I32).with_default(Literal::I32(5)),
        Field::new("label", TypeNode::bounded_string(4))
            .with_default(Literal::String("ab".to_string())),
    ]);

    let value = generate_value(&ty, &mut seeded_rng(99), &GeneratorConfig::default());
    assert_eq!(
        value,
        ValueTree::Nested(vec![
            leaf(Value::I32(5)),
            leaf(Value::String("ab".to_string()))
        ])
    );
}

#[test]
fn generate_value_uses_nested_defaults_before_sampling() {
    let ty = TypeNode::nested(vec![
        Field::new(
            "stamp",
            TypeNode::nested(vec![
                Field::new("sec", Primitive::I32).with_default(Literal::I32(1)),
                Field::new("nanosec", Primitive::U32).with_default(Literal::U32(2)),
            ]),
        )
        .with_default(Literal::Nested(vec![Literal::I32(3), Literal::U32(4)])),
        Field::new(
            "history",
            TypeNode::array(TypeNode::nested(vec![
                Field::new("sec", Primitive::I32),
                Field::new("nanosec", Primitive::U32),
            ])),
        )
        .with_default(Literal::Array(vec![
            Literal::Nested(vec![Literal::I32(5), Literal::U32(6)]),
            Literal::Nested(vec![Literal::I32(7), Literal::U32(8)]),
        ])),
    ]);

    let value = generate_value(&ty, &mut seeded_rng(101), &GeneratorConfig::default());
    assert_eq!(
        value,
        ValueTree::Nested(vec![
            ValueTree::Nested(vec![leaf(Value::I32(3)), leaf(Value::U32(4))]),
            ValueTree::Array(vec![
                ValueTree::Nested(vec![leaf(Value::I32(5)), leaf(Value::U32(6))]),
                ValueTree::Nested(vec![leaf(Value::I32(7)), leaf(Value::U32(8))]),
            ]),
        ])
    );
}

#[test]
fn official_jazzy_quaternion_defaults_flow_into_generation() {
    let share = Path::new("/opt/ros/jazzy/share");
    let pose = share.join("geometry_msgs/msg/Pose.msg");
    if !pose.exists() {
        return;
    }

    let interface = FileExtractor::new(vec![pose], vec![share.to_path_buf()])
        .extract()
        .unwrap()
        .remove(0);
    let value = generate_value(
        &TypeNode::nested(interface.fields.clone()),
        &mut seeded_rng(202),
        &GeneratorConfig::default(),
    );

    let ValueTree::Nested(fields) = value else {
        panic!("expected Pose to generate as nested value");
    };
    let ValueTree::Nested(orientation) = &fields[1] else {
        panic!("expected Pose.orientation to be nested");
    };
    assert_eq!(
        orientation,
        &vec![
            leaf(Value::F64(0.0)),
            leaf(Value::F64(0.0)),
            leaf(Value::F64(0.0)),
            leaf(Value::F64(1.0)),
        ]
    );
}

#[test]
fn generate_value_respects_bounded_string_and_array_limits() {
    let mut config = GeneratorConfig::default();
    config
        .per_type_value_ranges
        .insert(Primitive::String, ValueRange::new(5.0, 5.0));
    config.array_len_range = 4..=4;

    let ty = TypeNode::nested(vec![
        Field::new("name", TypeNode::bounded_string(3)),
        Field::new("samples", TypeNode::bounded_array(Primitive::U8.into(), 2)),
    ]);
    let value = generate_value(&ty, &mut seeded_rng(123), &config);
    let ValueTree::Nested(fields) = value else {
        panic!("expected nested value");
    };

    assert!(matches!(&fields[0], ValueTree::Leaf(Value::String(text)) if text.len() <= 3));
    assert!(matches!(&fields[1], ValueTree::Array(items) if items.len() <= 2));
}

// ---------------------------------------------------------------------------
// Flip：按位翻转 / 取反
// ---------------------------------------------------------------------------

#[test]
fn flip_inverts_bool_leaf() {
    let config = forced_config(&[OpKind::Flip], &[], &[], &[]);
    let mutator = Mutator::new(config);
    let out = mutator.mutate(
        &leaf(Value::Bool(true)),
        &Primitive::Bool.into(),
        &mut seeded_rng(1),
    );
    assert_eq!(out, leaf(Value::Bool(false)));
}

#[test]
fn flip_negates_f64_leaf() {
    let config = forced_config(&[OpKind::Flip], &[], &[], &[]);
    let mutator = Mutator::new(config);
    let out = mutator.mutate(
        &leaf(Value::F64(2.5)),
        &Primitive::F64.into(),
        &mut seeded_rng(1),
    );
    assert_eq!(out, leaf(Value::F64(-2.5)));
}

#[test]
fn flip_flips_exactly_one_bit_of_i32() {
    let config = forced_config(&[OpKind::Flip], &[], &[], &[]);
    let mutator = Mutator::new(config);
    let out = mutator.mutate(
        &leaf(Value::I32(0)),
        &Primitive::I32.into(),
        &mut seeded_rng(3),
    );
    assert!(matches!(out, ValueTree::Leaf(Value::I32(v)) if v.count_ones() == 1));
}

#[test]
fn flip_preserves_signedness_of_i8() {
    let config = forced_config(&[OpKind::Flip], &[], &[], &[]);
    let mutator = Mutator::new(config);
    let out = mutator.mutate(
        &leaf(Value::I8(0)),
        &Primitive::I8.into(),
        &mut seeded_rng(5),
    );
    assert!(matches!(out, ValueTree::Leaf(Value::I8(v)) if v > 0 && v.count_ones() == 1));
}

// ---------------------------------------------------------------------------
// Boundary：边界值替换
// ---------------------------------------------------------------------------

#[test]
fn boundary_replaces_i32_with_a_range_endpoint() {
    let config = forced_config(&[OpKind::Boundary], &[], &[], &[]);
    let mutator = Mutator::new(config);
    let out = mutator.mutate(
        &leaf(Value::I32(7)),
        &Primitive::I32.into(),
        &mut seeded_rng(11),
    );
    assert!(
        matches!(out, ValueTree::Leaf(Value::I32(v)) if [-1000, 0, 1000].contains(&v)),
        "got {out:?}"
    );
}

#[test]
fn boundary_on_u8_uses_valid_type_endpoints() {
    let config = forced_config(&[OpKind::Boundary], &[], &[], &[]);
    let mutator = Mutator::new(config);
    for seed in 0..32 {
        let out = mutator.mutate(
            &leaf(Value::U8(50)),
            &Primitive::U8.into(),
            &mut seeded_rng(seed),
        );
        assert!(
            matches!(out, ValueTree::Leaf(Value::U8(v)) if [0, 255].contains(&v)),
            "seed {seed} got {out:?}"
        );
    }
}

#[test]
fn boundary_returns_value_unchanged_when_all_candidates_equal_current() {
    let mut config = forced_config(&[OpKind::Boundary], &[], &[], &[]);
    config
        .per_type_value_ranges
        .insert(Primitive::I32, ValueRange::new(0.0, 0.0));
    let mutator = Mutator::new(config);
    // min == 0 == max == 当前值 -> 候选为空，原样返回
    let out = mutator.mutate(
        &leaf(Value::I32(0)),
        &Primitive::I32.into(),
        &mut seeded_rng(1),
    );
    assert_eq!(out, leaf(Value::I32(0)));
}

// ---------------------------------------------------------------------------
// Resize：字符串 / 字节序列 / 变长数组伸缩
// ---------------------------------------------------------------------------

#[test]
fn resize_truncates_string_to_configured_length() {
    let mut config = forced_config(&[], &[OpKind::Resize], &[], &[]);
    config
        .per_type_value_ranges
        .insert(Primitive::String, ValueRange::new(3.0, 3.0));
    let mutator = Mutator::new(config);
    let out = mutator.mutate(
        &leaf(Value::String("map://home/robot".to_string())),
        &Primitive::String.into(),
        &mut seeded_rng(1),
    );
    assert!(matches!(out, ValueTree::Leaf(Value::String(s)) if s.len() == 3));
}

#[test]
fn resize_respects_bounded_byte_array_limit() {
    let mut config = forced_config(&[], &[], &[OpKind::Resize], &[]);
    config.array_len_range = 5..=5;
    let mutator = Mutator::new(config);
    let out = mutator.mutate(
        &ValueTree::Array(vec![leaf(Value::U8(1)), leaf(Value::U8(2))]),
        &TypeNode::bounded_array(Primitive::U8.into(), 3),
        &mut seeded_rng(1),
    );
    assert!(matches!(out, ValueTree::Array(items) if items.len() == 3));
}

#[test]
fn resize_resizes_variable_array_to_configured_length() {
    let mut config = forced_config(&[], &[], &[OpKind::Resize], &[]);
    config.array_len_range = 4..=4;
    let mutator = Mutator::new(config);
    let out = mutator.mutate(
        &ValueTree::Array(vec![leaf(Value::F32(1.0)), leaf(Value::F32(2.0))]),
        &TypeNode::array(Primitive::F32.into()),
        &mut seeded_rng(1),
    );
    let ValueTree::Array(items) = out else {
        panic!("expected array");
    };
    assert_eq!(items.len(), 4);
    assert_eq!(items[0], leaf(Value::F32(1.0)));
}

#[test]
fn resize_cannot_apply_to_fixed_array_and_returns_value_unchanged() {
    let config = forced_config(&[], &[], &[OpKind::Resize], &[]);
    let mutator = Mutator::new(config);
    let value = ValueTree::Array(vec![
        leaf(Value::U8(1)),
        leaf(Value::U8(2)),
        leaf(Value::U8(3)),
    ]);
    let out = mutator.mutate(
        &value.clone(),
        &TypeNode::fixed_array(Primitive::U8.into(), 3),
        &mut seeded_rng(1),
    );
    assert_eq!(out, value);
}

// ---------------------------------------------------------------------------
// ByteEdit：单字节替换
// ---------------------------------------------------------------------------

#[test]
fn byte_edit_keeps_empty_string_empty() {
    let config = forced_config(&[], &[OpKind::ByteEdit], &[], &[]);
    let mutator = Mutator::new(config);
    let out = mutator.mutate(
        &leaf(Value::String(String::new())),
        &Primitive::String.into(),
        &mut seeded_rng(1),
    );
    assert_eq!(out, leaf(Value::String(String::new())));
}

#[test]
fn byte_edit_changes_one_byte_and_preserves_length() {
    let config = forced_config(&[], &[], &[OpKind::ByteEdit], &[]);
    let mutator = Mutator::new(config);
    let value = ValueTree::Array(vec![leaf(Value::U8(10)), leaf(Value::U8(20)), leaf(Value::U8(30))]);
    let out = mutator.mutate(
        &value,
        &TypeNode::array(Primitive::U8.into()),
        &mut seeded_rng(2),
    );
    let (ValueTree::Array(before), ValueTree::Array(after)) = (value, out) else { panic!("expected arrays") };
    assert_eq!(before.len(), after.len());
    assert_eq!(before.iter().zip(after).filter(|(a, b)| a != &b).count(), 1);
}

// ---------------------------------------------------------------------------
// Resample 与结构不变量
// ---------------------------------------------------------------------------

#[test]
fn resample_regenerates_type_conformant_subtree() {
    let config = forced_config(&[], &[], &[], &[OpKind::Resample]);
    let mutator = Mutator::new(config);
    let ty = TypeNode::nested(vec![
        Field::new("x", Primitive::F64),
        Field::new("y", Primitive::F64),
    ]);
    let value = ValueTree::Nested(vec![leaf(Value::F64(1.0)), leaf(Value::F64(2.0))]);
    let out = mutator.mutate(&value, &ty, &mut seeded_rng(9));
    // 重新生成的子树必须仍能序列化（形状合规）
    SimpleSerializer
        .serialize(&out, &ty)
        .expect("resampled tree must stay type-conformant");
}

#[test]
fn mutate_returns_value_unchanged_when_shape_mismatches_type() {
    // Leaf 值配 Nested 类型：pick_operator 无适用算子，提前退出
    let config = GeneratorConfig::default();
    let mutator = Mutator::new(config);
    let ty = TypeNode::nested(vec![Field::new("x", Primitive::F64)]);
    let value = leaf(Value::Bool(true));
    let out = mutator.mutate(&value, &ty, &mut seeded_rng(1));
    assert_eq!(out, value);
}

#[test]
fn mutate_is_deterministic_for_a_fixed_seed() {
    let config = GeneratorConfig {
        mutation_energy: 8,
        ..Default::default()
    };
    let ty = TypeNode::nested(vec![
        Field::new("text", Primitive::String),
        Field::new("items", TypeNode::array(Primitive::U32.into())),
    ]);
    let value = generate_value(&ty, &mut seeded_rng(100), &config);

    let mutator = Mutator::new(config);
    let out_a = mutator.mutate(&value, &ty, &mut seeded_rng(777));
    let out_b = mutator.mutate(&value, &ty, &mut seeded_rng(777));
    assert_eq!(out_a, out_b);
}

// ---------------------------------------------------------------------------
// 集成测试：真实 nav2 类型上的变异合规性
// ---------------------------------------------------------------------------

/// 从 fixtures 解析 sensor_msgs/LaserScan（nav2 costmap 的实际订阅输入），
/// 生成种子值并连续变异，断言每一代都保持类型合规（可序列化 + 可往返）。
#[test]
fn mutating_real_laser_scan_payloads_stays_type_conformant() {
    let scan = fixtures_root().join("sensor_msgs/msg/LaserScan.msg");
    let interface = FileExtractor::new(vec![scan], vec![fixtures_root()])
        .extract()
        .unwrap()
        .remove(0);
    let ty = TypeNode::nested(interface.fields.clone());

    let config = GeneratorConfig::default();
    let mut rng = seeded_rng(2024);
    let mut value = generate_value(&ty, &mut rng, &config);
    let mutator = Mutator::new(config);

    for round in 0..50 {
        value = mutator.mutate(&value, &ty, &mut rng);
        let bytes = SimpleSerializer
            .serialize(&value, &ty)
            .unwrap_or_else(|e| panic!("round {round}: mutated value broke type conformance: {e}"));
        let restored = SimpleSerializer
            .deserialize(&bytes, &ty)
            .unwrap_or_else(|e| panic!("round {round}: mutated bytes no longer decode: {e}"));
        assert_eq!(restored, value, "round {round}: round trip diverged");
    }
}

/// 变异真实 nav2_msgs/LoadMap 响应形状（OccupancyGrid 深层嵌套 + 变长 int8 数组），
/// 覆盖 apply_at 在多层数组/嵌套路径上的递归分发。
#[test]
fn mutating_load_map_response_traverses_deep_paths() {
    let load_map = fixtures_root().join("nav2_msgs/srv/LoadMap.srv");
    let interface = FileExtractor::new(vec![load_map], vec![fixtures_root()])
        .extract()
        .unwrap()
        .remove(0);
    let response = interface
        .service
        .as_ref()
        .expect("service missing")
        .response
        .clone();
    let ty = TypeNode::nested(response);

    let config = GeneratorConfig {
        mutation_energy: 16, // 深树需要更多能量才可能触达叶子
        max_recursion_depth: 16,
        ..Default::default()
    };
    let mut rng = seeded_rng(8);
    let mut value = generate_value(&ty, &mut rng, &config);
    let mutator = Mutator::new(config.clone());

    let mut changed = false;
    for round in 0..30 {
        let next = mutator.mutate(&value, &ty, &mut rng);
        if next != value {
            changed = true;
        }
        value = next;
        let bytes = SimpleSerializer
            .serialize(&value, &ty)
            .unwrap_or_else(|e| panic!("round {round}: deep mutation broke conformance: {e}"));
        assert_eq!(
            SimpleSerializer.deserialize(&bytes, &ty).unwrap(),
            value,
            "round {round}: round trip diverged"
        );
    }
    assert!(
        changed,
        "mutation should change at least one round of the deep tree"
    );
}
