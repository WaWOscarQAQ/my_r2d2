//! 序列化边界测试：针对 `SimpleSerializer` 的全部编码/解码分支与错误路径。
//!
//! 类型树取自 nav2 fuzzer 的实际目标（LaserScan / OccupancyGrid 形状），
//! 保证测试难度与真实负载一致。

use my_r2d2::interface_extractor::{Field, Primitive, TypeNode};
use my_r2d2::payload::{Error, Serializer, SimpleSerializer, Value, ValueTree};

fn round_trip(value: &ValueTree, ty: &TypeNode) -> ValueTree {
    let bytes = SimpleSerializer
        .serialize(value, ty)
        .expect("serialize failed");
    SimpleSerializer
        .deserialize(&bytes, ty)
        .expect("deserialize failed")
}

fn leaf(value: Value) -> ValueTree {
    ValueTree::Leaf(value)
}

/// sensor_msgs/LaserScan 的类型形状（nav2 costmap 的实际订阅输入）。
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

#[test]
fn round_trips_every_numeric_primitive() {
    let cases: Vec<(ValueTree, TypeNode)> = vec![
        (leaf(Value::Bool(true)), Primitive::Bool.into()),
        (leaf(Value::Bool(false)), Primitive::Bool.into()),
        (leaf(Value::I8(-5)), Primitive::I8.into()),
        (leaf(Value::I8(i8::MIN)), Primitive::I8.into()),
        (leaf(Value::U8(200)), Primitive::U8.into()),
        (leaf(Value::U8(u8::MAX)), Primitive::U8.into()),
        (leaf(Value::I16(-300)), Primitive::I16.into()),
        (leaf(Value::U16(60_000)), Primitive::U16.into()),
        (leaf(Value::I32(-100_000)), Primitive::I32.into()),
        (leaf(Value::U32(4_000_000_000)), Primitive::U32.into()),
        (leaf(Value::I64(i64::MIN)), Primitive::I64.into()),
        (leaf(Value::U64(u64::MAX)), Primitive::U64.into()),
        (leaf(Value::F32(1.5)), Primitive::F32.into()),
        (leaf(Value::F32(f32::MIN_POSITIVE)), Primitive::F32.into()),
        (leaf(Value::F64(-2.25)), Primitive::F64.into()),
        (leaf(Value::F64(f64::MAX)), Primitive::F64.into()),
    ];
    for (value, ty) in cases {
        assert_eq!(
            round_trip(&value, &ty),
            value,
            "round trip failed for {ty:?}"
        );
    }
}

#[test]
fn round_trips_string_with_length_prefix() {
    let value = leaf(Value::String("laser_frame".to_string()));
    let ty: TypeNode = Primitive::String.into();
    let bytes = SimpleSerializer.serialize(&value, &ty).unwrap();
    // u32 小端长度前缀 + UTF-8 字节
    assert_eq!(&bytes[..4], &(value_len(&value)).to_le_bytes());
    assert_eq!(&bytes[4..], b"laser_frame");
    assert_eq!(
        round_trip(&value, &ty),
        leaf(Value::String("laser_frame".to_string()))
    );
}

#[test]
fn round_trips_bounded_string_and_array_values() {
    let string_ty = TypeNode::bounded_string(3);
    let string_value = leaf(Value::String("abc".to_string()));
    assert_eq!(round_trip(&string_value, &string_ty), string_value);

    let array_ty = TypeNode::bounded_array(Primitive::U8.into(), 2);
    let array_value = ValueTree::Array(vec![leaf(Value::U8(1)), leaf(Value::U8(2))]);
    assert_eq!(round_trip(&array_value, &array_ty), array_value);
}

fn value_len(value: &ValueTree) -> u32 {
    match value {
        ValueTree::Leaf(Value::String(s)) => s.len() as u32,
        other => panic!("not a length-prefixed leaf: {other:?}"),
    }
}

#[test]
fn round_trips_laser_scan_shaped_nested_tree() {
    let ty = laser_scan_type();
    let value = ValueTree::Nested(vec![
        ValueTree::Nested(vec![
            ValueTree::Nested(vec![leaf(Value::I32(1_700_000_000)), leaf(Value::U32(500))]),
            leaf(Value::String("base_link".to_string())),
        ]),
        leaf(Value::F32(-1.57)),
        leaf(Value::F32(1.57)),
        leaf(Value::F32(0.0174533)),
        leaf(Value::F32(0.0001)),
        leaf(Value::F32(0.05)),
        leaf(Value::F32(0.12)),
        leaf(Value::F32(30.0)),
        ValueTree::Array(vec![
            leaf(Value::F32(1.0)),
            leaf(Value::F32(2.5)),
            leaf(Value::F32(2999.0)),
        ]),
        ValueTree::Array(vec![]), // 空变长数组
    ]);
    let restored = round_trip(&value, &ty);
    assert_eq!(restored, value);
}

#[test]
fn round_trips_fixed_covariance_array_from_pose_with_covariance() {
    let ty = TypeNode::fixed_array(Primitive::F64.into(), 36);
    let value = ValueTree::Array((0..36).map(|i| leaf(Value::F64(i as f64))).collect());
    assert_eq!(round_trip(&value, &ty), value);
}

/// NaN 与自身不相等（IEEE 语义），所以用位模式字节比较验证往返。
#[test]
fn round_trip_preserves_nan_bit_pattern() {
    let ty: TypeNode = Primitive::F32.into();
    let value = leaf(Value::F32(f32::NAN));
    let bytes = SimpleSerializer.serialize(&value, &ty).unwrap();
    assert_eq!(bytes, f32::NAN.to_bits().to_le_bytes());
    let restored = round_trip(&value, &ty);
    match restored {
        ValueTree::Leaf(Value::F32(v)) => assert!(v.is_nan()),
        other => panic!("expected F32 leaf, got {other:?}"),
    }
}

// ---------------------------------------------------------------------------
// 编码期类型不匹配
// ---------------------------------------------------------------------------

#[test]
fn encoding_leaf_against_wrong_primitive_reports_type_mismatch() {
    let value = leaf(Value::Bool(true));
    let ty: TypeNode = Primitive::I32.into();
    let err = SimpleSerializer.serialize(&value, &ty).unwrap_err();
    assert!(matches!(err, Error::TypeMismatch { .. }));
    assert!(err.to_string().contains("type mismatch"), "got: {err}");
}

#[test]
fn encoding_nested_with_wrong_field_count_reports_type_mismatch() {
    let ty = TypeNode::nested(vec![
        Field::new("x", Primitive::F64),
        Field::new("y", Primitive::F64),
    ]);
    let value = ValueTree::Nested(vec![leaf(Value::F64(1.0))]);
    let err = SimpleSerializer.serialize(&value, &ty).unwrap_err();
    assert!(matches!(err, Error::TypeMismatch { .. }));
    assert!(
        err.to_string().contains("nested message with 2 fields"),
        "got: {err}"
    );
}

#[test]
fn encoding_fixed_array_with_wrong_length_reports_type_mismatch() {
    let ty = TypeNode::fixed_array(Primitive::U8.into(), 3);
    let value = ValueTree::Array(vec![leaf(Value::U8(1)), leaf(Value::U8(2))]);
    let err = SimpleSerializer.serialize(&value, &ty).unwrap_err();
    assert!(matches!(err, Error::TypeMismatch { .. }));
    assert!(
        err.to_string().contains("fixed array of 3 elements"),
        "got: {err}"
    );
}

#[test]
fn encoding_array_value_against_nested_type_reports_type_mismatch() {
    let ty = TypeNode::nested(vec![Field::new("x", Primitive::F64)]);
    let value = ValueTree::Array(vec![leaf(Value::F64(1.0))]);
    let err = SimpleSerializer.serialize(&value, &ty).unwrap_err();
    assert!(matches!(err, Error::TypeMismatch { .. }));
}

#[test]
fn encoding_overlong_bounded_values_reports_type_mismatch() {
    let string_ty = TypeNode::bounded_string(3);
    let string_value = leaf(Value::String("toolong".to_string()));
    let err = SimpleSerializer
        .serialize(&string_value, &string_ty)
        .unwrap_err();
    assert!(matches!(err, Error::TypeMismatch { .. }));
    assert!(err.to_string().contains("at most 3 bytes"), "got: {err}");

    let array_ty = TypeNode::bounded_array(Primitive::U8.into(), 2);
    let array_value = ValueTree::Array(vec![
        leaf(Value::U8(1)),
        leaf(Value::U8(2)),
        leaf(Value::U8(3)),
    ]);
    let err = SimpleSerializer
        .serialize(&array_value, &array_ty)
        .unwrap_err();
    assert!(matches!(err, Error::TypeMismatch { .. }));
    assert!(err.to_string().contains("at most 2 elements"), "got: {err}");
}

// ---------------------------------------------------------------------------
// 解码期格式错误
// ---------------------------------------------------------------------------

#[test]
fn decoding_truncated_numeric_reports_missing_bytes() {
    let ty: TypeNode = Primitive::F64.into();
    let err = SimpleSerializer.deserialize(&[0u8, 1, 2], &ty).unwrap_err();
    assert!(matches!(err, Error::Malformed(_)));
    assert!(
        err.to_string().contains("expected 8 bytes, 3 remaining"),
        "got: {err}"
    );
}

#[test]
fn decoding_truncated_length_prefix_reports_missing_bytes() {
    let ty: TypeNode = Primitive::String.into();
    let err = SimpleSerializer.deserialize(&[2u8, 0], &ty).unwrap_err();
    assert!(
        err.to_string().contains("expected 4 bytes, 2 remaining"),
        "got: {err}"
    );
}

#[test]
fn decoding_string_shorter_than_declared_length_reports_missing_bytes() {
    let ty: TypeNode = Primitive::String.into();
    // 声明长度 10，实际只跟了 2 个字节
    let bytes = [10u8, 0, 0, 0, b'a', b'b'];
    let err = SimpleSerializer.deserialize(&bytes, &ty).unwrap_err();
    assert!(
        err.to_string().contains("expected 10 bytes, 2 remaining"),
        "got: {err}"
    );
}

#[test]
fn decoding_non_utf8_string_reports_malformed() {
    let ty: TypeNode = Primitive::String.into();
    let bytes = [2u8, 0, 0, 0, 0xFF, 0xFE];
    let err = SimpleSerializer.deserialize(&bytes, &ty).unwrap_err();
    assert!(err.to_string().contains("not valid UTF-8"), "got: {err}");
}

#[test]
fn decoding_fixed_array_with_wrong_count_reports_malformed() {
    let ty = TypeNode::fixed_array(Primitive::U8.into(), 4);
    // 长度前缀是 3，类型声明是 4
    let bytes = [3u8, 0, 0, 0, 1, 2, 3];
    let err = SimpleSerializer.deserialize(&bytes, &ty).unwrap_err();
    assert!(matches!(err, Error::Malformed(_)));
    assert!(
        err.to_string().contains("expected 4, found 3"),
        "got: {err}"
    );
}

#[test]
fn decoding_overlong_bounded_values_reports_type_mismatch() {
    let string_ty = TypeNode::bounded_string(3);
    let string_bytes = [4u8, 0, 0, 0, b'a', b'b', b'c', b'd'];
    let err = SimpleSerializer
        .deserialize(&string_bytes, &string_ty)
        .unwrap_err();
    assert!(matches!(err, Error::TypeMismatch { .. }));
    assert!(err.to_string().contains("at most 3 bytes"), "got: {err}");

    let array_ty = TypeNode::bounded_array(Primitive::U8.into(), 2);
    let array_bytes = [3u8, 0, 0, 0, 1, 2, 3];
    let err = SimpleSerializer
        .deserialize(&array_bytes, &array_ty)
        .unwrap_err();
    assert!(matches!(err, Error::TypeMismatch { .. }));
    assert!(err.to_string().contains("at most 2 elements"), "got: {err}");
}

#[test]
fn decoding_nested_tree_stops_at_first_truncated_field() {
    // stamp { i32 sec, u32 nanosec }：sec 读掉 4 字节后 nanosec 只剩 1 字节
    let ty = TypeNode::nested(vec![
        Field::new("sec", Primitive::I32),
        Field::new("nanosec", Primitive::U32),
    ]);
    let bytes = [1u8, 0, 0, 0, 9];
    let err = SimpleSerializer.deserialize(&bytes, &ty).unwrap_err();
    assert!(
        err.to_string().contains("expected 4 bytes, 1 remaining"),
        "got: {err}"
    );
}

#[test]
fn trailing_bytes_after_full_decode_are_rejected() {
    let ty: TypeNode = Primitive::I32.into();
    let bytes = [7u8, 0, 0, 0, 0xFF];
    let err = SimpleSerializer.deserialize(&bytes, &ty).unwrap_err();
    assert!(err.to_string().contains("1 trailing bytes"), "got: {err}");
}

#[test]
fn decoding_bool_treats_any_nonzero_byte_as_true() {
    let ty: TypeNode = Primitive::Bool.into();
    assert_eq!(
        SimpleSerializer.deserialize(&[5u8], &ty).unwrap(),
        leaf(Value::Bool(true))
    );
}

// ---------------------------------------------------------------------------
// 错误 Display
// ---------------------------------------------------------------------------

#[test]
fn error_display_messages_are_stable() {
    assert_eq!(
        Error::TypeMismatch {
            expected: "int32".into(),
            found: "bool".into(),
        }
        .to_string(),
        "type mismatch: expected int32, found bool"
    );
    assert_eq!(
        Error::Malformed("boom".into()).to_string(),
        "malformed payload bytes: boom"
    );
    assert_eq!(
        Error::Unsupported("actions".into()).to_string(),
        "unsupported: actions"
    );
}
