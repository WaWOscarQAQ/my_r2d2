use my_r2d2::interface_extractor::{Extractor, FileExtractor, Kind, Literal, Primitive, TypeNode};
use std::path::Path;

fn fixture(path: &str) -> std::path::PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures/ros_interfaces")
        .join(path)
}

#[test]
fn parses_real_geometry_msgs_twist_shape_and_dependency() {
    let twist = fixture("geometry_msgs/msg/Twist.msg");
    let interface = FileExtractor::from_file(twist).extract().unwrap().remove(0);

    assert_eq!(interface.name, "Twist");
    assert_eq!(interface.kind, Kind::Topic);
    assert_eq!(interface.fields.len(), 2);
    assert_eq!(interface.fields[0].name, "linear");
    assert_eq!(interface.fields[1].name, "angular");
    assert!(matches!(interface.fields[0].ty, TypeNode::Nested(_)));
    assert!(matches!(interface.fields[1].ty, TypeNode::Nested(_)));
    assert_eq!(interface.data_files.len(), 2);

    let TypeNode::Nested(fields) = &interface.fields[0].ty else {
        panic!("linear must be a nested Vector3");
    };
    assert_eq!(
        fields
            .iter()
            .map(|field| field.name.as_str())
            .collect::<Vec<_>>(),
        ["x", "y", "z"]
    );
    assert!(
        fields
            .iter()
            .all(|field| field.ty == TypeNode::Primitive(Primitive::F64))
    );
}

#[test]
fn parses_real_service_request_and_response() {
    let service = fixture("example_interfaces/srv/AddTwoInts.srv");
    let interface = FileExtractor::from_file(service)
        .extract()
        .unwrap()
        .remove(0);

    assert_eq!(interface.name, "AddTwoInts");
    assert_eq!(interface.kind, Kind::Service);
    assert_eq!(interface.fields.len(), 2);
    assert_eq!(interface.fields[0].name, "a");
    assert_eq!(interface.fields[1].name, "b");

    let service = interface.service.expect("service request/response missing");
    assert_eq!(service.request.len(), 2);
    assert_eq!(service.response.len(), 1);
    assert_eq!(service.response[0].name, "sum");
    assert_eq!(service.response[0].ty, TypeNode::Primitive(Primitive::I64));
}

#[test]
fn parses_arrays_and_ignores_comments() {
    let path = fixture("geometry_msgs/msg/Vector3.msg");
    let interface = FileExtractor::from_file(path).extract().unwrap().remove(0);
    assert_eq!(interface.fields.len(), 3);
}

/// Search root mimicking a ROS share directory so that cross-package
/// references like `nav_msgs/OccupancyGrid` resolve via search_paths.
fn fixtures_root() -> std::path::PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/ros_interfaces")
}

fn jazzy_share_root() -> std::path::PathBuf {
    Path::new("/opt/ros/jazzy/share").to_path_buf()
}

fn nested<'a>(ty: &'a TypeNode) -> &'a [my_r2d2::interface_extractor::Field] {
    let TypeNode::Nested(fields) = ty else {
        panic!("expected nested node, got {ty:?}");
    };
    fields
}

fn find<'a>(
    fields: &'a [my_r2d2::interface_extractor::Field],
    name: &str,
) -> &'a my_r2d2::interface_extractor::Field {
    fields
        .iter()
        .find(|field| field.name == name)
        .unwrap_or_else(|| panic!("field {name:?} not found"))
}

fn find_constant<'a>(
    interface: &'a my_r2d2::interface_extractor::Interface,
    name: &str,
) -> &'a my_r2d2::interface_extractor::Constant {
    interface.data_files[0]
        .constants
        .iter()
        .find(|constant| constant.name == name)
        .unwrap_or_else(|| panic!("constant {name:?} not found"))
}

/// nav2_msgs/srv/LoadMap —— nav2 fuzzer 的实际目标。
/// 覆盖：跨包解析、三层以上递归嵌套、空请求字段、变长数组。
#[test]
fn parses_nav2_load_map_deep_service_tree() {
    let load_map = fixture("nav2_msgs/srv/LoadMap.srv");
    let extractor = FileExtractor::new(vec![load_map], vec![fixtures_root()]);
    let interface = extractor.extract().unwrap().remove(0);

    assert_eq!(interface.name, "LoadMap");
    assert_eq!(interface.kind, Kind::Service);

    // request: string map_url
    let service = interface.service.as_ref().expect("service missing");
    assert_eq!(service.request.len(), 1);
    assert_eq!(service.request[0].name, "map_url");
    assert_eq!(
        service.request[0].ty,
        TypeNode::Primitive(Primitive::String)
    );

    // response: nav_msgs/OccupancyGrid map; uint8 result
    assert_eq!(service.response.len(), 2);
    assert_eq!(
        find(&service.response, "result").ty,
        TypeNode::Primitive(Primitive::U8)
    );

    // map -> [header, info, data]
    let grid = nested(&find(&service.response, "map").ty);
    assert_eq!(grid.len(), 3);

    // header -> [stamp (Time), frame_id]
    let header = nested(&find(grid, "header").ty);
    assert_eq!(
        find(header, "frame_id").ty,
        TypeNode::Primitive(Primitive::String)
    );
    let stamp = nested(&find(header, "stamp").ty);
    assert_eq!(stamp[0].ty, TypeNode::Primitive(Primitive::I32)); // int32 sec
    assert_eq!(stamp[1].ty, TypeNode::Primitive(Primitive::U32)); // uint32 nanosec

    // info (MapMetaData) -> [map_load_time, resolution, width, height, origin]
    let info = nested(&find(grid, "info").ty);
    assert_eq!(info.len(), 5);
    assert_eq!(
        find(info, "resolution").ty,
        TypeNode::Primitive(Primitive::F32)
    );

    // origin (Pose) -> [position (Point), orientation (Quaternion)]
    let origin = nested(&find(info, "origin").ty);
    assert_eq!(nested(&find(origin, "position").ty).len(), 3); // x, y, z
    assert_eq!(nested(&find(origin, "orientation").ty).len(), 4); // x, y, z, w

    // data -> variable-length int8 array
    match &find(grid, "data").ty {
        TypeNode::Array(element, None) => assert_eq!(**element, TypeNode::Primitive(Primitive::I8)),
        other => panic!("expected variable array, got {other:?}"),
    }

    // 溯源：自身 + OccupancyGrid + MapMetaData + Header + Time + Pose + Point + Quaternion
    assert_eq!(interface.data_files.len(), 8);
    assert!(interface.data_files[0].name.ends_with("LoadMap.srv"));
}

/// sensor_msgs/LaserScan —— nav2 costmap 的实际订阅输入。
#[test]
fn parses_sensor_msgs_laser_scan_topic() {
    let scan = fixture("sensor_msgs/msg/LaserScan.msg");
    let extractor = FileExtractor::new(vec![scan], vec![fixtures_root()]);
    let interface = extractor.extract().unwrap().remove(0);

    assert_eq!(interface.name, "LaserScan");
    assert_eq!(interface.kind, Kind::Topic);
    assert_eq!(interface.fields.len(), 10);

    // 8 个 float32 标量
    for name in [
        "angle_min",
        "angle_max",
        "angle_increment",
        "time_increment",
        "scan_time",
        "range_min",
        "range_max",
    ] {
        assert_eq!(
            find(&interface.fields, name).ty,
            TypeNode::Primitive(Primitive::F32)
        );
    }
    // 两个变长 float32 数组
    for name in ["ranges", "intensities"] {
        match &find(&interface.fields, name).ty {
            TypeNode::Array(element, None) => {
                assert_eq!(**element, TypeNode::Primitive(Primitive::F32))
            }
            other => panic!("expected variable array, got {other:?}"),
        }
    }
    // Header 经 search_paths 跨包解析
    let header = nested(&find(&interface.fields, "header").ty);
    assert_eq!(header.len(), 2);
    // 自身 + Header + Time
    assert_eq!(interface.data_files.len(), 3);
}

/// geometry_msgs/PoseWithCovarianceStamped —— amcl 初始位姿输入。
/// 覆盖：定长数组 float64[36]。
#[test]
fn parses_pose_with_covariance_stamped_fixed_array() {
    let stamped = fixture("geometry_msgs/msg/PoseWithCovarianceStamped.msg");
    let extractor = FileExtractor::new(vec![stamped], vec![fixtures_root()]);
    let interface = extractor.extract().unwrap().remove(0);

    assert_eq!(interface.name, "PoseWithCovarianceStamped");

    // pose -> [pose (Pose), covariance (float64[36])]
    let pose = nested(&find(&interface.fields, "pose").ty);
    match &find(pose, "covariance").ty {
        TypeNode::Array(element, Some(36)) => {
            assert_eq!(**element, TypeNode::Primitive(Primitive::F64))
        }
        other => panic!("expected fixed array of 36, got {other:?}"),
    }
}

/// nav2_msgs/srv/ClearEntireCostmap —— 空请求/空响应的边界情况。
#[test]
fn parses_nav2_clear_costmap_empty_sections() {
    let clear = fixture("nav2_msgs/srv/ClearEntireCostmap.srv");
    let extractor = FileExtractor::new(vec![clear], vec![fixtures_root()]);
    let interface = extractor.extract().unwrap().remove(0);

    assert_eq!(interface.kind, Kind::Service);
    assert!(interface.fields.is_empty());
    let service = interface.service.as_ref().expect("service missing");
    assert!(service.request.is_empty());
    assert!(service.response.is_empty());
}

// ---------------------------------------------------------------------------
// 错误分支：损坏的 fixture，每个对应解析器里一条显式报错路径。
// ---------------------------------------------------------------------------

fn extract_err(path: &str) -> String {
    let extractor = FileExtractor::new(vec![fixture(path)], vec![fixtures_root()]);
    extractor.extract().unwrap_err().to_string()
}

#[test]
fn missing_file_reports_canonicalize_error() {
    let err = extract_err("broken/msg/DoesNotExist.msg");
    assert!(err.contains("canonicalize"), "got: {err}");
}

#[test]
fn non_msg_srv_extension_is_rejected() {
    let err = extract_err("broken/NotInterface.txt");
    assert!(err.contains("is not a .msg or .srv file"), "got: {err}");
}

#[test]
fn parses_official_pointcloud2_shape() {
    let share = jazzy_share_root();
    let pointcloud = share.join("sensor_msgs/msg/PointCloud2.msg");
    if !pointcloud.exists() {
        return;
    }

    let extractor = FileExtractor::new(vec![pointcloud], vec![share]);
    let interface = extractor.extract().unwrap().remove(0);

    assert_eq!(interface.name, "PointCloud2");
    assert_eq!(interface.kind, Kind::Topic);
    assert_eq!(interface.fields.len(), 9);

    let TypeNode::Array(element, None) = &find(&interface.fields, "fields").ty else {
        panic!("PointCloud2.fields must be a variable-length PointField array");
    };
    let point_fields = nested(element);
    assert_eq!(
        point_fields
            .iter()
            .map(|field| field.name.as_str())
            .collect::<Vec<_>>(),
        ["name", "offset", "datatype", "count"]
    );
}

#[test]
fn parses_official_quaternion_shape() {
    let share = jazzy_share_root();
    let quaternion = share.join("geometry_msgs/msg/Quaternion.msg");
    if !quaternion.exists() {
        return;
    }

    let extractor = FileExtractor::new(vec![quaternion], vec![share]);
    let interface = extractor.extract().unwrap().remove(0);

    assert_eq!(interface.name, "Quaternion");
    assert_eq!(interface.fields.len(), 4);
    assert!(
        interface
            .fields
            .iter()
            .all(|field| field.ty == TypeNode::Primitive(Primitive::F64))
    );
}

#[test]
fn missing_field_name_is_rejected() {
    let err = extract_err("broken/msg/MissingName.msg");
    assert!(err.contains("missing field name"), "got: {err}");
}

#[test]
fn malformed_array_bound_is_rejected() {
    let err = extract_err("broken/msg/BadArray.msg");
    assert!(err.contains("unsupported array bound"), "got: {err}");
}

#[test]
fn parses_bounded_string_fixture() {
    let interface = FileExtractor::from_file(fixture("broken/msg/Bounded.msg"))
        .extract()
        .unwrap()
        .remove(0);

    let field = find(&interface.fields, "name");
    assert_eq!(field.ty, TypeNode::bounded_string(10));
    assert_eq!(field.default_value, None);
}

#[test]
fn parses_default_value_fixture() {
    let interface = FileExtractor::from_file(fixture("broken/msg/Defaults.msg"))
        .extract()
        .unwrap()
        .remove(0);

    let field = find(&interface.fields, "x");
    assert_eq!(field.ty, TypeNode::Primitive(Primitive::I32));
    assert_eq!(field.default_value, Some(Literal::I32(5)));
}

#[test]
fn parses_constant_fixture() {
    let interface = FileExtractor::from_file(fixture("broken/msg/Constants.msg"))
        .extract()
        .unwrap()
        .remove(0);

    assert!(interface.fields.is_empty());
    assert_eq!(interface.data_files[0].constants.len(), 1);
    let constant = find_constant(&interface, "X");
    assert_eq!(constant.ty, TypeNode::Primitive(Primitive::I32));
    assert_eq!(constant.value, Literal::I32(1));
}

#[test]
fn parses_nested_defaults_and_nested_array_defaults() {
    let interface = FileExtractor::new(
        vec![fixture("broken/msg/NestedDefaults.msg")],
        vec![fixtures_root()],
    )
    .extract()
    .unwrap()
    .remove(0);

    let stamp = find(&interface.fields, "stamp");
    assert_eq!(
        stamp.default_value,
        Some(Literal::Nested(vec![Literal::I32(1), Literal::U32(2)]))
    );

    let history = find(&interface.fields, "history");
    assert_eq!(
        history.default_value,
        Some(Literal::Array(vec![
            Literal::Nested(vec![Literal::I32(3), Literal::U32(4)]),
            Literal::Nested(vec![Literal::I32(5), Literal::U32(6)]),
        ]))
    );
}

#[test]
fn parses_nested_constant_fixture() {
    let interface = FileExtractor::new(
        vec![fixture("broken/msg/NestedConstants.msg")],
        vec![fixtures_root()],
    )
    .extract()
    .unwrap()
    .remove(0);

    assert!(interface.fields.is_empty());
    let constant = find_constant(&interface, "ZERO");
    assert_eq!(
        constant.value,
        Literal::Nested(vec![Literal::I32(0), Literal::U32(0)])
    );
}

#[test]
fn parses_official_test_msgs_defaults_constants_and_bounded_sequences() {
    let share = jazzy_share_root();

    let defaults = share.join("test_msgs/msg/Defaults.msg");
    if defaults.exists() {
        let interface = FileExtractor::new(vec![defaults], vec![share.clone()])
            .extract()
            .unwrap()
            .remove(0);
        assert_eq!(
            find(&interface.fields, "bool_value").default_value,
            Some(Literal::Bool(true))
        );
        assert_eq!(
            find(&interface.fields, "int32_value").default_value,
            Some(Literal::I32(-30_000))
        );
        assert_eq!(
            find(&interface.fields, "uint64_value").default_value,
            Some(Literal::U64(50_000_000))
        );
    }

    let constants = share.join("test_msgs/msg/Constants.msg");
    if constants.exists() {
        let interface = FileExtractor::new(vec![constants], vec![share.clone()])
            .extract()
            .unwrap()
            .remove(0);
        assert!(interface.fields.is_empty());
        assert_eq!(
            find_constant(&interface, "BOOL_CONST").value,
            Literal::Bool(true)
        );
        assert_eq!(
            find_constant(&interface, "UINT64_CONST").value,
            Literal::U64(50_000_000)
        );
    }

    let bounded = share.join("test_msgs/msg/BoundedSequences.msg");
    if bounded.exists() {
        let interface = FileExtractor::new(vec![bounded], vec![share])
            .extract()
            .unwrap()
            .remove(0);
        assert_eq!(
            find(&interface.fields, "bool_values").ty,
            TypeNode::bounded_array(Primitive::Bool.into(), 3)
        );
        assert_eq!(
            find(&interface.fields, "string_values_default").default_value,
            Some(Literal::Array(vec![
                Literal::String(String::new()),
                Literal::String("max value".to_string()),
                Literal::String("min value".to_string()),
            ]))
        );
        assert_eq!(
            find(&interface.fields, "alignment_check").ty,
            TypeNode::Primitive(Primitive::I32)
        );
    }
}

#[test]
fn unresolvable_nested_type_reports_token() {
    let err = extract_err("broken/msg/Unresolved.msg");
    assert!(
        err.contains("cannot resolve nested ROS message type \"geometry_msgs/DefinitelyMissing\""),
        "got: {err}"
    );
}

#[test]
fn service_without_separator_is_rejected() {
    let err = extract_err("broken/srv/MissingSep.srv");
    assert!(
        err.contains("must contain a line containing only ---"),
        "got: {err}"
    );
}

#[test]
fn service_with_multiple_separators_is_rejected() {
    let err = extract_err("broken/srv/TwoSeps.srv");
    assert!(err.contains("more than one --- separator"), "got: {err}");
}

#[test]
fn byte_and_char_aliases_map_to_u8() {
    let interface = FileExtractor::from_file(fixture("broken/msg/ByteAlias.msg"))
        .extract()
        .unwrap()
        .remove(0);
    assert_eq!(interface.fields.len(), 2);
    assert_eq!(interface.fields[0].name, "data");
    assert_eq!(interface.fields[1].name, "letter");
    assert!(
        interface
            .fields
            .iter()
            .all(|field| field.ty == TypeNode::Primitive(Primitive::U8))
    );
}
