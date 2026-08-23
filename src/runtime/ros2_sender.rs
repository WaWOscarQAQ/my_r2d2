use crate::interface_extractor::{Constraint, Field, Interface, Primitive, TypeNode};
use crate::payload::{Error, Payload, Value, ValueTree};
use crate::payload_generator::Sender;
use serde_yaml::{Mapping, Value as YamlValue};
use std::f64::consts::PI;
use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;

const RANGES_PER_SCAN: usize = 180;
const MAX_POINT_COUNT: u32 = 8;
const MAX_GRID_SIDE: u32 = 32;
const ROS2_CLI_TIMEOUT_SEC: u64 = 40;

#[derive(Debug, Clone)]
pub struct LaserScanSchedule {
    pub rate_hz: f64,
    pub duration_sec: f64,
    pub burst_count: u32,
    pub burst_gap_ms: u64,
    pub max_publishes: u64,
    pub stamp_mode: String,
}

impl LaserScanSchedule {
    pub fn expected_messages(&self) -> u64 {
        let ticks = (self.rate_hz * self.duration_sec) as u64;
        let uncapped = ticks.saturating_mul(self.burst_count as u64);
        if self.max_publishes > 0 {
            uncapped.min(self.max_publishes)
        } else {
            uncapped
        }
    }
}

pub struct Ros2LaserScanSender {
    ros_setup: PathBuf,
    install_setup: PathBuf,
    payload_file: PathBuf,
    domain_id: String,
    schedule: LaserScanSchedule,
}

impl Ros2LaserScanSender {
    pub fn new(
        ros_setup: impl Into<PathBuf>,
        install_setup: impl Into<PathBuf>,
        payload_file: impl Into<PathBuf>,
        domain_id: impl Into<String>,
        schedule: LaserScanSchedule,
    ) -> Self {
        Self {
            ros_setup: ros_setup.into(),
            install_setup: install_setup.into(),
            payload_file: payload_file.into(),
            domain_id: domain_id.into(),
            schedule,
        }
    }

    pub fn expected_messages(&self) -> u64 {
        self.schedule.expected_messages()
    }
}

impl Sender for Ros2LaserScanSender {
    fn send(&self, payload: &Payload) -> Result<(), Error> {
        write_payload_file(&self.payload_file, payload)?;
        let status = Command::new("bash")
            .arg("-c")
            .arg(
                "source \"$1\" && source \"$2\" && export ROS_DOMAIN_ID=\"$3\" && \
                 export ROS_LOG_DIR=\"${ROS_LOG_DIR:-/tmp/r2d2_ros_logs}\" && \
                 export ROS2CLI_NO_DAEMON=1 && mkdir -p \"$ROS_LOG_DIR\" && \
                 exec setarch x86_64 -R ros2 run r2d2_scan_bridge r2d2_scan_bridge \
                 \"$4\" \"$5\" \"$6\" \"$7\" \"$8\" \"$9\" \"${10}\"",
            )
            .arg("r2d2-ros2-sender")
            .arg(&self.ros_setup)
            .arg(&self.install_setup)
            .arg(&self.domain_id)
            .arg(&self.payload_file)
            .arg(self.schedule.rate_hz.to_string())
            .arg(self.schedule.duration_sec.to_string())
            .arg(self.schedule.burst_count.to_string())
            .arg(self.schedule.burst_gap_ms.to_string())
            .arg(self.schedule.max_publishes.to_string())
            .arg(&self.schedule.stamp_mode)
            .env_remove("LD_PRELOAD")
            .env_remove("ASAN_OPTIONS")
            .env_remove("TSAN_OPTIONS")
            .env_remove("COLCON_CURRENT_PREFIX")
            .status()
            .map_err(|error| {
                Error::Unsupported(format!("failed to start ROS scan bridge: {error}"))
            })?;
        if status.success() {
            Ok(())
        } else {
            Err(Error::Unsupported(format!(
                "ROS scan bridge exited with {status}"
            )))
        }
    }
}

#[derive(Debug, Clone)]
pub struct Ros2TopicOptions {
    pub qos_profile: Option<String>,
    pub qos_depth: Option<u32>,
    pub qos_history: Option<String>,
    pub qos_reliability: Option<String>,
    pub qos_durability: Option<String>,
    pub keep_alive_sec: f64,
}

impl Default for Ros2TopicOptions {
    fn default() -> Self {
        Self {
            qos_profile: None,
            qos_depth: None,
            qos_history: None,
            qos_reliability: None,
            qos_durability: None,
            keep_alive_sec: 0.2,
        }
    }
}

pub struct Ros2TopicSender {
    ros_setup: PathBuf,
    install_setup: PathBuf,
    domain_id: String,
    topic_name: String,
    message_type: String,
    interface: Interface,
    options: Ros2TopicOptions,
}

impl Ros2TopicSender {
    pub fn new(
        ros_setup: impl Into<PathBuf>,
        install_setup: impl Into<PathBuf>,
        domain_id: impl Into<String>,
        topic_name: impl Into<String>,
        message_type: impl Into<String>,
        interface: Interface,
        options: Ros2TopicOptions,
    ) -> Self {
        Self {
            ros_setup: ros_setup.into(),
            install_setup: install_setup.into(),
            domain_id: domain_id.into(),
            topic_name: topic_name.into(),
            message_type: message_type.into(),
            interface,
            options,
        }
    }
}

impl Sender for Ros2TopicSender {
    fn send(&self, payload: &Payload) -> Result<(), Error> {
        let normalized = normalize_value(&self.interface.name, payload);
        let values = render_cli_payload(&normalized, &self.interface.fields)?;

        let mut args = vec![
            "ros2".to_string(),
            "topic".to_string(),
            "pub".to_string(),
            "--once".to_string(),
            "--keep-alive".to_string(),
            self.options.keep_alive_sec.to_string(),
        ];
        if let Some(profile) = &self.options.qos_profile {
            args.push("--qos-profile".to_string());
            args.push(profile.clone());
        }
        if let Some(depth) = self.options.qos_depth {
            args.push("--qos-depth".to_string());
            args.push(depth.to_string());
        }
        if let Some(history) = &self.options.qos_history {
            args.push("--qos-history".to_string());
            args.push(history.clone());
        }
        if let Some(reliability) = &self.options.qos_reliability {
            args.push("--qos-reliability".to_string());
            args.push(reliability.clone());
        }
        if let Some(durability) = &self.options.qos_durability {
            args.push("--qos-durability".to_string());
            args.push(durability.clone());
        }
        args.push(self.topic_name.clone());
        args.push(self.message_type.clone());
        args.push(values);

        run_ros2_cli(
            &self.ros_setup,
            &self.install_setup,
            &self.domain_id,
            &args,
            &format!("topic {}", self.topic_name),
        )
    }
}

pub struct Ros2ServiceSender {
    ros_setup: PathBuf,
    install_setup: PathBuf,
    domain_id: String,
    service_name: String,
    service_type: String,
    interface: Interface,
}

impl Ros2ServiceSender {
    pub fn new(
        ros_setup: impl Into<PathBuf>,
        install_setup: impl Into<PathBuf>,
        domain_id: impl Into<String>,
        service_name: impl Into<String>,
        service_type: impl Into<String>,
        interface: Interface,
    ) -> Self {
        Self {
            ros_setup: ros_setup.into(),
            install_setup: install_setup.into(),
            domain_id: domain_id.into(),
            service_name: service_name.into(),
            service_type: service_type.into(),
            interface,
        }
    }
}

impl Sender for Ros2ServiceSender {
    fn send(&self, payload: &Payload) -> Result<(), Error> {
        let normalized = normalize_value(&self.interface.name, payload);
        let values = render_cli_payload(&normalized, &self.interface.fields)?;
        let args = vec![
            "ros2".to_string(),
            "service".to_string(),
            "call".to_string(),
            self.service_name.clone(),
            self.service_type.clone(),
            values,
        ];
        run_ros2_cli(
            &self.ros_setup,
            &self.install_setup,
            &self.domain_id,
            &args,
            &format!("service {}", self.service_name),
        )
    }
}

pub struct Ros2ParameterSender {
    ros_setup: PathBuf,
    install_setup: PathBuf,
    domain_id: String,
    node_name: String,
    parameter_name: String,
    interface: Interface,
}

impl Ros2ParameterSender {
    pub fn new(
        ros_setup: impl Into<PathBuf>,
        install_setup: impl Into<PathBuf>,
        domain_id: impl Into<String>,
        node_name: impl Into<String>,
        parameter_name: impl Into<String>,
        interface: Interface,
    ) -> Self {
        Self {
            ros_setup: ros_setup.into(),
            install_setup: install_setup.into(),
            domain_id: domain_id.into(),
            node_name: node_name.into(),
            parameter_name: parameter_name.into(),
            interface,
        }
    }

    pub fn send_value(&self, value: &ValueTree) -> Result<(), Error> {
        let field = parameter_value_field(&self.interface)?;
        let rendered = render_cli_value(value, &field.ty)?;
        let args = vec![
            "ros2".to_string(),
            "param".to_string(),
            "set".to_string(),
            self.node_name.clone(),
            self.parameter_name.clone(),
            rendered,
        ];
        run_ros2_cli(
            &self.ros_setup,
            &self.install_setup,
            &self.domain_id,
            &args,
            &format!("parameter {}:{}", self.node_name, self.parameter_name),
        )
    }
}

impl Sender for Ros2ParameterSender {
    fn send(&self, payload: &Payload) -> Result<(), Error> {
        let normalized = normalize_value(&self.interface.name, payload);
        let value = extract_parameter_value(&normalized, &self.interface)?;
        self.send_value(value)
    }
}

fn run_ros2_cli(
    ros_setup: &Path,
    install_setup: &Path,
    domain_id: &str,
    args: &[String],
    label: &str,
) -> Result<(), Error> {
    let status = ros2_cli_command(ros_setup, install_setup, domain_id, args)
        .status()
        .map_err(|error| Error::Unsupported(format!("failed to start {label}: {error}")))?;
    if status.success() {
        Ok(())
    } else if status.code() == Some(124) {
        Err(Error::Unsupported(format!(
            "{label} timed out after {ROS2_CLI_TIMEOUT_SEC}s"
        )))
    } else {
        Err(Error::Unsupported(format!("{label} exited with {status}")))
    }
}

pub fn ros2_cli_command(
    ros_setup: &Path,
    install_setup: &Path,
    domain_id: &str,
    args: &[String],
) -> Command {
    let mut command = Command::new("bash");
    command
        .arg("-lc")
        .arg("source \"$1\" && source \"$2\" && export ROS_DOMAIN_ID=\"$3\" && export ROS_LOG_DIR=\"${ROS_LOG_DIR:-/tmp/r2d2_ros_logs}\" && export ROS2CLI_NO_DAEMON=1 && mkdir -p \"$ROS_LOG_DIR\" && shift 3 && exec timeout --signal=INT --kill-after=5 \"${R2D2_ROS2_CLI_TIMEOUT_SEC}\" \"$@\"")
        .arg("r2d2-ros2-cli")
        .arg(ros_setup)
        .arg(install_setup)
        .arg(domain_id)
        .env("R2D2_ROS2_CLI_TIMEOUT_SEC", ROS2_CLI_TIMEOUT_SEC.to_string())
        .env_remove("LD_PRELOAD")
        .env_remove("ASAN_OPTIONS")
        .env_remove("TSAN_OPTIONS")
        .env_remove("COLCON_CURRENT_PREFIX");
    for arg in args {
        command.arg(arg);
    }
    command
}

fn render_cli_payload(value: &ValueTree, fields: &[Field]) -> Result<String, Error> {
    let yaml = render_message(value, fields)?;
    serde_json::to_string(&yaml)
        .map_err(|error| Error::Unsupported(format!("failed to render JSON payload: {error}")))
}

fn render_cli_value(value: &ValueTree, ty: &TypeNode) -> Result<String, Error> {
    let yaml = render_value(value, ty)?;
    serde_json::to_string(&yaml)
        .map_err(|error| Error::Unsupported(format!("failed to render parameter value: {error}")))
}

fn render_message(value: &ValueTree, fields: &[Field]) -> Result<YamlValue, Error> {
    let ValueTree::Nested(values) = value else {
        return Err(Error::TypeMismatch {
            expected: format!("nested message with {} fields", fields.len()),
            found: format!("{value:?}"),
        });
    };
    if values.len() != fields.len() {
        return Err(Error::TypeMismatch {
            expected: format!("nested message with {} fields", fields.len()),
            found: format!("nested value with {} fields", values.len()),
        });
    }
    let mut mapping = Mapping::new();
    for (field, value) in fields.iter().zip(values) {
        mapping.insert(
            YamlValue::String(field.name.clone()),
            render_value(value, &field.ty)?,
        );
    }
    Ok(YamlValue::Mapping(mapping))
}

fn render_value(value: &ValueTree, ty: &TypeNode) -> Result<YamlValue, Error> {
    match (value, ty) {
        (value, TypeNode::Constrained(inner, constraint)) => {
            validate_constraint(value, constraint)?;
            render_value(value, inner)
        }
        (ValueTree::Leaf(Value::Bool(v)), TypeNode::Primitive(Primitive::Bool)) => scalar_yaml(*v),
        (ValueTree::Leaf(Value::I8(v)), TypeNode::Primitive(Primitive::I8)) => scalar_yaml(*v),
        (ValueTree::Leaf(Value::U8(v)), TypeNode::Primitive(Primitive::U8)) => scalar_yaml(*v),
        (ValueTree::Leaf(Value::I16(v)), TypeNode::Primitive(Primitive::I16)) => scalar_yaml(*v),
        (ValueTree::Leaf(Value::U16(v)), TypeNode::Primitive(Primitive::U16)) => scalar_yaml(*v),
        (ValueTree::Leaf(Value::I32(v)), TypeNode::Primitive(Primitive::I32)) => scalar_yaml(*v),
        (ValueTree::Leaf(Value::U32(v)), TypeNode::Primitive(Primitive::U32)) => scalar_yaml(*v),
        (ValueTree::Leaf(Value::I64(v)), TypeNode::Primitive(Primitive::I64)) => scalar_yaml(*v),
        (ValueTree::Leaf(Value::U64(v)), TypeNode::Primitive(Primitive::U64)) => scalar_yaml(*v),
        (ValueTree::Leaf(Value::F32(v)), TypeNode::Primitive(Primitive::F32)) => scalar_yaml(*v),
        (ValueTree::Leaf(Value::F64(v)), TypeNode::Primitive(Primitive::F64)) => scalar_yaml(*v),
        (ValueTree::Leaf(Value::String(v)), TypeNode::Primitive(Primitive::String)) => {
            Ok(YamlValue::String(v.clone()))
        }
        (ValueTree::Leaf(Value::Bytes(v)), TypeNode::Primitive(Primitive::Bytes)) => Ok(
            YamlValue::Sequence(v.iter().map(|byte| YamlValue::from(*byte)).collect()),
        ),
        (ValueTree::Nested(_), TypeNode::Nested(fields)) => render_message(value, fields),
        (ValueTree::Array(items), TypeNode::Array(element, fixed_len)) => {
            if let Some(expected) = fixed_len
                && items.len() != *expected
            {
                return Err(Error::TypeMismatch {
                    expected: format!("fixed array of {expected} elements"),
                    found: format!("array of {} elements", items.len()),
                });
            }
            let mut out = Vec::with_capacity(items.len());
            for item in items {
                out.push(render_value(item, element)?);
            }
            Ok(YamlValue::Sequence(out))
        }
        (other, ty) => Err(Error::TypeMismatch {
            expected: format!("{ty:?}"),
            found: format!("{other:?}"),
        }),
    }
}

fn validate_constraint(value: &ValueTree, constraint: &Constraint) -> Result<(), Error> {
    match (value, constraint) {
        (ValueTree::Leaf(Value::String(text)), Constraint::StringMaxLen(max))
            if text.len() > *max =>
        {
            Err(Error::TypeMismatch {
                expected: format!("string of at most {max} bytes"),
                found: format!("string of {} bytes", text.len()),
            })
        }
        (ValueTree::Array(items), Constraint::ArrayMaxLen(max)) if items.len() > *max => {
            Err(Error::TypeMismatch {
                expected: format!("array of at most {max} elements"),
                found: format!("array of {} elements", items.len()),
            })
        }
        _ => Ok(()),
    }
}

fn scalar_yaml<T: serde::Serialize>(value: T) -> Result<YamlValue, Error> {
    serde_yaml::to_value(value)
        .map_err(|error| Error::Unsupported(format!("failed to serialize scalar: {error}")))
}

fn parameter_value_field(interface: &Interface) -> Result<&Field, Error> {
    if interface.fields.len() != 1 {
        return Err(Error::TypeMismatch {
            expected: "parameter interface with exactly one top-level field".to_string(),
            found: format!("{} top-level fields", interface.fields.len()),
        });
    }
    let field = &interface.fields[0];
    if field.name != "value" {
        return Err(Error::TypeMismatch {
            expected: "parameter top-level field named value".to_string(),
            found: format!("field {:?}", field.name),
        });
    }
    Ok(field)
}

fn extract_parameter_value<'a>(
    normalized: &'a ValueTree,
    interface: &Interface,
) -> Result<&'a ValueTree, Error> {
    let field = parameter_value_field(interface)?;
    let ValueTree::Nested(values) = normalized else {
        return Err(Error::TypeMismatch {
            expected: "nested parameter payload".to_string(),
            found: format!("{normalized:?}"),
        });
    };
    let Some(value) = values.first() else {
        return Err(Error::TypeMismatch {
            expected: "parameter payload with one value field".to_string(),
            found: "empty nested payload".to_string(),
        });
    };
    if values.len() != 1 {
        return Err(Error::TypeMismatch {
            expected: "parameter payload with one value field".to_string(),
            found: format!("nested payload with {} fields", values.len()),
        });
    }
    render_value(value, &field.ty)?;
    Ok(value)
}

fn write_payload_file(path: &Path, payload: &Payload) -> Result<(), Error> {
    let (amin, amax, ainc, tinc, stime, rmin, rmax, ranges) = scan_params(payload);
    let mut text = format!("{amin} {amax} {ainc} {tinc} {stime} {rmin} {rmax}\n");
    for (index, range) in ranges.iter().enumerate() {
        if index > 0 {
            text.push(' ');
        }
        text.push_str(&range.to_string());
    }
    text.push('\n');
    fs::write(path, text)
        .map_err(|error| Error::Unsupported(format!("failed to write {}: {error}", path.display())))
}

fn field_f32(fields: &[ValueTree], index: usize) -> f32 {
    match fields.get(index) {
        Some(ValueTree::Leaf(Value::F32(value))) => *value,
        _ => 0.0,
    }
}

fn scan_params(payload: &Payload) -> (f32, f32, f32, f32, f32, f32, f32, Vec<f32>) {
    let ValueTree::Nested(fields) = &payload.value else {
        return (0.0, 0.0, 0.01, 0.0, 0.05, 0.01, 12.0, Vec::new());
    };
    let angle_min = field_f32(fields, 1).clamp(-std::f32::consts::PI, std::f32::consts::PI);
    let angle_max = field_f32(fields, 2).clamp(angle_min + 0.01, angle_min + std::f32::consts::PI);
    let angle_increment = field_f32(fields, 3).abs().clamp(0.001, 0.35);
    let time_increment = field_f32(fields, 4).abs().min(0.1);
    let scan_time = field_f32(fields, 5).abs().min(0.5);
    let range_min = field_f32(fields, 6).clamp(0.01, 5.0);
    let range_max = field_f32(fields, 7).clamp(range_min + 0.1, 20.0);

    let ranges = match fields.get(8) {
        Some(ValueTree::Array(items)) => items
            .iter()
            .filter_map(|item| match item {
                ValueTree::Leaf(Value::F32(value)) => Some(*value),
                _ => None,
            })
            .collect::<Vec<_>>(),
        _ => Vec::new(),
    };
    let ranges = if ranges.is_empty() {
        vec![range_max; RANGES_PER_SCAN]
    } else {
        ranges
            .iter()
            .cycle()
            .take(RANGES_PER_SCAN)
            .map(|range| (*range).clamp(range_min, range_max))
            .collect()
    };
    (
        angle_min,
        angle_max,
        angle_increment,
        time_increment,
        scan_time,
        range_min,
        range_max,
        ranges,
    )
}

fn normalize_value(interface_name: &str, payload: &Payload) -> ValueTree {
    match interface_name {
        "PointCloud2" => normalize_point_cloud2(payload),
        "OccupancyGrid" => normalize_occupancy_grid(payload),
        "OccupancyGridUpdate" => normalize_occupancy_grid_update(payload),
        "GetCost" => normalize_get_cost(payload),
        "GetCostmap" => normalize_get_costmap(payload),
        "ClearCostmapExceptRegion" | "ClearCostmapAroundRobot" => {
            normalize_reset_distance_request(payload, 0.5)
        }
        "ClearCostmapAroundPose" => normalize_clear_around_pose(payload),
        _ => payload.value.clone(),
    }
}

fn normalize_point_cloud2(payload: &Payload) -> ValueTree {
    let count = payload
        .value
        .as_nested()
        .and_then(|fields| scalar_to_u32(fields.get(2)?))
        .unwrap_or(4)
        .clamp(1, MAX_POINT_COUNT) as usize;
    let mut blob = Vec::with_capacity(count * 12);
    for index in 0..count {
        let phase = ((payload.rng_seed.wrapping_add(index as u64) % 17) as f32) * 0.03;
        let x = 0.25 + phase;
        let y = (index as f32 - (count as f32 / 2.0)) * 0.08;
        let z = ((payload.rng_seed >> (index % 8)) & 1) as f32 * 0.02;
        blob.extend_from_slice(&x.to_le_bytes());
        blob.extend_from_slice(&y.to_le_bytes());
        blob.extend_from_slice(&z.to_le_bytes());
    }
    ValueTree::Nested(vec![
        header("laser_frame"),
        leaf_u32(1),
        leaf_u32(count as u32),
        ValueTree::Array(vec![
            point_field("x", 0),
            point_field("y", 4),
            point_field("z", 8),
        ]),
        ValueTree::Leaf(Value::Bool(false)),
        leaf_u32(12),
        leaf_u32((count * 12) as u32),
        byte_array(blob),
        ValueTree::Leaf(Value::Bool(true)),
    ])
}

fn normalize_occupancy_grid(payload: &Payload) -> ValueTree {
    let source = payload.value.as_nested();
    let info = source
        .and_then(|fields| fields.get(1))
        .and_then(ValueTree::as_nested);
    let width = info
        .and_then(|fields| scalar_to_u32(fields.get(2)?))
        .unwrap_or(8)
        .clamp(1, MAX_GRID_SIDE);
    let height = info
        .and_then(|fields| scalar_to_u32(fields.get(3)?))
        .unwrap_or(8)
        .clamp(1, MAX_GRID_SIDE);
    let resolution = info
        .and_then(|fields| scalar_to_f32(fields.get(1)?))
        .unwrap_or(0.05)
        .clamp(0.02, 1.0);
    let data = source
        .and_then(|fields| fields.get(2))
        .map(|values| int8_array(values, width as usize * height as usize))
        .unwrap_or_else(|| vec![0; width as usize * height as usize]);
    ValueTree::Nested(vec![
        header("map"),
        ValueTree::Nested(vec![
            time_zero(),
            ValueTree::Leaf(Value::F32(resolution)),
            leaf_u32(width),
            leaf_u32(height),
            pose(
                info.and_then(|fields| fields.get(4))
                    .and_then(ValueTree::as_nested)
                    .and_then(|pose| pose.get(0))
                    .and_then(ValueTree::as_nested)
                    .and_then(|position| scalar_to_f64(position.first()?))
                    .unwrap_or(0.0)
                    .clamp(-2.5, 2.5),
                info.and_then(|fields| fields.get(4))
                    .and_then(ValueTree::as_nested)
                    .and_then(|pose| pose.get(0))
                    .and_then(ValueTree::as_nested)
                    .and_then(|position| scalar_to_f64(position.get(1)?))
                    .unwrap_or(0.0)
                    .clamp(-2.5, 2.5),
                0.0,
                [0.0, 0.0, 0.0, 1.0],
            ),
        ]),
        int8_leaf_array(data),
    ])
}

fn normalize_occupancy_grid_update(payload: &Payload) -> ValueTree {
    let source = payload.value.as_nested();
    let width = source
        .and_then(|fields| scalar_to_u32(fields.get(3)?))
        .unwrap_or(8)
        .clamp(1, MAX_GRID_SIDE);
    let height = source
        .and_then(|fields| scalar_to_u32(fields.get(4)?))
        .unwrap_or(8)
        .clamp(1, MAX_GRID_SIDE);
    let x = source
        .and_then(|fields| scalar_to_i32(fields.get(1)?))
        .unwrap_or(0)
        .clamp(0, 99);
    let y = source
        .and_then(|fields| scalar_to_i32(fields.get(2)?))
        .unwrap_or(0)
        .clamp(0, 99);
    let data = source
        .and_then(|fields| fields.get(5))
        .map(|values| int8_array(values, width as usize * height as usize))
        .unwrap_or_else(|| vec![0; width as usize * height as usize]);
    ValueTree::Nested(vec![
        header("map"),
        ValueTree::Leaf(Value::I32(x)),
        ValueTree::Leaf(Value::I32(y)),
        leaf_u32(width),
        leaf_u32(height),
        int8_leaf_array(data),
    ])
}

fn normalize_get_cost(payload: &Payload) -> ValueTree {
    let source = payload.value.as_nested();
    ValueTree::Nested(vec![
        ValueTree::Leaf(Value::Bool(
            source
                .and_then(|fields| fields.first())
                .and_then(ValueTree::as_bool)
                .unwrap_or(false),
        )),
        ValueTree::Leaf(Value::F32(
            source
                .and_then(|fields| fields.get(1))
                .and_then(scalar_to_f32)
                .unwrap_or(0.0)
                .clamp(-2.5, 2.5),
        )),
        ValueTree::Leaf(Value::F32(
            source
                .and_then(|fields| fields.get(2))
                .and_then(scalar_to_f32)
                .unwrap_or(0.0)
                .clamp(-2.5, 2.5),
        )),
        ValueTree::Leaf(Value::F32(
            source
                .and_then(|fields| fields.get(3))
                .and_then(scalar_to_f32)
                .unwrap_or(0.0)
                .clamp(-(PI as f32), PI as f32),
        )),
    ])
}

fn normalize_get_costmap(payload: &Payload) -> ValueTree {
    let source = payload.value.as_nested();
    let specs = source
        .and_then(|fields| fields.first())
        .and_then(ValueTree::as_nested);
    let resolution = specs
        .and_then(|fields| scalar_to_f32(fields.get(3)?))
        .unwrap_or(0.05)
        .clamp(0.02, 1.0);
    let size_x = specs
        .and_then(|fields| scalar_to_u32(fields.get(4)?))
        .unwrap_or(16)
        .clamp(1, MAX_GRID_SIDE);
    let size_y = specs
        .and_then(|fields| scalar_to_u32(fields.get(5)?))
        .unwrap_or(16)
        .clamp(1, MAX_GRID_SIDE);
    ValueTree::Nested(vec![ValueTree::Nested(vec![
        time_zero(),
        time_zero(),
        ValueTree::Leaf(Value::String("master".to_string())),
        ValueTree::Leaf(Value::F32(resolution)),
        leaf_u32(size_x),
        leaf_u32(size_y),
        pose(0.0, 0.0, 0.0, [0.0, 0.0, 0.0, 1.0]),
    ])])
}

fn normalize_reset_distance_request(payload: &Payload, fallback: f32) -> ValueTree {
    let distance = payload
        .value
        .as_nested()
        .and_then(|fields| fields.first())
        .and_then(scalar_to_f32)
        .unwrap_or(fallback)
        .clamp(0.1, 5.0);
    ValueTree::Nested(vec![ValueTree::Leaf(Value::F32(distance))])
}

fn normalize_clear_around_pose(payload: &Payload) -> ValueTree {
    let source = payload.value.as_nested();
    let pose_value = source
        .and_then(|fields| fields.first())
        .and_then(ValueTree::as_nested);
    let pose_fields = pose_value
        .and_then(|fields| fields.get(1))
        .and_then(ValueTree::as_nested);
    let position = pose_fields
        .and_then(|fields| fields.first())
        .and_then(ValueTree::as_nested);
    let orientation = pose_fields
        .and_then(|fields| fields.get(1))
        .and_then(ValueTree::as_nested);

    let x = position
        .and_then(|fields| scalar_to_f64(fields.first()?))
        .unwrap_or(0.0)
        .clamp(-2.5, 2.5);
    let y = position
        .and_then(|fields| scalar_to_f64(fields.get(1)?))
        .unwrap_or(0.0)
        .clamp(-2.5, 2.5);
    let z = position
        .and_then(|fields| scalar_to_f64(fields.get(2)?))
        .unwrap_or(0.0)
        .clamp(-0.1, 0.1);
    let q = normalize_quaternion([
        orientation
            .and_then(|fields| scalar_to_f64(fields.first()?))
            .unwrap_or(0.0),
        orientation
            .and_then(|fields| scalar_to_f64(fields.get(1)?))
            .unwrap_or(0.0),
        orientation
            .and_then(|fields| scalar_to_f64(fields.get(2)?))
            .unwrap_or(0.0),
        orientation
            .and_then(|fields| scalar_to_f64(fields.get(3)?))
            .unwrap_or(1.0),
    ]);
    let reset_distance = source
        .and_then(|fields| fields.get(1))
        .and_then(scalar_to_f64)
        .unwrap_or(0.5)
        .clamp(0.1, 5.0);
    ValueTree::Nested(vec![
        ValueTree::Nested(vec![header("map"), pose(x, y, z, q)]),
        ValueTree::Leaf(Value::F64(reset_distance)),
    ])
}

fn point_field(name: &str, offset: u32) -> ValueTree {
    ValueTree::Nested(vec![
        ValueTree::Leaf(Value::String(name.to_string())),
        leaf_u32(offset),
        ValueTree::Leaf(Value::U8(7)),
        leaf_u32(1),
    ])
}

fn header(frame_id: &str) -> ValueTree {
    ValueTree::Nested(vec![
        time_zero(),
        ValueTree::Leaf(Value::String(frame_id.to_string())),
    ])
}

fn time_zero() -> ValueTree {
    ValueTree::Nested(vec![
        ValueTree::Leaf(Value::I32(0)),
        ValueTree::Leaf(Value::U32(0)),
    ])
}

fn pose(x: f64, y: f64, z: f64, quaternion: [f64; 4]) -> ValueTree {
    ValueTree::Nested(vec![
        ValueTree::Nested(vec![
            ValueTree::Leaf(Value::F64(x)),
            ValueTree::Leaf(Value::F64(y)),
            ValueTree::Leaf(Value::F64(z)),
        ]),
        ValueTree::Nested(vec![
            ValueTree::Leaf(Value::F64(quaternion[0])),
            ValueTree::Leaf(Value::F64(quaternion[1])),
            ValueTree::Leaf(Value::F64(quaternion[2])),
            ValueTree::Leaf(Value::F64(quaternion[3])),
        ]),
    ])
}

fn normalize_quaternion(mut quaternion: [f64; 4]) -> [f64; 4] {
    let norm = quaternion
        .iter()
        .map(|value| value * value)
        .sum::<f64>()
        .sqrt();
    if norm <= f64::EPSILON {
        quaternion[3] = 1.0;
        quaternion
    } else {
        quaternion.map(|value| value / norm)
    }
}

fn leaf_u32(value: u32) -> ValueTree {
    ValueTree::Leaf(Value::U32(value))
}

fn byte_array(bytes: Vec<u8>) -> ValueTree {
    ValueTree::Array(
        bytes
            .into_iter()
            .map(|value| ValueTree::Leaf(Value::U8(value)))
            .collect(),
    )
}

fn int8_leaf_array(values: Vec<i8>) -> ValueTree {
    ValueTree::Array(
        values
            .into_iter()
            .map(|value| ValueTree::Leaf(Value::I8(value)))
            .collect(),
    )
}

fn int8_array(tree: &ValueTree, len: usize) -> Vec<i8> {
    let source = match tree {
        ValueTree::Array(items) => items
            .iter()
            .map(|item| match item {
                ValueTree::Leaf(Value::I8(value)) => (*value).clamp(-1, 100),
                ValueTree::Leaf(Value::U8(value)) => (*value).min(100) as i8,
                _ => 0,
            })
            .collect::<Vec<_>>(),
        _ => Vec::new(),
    };
    fill_or_repeat(source, len, 0)
}

fn fill_or_repeat<T: Copy>(values: Vec<T>, len: usize, fill: T) -> Vec<T> {
    if values.is_empty() {
        vec![fill; len]
    } else {
        values.iter().copied().cycle().take(len).collect()
    }
}

fn scalar_to_u32(tree: &ValueTree) -> Option<u32> {
    match tree {
        ValueTree::Leaf(Value::U8(value)) => Some(*value as u32),
        ValueTree::Leaf(Value::U16(value)) => Some(*value as u32),
        ValueTree::Leaf(Value::U32(value)) => Some(*value),
        ValueTree::Leaf(Value::U64(value)) => u32::try_from(*value).ok(),
        ValueTree::Leaf(Value::I8(value)) if *value >= 0 => Some(*value as u32),
        ValueTree::Leaf(Value::I16(value)) if *value >= 0 => Some(*value as u32),
        ValueTree::Leaf(Value::I32(value)) if *value >= 0 => Some(*value as u32),
        ValueTree::Leaf(Value::I64(value)) if *value >= 0 => u32::try_from(*value).ok(),
        _ => None,
    }
}

fn scalar_to_i32(tree: &ValueTree) -> Option<i32> {
    match tree {
        ValueTree::Leaf(Value::U8(value)) => Some(*value as i32),
        ValueTree::Leaf(Value::U16(value)) => Some(*value as i32),
        ValueTree::Leaf(Value::U32(value)) => i32::try_from(*value).ok(),
        ValueTree::Leaf(Value::I8(value)) => Some(*value as i32),
        ValueTree::Leaf(Value::I16(value)) => Some(*value as i32),
        ValueTree::Leaf(Value::I32(value)) => Some(*value),
        ValueTree::Leaf(Value::I64(value)) => i32::try_from(*value).ok(),
        _ => None,
    }
}

fn scalar_to_f32(tree: &ValueTree) -> Option<f32> {
    match tree {
        ValueTree::Leaf(Value::U8(value)) => Some(*value as f32),
        ValueTree::Leaf(Value::U16(value)) => Some(*value as f32),
        ValueTree::Leaf(Value::U32(value)) => Some(*value as f32),
        ValueTree::Leaf(Value::I8(value)) => Some(*value as f32),
        ValueTree::Leaf(Value::I16(value)) => Some(*value as f32),
        ValueTree::Leaf(Value::I32(value)) => Some(*value as f32),
        ValueTree::Leaf(Value::F32(value)) => Some(*value),
        ValueTree::Leaf(Value::F64(value)) => Some(*value as f32),
        _ => None,
    }
}

fn scalar_to_f64(tree: &ValueTree) -> Option<f64> {
    match tree {
        ValueTree::Leaf(Value::U8(value)) => Some(*value as f64),
        ValueTree::Leaf(Value::U16(value)) => Some(*value as f64),
        ValueTree::Leaf(Value::U32(value)) => Some(*value as f64),
        ValueTree::Leaf(Value::U64(value)) => Some(*value as f64),
        ValueTree::Leaf(Value::I8(value)) => Some(*value as f64),
        ValueTree::Leaf(Value::I16(value)) => Some(*value as f64),
        ValueTree::Leaf(Value::I32(value)) => Some(*value as f64),
        ValueTree::Leaf(Value::I64(value)) => Some(*value as f64),
        ValueTree::Leaf(Value::F32(value)) => Some(*value as f64),
        ValueTree::Leaf(Value::F64(value)) => Some(*value),
        _ => None,
    }
}

trait ValueTreeExt {
    fn as_nested(&self) -> Option<&[ValueTree]>;
    fn as_bool(&self) -> Option<bool>;
}

impl ValueTreeExt for ValueTree {
    fn as_nested(&self) -> Option<&[ValueTree]> {
        match self {
            ValueTree::Nested(fields) => Some(fields),
            _ => None,
        }
    }

    fn as_bool(&self) -> Option<bool> {
        match self {
            ValueTree::Leaf(Value::Bool(value)) => Some(*value),
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{
        LaserScanSchedule, extract_parameter_value, normalize_point_cloud2, render_cli_payload,
        render_cli_value, render_message,
    };
    use crate::interface_extractor::{Field, Interface, Kind, Primitive, TypeNode};
    use crate::payload::{Payload, Value, ValueTree};

    #[test]
    fn expected_messages_matches_bridge_limits() {
        let schedule = LaserScanSchedule {
            rate_hz: 20.0,
            duration_sec: 20.0,
            burst_count: 2,
            burst_gap_ms: 0,
            max_publishes: 120,
            stamp_mode: "now".to_string(),
        };
        assert_eq!(schedule.expected_messages(), 120);

        let uncapped = LaserScanSchedule {
            duration_sec: 2.0,
            burst_count: 1,
            max_publishes: 0,
            ..schedule
        };
        assert_eq!(uncapped.expected_messages(), 40);
    }

    #[test]
    fn pointcloud2_normalization_keeps_blob_consistent() {
        let payload = Payload::new(
            "PointCloud2",
            crate::interface_extractor::Kind::Topic,
            ValueTree::Nested(vec![
                ValueTree::Nested(vec![]),
                ValueTree::Leaf(Value::U32(99)),
                ValueTree::Leaf(Value::U32(4)),
                ValueTree::Array(Vec::new()),
                ValueTree::Leaf(Value::Bool(false)),
                ValueTree::Leaf(Value::U32(0)),
                ValueTree::Leaf(Value::U32(0)),
                ValueTree::Array(Vec::new()),
                ValueTree::Leaf(Value::Bool(true)),
            ]),
            7,
        );
        let ValueTree::Nested(fields) = normalize_point_cloud2(&payload) else {
            panic!("must stay nested");
        };
        assert_eq!(fields[5], ValueTree::Leaf(Value::U32(12)));
        assert_eq!(fields[6], ValueTree::Leaf(Value::U32(48)));
        let ValueTree::Array(data) = &fields[7] else {
            panic!("data must be an array");
        };
        assert_eq!(data.len(), 48);
    }

    #[test]
    fn render_message_outputs_plain_cli_mapping() {
        let fields = vec![Field::new("request", TypeNode::Nested(Vec::new()))];
        let value = ValueTree::Nested(vec![ValueTree::Nested(Vec::new())]);
        let rendered = render_cli_payload(&value, &fields).unwrap();
        assert_eq!(rendered.trim(), "{\"request\":{}}");
        assert!(render_message(&value, &fields).is_ok());
    }

    #[test]
    fn render_uint8_array_as_json_sequence() {
        let fields = vec![Field::new(
            "data",
            TypeNode::Array(Box::new(TypeNode::Primitive(Primitive::U8)), None),
        )];
        let value = ValueTree::Nested(vec![ValueTree::Array(vec![
            ValueTree::Leaf(Value::U8(1)),
            ValueTree::Leaf(Value::U8(2)),
            ValueTree::Leaf(Value::U8(3)),
        ])]);
        let rendered = render_cli_payload(&value, &fields).unwrap();
        assert_eq!(rendered, "{\"data\":[1,2,3]}");
    }

    #[test]
    fn render_bounded_string_rejects_overflow() {
        let fields = vec![Field::new("name", TypeNode::bounded_string(3))];
        let value = ValueTree::Nested(vec![ValueTree::Leaf(Value::String("laser".to_string()))]);

        let err = render_cli_payload(&value, &fields).unwrap_err();
        assert!(err.to_string().contains("at most 3 bytes"), "got: {err}");
    }

    #[test]
    fn render_bounded_array_rejects_overflow() {
        let fields = vec![Field::new(
            "samples",
            TypeNode::bounded_array(Primitive::U8.into(), 2),
        )];
        let value = ValueTree::Nested(vec![ValueTree::Array(vec![
            ValueTree::Leaf(Value::U8(1)),
            ValueTree::Leaf(Value::U8(2)),
            ValueTree::Leaf(Value::U8(3)),
        ])]);

        let err = render_cli_payload(&value, &fields).unwrap_err();
        assert!(err.to_string().contains("at most 2 elements"), "got: {err}");
    }

    #[test]
    fn render_parameter_value_as_json_scalar() {
        let rendered = render_cli_value(
            &ValueTree::Leaf(Value::String("base_link".to_string())),
            &Primitive::String.into(),
        )
        .unwrap();
        assert_eq!(rendered, "\"base_link\"");
    }

    #[test]
    fn extract_parameter_value_requires_single_value_field() {
        let interface = Interface::new(
            "param:/costmap/width",
            Kind::Parameter,
            vec![Field::new("value", Primitive::I64)],
        );
        let payload = ValueTree::Nested(vec![ValueTree::Leaf(Value::I64(100))]);
        let value = extract_parameter_value(&payload, &interface).unwrap();
        assert_eq!(value, &ValueTree::Leaf(Value::I64(100)));
    }
}
