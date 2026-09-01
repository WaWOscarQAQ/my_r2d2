use crate::interface_extractor::{Constraint, Field, Interface, Primitive, TypeNode};
use crate::payload::{Error, Payload, Value, ValueTree};
use crate::payload_generator::Sender;
use crate::runtime::command_template::{CommandParameters, CommandTemplate};
use serde::{Deserialize, Serialize};
use serde_yaml::{Mapping, Value as YamlValue};
use std::f64::consts::PI;
use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::process::Stdio;

const RANGES_PER_SCAN: usize = 180;
const MAX_POINT_COUNT: u32 = 8;
const SAFE_GRID_SIDE: u32 = 64;
const SAFE_GRID_RESOLUTION: f32 = 0.1;
const SAFE_GRID_ORIGIN_X: f64 = -3.2;
const SAFE_GRID_ORIGIN_Y: f64 = -3.2;
const NAV2_COSTMAP_STREAM_SIDE: u32 = 16;
const NAV2_COSTMAP_UPDATE_SIDE: u32 = 8;
const NAV2_COSTMAP_STREAM_RESOLUTION: f32 = 0.05;
const SAFE_MAP_ENV: &str = "R2D2_NAV2_SAFE_MAP";
const SAFE_MAP_LIST_ENV: &str = "R2D2_NAV2_SAFE_MAPS";
const SAFE_MAP_RELATIVE: &str =
    "nav2_ws/src_lyrical/navigation2/nav2_bringup/maps/tb3_sandbox.yaml";
const NAV2_LOAD_MAP_RELATIVES: [&str; 4] = [
    SAFE_MAP_RELATIVE,
    "nav2_ws/src_lyrical/navigation2/nav2_bringup/maps/warehouse.yaml",
    "nav2_ws/src_lyrical/navigation2/nav2_bringup/maps/warehouse_keepout.yaml",
    "nav2_ws/src_lyrical/navigation2/nav2_bringup/maps/warehouse_speed.yaml",
];
const NAV2_SAFE_START_X: f64 = 0.618_745_446_205_139_2;
const NAV2_SAFE_START_Y: f64 = -1.228_127_241_134_643_6;
const NAV2_BEHAVIOR_TIME_ALLOWANCE_SEC: i32 = 8;
const NAV2_FOLLOW_PATH_STEP_METERS: f64 = 0.25;
const NAV2_FOLLOW_PATH_POSES: usize = 5;
const NAV2_LOCAL_PATH_STEP_METERS: f64 = 0.22;
const NAV2_SPIN_TIME_ALLOWANCE_SEC: i32 = 5;
const NAV2_SMOOTH_MAX_DURATION_SEC: i32 = 3;
const NAV2_WAIT_DURATION_SEC: i32 = 3;
const NAV2_BT_TREE_RELATIVE_DIR: &str =
    "nav2_ws/src_lyrical/navigation2/nav2_bt_navigator/behavior_trees";
const NAV2_NAV_TO_POSE_BT_XMLS: [&str; 12] = [
    "",
    "follow_point.xml",
    "navigate_to_pose_w_bounds_check.xml",
    "navigate_to_pose_w_replanning_and_recovery.xml",
    "navigate_to_pose_w_replanning_goal_patience_and_recovery.xml",
    "navigate_w_recovery_and_replanning_only_if_path_becomes_invalid.xml",
    "navigate_w_replanning_time.xml",
    "navigate_w_replanning_distance.xml",
    "navigate_w_replanning_speed.xml",
    "navigate_w_replanning_only_if_goal_is_updated.xml",
    "navigate_w_replanning_only_if_path_becomes_invalid.xml",
    "nav_to_pose_with_consistent_replanning_and_if_path_becomes_invalid.xml",
];
const NAV2_NAV_THROUGH_POSES_BT_XMLS: [&str; 2] =
    ["", "navigate_through_poses_w_replanning_and_recovery.xml"];
const NAV2_PLANNER_IDS: [&str; 1] = ["GridBased"];
const NAV2_CONTROLLER_IDS: [&str; 1] = ["FollowPath"];
const NAV2_GOAL_CHECKER_IDS: [&str; 1] = ["general_goal_checker"];
const NAV2_PROGRESS_CHECKER_IDS: [&str; 1] = ["progress_checker"];
const NAV2_PATH_HANDLER_IDS: [&str; 1] = ["PathHandler"];
const NAV2_SMOOTHER_IDS: [&str; 2] = ["simple_smoother", "route_smoother"];
const NAV2_SAFE_WAYPOINTS: [(f64, f64); 16] = [
    (0.506_251_871_585_846, 0.509_373_068_809_509_3),
    (0.525, -0.825),
    (1.025, -0.325),
    (0.700, -0.500),
    (0.400, -1.100),
    (0.000, -0.700),
    (-0.400, -0.700),
    (-0.975, -0.325),
    (1.525, 0.675),
    (1.300, 0.500),
    (-1.475, 0.175),
    (-0.900, 0.400),
    (-0.400, 0.300),
    (0.525, 1.175),
    (-0.475, 1.675),
    (-1.500, 1.000),
];
const NAV2_LOCAL_PATH_ROUTES: [[(f64, f64); 3]; 5] = [
    [
        (NAV2_SAFE_START_X, NAV2_SAFE_START_Y),
        (0.400, -1.100),
        (0.525, -0.825),
    ],
    [
        (NAV2_SAFE_START_X, NAV2_SAFE_START_Y),
        (0.525, -0.825),
        (0.700, -0.500),
    ],
    [
        (NAV2_SAFE_START_X, NAV2_SAFE_START_Y),
        (0.400, -1.100),
        (0.000, -0.700),
    ],
    [
        (NAV2_SAFE_START_X, NAV2_SAFE_START_Y),
        (0.525, -0.825),
        (1.025, -0.325),
    ],
    [
        (NAV2_SAFE_START_X, NAV2_SAFE_START_Y),
        (0.400, -1.100),
        (-0.400, -0.700),
    ],
];
const ROS2_CLI_TIMEOUT_SEC: u64 = 20;
const ROS2_CLI_TIMEOUT_ENV: &str = "R2D2_ROS2_CLI_TIMEOUT_SEC";
const ROS2_TOPIC_TIMEOUT_SEC: u64 = 8;
const ROS2_TOPIC_TIMEOUT_ENV: &str = "R2D2_ROS2_TOPIC_TIMEOUT_SEC";
const ROS2_SERVICE_TIMEOUT_SEC: u64 = 8;
const ROS2_SERVICE_TIMEOUT_ENV: &str = "R2D2_ROS2_SERVICE_TIMEOUT_SEC";
const ROS2_ACTION_TIMEOUT_SEC: u64 = 5;
const ROS2_ACTION_TIMEOUT_ENV: &str = "R2D2_ROS2_ACTION_TIMEOUT_SEC";
const ROS2_ACTION_CLI_TIMEOUT_ENV: &str = "R2D2_ROS2_ACTION_CLI_TIMEOUT_SEC";
const ROS2_SENDERS_CONFIG_VERSION: u32 = 1;
const ROS2_SENDERS_CONFIG_ENV: &str = "R2D2_ROS2_SENDERS_CONFIG";
const ROS2_CLI_SETUP_ENV: &str = "R2D2_ROS2_CLI_SETUP";
const ROS2_CLI_PLAIN_SETUP_CANDIDATES: [&str; 3] = [
    "nav2_ws/install_lyrical_fresh/setup.bash",
    "nav2_ws/install_lyrical/setup.bash",
    "nav2_ws/install/setup.bash",
];

#[derive(Debug, Clone, Deserialize)]
struct Ros2SenderCommands {
    version: u32,
    scan_bridge: CommandTemplate,
    ros2_cli: CommandTemplate,
    topic: TopicCommand,
    service: PrefixedCommand,
    action: ActionCommand,
    parameter: PrefixedCommand,
}

#[derive(Debug, Clone, Deserialize)]
struct TopicCommand {
    prefix: Vec<String>,
    wait_matching_subscriptions_option: String,
    keep_alive_option: String,
    qos_profile_option: String,
    qos_depth_option: String,
    qos_history_option: String,
    qos_reliability_option: String,
    qos_durability_option: String,
}

#[derive(Debug, Clone, Deserialize)]
struct PrefixedCommand {
    prefix: Vec<String>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct Ros2CommandPreview {
    pub label: String,
    pub args: Vec<String>,
    pub payload_yaml: String,
    pub command_line: String,
}

#[derive(Debug, Clone, Deserialize)]
struct ActionCommand {
    prefix: Vec<String>,
    timeout_option: String,
}

fn ros2_senders_config_path() -> PathBuf {
    std::env::var_os(ROS2_SENDERS_CONFIG_ENV)
        .filter(|path| !path.is_empty())
        .map(PathBuf::from)
        .unwrap_or_else(|| Path::new(env!("CARGO_MANIFEST_DIR")).join("config/ros2_senders.yaml"))
}

fn load_ros2_sender_commands() -> Result<Ros2SenderCommands, Error> {
    let path = ros2_senders_config_path();
    let content = fs::read_to_string(&path).map_err(|error| {
        Error::Unsupported(format!(
            "failed to read ROS 2 sender commands from {}: {error}",
            path.display()
        ))
    })?;
    let commands: Ros2SenderCommands = serde_yaml::from_str(&content).map_err(|error| {
        Error::Unsupported(format!(
            "failed to parse ROS 2 sender commands from {}: {error}",
            path.display()
        ))
    })?;
    commands.validate(&path)?;
    Ok(commands)
}

impl Ros2SenderCommands {
    fn validate(&self, path: &Path) -> Result<(), Error> {
        if self.version != ROS2_SENDERS_CONFIG_VERSION {
            return Err(Error::Unsupported(format!(
                "unsupported command version {} in {}",
                self.version,
                path.display()
            )));
        }
        for (name, template) in [
            ("scan_bridge", &self.scan_bridge),
            ("ros2_cli", &self.ros2_cli),
        ] {
            template
                .validate(name)
                .map_err(|error| Error::Unsupported(format!("{error} in {}", path.display())))?;
        }
        if [
            &self.topic.prefix,
            &self.service.prefix,
            &self.action.prefix,
            &self.parameter.prefix,
        ]
        .iter()
        .any(|prefix| prefix.is_empty())
        {
            return Err(Error::Unsupported(format!(
                "empty ROS 2 command prefix in {}",
                path.display()
            )));
        }
        Ok(())
    }
}

fn tsan_ld_preload() -> Option<&'static str> {
    match std::env::var("R2D2_PROFILE").as_deref() {
        Ok("tsan") => Some("/lib/x86_64-linux-gnu/libtsan.so.2"),
        _ => None,
    }
}

fn ros2_cli_tsan_ld_preload() -> Option<&'static str> {
    match std::env::var("R2D2_PROFILE").as_deref() {
        Ok("tsan") => Some("/lib/x86_64-linux-gnu/libtsan.so.2"),
        _ => None,
    }
}

#[derive(Debug, Clone)]
pub struct LaserScanSchedule {
    pub rate_hz: f64,
    pub duration_sec: f64,
    pub burst_count: u32,
    pub burst_gap_ms: u64,
    pub max_publishes: u64,
    pub stamp_mode: String,
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
}

impl Sender for Ros2LaserScanSender {
    fn send(&self, payload: &Payload) -> Result<(), Error> {
        write_payload_file(&self.payload_file, payload)?;
        let commands = load_ros2_sender_commands()?;
        let mut command = build_scan_bridge_command(
            &commands.scan_bridge,
            &self.ros_setup,
            &self.install_setup,
            &self.payload_file,
            &self.domain_id,
            &self.schedule,
        )?;
        let status = command.status().map_err(|error| {
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

fn build_scan_bridge_command(
    template: &CommandTemplate,
    ros_setup: &Path,
    install_setup: &Path,
    payload_file: &Path,
    domain_id: &str,
    schedule: &LaserScanSchedule,
) -> Result<Command, Error> {
    let mut parameters = CommandParameters::new();
    parameters
        .insert_path("ros_setup", ros_setup)
        .insert_path("install_setup", install_setup)
        .insert("domain_id", domain_id)
        .insert_path("payload_file", payload_file)
        .insert("rate_hz", schedule.rate_hz.to_string())
        .insert("duration_sec", schedule.duration_sec.to_string())
        .insert("burst_count", schedule.burst_count.to_string())
        .insert("burst_gap_ms", schedule.burst_gap_ms.to_string())
        .insert("max_publishes", schedule.max_publishes.to_string())
        .insert("stamp_mode", &schedule.stamp_mode);
    let mut command = template
        .build("scan_bridge", &parameters)
        .map_err(|error| Error::Unsupported(error.to_string()))?;
    command
        .env_remove("ASAN_OPTIONS")
        .env_remove("LTTNG_HOME")
        .env_remove("R2D2_SHM_PATH")
        .env_remove("R2D2_TRACER_MODE")
        .env_remove("COLCON_CURRENT_PREFIX")
        .env_remove("LD_PRELOAD");
    if let Some(preload) = tsan_ld_preload() {
        command.env("R2D2_TSAN_LD_PRELOAD", preload);
        command.env(
            "TSAN_OPTIONS",
            "halt_on_error=0:exitcode=0:report_signal_unsafe=0:symbolize=0",
        );
    } else {
        command.env_remove("R2D2_TSAN_LD_PRELOAD");
        command.env_remove("TSAN_OPTIONS");
    }
    Ok(command)
}

#[derive(Debug, Clone)]
pub struct Ros2TopicOptions {
    pub qos_profile: Option<String>,
    pub qos_depth: Option<u32>,
    pub qos_history: Option<String>,
    pub qos_reliability: Option<String>,
    pub qos_durability: Option<String>,
    pub wait_matching_subscriptions: Option<u32>,
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
            wait_matching_subscriptions: Some(0),
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
        let commands = load_ros2_sender_commands()?;
        let args = build_topic_args(
            &commands.topic,
            &self.options,
            &self.topic_name,
            &self.message_type,
            values,
        );

        run_ros2_cli_with_timeout(
            &self.ros_setup,
            &self.install_setup,
            &self.domain_id,
            &args,
            &format!("topic {}", self.topic_name),
            &commands.ros2_cli,
            ros2_topic_timeout_sec(),
        )
    }
}

fn build_topic_args(
    template: &TopicCommand,
    options: &Ros2TopicOptions,
    topic_name: &str,
    message_type: &str,
    values: String,
) -> Vec<String> {
    let mut args = template.prefix.clone();
    if let Some(wait_matching_subscriptions) = options.wait_matching_subscriptions {
        args.push(template.wait_matching_subscriptions_option.clone());
        args.push(wait_matching_subscriptions.to_string());
    }
    args.push(template.keep_alive_option.clone());
    args.push(options.keep_alive_sec.to_string());
    if let Some(profile) = &options.qos_profile {
        args.push(template.qos_profile_option.clone());
        args.push(profile.clone());
    }
    if let Some(depth) = options.qos_depth {
        args.push(template.qos_depth_option.clone());
        args.push(depth.to_string());
    }
    if let Some(history) = &options.qos_history {
        args.push(template.qos_history_option.clone());
        args.push(history.clone());
    }
    if let Some(reliability) = &options.qos_reliability {
        args.push(template.qos_reliability_option.clone());
        args.push(reliability.clone());
    }
    if let Some(durability) = &options.qos_durability {
        args.push(template.qos_durability_option.clone());
        args.push(durability.clone());
    }
    args.push(topic_name.to_string());
    args.push(message_type.to_string());
    args.push(values);
    args
}

pub fn preview_ros2_topic_command(
    interface: &Interface,
    topic_name: &str,
    message_type: &str,
    options: &Ros2TopicOptions,
    payload: &Payload,
) -> Result<Ros2CommandPreview, Error> {
    let normalized = normalize_value(&interface.name, payload);
    let values = render_cli_payload(&normalized, &interface.fields)?;
    let commands = load_ros2_sender_commands()?;
    let args = build_topic_args(
        &commands.topic,
        options,
        topic_name,
        message_type,
        values.clone(),
    );
    Ok(Ros2CommandPreview {
        label: format!("topic {topic_name}"),
        command_line: render_ros2_cli_invocation(&args),
        args,
        payload_yaml: values,
    })
}

pub struct Ros2ServiceSender {
    ros_setup: PathBuf,
    install_setup: PathBuf,
    domain_id: String,
    service_name: String,
    service_type: String,
    interface: Interface,
    timeout_sec: Option<u64>,
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
            timeout_sec: None,
        }
    }

    pub fn with_timeout_sec(mut self, timeout_sec: Option<u64>) -> Self {
        self.timeout_sec = timeout_sec.filter(|value| *value > 0);
        self
    }
}

impl Sender for Ros2ServiceSender {
    fn send(&self, payload: &Payload) -> Result<(), Error> {
        let normalized = normalize_value(&self.interface.name, payload);
        let values = render_cli_payload(&normalized, &self.interface.fields)?;
        let commands = load_ros2_sender_commands()?;
        let mut args = commands.service.prefix.clone();
        args.extend([self.service_name.clone(), self.service_type.clone(), values]);
        run_ros2_cli_with_timeout(
            &self.ros_setup,
            &self.install_setup,
            &self.domain_id,
            &args,
            &format!("service {}", self.service_name),
            &commands.ros2_cli,
            self.timeout_sec.unwrap_or_else(ros2_service_timeout_sec),
        )
    }
}

pub struct Ros2ActionSender {
    ros_setup: PathBuf,
    install_setup: PathBuf,
    domain_id: String,
    action_name: String,
    action_type: String,
    interface: Interface,
}

impl Ros2ActionSender {
    pub fn new(
        ros_setup: impl Into<PathBuf>,
        install_setup: impl Into<PathBuf>,
        domain_id: impl Into<String>,
        action_name: impl Into<String>,
        action_type: impl Into<String>,
        interface: Interface,
    ) -> Self {
        Self {
            ros_setup: ros_setup.into(),
            install_setup: install_setup.into(),
            domain_id: domain_id.into(),
            action_name: action_name.into(),
            action_type: action_type.into(),
            interface,
        }
    }
}

impl Sender for Ros2ActionSender {
    fn send(&self, payload: &Payload) -> Result<(), Error> {
        let normalized = normalize_value(&self.interface.name, payload);
        let values = render_cli_payload(&normalized, &self.interface.fields)?;
        let commands = load_ros2_sender_commands()?;
        let args = build_action_args(
            &commands.action,
            &self.action_name,
            &self.action_type,
            values,
        );
        run_ros2_cli_with_timeout(
            &self.ros_setup,
            &self.install_setup,
            &self.domain_id,
            &args,
            &format!("action {}", self.action_name),
            &commands.ros2_cli,
            ros2_action_cli_timeout_sec(),
        )
    }
}

pub fn send_ros2_topic_yaml(
    ros_setup: &Path,
    install_setup: &Path,
    domain_id: &str,
    topic_name: &str,
    message_type: &str,
    values: &YamlValue,
    options: &Ros2TopicOptions,
) -> Result<(), Error> {
    let commands = load_ros2_sender_commands()?;
    let args = build_topic_args(
        &commands.topic,
        options,
        topic_name,
        message_type,
        render_raw_cli_payload(values)?,
    );
    run_ros2_cli_with_timeout(
        ros_setup,
        install_setup,
        domain_id,
        &args,
        &format!("topic {topic_name}"),
        &commands.ros2_cli,
        ros2_topic_timeout_sec(),
    )
}

pub fn send_ros2_service_yaml(
    ros_setup: &Path,
    install_setup: &Path,
    domain_id: &str,
    service_name: &str,
    service_type: &str,
    values: &YamlValue,
) -> Result<(), Error> {
    let commands = load_ros2_sender_commands()?;
    let mut args = commands.service.prefix.clone();
    args.extend([
        service_name.to_string(),
        service_type.to_string(),
        render_raw_cli_payload(values)?,
    ]);
    run_ros2_cli_with_timeout(
        ros_setup,
        install_setup,
        domain_id,
        &args,
        &format!("service {service_name}"),
        &commands.ros2_cli,
        ros2_service_timeout_sec(),
    )
}

pub fn send_ros2_parameter_value(
    ros_setup: &Path,
    install_setup: &Path,
    domain_id: &str,
    node_name: &str,
    parameter_name: &str,
    value: &YamlValue,
) -> Result<(), Error> {
    let commands = load_ros2_sender_commands()?;
    let mut args = commands.parameter.prefix.clone();
    args.extend([
        node_name.to_string(),
        parameter_name.to_string(),
        cli_scalar_or_yaml(value)?,
    ]);
    run_ros2_cli(
        ros_setup,
        install_setup,
        domain_id,
        &args,
        &format!("parameter {node_name}.{parameter_name}"),
        &commands.ros2_cli,
    )
}

fn run_ros2_cli(
    ros_setup: &Path,
    install_setup: &Path,
    domain_id: &str,
    args: &[String],
    label: &str,
    cli_template: &CommandTemplate,
) -> Result<(), Error> {
    let timeout_sec = ros2_cli_timeout_sec();
    run_ros2_cli_with_timeout(
        ros_setup,
        install_setup,
        domain_id,
        args,
        label,
        cli_template,
        timeout_sec,
    )
}

fn run_ros2_cli_with_timeout(
    ros_setup: &Path,
    install_setup: &Path,
    domain_id: &str,
    args: &[String],
    label: &str,
    cli_template: &CommandTemplate,
    timeout_sec: u64,
) -> Result<(), Error> {
    let output = build_ros2_cli_command_with_timeout(
        cli_template,
        ros_setup,
        install_setup,
        domain_id,
        args,
        timeout_sec,
    )?
    .stdout(Stdio::null())
    .output()
    .map_err(|error| Error::Unsupported(format!("failed to start {label}: {error}")))?;
    if output.status.success() {
        Ok(())
    } else if output.status.code() == Some(124) {
        Err(Error::Unsupported(format!(
            "{label} timed out after {timeout_sec}s"
        )))
    } else {
        let stderr = String::from_utf8_lossy(&output.stderr).trim().to_string();
        if stderr.is_empty() {
            Err(Error::Unsupported(format!(
                "{label} exited with {}",
                output.status
            )))
        } else {
            Err(Error::Unsupported(format!("{label} failed: {stderr}")))
        }
    }
}

pub fn ros2_cli_command(
    ros_setup: &Path,
    install_setup: &Path,
    domain_id: &str,
    args: &[String],
) -> Command {
    let commands = load_ros2_sender_commands().unwrap_or_else(|error| {
        panic!("cannot construct ROS 2 CLI command: {error}");
    });
    build_ros2_cli_command_with_timeout(
        &commands.ros2_cli,
        ros_setup,
        install_setup,
        domain_id,
        args,
        ros2_cli_timeout_sec(),
    )
    .unwrap_or_else(|error| panic!("cannot construct ROS 2 CLI command: {error}"))
}

fn build_ros2_cli_command_with_timeout(
    template: &CommandTemplate,
    ros_setup: &Path,
    install_setup: &Path,
    domain_id: &str,
    args: &[String],
    timeout_sec: u64,
) -> Result<Command, Error> {
    let mut parameters = CommandParameters::new();
    parameters
        .insert_path("ros_setup", ros_setup)
        .insert_path("install_setup", install_setup)
        .insert("domain_id", domain_id)
        .insert_many("cli_args", args.iter().cloned());
    let mut command = template
        .build("ros2_cli", &parameters)
        .map_err(|error| Error::Unsupported(error.to_string()))?;
    let timeout_sec = timeout_sec.to_string();
    command
        .env(ROS2_CLI_TIMEOUT_ENV, timeout_sec)
        .env_remove("ASAN_OPTIONS")
        .env_remove("LTTNG_HOME")
        .env_remove("R2D2_SHM_PATH")
        .env_remove("R2D2_TRACER_MODE")
        .env_remove("COLCON_CURRENT_PREFIX")
        .env_remove("LD_PRELOAD");
    if let Some(cli_setup) = ros2_cli_plain_setup_path() {
        command.env(ROS2_CLI_SETUP_ENV, cli_setup);
    } else {
        command.env_remove(ROS2_CLI_SETUP_ENV);
    }
    if let Some(preload) = ros2_cli_tsan_ld_preload() {
        command.env("R2D2_TSAN_LD_PRELOAD", preload);
        command.env(
            "TSAN_OPTIONS",
            "halt_on_error=0:exitcode=0:report_signal_unsafe=0:symbolize=0",
        );
    } else {
        command.env_remove("R2D2_TSAN_LD_PRELOAD");
        command.env_remove("TSAN_OPTIONS");
    }
    Ok(command)
}

fn ros2_cli_timeout_sec() -> u64 {
    timeout_env_or_default(ROS2_CLI_TIMEOUT_ENV, ROS2_CLI_TIMEOUT_SEC)
}

fn ros2_topic_timeout_sec() -> u64 {
    timeout_env_or_default(ROS2_TOPIC_TIMEOUT_ENV, ROS2_TOPIC_TIMEOUT_SEC)
}

fn ros2_service_timeout_sec() -> u64 {
    timeout_env_or_default(ROS2_SERVICE_TIMEOUT_ENV, ROS2_SERVICE_TIMEOUT_SEC)
}

fn ros2_action_cli_timeout_sec() -> u64 {
    std::env::var(ROS2_ACTION_CLI_TIMEOUT_ENV)
        .ok()
        .and_then(|value| value.parse::<u64>().ok())
        .filter(|value| *value > 0)
        .unwrap_or_else(|| ros2_action_timeout_sec().saturating_add(5).max(8))
}

fn timeout_env_or_default(env_name: &str, default_value: u64) -> u64 {
    std::env::var(env_name)
        .ok()
        .and_then(|value| value.parse::<u64>().ok())
        .filter(|value| *value > 0)
        .unwrap_or(default_value)
}

fn ros2_action_timeout_sec() -> u64 {
    std::env::var(ROS2_ACTION_TIMEOUT_ENV)
        .ok()
        .and_then(|value| value.parse::<u64>().ok())
        .filter(|value| *value > 0)
        .unwrap_or(ROS2_ACTION_TIMEOUT_SEC)
}

fn ros2_cli_plain_setup_path() -> Option<PathBuf> {
    if !matches!(std::env::var("R2D2_PROFILE").as_deref(), Ok("sancov")) {
        return None;
    }
    std::env::var_os(ROS2_CLI_SETUP_ENV)
        .filter(|path| !path.is_empty())
        .map(PathBuf::from)
        .or_else(|| {
            ROS2_CLI_PLAIN_SETUP_CANDIDATES
                .iter()
                .map(|relative| Path::new(env!("CARGO_MANIFEST_DIR")).join(relative))
                .find(|path| path.is_file())
        })
}

fn build_action_args(
    template: &ActionCommand,
    action_name: &str,
    action_type: &str,
    values: String,
) -> Vec<String> {
    let mut args = template.prefix.clone();
    args.push(template.timeout_option.clone());
    args.push(ros2_action_timeout_sec().to_string());
    args.push(action_name.to_string());
    args.push(action_type.to_string());
    args.push(values);
    args
}

pub fn preview_ros2_service_command(
    interface: &Interface,
    service_name: &str,
    service_type: &str,
    payload: &Payload,
) -> Result<Ros2CommandPreview, Error> {
    let normalized = normalize_value(&interface.name, payload);
    let values = render_cli_payload(&normalized, &interface.fields)?;
    let commands = load_ros2_sender_commands()?;
    let mut args = commands.service.prefix.clone();
    args.extend([
        service_name.to_string(),
        service_type.to_string(),
        values.clone(),
    ]);
    Ok(Ros2CommandPreview {
        label: format!("service {service_name}"),
        command_line: render_ros2_cli_invocation(&args),
        args,
        payload_yaml: values,
    })
}

pub fn preview_ros2_action_command(
    interface: &Interface,
    action_name: &str,
    action_type: &str,
    payload: &Payload,
) -> Result<Ros2CommandPreview, Error> {
    let normalized = normalize_value(&interface.name, payload);
    let values = render_cli_payload(&normalized, &interface.fields)?;
    let commands = load_ros2_sender_commands()?;
    let args = build_action_args(&commands.action, action_name, action_type, values.clone());
    Ok(Ros2CommandPreview {
        label: format!("action {action_name}"),
        command_line: render_ros2_cli_invocation(&args),
        args,
        payload_yaml: values,
    })
}

pub fn preview_ros2_parameter_command(
    node_name: &str,
    parameter_name: &str,
    value: &YamlValue,
) -> Result<Ros2CommandPreview, Error> {
    let commands = load_ros2_sender_commands()?;
    let value_arg = cli_scalar_or_yaml(value)?;
    let mut args = commands.parameter.prefix.clone();
    args.extend([
        node_name.to_string(),
        parameter_name.to_string(),
        value_arg.clone(),
    ]);
    Ok(Ros2CommandPreview {
        label: format!("parameter {node_name}.{parameter_name}"),
        command_line: render_ros2_cli_invocation(&args),
        args,
        payload_yaml: value_arg,
    })
}

pub fn render_ros2_cli_invocation(args: &[String]) -> String {
    let has_executable = args.first().is_some_and(|arg| arg == "ros2");
    let prefix = (!has_executable).then(|| "ros2".to_string());
    prefix
        .into_iter()
        .chain(args.iter().map(|arg| shell_quote(arg)))
        .collect::<Vec<_>>()
        .join(" ")
}

fn shell_quote(value: &str) -> String {
    if !value.is_empty()
        && value
            .chars()
            .all(|ch| ch.is_ascii_alphanumeric() || matches!(ch, '/' | '.' | '-' | '_' | ':' | '='))
    {
        return value.to_string();
    }
    format!("'{}'", value.replace('\'', "'\"'\"'"))
}

fn render_cli_payload(value: &ValueTree, fields: &[Field]) -> Result<String, Error> {
    let yaml = render_message(value, fields)?;
    serde_json::to_string(&yaml)
        .map_err(|error| Error::Unsupported(format!("failed to render JSON payload: {error}")))
}

fn render_raw_cli_payload(value: &YamlValue) -> Result<String, Error> {
    serde_json::to_string(value)
        .map_err(|error| Error::Unsupported(format!("failed to render JSON payload: {error}")))
}

fn cli_scalar_or_yaml(value: &YamlValue) -> Result<String, Error> {
    match value {
        YamlValue::Null => Ok("null".to_string()),
        YamlValue::Bool(value) => Ok(value.to_string()),
        YamlValue::Number(value) => Ok(value.to_string()),
        YamlValue::String(value) => Ok(value.clone()),
        _ => render_raw_cli_payload(value),
    }
}

pub fn validate_yaml_payload(value: &YamlValue, fields: &[Field]) -> Result<(), Error> {
    validate_yaml_mapping(value, fields, "$")
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

fn validate_yaml_mapping(value: &YamlValue, fields: &[Field], path: &str) -> Result<(), Error> {
    let YamlValue::Mapping(mapping) = value else {
        return Err(Error::TypeMismatch {
            expected: format!("YAML mapping for {path}"),
            found: yaml_type_name(value).to_string(),
        });
    };
    for key in mapping.keys() {
        let Some(name) = key.as_str() else {
            return Err(Error::TypeMismatch {
                expected: format!("string field name in {path}"),
                found: yaml_type_name(key).to_string(),
            });
        };
        let Some(field) = fields.iter().find(|field| field.name == name) else {
            return Err(Error::Unsupported(format!(
                "unknown field {path}.{name} in YAML payload"
            )));
        };
        if let Some(child) = mapping.get(key) {
            validate_yaml_value(child, &field.ty, &format!("{path}.{name}"))?;
        }
    }
    Ok(())
}

fn validate_yaml_value(value: &YamlValue, ty: &TypeNode, path: &str) -> Result<(), Error> {
    match ty {
        TypeNode::Constrained(inner, constraint) => {
            validate_yaml_value(value, inner, path)?;
            validate_yaml_constraint(value, constraint, path)
        }
        TypeNode::Primitive(primitive) => validate_yaml_primitive(value, *primitive, path),
        TypeNode::Nested(fields) => validate_yaml_mapping(value, fields, path),
        TypeNode::Array(element, fixed_len) => {
            let YamlValue::Sequence(items) = value else {
                return Err(Error::TypeMismatch {
                    expected: format!("YAML sequence for {path}"),
                    found: yaml_type_name(value).to_string(),
                });
            };
            if let Some(expected) = fixed_len
                && items.len() != *expected
            {
                return Err(Error::TypeMismatch {
                    expected: format!("fixed array of {expected} elements at {path}"),
                    found: format!("array of {} elements", items.len()),
                });
            }
            for (index, item) in items.iter().enumerate() {
                validate_yaml_value(item, element, &format!("{path}[{index}]"))?;
            }
            Ok(())
        }
    }
}

fn validate_yaml_primitive(
    value: &YamlValue,
    primitive: Primitive,
    path: &str,
) -> Result<(), Error> {
    let ok = match primitive {
        Primitive::Bool => matches!(value, YamlValue::Bool(_)),
        Primitive::I8 => yaml_i64(value).is_some_and(|value| i8::try_from(value).is_ok()),
        Primitive::U8 => yaml_u64(value).is_some_and(|value| u8::try_from(value).is_ok()),
        Primitive::I16 => yaml_i64(value).is_some_and(|value| i16::try_from(value).is_ok()),
        Primitive::U16 => yaml_u64(value).is_some_and(|value| u16::try_from(value).is_ok()),
        Primitive::I32 => yaml_i64(value).is_some_and(|value| i32::try_from(value).is_ok()),
        Primitive::U32 => yaml_u64(value).is_some_and(|value| u32::try_from(value).is_ok()),
        Primitive::I64 => yaml_i64(value).is_some(),
        Primitive::U64 => yaml_u64(value).is_some(),
        Primitive::F32 | Primitive::F64 => yaml_f64(value).is_some(),
        Primitive::String => matches!(value, YamlValue::String(_)),
    };
    if ok {
        Ok(())
    } else {
        Err(Error::TypeMismatch {
            expected: format!("{primitive:?} value at {path}"),
            found: yaml_type_name(value).to_string(),
        })
    }
}

fn validate_yaml_constraint(
    value: &YamlValue,
    constraint: &Constraint,
    path: &str,
) -> Result<(), Error> {
    match (value, constraint) {
        (YamlValue::String(text), Constraint::StringMaxLen(max)) if text.len() > *max => {
            Err(Error::TypeMismatch {
                expected: format!("string of at most {max} bytes at {path}"),
                found: format!("string of {} bytes", text.len()),
            })
        }
        (YamlValue::Sequence(items), Constraint::ArrayMaxLen(max)) if items.len() > *max => {
            Err(Error::TypeMismatch {
                expected: format!("array of at most {max} elements at {path}"),
                found: format!("array of {} elements", items.len()),
            })
        }
        _ => Ok(()),
    }
}

fn yaml_i64(value: &YamlValue) -> Option<i64> {
    value
        .as_i64()
        .or_else(|| value.as_u64().and_then(|value| i64::try_from(value).ok()))
}

fn yaml_u64(value: &YamlValue) -> Option<u64> {
    value.as_u64()
}

fn yaml_f64(value: &YamlValue) -> Option<f64> {
    value
        .as_f64()
        .or_else(|| yaml_i64(value).map(|value| value as f64))
}

fn yaml_type_name(value: &YamlValue) -> &'static str {
    match value {
        YamlValue::Null => "null",
        YamlValue::Bool(_) => "bool",
        YamlValue::Number(_) => "number",
        YamlValue::String(_) => "string",
        YamlValue::Sequence(_) => "sequence",
        YamlValue::Mapping(_) => "mapping",
        YamlValue::Tagged(_) => "tagged",
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
    if let Some(tf_message) = normalize_nav2_tf(interface_name, payload) {
        return tf_message;
    }
    if let Some(costmap_stream) = normalize_nav2_costmap_stream(interface_name, payload) {
        return costmap_stream;
    }
    if let Some(selector) = normalize_nav2_selector(interface_name, payload) {
        return selector;
    }
    match canonical_interface_name(interface_name) {
        "SpeedLimit" => normalize_speed_limit(payload),
        "Twist" => normalize_twist(payload),
        "TwistStamped" => normalize_twist_stamped(payload),
        "Polygon" => normalize_polygon(payload),
        "PolygonStamped" => normalize_polygon_stamped(payload, "map"),
        "PointCloud2" => normalize_point_cloud2(payload),
        "PoseStamped" => pose_stamped_from_payload(payload, 0.5),
        "PoseWithCovarianceStamped" => normalize_initial_pose(payload),
        "Odometry" => normalize_odometry(payload),
        "OccupancyGrid" => normalize_occupancy_grid(payload),
        "OccupancyGridUpdate" => normalize_occupancy_grid_update(payload),
        "Costmap" => normalize_costmap(payload, "map", "costmap"),
        "CostmapUpdate" => normalize_costmap_update(payload, "map"),
        "TFMessage" => normalize_tf_message(payload),
        "GetCosts" => normalize_get_costs(payload),
        "GetCostmap" => normalize_get_costmap(payload),
        "ClearCostmapExceptRegion" | "ClearCostmapAroundRobot" => {
            normalize_reset_distance_request(payload, 0.5)
        }
        "ClearCostmapAroundPose" => normalize_clear_around_pose(payload),
        "ClearEntireCostmap" => normalize_clear_entire_costmap(),
        "SetInitialPose" => normalize_set_initial_pose(payload),
        "SetBool" => normalize_set_bool(payload),
        "LoadMap" => normalize_load_map(payload),
        "IsPathValid" => normalize_is_path_valid(payload),
        "NavigateToPose" => normalize_navigate_to_pose(payload),
        "NavigateThroughPoses" => normalize_navigate_through_poses(payload),
        "ComputePathToPose" => normalize_compute_path_to_pose(payload),
        "ComputePathThroughPoses" => normalize_compute_path_through_poses(payload),
        "FollowPath" => normalize_follow_path(payload),
        "SmoothPath" => normalize_smooth_path(payload),
        "Spin" => normalize_spin(payload),
        "BackUp" => normalize_drive_on_heading(payload, -1.0),
        "DriveOnHeading" => normalize_drive_on_heading(payload, 1.0),
        "AssistedTeleop" => normalize_assisted_teleop(payload),
        "Wait" => normalize_wait(payload),
        _ => payload.value.clone(),
    }
}

fn normalize_nav2_tf(interface_name: &str, payload: &Payload) -> Option<ValueTree> {
    match interface_name {
        "TFMessage" => Some(normalize_tf_message(payload)),
        "TFMessage@tf_static" => Some(normalize_tf_static_message()),
        _ => None,
    }
}

fn normalize_nav2_selector(interface_name: &str, payload: &Payload) -> Option<ValueTree> {
    let candidates = match interface_name {
        "String@planner_selector" => &NAV2_PLANNER_IDS[..],
        "String@controller_selector" => &NAV2_CONTROLLER_IDS[..],
        "String@goal_checker_selector" => &NAV2_GOAL_CHECKER_IDS[..],
        "String@progress_checker_selector" => &NAV2_PROGRESS_CHECKER_IDS[..],
        "String@path_handler_selector" => &NAV2_PATH_HANDLER_IDS[..],
        _ => return None,
    };
    let index = (payload.rng_seed as usize) % candidates.len();
    Some(ValueTree::Nested(vec![ValueTree::Leaf(Value::String(
        candidates[index].to_string(),
    ))]))
}

fn normalize_nav2_costmap_stream(interface_name: &str, payload: &Payload) -> Option<ValueTree> {
    match interface_name {
        "Costmap@local_costmap_raw" => Some(normalize_costmap(payload, "odom", "local_costmap")),
        "CostmapUpdate@local_costmap_raw_updates" => {
            Some(normalize_costmap_update(payload, "odom"))
        }
        "Costmap@global_costmap_raw" => Some(normalize_costmap(payload, "map", "global_costmap")),
        "CostmapUpdate@global_costmap_raw_updates" => {
            Some(normalize_costmap_update(payload, "map"))
        }
        "CostmapFilterInfo@keepout" => Some(normalize_costmap_filter_info(
            0,
            "keepout_filter_mask",
            0.0,
            1.0,
        )),
        "CostmapFilterInfo@speed" => Some(normalize_costmap_filter_info(
            1,
            "speed_filter_mask",
            100.0,
            -1.0,
        )),
        "OccupancyGrid@keepout_filter_mask" => Some(normalize_costmap_filter_mask(
            payload,
            FilterMaskKind::Keepout,
        )),
        "OccupancyGrid@speed_filter_mask" => Some(normalize_costmap_filter_mask(
            payload,
            FilterMaskKind::Speed,
        )),
        "PolygonStamped@local_published_footprint" => {
            Some(normalize_polygon_stamped(payload, "odom"))
        }
        "PolygonStamped@global_published_footprint" => {
            Some(normalize_polygon_stamped(payload, "map"))
        }
        _ => None,
    }
}

fn canonical_interface_name(interface_name: &str) -> &str {
    interface_name
        .split_once('@')
        .map(|(base, _)| base)
        .unwrap_or(interface_name)
}

fn normalize_speed_limit(payload: &Payload) -> ValueTree {
    let source = payload.value.as_nested();
    let percentage = source
        .and_then(|fields| fields.get(1))
        .and_then(ValueTree::as_bool)
        .unwrap_or_else(|| seeded_bool(payload.rng_seed, 22));
    let source_limit = source
        .and_then(|fields| fields.get(2))
        .and_then(scalar_to_f64)
        .filter(|value| value.is_finite());
    let speed_limit = if percentage {
        source_limit
            .map(|value| value.abs().clamp(1.0, 100.0))
            .unwrap_or_else(|| 10.0 + seeded_unit_interval(payload.rng_seed, 23) * 90.0)
    } else {
        source_limit
            .map(|value| value.abs().clamp(0.05, 0.80))
            .unwrap_or_else(|| 0.10 + ((payload.rng_seed % 40) as f64 * 0.01))
    };
    ValueTree::Nested(vec![
        header(""),
        ValueTree::Leaf(Value::Bool(percentage)),
        ValueTree::Leaf(Value::F64(speed_limit)),
    ])
}

fn normalize_twist(payload: &Payload) -> ValueTree {
    let linear = match payload.rng_seed % 3 {
        0 => 0.0,
        1 => 0.05,
        _ => -0.03,
    };
    let angular = match (payload.rng_seed / 3) % 3 {
        0 => 0.0,
        1 => 0.15,
        _ => -0.15,
    };
    ValueTree::Nested(vec![point(linear, 0.0, 0.0), point(0.0, 0.0, angular)])
}

fn normalize_twist_stamped(payload: &Payload) -> ValueTree {
    ValueTree::Nested(vec![header("base_footprint"), normalize_twist(payload)])
}

fn normalize_polygon(payload: &Payload) -> ValueTree {
    let front = 0.18 + seeded_unit_interval(payload.rng_seed, 24) as f32 * 0.08;
    let rear = 0.18 + seeded_unit_interval(payload.rng_seed, 25) as f32 * 0.08;
    let half_y = 0.14 + seeded_unit_interval(payload.rng_seed, 26) as f32 * 0.06;
    let shift_x = match payload.rng_seed % 3 {
        0 => -0.03,
        1 => 0.0,
        _ => 0.03,
    };
    ValueTree::Nested(vec![ValueTree::Array(vec![
        point32(shift_x - rear, -half_y, 0.0),
        point32(shift_x - rear, half_y, 0.0),
        point32(shift_x + front, half_y, 0.0),
        point32(shift_x + front, -half_y, 0.0),
    ])])
}

fn normalize_polygon_stamped(payload: &Payload, frame_id: &str) -> ValueTree {
    ValueTree::Nested(vec![header(frame_id), normalize_polygon(payload)])
}

fn normalize_costmap(payload: &Payload, frame_id: &str, layer: &str) -> ValueTree {
    let side = NAV2_COSTMAP_STREAM_SIDE;
    let len = side as usize * side as usize;
    let source = payload.value.as_nested();
    let data = source
        .and_then(|fields| fields.get(2))
        .map(|values| u8_cost_array(values, len))
        .unwrap_or_else(|| seeded_costmap_data(payload.rng_seed, side, side));
    ValueTree::Nested(vec![
        header(frame_id),
        costmap_metadata(frame_id, layer, side, side),
        u8_leaf_array(data),
    ])
}

fn normalize_costmap_update(payload: &Payload, frame_id: &str) -> ValueTree {
    let side = NAV2_COSTMAP_UPDATE_SIDE;
    let len = side as usize * side as usize;
    let source = payload.value.as_nested();
    let data = source
        .and_then(|fields| fields.get(5))
        .map(|values| u8_cost_array(values, len))
        .unwrap_or_else(|| seeded_costmap_data(payload.rng_seed.rotate_left(7), side, side));
    ValueTree::Nested(vec![
        header(frame_id),
        leaf_u32(0),
        leaf_u32(0),
        leaf_u32(side),
        leaf_u32(side),
        u8_leaf_array(data),
    ])
}

fn normalize_tf_message(payload: &Payload) -> ValueTree {
    let jitter_x = (seeded_unit_interval(payload.rng_seed, 61) - 0.5) * 0.08;
    let jitter_y = (seeded_unit_interval(payload.rng_seed, 62) - 0.5) * 0.08;
    let yaw = (seeded_unit_interval(payload.rng_seed, 63) - 0.5) * 0.5;
    ValueTree::Nested(vec![ValueTree::Array(vec![
        transform_stamped("map", "odom", 0.0, 0.0, 0.0, [0.0, 0.0, 0.0, 1.0]),
        transform_stamped(
            "odom",
            "base_footprint",
            NAV2_SAFE_START_X + jitter_x,
            NAV2_SAFE_START_Y + jitter_y,
            0.0,
            yaw_quaternion(yaw),
        ),
    ])])
}

fn normalize_tf_static_message() -> ValueTree {
    ValueTree::Nested(vec![ValueTree::Array(vec![
        laser_frame_transform("laser_frame"),
        laser_frame_transform("base_scan"),
        laser_frame_transform("rplidar_link"),
    ])])
}

fn laser_frame_transform(child_frame_id: &str) -> ValueTree {
    transform_stamped(
        "base_footprint",
        child_frame_id,
        0.12,
        0.0,
        0.18,
        [0.0, 0.0, 0.0, 1.0],
    )
}

fn costmap_metadata(frame_id: &str, layer: &str, size_x: u32, size_y: u32) -> ValueTree {
    let origin = match frame_id {
        "odom" => pose(
            NAV2_SAFE_START_X - NAV2_COSTMAP_STREAM_RESOLUTION as f64 * size_x as f64 * 0.5,
            NAV2_SAFE_START_Y - NAV2_COSTMAP_STREAM_RESOLUTION as f64 * size_y as f64 * 0.5,
            0.0,
            [0.0, 0.0, 0.0, 1.0],
        ),
        _ => pose(
            -NAV2_COSTMAP_STREAM_RESOLUTION as f64 * size_x as f64 * 0.5,
            -NAV2_COSTMAP_STREAM_RESOLUTION as f64 * size_y as f64 * 0.5,
            0.0,
            [0.0, 0.0, 0.0, 1.0],
        ),
    };
    ValueTree::Nested(vec![
        time_zero(),
        time_zero(),
        ValueTree::Leaf(Value::String(layer.to_string())),
        ValueTree::Leaf(Value::F32(NAV2_COSTMAP_STREAM_RESOLUTION)),
        leaf_u32(size_x),
        leaf_u32(size_y),
        origin,
    ])
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

fn normalize_initial_pose(payload: &Payload) -> ValueTree {
    let source = payload.value.as_nested();
    let x = source
        .and_then(|fields| scalar_to_f64(fields.first()?))
        .unwrap_or(NAV2_SAFE_START_X)
        .clamp(-2.5, 2.5);
    let y = source
        .and_then(|fields| scalar_to_f64(fields.get(1)?))
        .unwrap_or(NAV2_SAFE_START_Y)
        .clamp(-2.5, 2.5);
    let pose = pose(x, y, 0.0, [0.0, 0.0, 0.0, 1.0]);
    ValueTree::Nested(vec![header("map"), pose_with_covariance(pose)])
}

fn normalize_odometry(_payload: &Payload) -> ValueTree {
    ValueTree::Nested(vec![
        header("odom"),
        ValueTree::Leaf(Value::String("base_footprint".to_string())),
        pose_with_covariance(pose(
            NAV2_SAFE_START_X,
            NAV2_SAFE_START_Y,
            0.0,
            [0.0, 0.0, 0.0, 1.0],
        )),
        twist_with_covariance(0.0, 0.0),
    ])
}

fn normalize_occupancy_grid(payload: &Payload) -> ValueTree {
    let source = payload.value.as_nested();
    let len = SAFE_GRID_SIDE as usize * SAFE_GRID_SIDE as usize;
    let mut data = source
        .and_then(|fields| fields.get(2))
        .map(|values| int8_array(values, len))
        .unwrap_or_else(|| vec![0; len]);
    keep_safe_waypoints_free(&mut data, SAFE_GRID_SIDE, SAFE_GRID_SIDE);
    ValueTree::Nested(vec![
        header("map"),
        ValueTree::Nested(vec![
            time_zero(),
            ValueTree::Leaf(Value::F32(SAFE_GRID_RESOLUTION)),
            leaf_u32(SAFE_GRID_SIDE),
            leaf_u32(SAFE_GRID_SIDE),
            pose(
                SAFE_GRID_ORIGIN_X,
                SAFE_GRID_ORIGIN_Y,
                0.0,
                [0.0, 0.0, 0.0, 1.0],
            ),
        ]),
        int8_leaf_array(data),
    ])
}

#[derive(Clone, Copy)]
enum FilterMaskKind {
    Keepout,
    Speed,
}

fn normalize_costmap_filter_info(
    filter_type: u8,
    mask_topic: &str,
    base: f32,
    multiplier: f32,
) -> ValueTree {
    ValueTree::Nested(vec![
        header("map"),
        ValueTree::Leaf(Value::U8(filter_type)),
        ValueTree::Leaf(Value::String(mask_topic.to_string())),
        ValueTree::Leaf(Value::F32(base)),
        ValueTree::Leaf(Value::F32(multiplier)),
    ])
}

fn normalize_costmap_filter_mask(payload: &Payload, kind: FilterMaskKind) -> ValueTree {
    let source = payload.value.as_nested();
    let len = SAFE_GRID_SIDE as usize * SAFE_GRID_SIDE as usize;
    let mut data = source
        .and_then(|fields| fields.get(2))
        .map(|values| int8_array(values, len))
        .unwrap_or_else(|| {
            seeded_filter_mask_data(payload.rng_seed, SAFE_GRID_SIDE, SAFE_GRID_SIDE, kind)
        });
    keep_safe_waypoints_free(&mut data, SAFE_GRID_SIDE, SAFE_GRID_SIDE);
    ValueTree::Nested(vec![
        header("map"),
        ValueTree::Nested(vec![
            time_zero(),
            ValueTree::Leaf(Value::F32(SAFE_GRID_RESOLUTION)),
            leaf_u32(SAFE_GRID_SIDE),
            leaf_u32(SAFE_GRID_SIDE),
            pose(
                SAFE_GRID_ORIGIN_X,
                SAFE_GRID_ORIGIN_Y,
                0.0,
                [0.0, 0.0, 0.0, 1.0],
            ),
        ]),
        int8_leaf_array(data),
    ])
}

fn normalize_occupancy_grid_update(payload: &Payload) -> ValueTree {
    let source = payload.value.as_nested();
    let width = 8;
    let height = 8;
    let x = 0;
    let y = 0;
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

fn normalize_get_costs(payload: &Payload) -> ValueTree {
    let source = payload.value.as_nested();
    ValueTree::Nested(vec![
        ValueTree::Leaf(Value::Bool(
            source
                .and_then(|fields| fields.first())
                .and_then(ValueTree::as_bool)
                .unwrap_or_else(|| seeded_bool(payload.rng_seed, 22)),
        )),
        ValueTree::Array(cost_query_poses_from_payload(payload)),
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
        .clamp(1, SAFE_GRID_SIDE);
    let size_y = specs
        .and_then(|fields| scalar_to_u32(fields.get(5)?))
        .unwrap_or(16)
        .clamp(1, SAFE_GRID_SIDE);
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
    ValueTree::Nested(vec![
        ValueTree::Leaf(Value::F32(distance)),
        empty_string_array(),
    ])
}

fn normalize_clear_around_pose(payload: &Payload) -> ValueTree {
    let source = payload.value.as_nested();
    let fallback_pose = safe_waypoint_for_payload(payload, 26);
    let fallback_quaternion = yaw_quaternion(yaw_from_seed(payload.rng_seed, 27));
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
        .unwrap_or(fallback_pose.0)
        .clamp(-2.5, 2.5);
    let y = position
        .and_then(|fields| scalar_to_f64(fields.get(1)?))
        .unwrap_or(fallback_pose.1)
        .clamp(-2.5, 2.5);
    let z = position
        .and_then(|fields| scalar_to_f64(fields.get(2)?))
        .unwrap_or(0.0)
        .clamp(-0.1, 0.1);
    let q = normalize_quaternion([
        orientation
            .and_then(|fields| scalar_to_f64(fields.first()?))
            .unwrap_or(fallback_quaternion[0]),
        orientation
            .and_then(|fields| scalar_to_f64(fields.get(1)?))
            .unwrap_or(fallback_quaternion[1]),
        orientation
            .and_then(|fields| scalar_to_f64(fields.get(2)?))
            .unwrap_or(fallback_quaternion[2]),
        orientation
            .and_then(|fields| scalar_to_f64(fields.get(3)?))
            .unwrap_or(fallback_quaternion[3]),
    ]);
    let reset_distance = source
        .and_then(|fields| fields.get(1))
        .and_then(scalar_to_f64)
        .unwrap_or(0.5)
        .clamp(0.1, 5.0);
    ValueTree::Nested(vec![
        ValueTree::Nested(vec![header("map"), pose(x, y, z, q)]),
        ValueTree::Leaf(Value::F64(reset_distance)),
        empty_string_array(),
    ])
}

fn normalize_clear_entire_costmap() -> ValueTree {
    ValueTree::Nested(vec![empty_string_array()])
}

fn normalize_set_initial_pose(payload: &Payload) -> ValueTree {
    ValueTree::Nested(vec![normalize_initial_pose(payload)])
}

fn normalize_set_bool(payload: &Payload) -> ValueTree {
    ValueTree::Nested(vec![ValueTree::Leaf(Value::Bool(seeded_bool(
        payload.rng_seed,
        0,
    )))])
}

fn normalize_load_map(payload: &Payload) -> ValueTree {
    let map_url = load_map_url_from_payload(payload);
    ValueTree::Nested(vec![ValueTree::Leaf(Value::String(map_url))])
}

fn load_map_url_from_payload(payload: &Payload) -> String {
    if let Some(map_url) = payload
        .value
        .as_nested()
        .and_then(|fields| fields.first())
        .and_then(ValueTree::as_string)
        .map(str::trim)
        .filter(|value| !value.is_empty())
        .filter(|value| Path::new(value).is_file())
    {
        return map_url.to_string();
    }

    if let Some(map_url) = std::env::var(SAFE_MAP_ENV)
        .ok()
        .map(|value| value.trim().to_string())
        .filter(|value| !value.is_empty())
    {
        return map_url;
    }

    let candidates = configured_safe_map_candidates();
    let index = seeded_index(payload.rng_seed, 11, candidates.len());
    candidates[index].clone()
}

fn configured_safe_map_candidates() -> Vec<String> {
    let configured = std::env::var(SAFE_MAP_LIST_ENV)
        .ok()
        .into_iter()
        .flat_map(|value| {
            value
                .split([':', ';'])
                .map(str::trim)
                .filter(|item| !item.is_empty())
                .map(ToOwned::to_owned)
                .collect::<Vec<_>>()
        })
        .filter(|value| Path::new(value).is_file())
        .collect::<Vec<_>>();
    if !configured.is_empty() {
        return configured;
    }

    let built_in = NAV2_LOAD_MAP_RELATIVES
        .iter()
        .map(|relative| {
            Path::new(env!("CARGO_MANIFEST_DIR"))
                .join(relative)
                .display()
                .to_string()
        })
        .filter(|value| Path::new(value).is_file())
        .collect::<Vec<_>>();
    if built_in.is_empty() {
        vec![
            Path::new(env!("CARGO_MANIFEST_DIR"))
                .join(SAFE_MAP_RELATIVE)
                .display()
                .to_string(),
        ]
    } else {
        built_in
    }
}

fn normalize_is_path_valid(payload: &Payload) -> ValueTree {
    let source = payload.value.as_nested();
    ValueTree::Nested(vec![
        path_from_payload(payload),
        ValueTree::Leaf(Value::U8(254)),
        ValueTree::Leaf(Value::Bool(
            source
                .and_then(|fields| fields.get(2))
                .and_then(ValueTree::as_bool)
                .unwrap_or_else(|| seeded_bool(payload.rng_seed, 28)),
        )),
        ValueTree::Leaf(Value::String(String::new())),
        ValueTree::Leaf(Value::String(String::new())),
        ValueTree::Leaf(Value::Bool(
            source
                .and_then(|fields| fields.get(5))
                .and_then(ValueTree::as_bool)
                .unwrap_or_else(|| seeded_bool(payload.rng_seed, 29)),
        )),
        ValueTree::Leaf(Value::F64(path_lookahead_from_payload(payload, 30))),
    ])
}

fn normalize_navigate_to_pose(payload: &Payload) -> ValueTree {
    ValueTree::Nested(vec![
        pose_stamped_from_payload(payload, 0.8),
        ValueTree::Leaf(Value::String(nav_to_pose_behavior_tree(payload))),
    ])
}

fn normalize_navigate_through_poses(payload: &Payload) -> ValueTree {
    ValueTree::Nested(vec![
        nav_goals_from_payload(payload, 1, 2, 4),
        ValueTree::Leaf(Value::String(nav_through_poses_behavior_tree(payload))),
    ])
}

fn normalize_compute_path_to_pose(payload: &Payload) -> ValueTree {
    ValueTree::Nested(vec![
        planner_goal_pose_from_payload(payload, 8),
        pose_stamped_with_yaw(
            NAV2_SAFE_START_X,
            NAV2_SAFE_START_Y,
            0.0,
            yaw_from_seed(payload.rng_seed, 4),
        ),
        planner_viapoints_from_payload(payload, 9),
        ValueTree::Leaf(Value::String("GridBased".to_string())),
        ValueTree::Leaf(Value::Bool(planner_use_start(payload.rng_seed, 5))),
    ])
}

fn normalize_compute_path_through_poses(payload: &Payload) -> ValueTree {
    ValueTree::Nested(vec![
        nav_goals_from_payload(payload, 3, 2, 5),
        pose_stamped_with_yaw(
            NAV2_SAFE_START_X,
            NAV2_SAFE_START_Y,
            0.0,
            yaw_from_seed(payload.rng_seed, 6),
        ),
        ValueTree::Leaf(Value::String("GridBased".to_string())),
        ValueTree::Leaf(Value::Bool(planner_use_start(payload.rng_seed, 7))),
    ])
}

fn normalize_follow_path(payload: &Payload) -> ValueTree {
    ValueTree::Nested(vec![
        follow_path_from_payload(payload),
        ValueTree::Leaf(Value::String("FollowPath".to_string())),
        ValueTree::Leaf(Value::String("general_goal_checker".to_string())),
        ValueTree::Leaf(Value::String("progress_checker".to_string())),
        ValueTree::Leaf(Value::String("PathHandler".to_string())),
    ])
}

fn normalize_smooth_path(payload: &Payload) -> ValueTree {
    ValueTree::Nested(vec![
        path_from_payload(payload),
        ValueTree::Leaf(Value::String(smoother_id_from_payload(payload))),
        duration(
            bounded_i32_from_seed(payload.rng_seed, 11, NAV2_SMOOTH_MAX_DURATION_SEC, 5),
            0,
        ),
        ValueTree::Leaf(Value::Bool(seeded_bool(payload.rng_seed, 12))),
    ])
}

fn normalize_spin(payload: &Payload) -> ValueTree {
    let yaw = payload
        .value
        .as_nested()
        .and_then(|fields| fields.first())
        .and_then(scalar_to_f32)
        .map(wrap_angle_f32)
        .unwrap_or_else(|| {
            let fraction = seeded_unit_interval(payload.rng_seed, 13) as f32;
            -PI as f32 + fraction * (2.0 * PI as f32)
        });
    ValueTree::Nested(vec![
        ValueTree::Leaf(Value::F32(yaw)),
        duration(
            bounded_i32_from_seed(payload.rng_seed, 14, NAV2_SPIN_TIME_ALLOWANCE_SEC, 10),
            0,
        ),
        ValueTree::Leaf(Value::Bool(seeded_bool(payload.rng_seed, 15))),
    ])
}

fn normalize_drive_on_heading(payload: &Payload, direction: f64) -> ValueTree {
    let direction = direction.signum();
    let distance = motion_distance_from_payload(payload, 16);
    let speed = motion_speed_from_payload(payload, 17) * direction;
    ValueTree::Nested(vec![
        point(distance * direction, 0.0, 0.0),
        ValueTree::Leaf(Value::F32(speed as f32)),
        duration(
            bounded_i32_from_seed(payload.rng_seed, 18, NAV2_BEHAVIOR_TIME_ALLOWANCE_SEC, 12),
            0,
        ),
        ValueTree::Leaf(Value::Bool(seeded_bool(payload.rng_seed, 19))),
    ])
}

fn normalize_assisted_teleop(payload: &Payload) -> ValueTree {
    ValueTree::Nested(vec![duration(
        bounded_i32_from_seed(payload.rng_seed, 20, NAV2_BEHAVIOR_TIME_ALLOWANCE_SEC, 12),
        0,
    )])
}

fn normalize_wait(payload: &Payload) -> ValueTree {
    let secs = bounded_i32_from_seed(payload.rng_seed, 21, NAV2_WAIT_DURATION_SEC, 6);
    ValueTree::Nested(vec![duration(secs, 0)])
}

fn pose_stamped_from_payload(payload: &Payload, fallback_x: f64) -> ValueTree {
    let (x, y) = safe_pose_xy_from_payload(payload, fallback_x, 0.0);
    pose_stamped_with_yaw(x, y, 0.0, yaw_from_seed(payload.rng_seed, 2))
}

fn pose_stamped_with_yaw(x: f64, y: f64, z: f64, yaw: f64) -> ValueTree {
    ValueTree::Nested(vec![header("map"), pose(x, y, z, yaw_quaternion(yaw))])
}

fn nav_goals_from_payload(
    payload: &Payload,
    first_salt: usize,
    min: usize,
    max: usize,
) -> ValueTree {
    let count = bounded_count_from_seed(payload.rng_seed, first_salt, min, max);
    let points = (0..count)
        .map(|index| {
            let (x, y) = safe_waypoint_for_payload(payload, first_salt + index);
            pose_stamped_with_yaw(
                x,
                y,
                0.0,
                yaw_from_seed(payload.rng_seed, first_salt + index),
            )
        })
        .collect::<Vec<_>>();
    ValueTree::Nested(vec![header("map"), ValueTree::Array(points)])
}

fn planner_goal_pose_from_payload(payload: &Payload, salt: usize) -> ValueTree {
    let (x, y) = safe_waypoint_for_payload(payload, salt);
    pose_stamped_with_yaw(x, y, 0.0, yaw_from_seed(payload.rng_seed, salt + 1))
}

fn planner_viapoints_from_payload(payload: &Payload, first_salt: usize) -> ValueTree {
    let count = bounded_count_from_seed(payload.rng_seed, first_salt, 0, 2);
    let mut points = Vec::with_capacity(count);
    for index in 0..count {
        let (x, y) = safe_waypoint_for_payload(payload, first_salt + index);
        points.push(pose_stamped_with_yaw(
            x,
            y,
            0.0,
            yaw_from_seed(payload.rng_seed, first_salt + index + 1),
        ));
    }
    ValueTree::Array(points)
}

fn planner_use_start(seed: u64, salt: usize) -> bool {
    // Most planner rounds use the explicit safe start pose, while one out of
    // four still exercises Nav2's "use current robot pose" path.
    !seed.rotate_right((salt as u32) % 32).is_multiple_of(4)
}

fn cost_query_poses_from_payload(payload: &Payload) -> Vec<ValueTree> {
    let count = bounded_count_from_seed(payload.rng_seed, 23, 1, 4);
    let mut poses = Vec::with_capacity(count);
    if let Some((x, y)) = extract_pose_xy(&payload.value) {
        let (x, y) = nearest_safe_waypoint((x, y));
        poses.push(pose_stamped_with_yaw(
            x,
            y,
            0.0,
            yaw_from_seed(payload.rng_seed, 24),
        ));
    }
    while poses.len() < count {
        let salt = 24 + poses.len();
        let (x, y) = safe_waypoint_for_payload(payload, salt);
        poses.push(pose_stamped_with_yaw(
            x,
            y,
            0.0,
            yaw_from_seed(payload.rng_seed, salt),
        ));
    }
    poses
}

fn bounded_count_from_seed(seed: u64, salt: usize, min: usize, max: usize) -> usize {
    if max <= min {
        return min;
    }
    let span = max - min + 1;
    min + ((seed.rotate_right((salt as u32) % 32) as usize) % span)
}

fn bounded_i32_from_seed(seed: u64, salt: usize, min: i32, max: i32) -> i32 {
    if max <= min {
        return min;
    }
    let span = (max - min + 1) as u64;
    min + (seed.rotate_right((salt as u32) % 32) % span) as i32
}

fn seeded_index(seed: u64, salt: usize, len: usize) -> usize {
    if len == 0 {
        return 0;
    }
    let mut mixed = seed ^ (salt as u64).wrapping_mul(0x9e37_79b9_7f4a_7c15);
    mixed ^= mixed >> 33;
    mixed = mixed.wrapping_mul(0xff51_afd7_ed55_8ccd);
    mixed ^= mixed >> 33;
    (mixed % len as u64) as usize
}

fn seeded_bool(seed: u64, salt: usize) -> bool {
    seed.rotate_right((salt as u32) % 32) & 1 == 1
}

fn seeded_unit_interval(seed: u64, salt: usize) -> f64 {
    let bucket = seed.rotate_right((salt as u32) % 32) % 10_000;
    bucket as f64 / 9_999.0
}

fn nav_to_pose_behavior_tree(payload: &Payload) -> String {
    nav2_behavior_tree_from_seed(payload.rng_seed, &NAV2_NAV_TO_POSE_BT_XMLS)
}

fn nav_through_poses_behavior_tree(payload: &Payload) -> String {
    nav2_behavior_tree_from_seed(payload.rng_seed, &NAV2_NAV_THROUGH_POSES_BT_XMLS)
}

fn nav2_behavior_tree_from_seed(seed: u64, candidates: &[&str]) -> String {
    let index = (seed as usize) % candidates.len();
    let file_name = candidates[index];
    if file_name.is_empty() {
        String::new()
    } else {
        nav2_behavior_tree_path(file_name)
    }
}

fn nav2_behavior_tree_path(file_name: &str) -> String {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join(NAV2_BT_TREE_RELATIVE_DIR)
        .join(file_name)
        .display()
        .to_string()
}

fn yaw_from_seed(seed: u64, salt: usize) -> f64 {
    -PI + seeded_unit_interval(seed, salt) * (2.0 * PI)
}

fn smoother_id_from_payload(payload: &Payload) -> String {
    let index = (payload.rng_seed as usize) % NAV2_SMOOTHER_IDS.len();
    NAV2_SMOOTHER_IDS[index].to_string()
}

fn wrap_angle_f32(value: f32) -> f32 {
    if !value.is_finite() {
        return 0.0;
    }
    let two_pi = 2.0 * PI as f32;
    let wrapped = (value + PI as f32).rem_euclid(two_pi) - PI as f32;
    if wrapped == -PI as f32 {
        PI as f32
    } else {
        wrapped
    }
}

fn motion_distance_from_payload(payload: &Payload, salt: usize) -> f64 {
    payload
        .value
        .as_nested()
        .and_then(|fields| fields.first())
        .and_then(extract_motion_distance)
        .filter(|value| value.is_finite())
        .map(|value| value.abs().clamp(0.15, 0.8))
        .unwrap_or_else(|| 0.15 + seeded_unit_interval(payload.rng_seed, salt) * 0.65)
}

fn extract_motion_distance(value: &ValueTree) -> Option<f64> {
    match value {
        ValueTree::Nested(fields) => fields.first().and_then(scalar_to_f64),
        _ => scalar_to_f64(value),
    }
}

fn motion_speed_from_payload(payload: &Payload, salt: usize) -> f64 {
    payload
        .value
        .as_nested()
        .and_then(|fields| fields.get(1))
        .and_then(scalar_to_f64)
        .filter(|value| value.is_finite())
        .map(|value| value.abs().clamp(0.05, 0.30))
        .unwrap_or_else(|| 0.05 + seeded_unit_interval(payload.rng_seed, salt) * 0.25)
}

fn path_lookahead_from_payload(payload: &Payload, salt: usize) -> f64 {
    payload
        .value
        .as_nested()
        .and_then(|fields| fields.get(6))
        .and_then(scalar_to_f64)
        .filter(|value| value.is_finite())
        .map(|value| {
            if value <= 0.0 {
                -1.0
            } else {
                value.clamp(0.25, 3.0)
            }
        })
        .unwrap_or_else(|| {
            if seeded_bool(payload.rng_seed, salt) {
                -1.0
            } else {
                0.25 + seeded_unit_interval(payload.rng_seed, salt + 1) * 2.75
            }
        })
}

fn safe_pose_xy_from_payload(payload: &Payload, fallback_x: f64, fallback_y: f64) -> (f64, f64) {
    payload
        .value
        .as_nested()
        .and_then(|fields| {
            let x = fields.first().and_then(scalar_to_f64)?;
            let y = fields.get(1).and_then(scalar_to_f64)?;
            Some((x, y))
        })
        .or_else(|| extract_pose_xy(&payload.value))
        .map(nearest_safe_waypoint)
        .unwrap_or_else(|| {
            let default = nearest_safe_waypoint((fallback_x, fallback_y));
            let index =
                ((payload.rng_seed as usize) + waypoint_index(default)) % NAV2_SAFE_WAYPOINTS.len();
            NAV2_SAFE_WAYPOINTS[index]
        })
}

fn safe_waypoint_for_payload(payload: &Payload, salt: usize) -> (f64, f64) {
    if let Some((x, y)) = extract_pose_xy(&payload.value) {
        let index =
            (waypoint_index(nearest_safe_waypoint((x, y))) + salt) % NAV2_SAFE_WAYPOINTS.len();
        NAV2_SAFE_WAYPOINTS[index]
    } else {
        NAV2_SAFE_WAYPOINTS[((payload.rng_seed as usize) + salt) % NAV2_SAFE_WAYPOINTS.len()]
    }
}

fn extract_pose_xy(value: &ValueTree) -> Option<(f64, f64)> {
    let ValueTree::Nested(fields) = value else {
        return None;
    };
    if let Some(position) = pose_position_fields(fields) {
        let x = position.first().and_then(scalar_to_f64)?;
        let y = position.get(1).and_then(scalar_to_f64)?;
        return Some((x, y));
    }
    for child in fields {
        if let Some(xy) = extract_pose_xy(child) {
            return Some(xy);
        }
    }
    None
}

fn pose_position_fields(fields: &[ValueTree]) -> Option<&[ValueTree]> {
    let position = fields.first()?.as_nested()?;
    let orientation = fields.get(1)?.as_nested()?;
    if position.len() >= 3
        && orientation.len() >= 4
        && position
            .iter()
            .take(3)
            .all(|field| scalar_to_f64(field).is_some())
        && orientation
            .iter()
            .take(4)
            .all(|field| scalar_to_f64(field).is_some())
    {
        Some(position)
    } else {
        None
    }
}

fn nearest_safe_waypoint((x, y): (f64, f64)) -> (f64, f64) {
    NAV2_SAFE_WAYPOINTS
        .iter()
        .copied()
        .min_by(|a, b| squared_distance(*a, (x, y)).total_cmp(&squared_distance(*b, (x, y))))
        .unwrap_or((NAV2_SAFE_START_X, NAV2_SAFE_START_Y))
}

fn waypoint_index(point: (f64, f64)) -> usize {
    NAV2_SAFE_WAYPOINTS
        .iter()
        .position(|candidate| *candidate == point)
        .unwrap_or(0)
}

fn squared_distance(a: (f64, f64), b: (f64, f64)) -> f64 {
    (a.0 - b.0).powi(2) + (a.1 - b.1).powi(2)
}

fn path_from_payload(payload: &Payload) -> ValueTree {
    let points = local_path_points_from_payload(payload);
    let poses = points
        .iter()
        .enumerate()
        .map(|(index, point)| {
            let yaw = points
                .get(index + 1)
                .map(|next| (next.1 - point.1).atan2(next.0 - point.0))
                .or_else(|| {
                    index
                        .checked_sub(1)
                        .and_then(|previous_index| points.get(previous_index))
                        .map(|previous| (point.1 - previous.1).atan2(point.0 - previous.0))
                })
                .unwrap_or_else(|| yaw_from_seed(payload.rng_seed, index + 31));
            pose_stamped_with_yaw(point.0, point.1, 0.0, yaw)
        })
        .collect::<Vec<_>>();
    ValueTree::Nested(vec![header("map"), ValueTree::Array(poses)])
}

fn local_path_points_from_payload(payload: &Payload) -> Vec<(f64, f64)> {
    let route_index = seeded_index(payload.rng_seed, 3, NAV2_LOCAL_PATH_ROUTES.len());
    let mut route = NAV2_LOCAL_PATH_ROUTES[route_index].to_vec();
    if let Some(target) = extract_pose_xy(&payload.value).map(nearest_local_path_waypoint)
        && target != route[0]
    {
        route[2] = target;
    }
    densify_route(&route, NAV2_LOCAL_PATH_STEP_METERS)
}

fn nearest_local_path_waypoint((x, y): (f64, f64)) -> (f64, f64) {
    NAV2_LOCAL_PATH_ROUTES
        .iter()
        .flat_map(|route| route.iter().copied())
        .min_by(|a, b| squared_distance(*a, (x, y)).total_cmp(&squared_distance(*b, (x, y))))
        .unwrap_or((NAV2_SAFE_START_X, NAV2_SAFE_START_Y))
}

fn densify_route(route: &[(f64, f64)], max_step: f64) -> Vec<(f64, f64)> {
    let mut points = Vec::new();
    let Some(first) = route.first().copied() else {
        return points;
    };
    points.push(first);
    for segment in route.windows(2) {
        let start = segment[0];
        let end = segment[1];
        let distance = squared_distance(start, end).sqrt();
        let steps = (distance / max_step).ceil().max(1.0) as usize;
        for step in 1..=steps {
            let t = step as f64 / steps as f64;
            let x = start.0 + (end.0 - start.0) * t;
            let y = start.1 + (end.1 - start.1) * t;
            if points
                .last()
                .is_none_or(|previous| squared_distance(*previous, (x, y)) > 1e-12)
            {
                points.push((x, y));
            }
        }
    }
    points
}

fn follow_path_from_payload(payload: &Payload) -> ValueTree {
    let anchor = (NAV2_SAFE_START_X, NAV2_SAFE_START_Y);
    let waypoint = safe_waypoint_for_payload(payload, 1);
    let dx = waypoint.0 - anchor.0;
    let dy = waypoint.1 - anchor.1;
    let distance = (dx * dx + dy * dy).sqrt();
    let direction = if distance > f64::EPSILON {
        (dx / distance, dy / distance)
    } else {
        (1.0, 0.0)
    };
    let yaw = direction.1.atan2(direction.0);
    let poses = (0..NAV2_FOLLOW_PATH_POSES)
        .map(|index| {
            let offset = NAV2_FOLLOW_PATH_STEP_METERS * index as f64;
            let x = anchor.0 + direction.0 * offset;
            let y = anchor.1 + direction.1 * offset;
            pose_stamped_with_yaw(x, y, 0.0, yaw)
        })
        .collect();

    ValueTree::Nested(vec![header("map"), ValueTree::Array(poses)])
}

fn empty_string_array() -> ValueTree {
    ValueTree::Array(Vec::new())
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

fn duration(sec: i32, nanosec: u32) -> ValueTree {
    ValueTree::Nested(vec![
        ValueTree::Leaf(Value::I32(sec)),
        ValueTree::Leaf(Value::U32(nanosec)),
    ])
}

fn pose_with_covariance(pose_value: ValueTree) -> ValueTree {
    ValueTree::Nested(vec![pose_value, covariance()])
}

fn twist_with_covariance(linear_x: f64, angular_z: f64) -> ValueTree {
    ValueTree::Nested(vec![
        ValueTree::Nested(vec![point(linear_x, 0.0, 0.0), point(0.0, 0.0, angular_z)]),
        covariance(),
    ])
}

fn covariance() -> ValueTree {
    ValueTree::Array(
        (0..36)
            .map(|index| {
                let value = if matches!(index, 0 | 7 | 35) {
                    0.25
                } else {
                    0.0
                };
                ValueTree::Leaf(Value::F64(value))
            })
            .collect(),
    )
}

fn point(x: f64, y: f64, z: f64) -> ValueTree {
    ValueTree::Nested(vec![
        ValueTree::Leaf(Value::F64(x)),
        ValueTree::Leaf(Value::F64(y)),
        ValueTree::Leaf(Value::F64(z)),
    ])
}

fn point32(x: f32, y: f32, z: f32) -> ValueTree {
    ValueTree::Nested(vec![
        ValueTree::Leaf(Value::F32(x)),
        ValueTree::Leaf(Value::F32(y)),
        ValueTree::Leaf(Value::F32(z)),
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

fn transform_stamped(
    frame_id: &str,
    child_frame_id: &str,
    x: f64,
    y: f64,
    z: f64,
    quaternion: [f64; 4],
) -> ValueTree {
    ValueTree::Nested(vec![
        header(frame_id),
        ValueTree::Leaf(Value::String(child_frame_id.to_string())),
        pose(x, y, z, quaternion),
    ])
}

fn yaw_quaternion(yaw: f64) -> [f64; 4] {
    [0.0, 0.0, (yaw * 0.5).sin(), (yaw * 0.5).cos()]
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

fn u8_leaf_array(values: Vec<u8>) -> ValueTree {
    ValueTree::Array(
        values
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

fn u8_cost_array(tree: &ValueTree, len: usize) -> Vec<u8> {
    let source = match tree {
        ValueTree::Array(items) => items.iter().map(cost_value_to_u8).collect::<Vec<_>>(),
        _ => Vec::new(),
    };
    fill_or_repeat(source, len, 0)
}

fn cost_value_to_u8(tree: &ValueTree) -> u8 {
    match tree {
        ValueTree::Leaf(Value::Bool(value)) => {
            if *value {
                254
            } else {
                0
            }
        }
        ValueTree::Leaf(Value::I8(value)) => (*value).max(0) as u8,
        ValueTree::Leaf(Value::U8(value)) => *value,
        ValueTree::Leaf(Value::I16(value)) => (*value).clamp(0, 255) as u8,
        ValueTree::Leaf(Value::U16(value)) => (*value).min(255) as u8,
        ValueTree::Leaf(Value::I32(value)) => (*value).clamp(0, 255) as u8,
        ValueTree::Leaf(Value::U32(value)) => (*value).min(255) as u8,
        ValueTree::Leaf(Value::I64(value)) => (*value).clamp(0, 255) as u8,
        ValueTree::Leaf(Value::U64(value)) => (*value).min(255) as u8,
        ValueTree::Leaf(Value::F32(value)) if value.is_finite() => {
            value.abs().round().clamp(0.0, 255.0) as u8
        }
        ValueTree::Leaf(Value::F64(value)) if value.is_finite() => {
            value.abs().round().clamp(0.0, 255.0) as u8
        }
        _ => 0,
    }
}

fn seeded_costmap_data(seed: u64, width: u32, height: u32) -> Vec<u8> {
    let len = width as usize * height as usize;
    let mut data = vec![0; len];
    for (index, cell) in data.iter_mut().enumerate().take(len) {
        let mixed = seed
            .wrapping_add((index as u64).wrapping_mul(0x9e37_79b9_7f4a_7c15))
            .rotate_left((index % 31) as u32);
        *cell = match mixed % 37 {
            0 => 254,
            1 => 253,
            2 => 255,
            3 => 128,
            _ => 0,
        };
    }
    clear_costmap_center(&mut data, width, height);
    data
}

fn seeded_filter_mask_data(seed: u64, width: u32, height: u32, kind: FilterMaskKind) -> Vec<i8> {
    let len = width as usize * height as usize;
    let mut data = vec![0; len];
    for (index, cell) in data.iter_mut().enumerate().take(len) {
        let mixed = seed
            .wrapping_add((index as u64).wrapping_mul(0xd1b5_4a32_d192_ed03))
            .rotate_left((index % 29) as u32);
        *cell = match kind {
            FilterMaskKind::Keepout => match mixed % 53 {
                0 | 1 => 100,
                2 => 50,
                _ => 0,
            },
            FilterMaskKind::Speed => match mixed % 47 {
                0 => 90,
                1 | 2 => 50,
                3 => 20,
                _ => 0,
            },
        };
    }
    data
}

fn clear_costmap_center(data: &mut [u8], width: u32, height: u32) {
    let cx = (width / 2) as i32;
    let cy = (height / 2) as i32;
    for dy in -1..=1 {
        for dx in -1..=1 {
            let x = cx + dx;
            let y = cy + dy;
            if x >= 0 && y >= 0 && x < width as i32 && y < height as i32 {
                let index = y as usize * width as usize + x as usize;
                if let Some(cell) = data.get_mut(index) {
                    *cell = 0;
                }
            }
        }
    }
}

fn keep_safe_waypoints_free(data: &mut [i8], width: u32, height: u32) {
    let mut points = Vec::from(NAV2_SAFE_WAYPOINTS);
    points.push((NAV2_SAFE_START_X, NAV2_SAFE_START_Y));
    for (x, y) in points {
        let mx = ((x - SAFE_GRID_ORIGIN_X) / SAFE_GRID_RESOLUTION as f64).floor() as i32;
        let my = ((y - SAFE_GRID_ORIGIN_Y) / SAFE_GRID_RESOLUTION as f64).floor() as i32;
        for dy in -1..=1 {
            for dx in -1..=1 {
                let cx = mx + dx;
                let cy = my + dy;
                if cx >= 0 && cy >= 0 && cx < width as i32 && cy < height as i32 {
                    let index = cy as usize * width as usize + cx as usize;
                    if let Some(cell) = data.get_mut(index) {
                        *cell = 0;
                    }
                }
            }
        }
    }
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
    fn as_string(&self) -> Option<&str>;
    fn as_bool(&self) -> Option<bool>;
}

impl ValueTreeExt for ValueTree {
    fn as_nested(&self) -> Option<&[ValueTree]> {
        match self {
            ValueTree::Nested(fields) => Some(fields),
            _ => None,
        }
    }

    fn as_string(&self) -> Option<&str> {
        match self {
            ValueTree::Leaf(Value::String(value)) => Some(value),
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
        LaserScanSchedule, NAV2_BEHAVIOR_TIME_ALLOWANCE_SEC, NAV2_COSTMAP_STREAM_RESOLUTION,
        NAV2_COSTMAP_STREAM_SIDE, NAV2_COSTMAP_UPDATE_SIDE, NAV2_FOLLOW_PATH_POSES,
        NAV2_FOLLOW_PATH_STEP_METERS, NAV2_LOCAL_PATH_STEP_METERS, NAV2_NAV_TO_POSE_BT_XMLS,
        NAV2_SAFE_START_X, NAV2_SAFE_START_Y, NAV2_SMOOTH_MAX_DURATION_SEC, NAV2_SMOOTHER_IDS,
        NAV2_SPIN_TIME_ALLOWANCE_SEC, NAV2_WAIT_DURATION_SEC, Ros2TopicOptions, SAFE_GRID_ORIGIN_X,
        SAFE_GRID_ORIGIN_Y, SAFE_GRID_RESOLUTION, SAFE_GRID_SIDE, SAFE_MAP_ENV, SAFE_MAP_LIST_ENV,
        ValueTreeExt, build_action_args, build_ros2_cli_command_with_timeout,
        build_scan_bridge_command, build_topic_args, configured_safe_map_candidates,
        load_map_url_from_payload, load_ros2_sender_commands, local_path_points_from_payload,
        nav2_behavior_tree_path, normalize_point_cloud2, normalize_value, render_cli_payload,
        render_message, render_raw_cli_payload, render_ros2_cli_invocation, ros2_cli_command,
        squared_distance, validate_yaml_payload,
    };
    use crate::interface_extractor::{
        Extractor, Field, FileExtractor, Interface, Kind, Primitive, TypeNode,
    };
    use crate::payload::{Payload, Value, ValueTree};
    use std::f32::consts::PI as PI_F32;
    use std::path::{Path, PathBuf};

    #[test]
    fn ros2_cli_invocation_does_not_duplicate_configured_executable() {
        let invocation = render_ros2_cli_invocation(&[
            "ros2".to_string(),
            "topic".to_string(),
            "pub".to_string(),
            "/goal_pose".to_string(),
            "{\"frame_id\":\"map\"}".to_string(),
        ]);
        assert!(invocation.starts_with("ros2 topic pub /goal_pose "));
        assert!(!invocation.starts_with("ros2 ros2 "));
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
    fn render_raw_yaml_payload_as_single_json_argument() {
        let value: serde_yaml::Value = serde_yaml::from_str(
            r#"
pose:
  header:
    frame_id: map
  pose:
    position: {x: 0.5, y: 0.5, z: 0.0}
behavior_tree: ""
"#,
        )
        .unwrap();

        let rendered = render_raw_cli_payload(&value).unwrap();
        assert!(rendered.contains("\"pose\""));
        assert!(rendered.contains("\"behavior_tree\""));
        assert!(!rendered.contains('\n'));
    }

    #[test]
    fn validate_yaml_payload_allows_partial_ros_cli_mapping() {
        let fields = vec![
            Field::new(
                "pose",
                TypeNode::Nested(vec![Field::new(
                    "header",
                    TypeNode::Nested(vec![Field::new(
                        "frame_id",
                        TypeNode::Primitive(Primitive::String),
                    )]),
                )]),
            ),
            Field::new("behavior_tree", TypeNode::Primitive(Primitive::String)),
        ];
        let value: serde_yaml::Value = serde_yaml::from_str(
            r#"
pose:
  header:
    frame_id: map
"#,
        )
        .unwrap();

        validate_yaml_payload(&value, &fields).unwrap();
    }

    #[test]
    fn validate_yaml_payload_rejects_unknown_or_wrongly_typed_fields() {
        let fields = vec![Field::new(
            "frame_id",
            TypeNode::Primitive(Primitive::String),
        )];
        let unknown: serde_yaml::Value = serde_yaml::from_str("frame_id: map\nextra: 1\n").unwrap();
        let wrong_type: serde_yaml::Value = serde_yaml::from_str("frame_id: 7\n").unwrap();

        assert!(
            validate_yaml_payload(&unknown, &fields)
                .unwrap_err()
                .to_string()
                .contains("unknown field")
        );
        assert!(
            validate_yaml_payload(&wrong_type, &fields)
                .unwrap_err()
                .to_string()
                .contains("String")
        );
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
    fn nav2_lyrical_action_normalizers_render_against_real_goal_interfaces() {
        for name in [
            "NavigateToPose",
            "NavigateThroughPoses",
            "ComputePathToPose",
            "ComputePathThroughPoses",
            "FollowPath",
            "SmoothPath",
            "Spin",
            "BackUp",
            "DriveOnHeading",
            "AssistedTeleop",
            "Wait",
        ] {
            let interface = lyrical_nav2_interface(&format!("nav2_msgs/action/{name}.action"));
            let payload = Payload::new(name, Kind::Action, ValueTree::Nested(Vec::new()), 42);
            let normalized = normalize_value(name, &payload);

            render_cli_payload(&normalized, &interface.fields).unwrap_or_else(|error| {
                panic!("{name} did not render against real interface: {error}")
            });
        }
    }

    #[test]
    fn nav2_action_normalizers_keep_legal_semantic_variety() {
        let navigate_default = normalize_value(
            "NavigateToPose",
            &Payload::new(
                "NavigateToPose",
                Kind::Action,
                ValueTree::Nested(Vec::new()),
                48,
            ),
        );
        let navigate_with_bt = normalize_value(
            "NavigateToPose",
            &Payload::new(
                "NavigateToPose",
                Kind::Action,
                ValueTree::Nested(Vec::new()),
                49,
            ),
        );
        assert_eq!(nested_string(&navigate_default, &[1]), Some(""));
        let bt_xml = nested_string(&navigate_with_bt, &[1]).expect("behavior_tree field");
        assert!(
            NAV2_NAV_TO_POSE_BT_XMLS
                .iter()
                .skip(1)
                .any(|candidate| bt_xml.ends_with(candidate)),
            "unexpected behavior tree path: {bt_xml}"
        );
        assert!(
            Path::new(bt_xml).exists(),
            "normalizer should select an existing Nav2 behavior tree"
        );
        for candidate in NAV2_NAV_TO_POSE_BT_XMLS.iter().skip(1) {
            let candidate_path = nav2_behavior_tree_path(candidate);
            assert!(
                Path::new(&candidate_path).exists(),
                "Nav2 behavior tree candidate should exist: {candidate}"
            );
        }
        let navigate_qz = nested_f64(&navigate_with_bt, &[0, 1, 1, 2]).expect("goal orientation z");
        let navigate_qw = nested_f64(&navigate_with_bt, &[0, 1, 1, 3]).expect("goal orientation w");
        assert!(
            (navigate_qz * navigate_qz + navigate_qw * navigate_qw - 1.0).abs() < 1e-9,
            "NavigateToPose yaw quaternion should remain normalized"
        );

        for seed in 40..48 {
            let through = normalize_value(
                "NavigateThroughPoses",
                &Payload::new(
                    "NavigateThroughPoses",
                    Kind::Action,
                    ValueTree::Nested(Vec::new()),
                    seed,
                ),
            );
            let compute = normalize_value(
                "ComputePathThroughPoses",
                &Payload::new(
                    "ComputePathThroughPoses",
                    Kind::Action,
                    ValueTree::Nested(Vec::new()),
                    seed,
                ),
            );
            let compute_to_pose = normalize_value(
                "ComputePathToPose",
                &Payload::new(
                    "ComputePathToPose",
                    Kind::Action,
                    ValueTree::Nested(Vec::new()),
                    seed,
                ),
            );
            let smooth = normalize_value(
                "SmoothPath",
                &Payload::new(
                    "SmoothPath",
                    Kind::Action,
                    ValueTree::Nested(Vec::new()),
                    seed,
                ),
            );
            assert!(
                array_len(&through, &[0, 1]).is_some_and(|len| (2..=4).contains(&len)),
                "NavigateThroughPoses should keep a bounded non-empty goal list"
            );
            assert!(
                array_len(&compute, &[0, 1]).is_some_and(|len| (2..=5).contains(&len)),
                "ComputePathThroughPoses should keep a bounded non-empty goal list"
            );
            assert!(
                nested_bool(&compute_to_pose, &[4]).is_some(),
                "ComputePathToPose should keep a legal use_start bool"
            );
            assert!(
                nested_bool(&compute, &[3]).is_some(),
                "ComputePathThroughPoses should keep a legal use_start bool"
            );
            assert!(
                array_len(&smooth, &[0, 1]).is_some_and(|len| (3..=9).contains(&len)),
                "SmoothPath should keep a bounded contiguous path"
            );
        }
        let through_with_bt = normalize_value(
            "NavigateThroughPoses",
            &Payload::new(
                "NavigateThroughPoses",
                Kind::Action,
                ValueTree::Nested(Vec::new()),
                41,
            ),
        );
        let through_bt =
            nested_string(&through_with_bt, &[1]).expect("through poses behavior_tree");
        assert!(
            through_bt.ends_with("navigate_through_poses_w_replanning_and_recovery.xml"),
            "unexpected NavigateThroughPoses behavior tree path: {through_bt}"
        );
        assert!(
            Path::new(through_bt).exists(),
            "NavigateThroughPoses behavior tree should exist"
        );
    }

    #[test]
    fn nav2_follow_path_normalizer_keeps_controller_path_local() {
        let follow_path = normalize_value(
            "FollowPath",
            &Payload::new(
                "FollowPath",
                Kind::Action,
                ValueTree::Nested(Vec::new()),
                52,
            ),
        );
        let fields = follow_path.as_nested().expect("FollowPath goal is nested");
        let path = fields[0].as_nested().expect("FollowPath.path is nested");
        let ValueTree::Array(poses) = &path[1] else {
            panic!("FollowPath.path.poses is an array");
        };
        assert_eq!(poses.len(), NAV2_FOLLOW_PATH_POSES);

        let first_x = nested_f64(&poses[0], &[1, 0, 0]).expect("first pose x");
        let first_y = nested_f64(&poses[0], &[1, 0, 1]).expect("first pose y");
        assert!(
            ((first_x - NAV2_SAFE_START_X).powi(2) + (first_y - NAV2_SAFE_START_Y).powi(2)).sqrt()
                < 1e-9,
            "FollowPath should start at the full-stack startup pose so the controller can make progress"
        );

        for pair in poses.windows(2) {
            let x0 = nested_f64(&pair[0], &[1, 0, 0]).expect("pose x");
            let y0 = nested_f64(&pair[0], &[1, 0, 1]).expect("pose y");
            let x1 = nested_f64(&pair[1], &[1, 0, 0]).expect("pose x");
            let y1 = nested_f64(&pair[1], &[1, 0, 1]).expect("pose y");
            let step = ((x1 - x0).powi(2) + (y1 - y0).powi(2)).sqrt();
            assert!(
                step <= NAV2_FOLLOW_PATH_STEP_METERS + 1e-9,
                "FollowPath local step {step} exceeds the local costmap-friendly spacing"
            );
        }
        for pose in poses {
            let qz = nested_f64(pose, &[1, 1, 2]).expect("pose orientation z");
            let qw = nested_f64(pose, &[1, 1, 3]).expect("pose orientation w");
            assert!(
                (qz * qz + qw * qw - 1.0).abs() < 1e-9,
                "FollowPath path orientation should remain a normalized yaw quaternion"
            );
        }
        let last_x = nested_f64(poses.last().expect("last pose"), &[1, 0, 0]).expect("last pose x");
        let last_y = nested_f64(poses.last().expect("last pose"), &[1, 0, 1]).expect("last pose y");
        assert!(
            ((last_x - first_x).powi(2) + (last_y - first_y).powi(2)).sqrt()
                >= NAV2_FOLLOW_PATH_STEP_METERS,
            "FollowPath should still move away from the startup pose under payload-derived direction"
        );
    }

    #[test]
    fn nav2_behavior_action_normalizers_keep_reachable_motion_semantics() {
        let backup = normalize_value(
            "BackUp",
            &Payload::new("BackUp", Kind::Action, ValueTree::Nested(Vec::new()), 42),
        );
        let drive = normalize_value(
            "DriveOnHeading",
            &Payload::new(
                "DriveOnHeading",
                Kind::Action,
                ValueTree::Nested(Vec::new()),
                42,
            ),
        );
        let spin = normalize_value(
            "Spin",
            &Payload::new("Spin", Kind::Action, ValueTree::Nested(Vec::new()), 42),
        );
        let smooth = normalize_value(
            "SmoothPath",
            &Payload::new(
                "SmoothPath",
                Kind::Action,
                ValueTree::Nested(Vec::new()),
                42,
            ),
        );
        let assisted = normalize_value(
            "AssistedTeleop",
            &Payload::new(
                "AssistedTeleop",
                Kind::Action,
                ValueTree::Nested(Vec::new()),
                42,
            ),
        );
        let wait = normalize_value(
            "Wait",
            &Payload::new("Wait", Kind::Action, ValueTree::Nested(Vec::new()), 42),
        );

        assert!(nested_f64(&backup, &[0, 0]).is_some_and(|value| (-0.8..=-0.15).contains(&value)));
        assert_eq!(nested_f64(&backup, &[0, 1]), Some(0.0));
        assert_eq!(nested_f64(&backup, &[0, 2]), Some(0.0));
        assert!(nested_f32(&backup, &[1]).is_some_and(|value| (-0.30..=-0.05).contains(&value)));
        assert!(
            nested_i32(&backup, &[2, 0])
                .is_some_and(|value| { (NAV2_BEHAVIOR_TIME_ALLOWANCE_SEC..=12).contains(&value) })
        );
        assert!(nested_bool(&backup, &[3]).is_some());

        assert!(nested_f64(&drive, &[0, 0]).is_some_and(|value| (0.15..=0.8).contains(&value)));
        assert_eq!(nested_f64(&drive, &[0, 1]), Some(0.0));
        assert_eq!(nested_f64(&drive, &[0, 2]), Some(0.0));
        assert!(nested_f32(&drive, &[1]).is_some_and(|value| (0.05..=0.30).contains(&value)));
        assert!(
            nested_i32(&drive, &[2, 0])
                .is_some_and(|value| { (NAV2_BEHAVIOR_TIME_ALLOWANCE_SEC..=12).contains(&value) })
        );
        assert!(nested_bool(&drive, &[3]).is_some());

        assert!(nested_f32(&spin, &[0]).is_some_and(|value| (-PI_F32..=PI_F32).contains(&value)));
        assert!(
            nested_i32(&spin, &[1, 0])
                .is_some_and(|value| { (NAV2_SPIN_TIME_ALLOWANCE_SEC..=10).contains(&value) })
        );
        assert!(nested_bool(&spin, &[2]).is_some());

        assert!(
            nested_string(&smooth, &[1])
                .is_some_and(|value| { NAV2_SMOOTHER_IDS.contains(&value) })
        );
        assert!(
            nested_i32(&smooth, &[2, 0])
                .is_some_and(|value| { (NAV2_SMOOTH_MAX_DURATION_SEC..=5).contains(&value) })
        );
        assert!(nested_bool(&smooth, &[3]).is_some());

        assert!(
            nested_i32(&assisted, &[0, 0])
                .is_some_and(|value| { (NAV2_BEHAVIOR_TIME_ALLOWANCE_SEC..=12).contains(&value) })
        );
        assert!(
            nested_i32(&wait, &[0, 0])
                .is_some_and(|value| { (NAV2_WAIT_DURATION_SEC..=6).contains(&value) })
        );

        let smooth_next = normalize_value(
            "SmoothPath",
            &Payload::new(
                "SmoothPath",
                Kind::Action,
                ValueTree::Nested(Vec::new()),
                43,
            ),
        );
        assert_ne!(
            nested_string(&smooth, &[1]),
            nested_string(&smooth_next, &[1])
        );
    }

    #[test]
    fn nav2_topic_normalizers_keep_legal_semantic_variety() {
        let percent_speed = normalize_value(
            "SpeedLimit",
            &Payload::new(
                "SpeedLimit",
                Kind::Topic,
                ValueTree::Nested(vec![
                    ValueTree::Nested(Vec::new()),
                    ValueTree::Leaf(Value::Bool(true)),
                    ValueTree::Leaf(Value::F64(150.0)),
                ]),
                42,
            ),
        );
        assert_eq!(nested_bool(&percent_speed, &[1]), Some(true));
        assert!(
            nested_f64(&percent_speed, &[2]).is_some_and(|value| (1.0..=100.0).contains(&value))
        );

        let absolute_speed = normalize_value(
            "SpeedLimit",
            &Payload::new(
                "SpeedLimit",
                Kind::Topic,
                ValueTree::Nested(vec![
                    ValueTree::Nested(Vec::new()),
                    ValueTree::Leaf(Value::Bool(false)),
                    ValueTree::Leaf(Value::F64(2.0)),
                ]),
                43,
            ),
        );
        assert_eq!(nested_bool(&absolute_speed, &[1]), Some(false));
        assert!(
            nested_f64(&absolute_speed, &[2]).is_some_and(|value| (0.05..=0.80).contains(&value))
        );

        let footprint_a = normalize_value(
            "Polygon",
            &Payload::new("Polygon", Kind::Topic, ValueTree::Nested(Vec::new()), 42),
        );
        let footprint_b = normalize_value(
            "Polygon",
            &Payload::new("Polygon", Kind::Topic, ValueTree::Nested(Vec::new()), 43),
        );
        assert_eq!(array_len(&footprint_a, &[0]), Some(4));
        assert_eq!(array_len(&footprint_b, &[0]), Some(4));
        assert_ne!(footprint_a, footprint_b);
        for footprint in [&footprint_a, &footprint_b] {
            let ValueTree::Nested(fields) = footprint else {
                panic!("Polygon should be nested");
            };
            let ValueTree::Array(points) = &fields[0] else {
                panic!("Polygon.points should be an array");
            };
            for point in points {
                let x = nested_f32(point, &[0]).expect("point x");
                let y = nested_f32(point, &[1]).expect("point y");
                assert!((-0.35..=0.35).contains(&x));
                assert!((-0.25..=0.25).contains(&y));
            }
        }

        let stamped_footprint = normalize_value(
            "PolygonStamped@local_published_footprint",
            &Payload::new(
                "PolygonStamped@local_published_footprint",
                Kind::Topic,
                ValueTree::Nested(Vec::new()),
                42,
            ),
        );
        assert_eq!(nested_string(&stamped_footprint, &[0, 1]), Some("odom"));
        assert_eq!(array_len(&stamped_footprint, &[1, 0]), Some(4));

        let raw_costmap = normalize_value(
            "Costmap@global_costmap_raw",
            &Payload::new(
                "Costmap@global_costmap_raw",
                Kind::Topic,
                ValueTree::Nested(Vec::new()),
                42,
            ),
        );
        assert_eq!(nested_string(&raw_costmap, &[0, 1]), Some("map"));
        assert_eq!(nested_string(&raw_costmap, &[1, 2]), Some("global_costmap"));
        assert_eq!(
            nested_f32(&raw_costmap, &[1, 3]),
            Some(NAV2_COSTMAP_STREAM_RESOLUTION)
        );
        assert_eq!(
            nested_u32(&raw_costmap, &[1, 4]),
            Some(NAV2_COSTMAP_STREAM_SIDE)
        );
        assert_eq!(
            nested_u32(&raw_costmap, &[1, 5]),
            Some(NAV2_COSTMAP_STREAM_SIDE)
        );
        assert_eq!(
            array_len(&raw_costmap, &[2]),
            Some(NAV2_COSTMAP_STREAM_SIDE as usize * NAV2_COSTMAP_STREAM_SIDE as usize)
        );

        let raw_update = normalize_value(
            "CostmapUpdate@local_costmap_raw_updates",
            &Payload::new(
                "CostmapUpdate@local_costmap_raw_updates",
                Kind::Topic,
                ValueTree::Nested(Vec::new()),
                43,
            ),
        );
        assert_eq!(nested_string(&raw_update, &[0, 1]), Some("odom"));
        assert_eq!(
            nested_u32(&raw_update, &[3]),
            Some(NAV2_COSTMAP_UPDATE_SIDE)
        );
        assert_eq!(
            nested_u32(&raw_update, &[4]),
            Some(NAV2_COSTMAP_UPDATE_SIDE)
        );
        assert_eq!(
            array_len(&raw_update, &[5]),
            Some(NAV2_COSTMAP_UPDATE_SIDE as usize * NAV2_COSTMAP_UPDATE_SIDE as usize)
        );

        let keepout_info = normalize_value(
            "CostmapFilterInfo@keepout",
            &Payload::new(
                "CostmapFilterInfo@keepout",
                Kind::Topic,
                ValueTree::Nested(Vec::new()),
                42,
            ),
        );
        assert_eq!(nested_u8(&keepout_info, &[1]), Some(0));
        assert_eq!(
            nested_string(&keepout_info, &[2]),
            Some("keepout_filter_mask")
        );

        let speed_info = normalize_value(
            "CostmapFilterInfo@speed",
            &Payload::new(
                "CostmapFilterInfo@speed",
                Kind::Topic,
                ValueTree::Nested(Vec::new()),
                42,
            ),
        );
        assert_eq!(nested_u8(&speed_info, &[1]), Some(1));
        assert_eq!(nested_string(&speed_info, &[2]), Some("speed_filter_mask"));

        let keepout_mask = normalize_value(
            "OccupancyGrid@keepout_filter_mask",
            &Payload::new(
                "OccupancyGrid@keepout_filter_mask",
                Kind::Topic,
                ValueTree::Nested(Vec::new()),
                42,
            ),
        );
        assert_eq!(nested_string(&keepout_mask, &[0, 1]), Some("map"));
        assert_eq!(nested_u32(&keepout_mask, &[1, 2]), Some(SAFE_GRID_SIDE));
        assert_eq!(
            array_len(&keepout_mask, &[2]),
            Some(SAFE_GRID_SIDE as usize * SAFE_GRID_SIDE as usize)
        );

        let dynamic_tf = normalize_value(
            "TFMessage",
            &Payload::new("TFMessage", Kind::Topic, ValueTree::Nested(Vec::new()), 42),
        );
        assert_eq!(array_len(&dynamic_tf, &[0]), Some(2));
        assert_eq!(tf_transform_string(&dynamic_tf, 0, &[0, 1]), Some("map"));
        assert_eq!(tf_transform_string(&dynamic_tf, 0, &[1]), Some("odom"));
        assert_eq!(tf_transform_string(&dynamic_tf, 1, &[0, 1]), Some("odom"));
        assert_eq!(
            tf_transform_string(&dynamic_tf, 1, &[1]),
            Some("base_footprint")
        );
        assert!(
            tf_transform_f64(&dynamic_tf, 1, &[2, 0, 0])
                .is_some_and(|x| (NAV2_SAFE_START_X - 0.05..=NAV2_SAFE_START_X + 0.05)
                    .contains(&x))
        );
        assert!(
            tf_transform_f64(&dynamic_tf, 1, &[2, 0, 1])
                .is_some_and(|y| (NAV2_SAFE_START_Y - 0.05..=NAV2_SAFE_START_Y + 0.05)
                    .contains(&y))
        );

        let static_tf = normalize_value(
            "TFMessage@tf_static",
            &Payload::new(
                "TFMessage@tf_static",
                Kind::Topic,
                ValueTree::Nested(Vec::new()),
                42,
            ),
        );
        assert_eq!(array_len(&static_tf, &[0]), Some(3));
        assert_eq!(
            tf_transform_string(&static_tf, 0, &[0, 1]),
            Some("base_footprint")
        );
        assert_eq!(
            tf_transform_string(&static_tf, 0, &[1]),
            Some("laser_frame")
        );
        assert_eq!(tf_transform_string(&static_tf, 1, &[1]), Some("base_scan"));
        assert_eq!(
            tf_transform_string(&static_tf, 2, &[1]),
            Some("rplidar_link")
        );

        let stamped_cmd_a = normalize_value(
            "TwistStamped",
            &Payload::new(
                "TwistStamped",
                Kind::Topic,
                ValueTree::Nested(Vec::new()),
                42,
            ),
        );
        let stamped_cmd_b = normalize_value(
            "TwistStamped",
            &Payload::new(
                "TwistStamped",
                Kind::Topic,
                ValueTree::Nested(Vec::new()),
                43,
            ),
        );
        assert_eq!(
            nested_string(&stamped_cmd_a, &[0, 1]),
            Some("base_footprint")
        );
        assert_ne!(stamped_cmd_a, stamped_cmd_b);
        assert!(
            nested_f64(&stamped_cmd_a, &[1, 0, 0]).is_some_and(|x| (-0.03..=0.05).contains(&x))
        );
        assert_eq!(nested_f64(&stamped_cmd_a, &[1, 0, 1]), Some(0.0));
        assert_eq!(nested_f64(&stamped_cmd_a, &[1, 0, 2]), Some(0.0));
        assert!(
            nested_f64(&stamped_cmd_a, &[1, 1, 2]).is_some_and(|z| (-0.15..=0.15).contains(&z))
        );

        for (name, expected) in [
            ("String@planner_selector", "GridBased"),
            ("String@controller_selector", "FollowPath"),
            ("String@goal_checker_selector", "general_goal_checker"),
            ("String@progress_checker_selector", "progress_checker"),
            ("String@path_handler_selector", "PathHandler"),
        ] {
            let selector = normalize_value(
                name,
                &Payload::new(name, Kind::Topic, ValueTree::Nested(Vec::new()), 42),
            );
            assert_eq!(nested_string(&selector, &[0]), Some(expected));
        }
    }

    #[test]
    fn nav2_lyrical_topic_normalizers_render_against_real_message_interfaces() {
        for (name, relative) in [
            (
                "PoseWithCovarianceStamped",
                "geometry_msgs/msg/PoseWithCovarianceStamped.msg",
            ),
            ("Odometry", "nav_msgs/msg/Odometry.msg"),
            ("PoseStamped", "geometry_msgs/msg/PoseStamped.msg"),
            ("SpeedLimit", "nav2_msgs/msg/SpeedLimit.msg"),
            ("TwistStamped", "geometry_msgs/msg/TwistStamped.msg"),
            ("Polygon", "geometry_msgs/msg/Polygon.msg"),
            (
                "PolygonStamped@local_published_footprint",
                "geometry_msgs/msg/PolygonStamped.msg",
            ),
            (
                "PolygonStamped@global_published_footprint",
                "geometry_msgs/msg/PolygonStamped.msg",
            ),
            ("Costmap@local_costmap_raw", "nav2_msgs/msg/Costmap.msg"),
            ("Costmap@global_costmap_raw", "nav2_msgs/msg/Costmap.msg"),
            (
                "CostmapUpdate@local_costmap_raw_updates",
                "nav2_msgs/msg/CostmapUpdate.msg",
            ),
            (
                "CostmapUpdate@global_costmap_raw_updates",
                "nav2_msgs/msg/CostmapUpdate.msg",
            ),
            (
                "CostmapFilterInfo@keepout",
                "nav2_msgs/msg/CostmapFilterInfo.msg",
            ),
            (
                "CostmapFilterInfo@speed",
                "nav2_msgs/msg/CostmapFilterInfo.msg",
            ),
            (
                "OccupancyGrid@keepout_filter_mask",
                "nav_msgs/msg/OccupancyGrid.msg",
            ),
            (
                "OccupancyGrid@speed_filter_mask",
                "nav_msgs/msg/OccupancyGrid.msg",
            ),
            ("TFMessage", "tf2_msgs/msg/TFMessage.msg"),
            ("TFMessage@tf_static", "tf2_msgs/msg/TFMessage.msg"),
            ("String@planner_selector", "std_msgs/msg/String.msg"),
            ("String@controller_selector", "std_msgs/msg/String.msg"),
            ("String@goal_checker_selector", "std_msgs/msg/String.msg"),
            (
                "String@progress_checker_selector",
                "std_msgs/msg/String.msg",
            ),
            ("String@path_handler_selector", "std_msgs/msg/String.msg"),
        ] {
            let interface = lyrical_nav2_interface(relative);
            let payload = Payload::new(name, Kind::Topic, ValueTree::Nested(Vec::new()), 42);
            let normalized = normalize_value(name, &payload);

            render_cli_payload(&normalized, &interface.fields).unwrap_or_else(|error| {
                panic!("{name} did not render against real interface: {error}")
            });
        }
    }

    #[test]
    fn nav2_lyrical_service_normalizers_render_against_real_request_interfaces() {
        for name in [
            "GetCosts",
            "GetCostmap",
            "ClearCostmapExceptRegion",
            "ClearCostmapAroundRobot",
            "ClearCostmapAroundPose",
            "ClearEntireCostmap",
            "SetInitialPose",
            "LoadMap",
            "IsPathValid",
        ] {
            let interface = lyrical_nav2_interface(&format!("nav2_msgs/srv/{name}.srv"));
            let payload = Payload::new(name, Kind::Service, ValueTree::Nested(Vec::new()), 42);
            let normalized = normalize_value(name, &payload);

            render_cli_payload(&normalized, &interface.fields).unwrap_or_else(|error| {
                panic!("{name} did not render against real interface: {error}")
            });
        }
    }

    #[test]
    fn nav2_load_map_normalizer_uses_existing_legal_maps() {
        let candidates = configured_safe_map_candidates();
        assert!(
            !candidates.is_empty(),
            "at least one legal Nav2 map candidate should be available"
        );
        for candidate in &candidates {
            assert!(
                Path::new(candidate).is_file(),
                "LoadMap candidate should exist: {candidate}"
            );
        }

        let explicit = Payload::new(
            "LoadMap",
            Kind::Service,
            ValueTree::Nested(vec![ValueTree::Leaf(Value::String(candidates[0].clone()))]),
            99,
        );
        assert_eq!(load_map_url_from_payload(&explicit), candidates[0]);

        let mut observed = std::collections::BTreeSet::new();
        for seed in 0..8 {
            let load_map = normalize_value(
                "LoadMap",
                &Payload::new(
                    "LoadMap",
                    Kind::Service,
                    ValueTree::Nested(Vec::new()),
                    seed,
                ),
            );
            let map_url = nested_string(&load_map, &[0]).expect("LoadMap.map_url");
            assert!(
                Path::new(map_url).is_file(),
                "LoadMap should render an existing map yaml: {map_url}"
            );
            observed.insert(map_url.to_string());
        }

        if std::env::var_os(SAFE_MAP_ENV).is_none()
            && std::env::var_os(SAFE_MAP_LIST_ENV).is_none()
            && candidates.len() > 1
        {
            assert!(
                observed.len() > 1,
                "built-in LoadMap candidates should vary with the payload seed"
            );
        }
    }

    #[test]
    fn nav2_service_normalizers_keep_legal_semantic_variety() {
        let get_costs = normalize_value(
            "GetCosts",
            &Payload::new("GetCosts", Kind::Service, ValueTree::Nested(Vec::new()), 42),
        );
        assert!(nested_bool(&get_costs, &[0]).is_some());
        assert!(array_len(&get_costs, &[1]).is_some_and(|len| (1..=4).contains(&len)));

        let get_costs_next = normalize_value(
            "GetCosts@global_costmap",
            &Payload::new(
                "GetCosts@global_costmap",
                Kind::Service,
                ValueTree::Nested(Vec::new()),
                43,
            ),
        );
        assert_ne!(
            get_costs, get_costs_next,
            "GetCosts should not collapse every seed to the same request"
        );

        let clear_pose = normalize_value(
            "ClearCostmapAroundPose",
            &Payload::new(
                "ClearCostmapAroundPose",
                Kind::Service,
                ValueTree::Nested(Vec::new()),
                44,
            ),
        );
        assert!(nested_f64(&clear_pose, &[0, 1, 0, 0]).is_some_and(|x| (-2.5..=2.5).contains(&x)));
        assert!(nested_f64(&clear_pose, &[0, 1, 0, 1]).is_some_and(|y| (-2.5..=2.5).contains(&y)));
        let qz = nested_f64(&clear_pose, &[0, 1, 1, 2]).expect("clear pose orientation z");
        let qw = nested_f64(&clear_pose, &[0, 1, 1, 3]).expect("clear pose orientation w");
        assert!(
            (qz * qz + qw * qw - 1.0).abs() < 1e-9,
            "ClearCostmapAroundPose yaw quaternion should remain normalized"
        );

        let path_valid = normalize_value(
            "IsPathValid",
            &Payload::new(
                "IsPathValid",
                Kind::Service,
                ValueTree::Nested(Vec::new()),
                45,
            ),
        );
        assert!(array_len(&path_valid, &[0, 1]).is_some_and(|len| (3..=9).contains(&len)));
        assert!(nested_bool(&path_valid, &[2]).is_some());
        assert!(nested_bool(&path_valid, &[5]).is_some());
        assert!(
            nested_f64(&path_valid, &[6])
                .is_some_and(|value| value == -1.0 || (0.25..=3.0).contains(&value))
        );
        let path_points = local_path_points_from_payload(&Payload::new(
            "IsPathValid",
            Kind::Service,
            ValueTree::Nested(Vec::new()),
            45,
        ));
        assert!(path_points.len() >= 3);
        assert!(
            ((path_points[0].0 - NAV2_SAFE_START_X).powi(2)
                + (path_points[0].1 - NAV2_SAFE_START_Y).powi(2))
            .sqrt()
                < 1e-9,
            "IsPathValid path should start at the full-stack startup pose"
        );
        for pair in path_points.windows(2) {
            let step = squared_distance(pair[0], pair[1]).sqrt();
            assert!(
                step <= NAV2_LOCAL_PATH_STEP_METERS + 1e-9,
                "IsPathValid/SmoothPath path segment {step} should stay local and contiguous"
            );
        }

        let toggle_a = normalize_value(
            "SetBool@global_speed_filter",
            &Payload::new(
                "SetBool@global_speed_filter",
                Kind::Service,
                ValueTree::Nested(Vec::new()),
                42,
            ),
        );
        let toggle_b = normalize_value(
            "SetBool@global_speed_filter",
            &Payload::new(
                "SetBool@global_speed_filter",
                Kind::Service,
                ValueTree::Nested(Vec::new()),
                43,
            ),
        );
        assert!(nested_bool(&toggle_a, &[0]).is_some());
        assert!(nested_bool(&toggle_b, &[0]).is_some());
        assert_ne!(toggle_a, toggle_b);
    }

    #[test]
    fn nav2_aliased_normalizers_render_against_real_interfaces() {
        for (name, relative) in [
            ("GetCosts@global_costmap", "nav2_msgs/srv/GetCosts.srv"),
            (
                "GetCostmap@global_static_layer",
                "nav2_msgs/srv/GetCostmap.srv",
            ),
            (
                "ClearCostmapAroundRobot@global_costmap",
                "nav2_msgs/srv/ClearCostmapAroundRobot.srv",
            ),
            (
                "ClearEntireCostmap@global_costmap",
                "nav2_msgs/srv/ClearEntireCostmap.srv",
            ),
            ("SetBool@global_speed_filter", "std_srvs/srv/SetBool.srv"),
            ("Polygon@global_costmap", "geometry_msgs/msg/Polygon.msg"),
            (
                "PolygonStamped@global_published_footprint",
                "geometry_msgs/msg/PolygonStamped.msg",
            ),
            ("Costmap@global_costmap_raw", "nav2_msgs/msg/Costmap.msg"),
            (
                "CostmapUpdate@global_costmap_raw_updates",
                "nav2_msgs/msg/CostmapUpdate.msg",
            ),
            ("TFMessage@tf_static", "tf2_msgs/msg/TFMessage.msg"),
        ] {
            let interface = lyrical_nav2_interface(relative);
            let payload = Payload::new(name, Kind::Topic, ValueTree::Nested(Vec::new()), 42);
            let normalized = normalize_value(name, &payload);

            render_cli_payload(&normalized, &interface.fields).unwrap_or_else(|error| {
                panic!("{name} did not render against real interface: {error}")
            });
        }
    }

    #[test]
    fn occupancy_grid_normalization_keeps_nav2_safe_region_in_bounds() {
        let interface = lyrical_nav2_interface("nav_msgs/msg/OccupancyGrid.msg");
        let payload = Payload::new(
            "OccupancyGrid",
            Kind::Topic,
            ValueTree::Nested(Vec::new()),
            42,
        );
        let normalized = normalize_value("OccupancyGrid", &payload);
        render_cli_payload(&normalized, &interface.fields).unwrap();

        let ValueTree::Nested(fields) = normalized else {
            panic!("OccupancyGrid must be a nested message");
        };
        let ValueTree::Nested(info) = &fields[1] else {
            panic!("OccupancyGrid.info must be nested");
        };
        assert_eq!(info[1], ValueTree::Leaf(Value::F32(SAFE_GRID_RESOLUTION)));
        assert_eq!(info[2], ValueTree::Leaf(Value::U32(SAFE_GRID_SIDE)));
        assert_eq!(info[3], ValueTree::Leaf(Value::U32(SAFE_GRID_SIDE)));
        let max_x = SAFE_GRID_ORIGIN_X + SAFE_GRID_RESOLUTION as f64 * SAFE_GRID_SIDE as f64;
        let max_y = SAFE_GRID_ORIGIN_Y + SAFE_GRID_RESOLUTION as f64 * SAFE_GRID_SIDE as f64;
        assert!(NAV2_SAFE_START_X >= SAFE_GRID_ORIGIN_X && NAV2_SAFE_START_X < max_x);
        assert!(NAV2_SAFE_START_Y >= SAFE_GRID_ORIGIN_Y && NAV2_SAFE_START_Y < max_y);
    }

    #[test]
    fn scan_bridge_parameters_keep_the_original_order_and_text() {
        let commands = load_ros2_sender_commands().unwrap();
        let schedule = LaserScanSchedule {
            rate_hz: 12.5,
            duration_sec: 3.25,
            burst_count: 4,
            burst_gap_ms: 75,
            max_publishes: 19,
            stamp_mode: "system".to_string(),
        };
        let command = build_scan_bridge_command(
            &commands.scan_bridge,
            Path::new("/opt/ros/jazzy/setup.bash"),
            Path::new("/workspace/install/setup.bash"),
            Path::new("/tmp/payload with spaces.txt"),
            "190",
            &schedule,
        )
        .unwrap();
        let args = command
            .get_args()
            .map(|arg| arg.to_string_lossy().into_owned())
            .collect::<Vec<_>>();
        assert_eq!(
            &args[3..],
            [
                "/opt/ros/jazzy/setup.bash",
                "/workspace/install/setup.bash",
                "190",
                "/tmp/payload with spaces.txt",
                "12.5",
                "3.25",
                "4",
                "75",
                "19",
                "system",
            ]
        );
    }

    #[test]
    fn topic_and_cli_parameters_keep_the_original_order_and_text() {
        let commands = load_ros2_sender_commands().unwrap();
        let options = Ros2TopicOptions {
            qos_profile: Some("sensor_data".to_string()),
            qos_depth: Some(7),
            qos_history: Some("keep_last".to_string()),
            qos_reliability: Some("best_effort".to_string()),
            qos_durability: Some("volatile".to_string()),
            wait_matching_subscriptions: Some(0),
            keep_alive_sec: 0.2,
        };
        let payload = "{\"text\":\"spaces and \\\"quotes\\\" stay intact\"}".to_string();
        let args = build_topic_args(
            &commands.topic,
            &options,
            "/scan input",
            "example_msgs/msg/Example",
            payload.clone(),
        );
        assert_eq!(
            args,
            [
                "ros2",
                "topic",
                "pub",
                "--once",
                "--wait-matching-subscriptions",
                "0",
                "--keep-alive",
                "0.2",
                "--qos-profile",
                "sensor_data",
                "--qos-depth",
                "7",
                "--qos-history",
                "keep_last",
                "--qos-reliability",
                "best_effort",
                "--qos-durability",
                "volatile",
                "/scan input",
                "example_msgs/msg/Example",
                payload.as_str(),
            ]
        );

        let command = ros2_cli_command(
            Path::new("/opt/ros/jazzy/setup.bash"),
            Path::new("/workspace/install/setup.bash"),
            "190",
            &args,
        );
        let cli_args = command
            .get_args()
            .map(|arg| arg.to_string_lossy().into_owned())
            .collect::<Vec<_>>();
        assert_eq!(cli_args[3], "/opt/ros/jazzy/setup.bash");
        assert_eq!(cli_args[4], "/workspace/install/setup.bash");
        assert_eq!(cli_args[5], "190");
        assert_eq!(&cli_args[6..], args);
    }

    #[test]
    fn parameter_preview_uses_ros2_param_set_shape() {
        let value = serde_yaml::Value::from(0.125);
        let preview = super::preview_ros2_parameter_command(
            "/local_costmap/local_costmap",
            "footprint_padding",
            &value,
        )
        .unwrap();

        assert_eq!(
            preview.label,
            "parameter /local_costmap/local_costmap.footprint_padding"
        );
        assert_eq!(
            preview.args,
            [
                "ros2",
                "param",
                "set",
                "/local_costmap/local_costmap",
                "footprint_padding",
                "0.125"
            ]
        );
        assert_eq!(
            preview.command_line,
            "ros2 param set /local_costmap/local_costmap footprint_padding 0.125"
        );
    }

    #[test]
    fn ros2_cli_command_can_use_topic_specific_timeout() {
        let commands = load_ros2_sender_commands().unwrap();
        let args = vec![
            "ros2".to_string(),
            "topic".to_string(),
            "pub".to_string(),
            "--once".to_string(),
            "/odom".to_string(),
            "nav_msgs/msg/Odometry".to_string(),
            "{}".to_string(),
        ];
        let command = build_ros2_cli_command_with_timeout(
            &commands.ros2_cli,
            Path::new("/opt/ros/jazzy/setup.bash"),
            Path::new("/workspace/install/setup.bash"),
            "91",
            &args,
            8,
        )
        .unwrap();
        let timeout = command
            .get_envs()
            .find_map(|(key, value)| {
                if key == "R2D2_ROS2_CLI_TIMEOUT_SEC" {
                    value.map(|value| value.to_string_lossy().into_owned())
                } else {
                    None
                }
            })
            .unwrap();
        assert_eq!(timeout, "8");
    }

    #[test]
    fn ros2_cli_command_can_use_service_specific_timeout() {
        let commands = load_ros2_sender_commands().unwrap();
        let args = vec![
            "ros2".to_string(),
            "service".to_string(),
            "call".to_string(),
            "/global_costmap/clear_entirely_global_costmap".to_string(),
            "nav2_msgs/srv/ClearEntireCostmap".to_string(),
            "{}".to_string(),
        ];
        let command = build_ros2_cli_command_with_timeout(
            &commands.ros2_cli,
            Path::new("/opt/ros/jazzy/setup.bash"),
            Path::new("/workspace/install/setup.bash"),
            "94",
            &args,
            8,
        )
        .unwrap();
        let timeout = command
            .get_envs()
            .find_map(|(key, value)| {
                if key == "R2D2_ROS2_CLI_TIMEOUT_SEC" {
                    value.map(|value| value.to_string_lossy().into_owned())
                } else {
                    None
                }
            })
            .unwrap();
        assert_eq!(timeout, "8");
    }

    #[test]
    fn action_parameters_include_bounded_result_timeout() {
        let commands = load_ros2_sender_commands().unwrap();
        let payload = "{\"pose\":{\"header\":{\"frame_id\":\"map\"}}}".to_string();
        let args = build_action_args(
            &commands.action,
            "/navigate_to_pose",
            "nav2_msgs/action/NavigateToPose",
            payload.clone(),
        );

        assert_eq!(
            args,
            [
                "ros2",
                "action",
                "send_goal",
                "--timeout",
                "5",
                "/navigate_to_pose",
                "nav2_msgs/action/NavigateToPose",
                payload.as_str(),
            ]
        );
    }

    fn lyrical_nav2_interface(relative: &str) -> Interface {
        let repo_root = Path::new(env!("CARGO_MANIFEST_DIR"));
        let ros_share = PathBuf::from("/opt/ros/lyrical/share");
        let nav2_root = repo_root.join("nav2_ws/src_lyrical/navigation2");
        let path = [nav2_root.join(relative), ros_share.join(relative)]
            .into_iter()
            .find(|path| path.is_file())
            .unwrap_or_else(|| nav2_root.join(relative));
        assert!(
            path.is_file(),
            "missing checked-in Lyrical Nav2 interface {}",
            path.display()
        );
        FileExtractor::new(vec![path], vec![ros_share, nav2_root])
            .extract()
            .unwrap_or_else(|error| panic!("failed to extract {relative}: {error}"))
            .into_iter()
            .next()
            .unwrap_or_else(|| panic!("no interface extracted for {relative}"))
    }

    fn nested_at<'a>(tree: &'a ValueTree, path: &[usize]) -> Option<&'a ValueTree> {
        let mut current = tree;
        for index in path {
            let ValueTree::Nested(fields) = current else {
                return None;
            };
            current = fields.get(*index)?;
        }
        Some(current)
    }

    fn array_len(tree: &ValueTree, path: &[usize]) -> Option<usize> {
        match nested_at(tree, path)? {
            ValueTree::Array(items) => Some(items.len()),
            _ => None,
        }
    }

    fn nested_string<'a>(tree: &'a ValueTree, path: &[usize]) -> Option<&'a str> {
        match nested_at(tree, path)? {
            ValueTree::Leaf(Value::String(value)) => Some(value),
            _ => None,
        }
    }

    fn nested_i32(tree: &ValueTree, path: &[usize]) -> Option<i32> {
        match nested_at(tree, path)? {
            ValueTree::Leaf(Value::I32(value)) => Some(*value),
            _ => None,
        }
    }

    fn nested_u32(tree: &ValueTree, path: &[usize]) -> Option<u32> {
        match nested_at(tree, path)? {
            ValueTree::Leaf(Value::U32(value)) => Some(*value),
            _ => None,
        }
    }

    fn nested_u8(tree: &ValueTree, path: &[usize]) -> Option<u8> {
        match nested_at(tree, path)? {
            ValueTree::Leaf(Value::U8(value)) => Some(*value),
            _ => None,
        }
    }

    fn nested_f32(tree: &ValueTree, path: &[usize]) -> Option<f32> {
        match nested_at(tree, path)? {
            ValueTree::Leaf(Value::F32(value)) => Some(*value),
            _ => None,
        }
    }

    fn nested_f64(tree: &ValueTree, path: &[usize]) -> Option<f64> {
        match nested_at(tree, path)? {
            ValueTree::Leaf(Value::F64(value)) => Some(*value),
            _ => None,
        }
    }

    fn nested_bool(tree: &ValueTree, path: &[usize]) -> Option<bool> {
        match nested_at(tree, path)? {
            ValueTree::Leaf(Value::Bool(value)) => Some(*value),
            _ => None,
        }
    }

    fn tf_transform_at<'a>(
        tree: &'a ValueTree,
        transform_index: usize,
        path: &[usize],
    ) -> Option<&'a ValueTree> {
        let ValueTree::Array(transforms) = nested_at(tree, &[0])? else {
            return None;
        };
        nested_at(transforms.get(transform_index)?, path)
    }

    fn tf_transform_string<'a>(
        tree: &'a ValueTree,
        transform_index: usize,
        path: &[usize],
    ) -> Option<&'a str> {
        match tf_transform_at(tree, transform_index, path)? {
            ValueTree::Leaf(Value::String(value)) => Some(value),
            _ => None,
        }
    }

    fn tf_transform_f64(tree: &ValueTree, transform_index: usize, path: &[usize]) -> Option<f64> {
        match tf_transform_at(tree, transform_index, path)? {
            ValueTree::Leaf(Value::F64(value)) => Some(*value),
            _ => None,
        }
    }
}
