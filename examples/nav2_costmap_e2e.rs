//! Jazzy nav2 costmap e2e harness.
//!
//! Read in this order: startup barrier -> dry run bindings -> benchmark -> fuzz loop -> coverage.

use my_r2d2::callback_profile::{CallbackRegistry, profile_trace};
use my_r2d2::interface_extractor::{Extractor, Field, FileExtractor, Interface, Kind, Primitive};
use my_r2d2::payload::{Payload, Value, ValueTree};
use my_r2d2::payload_generator::{FreshSelectionPolicy, GeneratorConfig, PayloadGenerator, Sender};
use my_r2d2::runtime::input_sequence::InputSequence;
use my_r2d2::runtime::ros2_sender::{
    LaserScanSchedule, Ros2ActionSender, Ros2LaserScanSender, Ros2ServiceSender, Ros2TopicOptions,
    Ros2TopicSender, ros2_cli_command,
};
use my_r2d2::runtime::state_oracle::{
    BenchmarkBuilder, BenchmarkModel, BenchmarkStateOracle, DeviationThresholds, TraceDisposition,
};
use my_r2d2::seed_corpus::{Schedule, load_schedules};
use my_r2d2::trace_buffer::{RuntimeDrain, TraceReader, TraceSession};
use my_r2d2::utils::yaml_reader::YamlEnv;
use rand::SeedableRng;
use rand::rngs::StdRng;
use rand::seq::SliceRandom;
use serde_json::json;
use std::collections::BTreeMap;
use std::collections::BTreeSet;
use std::fs;
use std::path::{Path, PathBuf};
use std::thread;
use std::time::{Duration, Instant};
/// startup barrier 的总超时；系统在此之前没有进入可交互稳定态，就直接失败，
/// 不允许 benchmark 吞掉启动竞态。
const DEFAULT_STARTUP_TIMEOUT: Duration = Duration::from_secs(600);
const STARTUP_TIMEOUT_ENV: &str = "R2D2_STACK_STARTUP_TIMEOUT_SEC";
const STARTUP_POLL: Duration = Duration::from_millis(500);
/// 每轮 payload 发送完成后继续 drain，直到 runtime 事件静默收敛。
const ROUND_SETTLE_POLL: Duration = Duration::from_millis(100);
const ROUND_SETTLE_POLLS: usize = 3;
const DEFAULT_ROUND_SETTLE_TIMEOUT: Duration = Duration::from_secs(3);
const DEFAULT_INPUT_SEQUENCE: &str = "config/nav2_sequences/full_stack_bootstrap.yaml";

fn startup_timeout() -> Duration {
    std::env::var(STARTUP_TIMEOUT_ENV)
        .ok()
        .and_then(|value| value.parse::<u64>().ok())
        .filter(|seconds| *seconds > 0)
        .map(Duration::from_secs)
        .unwrap_or(DEFAULT_STARTUP_TIMEOUT)
}

#[derive(Clone)]
enum EndpointBinding {
    LaserScan {
        topic_name: String,
    },
    Topic {
        topic_name: String,
        message_type: String,
        options: Ros2TopicOptions,
    },
    Service {
        service_name: String,
        service_type: String,
        timeout_sec: Option<u64>,
    },
    Action {
        action_name: String,
        action_type: String,
    },
    Parameter {
        endpoint_name: String,
        node_name: String,
        parameter_name: String,
        profile: SafeParameterProfile,
    },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
enum InputSource {
    Manifest,
    RuntimeExtension,
    SafeParameterProfile,
}

impl InputSource {
    fn as_str(self) -> &'static str {
        match self {
            Self::Manifest => "manifest",
            Self::RuntimeExtension => "runtime-extension",
            Self::SafeParameterProfile => "safe-parameter-profile",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum SafeParameterProfile {
    F64 { restore: f64, min: f64, max: f64 },
}

impl SafeParameterProfile {
    fn primitive(self) -> Primitive {
        match self {
            Self::F64 { .. } => Primitive::F64,
        }
    }
}

#[derive(Clone)]
struct InterfaceBinding {
    interface_id: String,
    interface: Interface,
    endpoint: EndpointBinding,
    source: InputSource,
}

impl InterfaceBinding {
    fn new(interface: Interface, endpoint: EndpointBinding) -> Self {
        Self {
            interface_id: interface.name.clone(),
            interface,
            endpoint,
            source: InputSource::RuntimeExtension,
        }
    }

    fn alias(
        interface_id: impl Into<String>,
        interface: Interface,
        endpoint: EndpointBinding,
    ) -> Self {
        Self {
            interface_id: interface_id.into(),
            interface,
            endpoint,
            source: InputSource::RuntimeExtension,
        }
    }

    fn with_source(mut self, source: InputSource) -> Self {
        self.source = source;
        self
    }

    fn generator_interface(&self) -> Interface {
        let mut interface = self.interface.clone();
        interface.name = self.interface_id.clone();
        interface
    }

    fn endpoint_name(&self) -> &str {
        match &self.endpoint {
            EndpointBinding::LaserScan { topic_name } => topic_name,
            EndpointBinding::Topic { topic_name, .. } => topic_name,
            EndpointBinding::Service { service_name, .. } => service_name,
            EndpointBinding::Action { action_name, .. } => action_name,
            EndpointBinding::Parameter { endpoint_name, .. } => endpoint_name,
        }
    }

    fn input_kind(&self) -> Kind {
        match self.endpoint {
            EndpointBinding::LaserScan { .. } | EndpointBinding::Topic { .. } => Kind::Topic,
            EndpointBinding::Service { .. } => Kind::Service,
            EndpointBinding::Action { .. } => Kind::Action,
            EndpointBinding::Parameter { .. } => Kind::Parameter,
        }
    }

    fn input_kind_label(&self) -> &'static str {
        self.input_kind().as_str()
    }

    fn input_source_label(&self) -> &'static str {
        self.source.as_str()
    }
}

fn safe_parameter_payload_value(
    payload: &Payload,
    profile: SafeParameterProfile,
) -> serde_yaml::Value {
    match profile {
        SafeParameterProfile::F64 { restore, min, max } => {
            let value = payload_f64_value(&payload.value)
                .filter(|value| value.is_finite())
                .unwrap_or(restore)
                .clamp(min, max);
            serde_yaml::to_value(value).unwrap_or_else(|_| serde_yaml::Value::from(restore))
        }
    }
}

fn safe_parameter_restore_value(profile: SafeParameterProfile) -> serde_yaml::Value {
    match profile {
        SafeParameterProfile::F64 { restore, .. } => {
            serde_yaml::to_value(restore).unwrap_or_else(|_| serde_yaml::Value::from(restore))
        }
    }
}

fn payload_f64_value(value: &ValueTree) -> Option<f64> {
    match value {
        ValueTree::Leaf(value) => payload_number_value(value),
        ValueTree::Nested(values) | ValueTree::Array(values) => {
            values.iter().find_map(payload_f64_value)
        }
    }
}

fn payload_number_value(value: &Value) -> Option<f64> {
    match value {
        Value::Bool(value) => Some(f64::from(u8::from(*value))),
        Value::I8(value) => Some(*value as f64),
        Value::U8(value) => Some(*value as f64),
        Value::I16(value) => Some(*value as f64),
        Value::U16(value) => Some(*value as f64),
        Value::I32(value) => Some(*value as f64),
        Value::U32(value) => Some(*value as f64),
        Value::I64(value) => Some(*value as f64),
        Value::U64(value) => Some(*value as f64),
        Value::F32(value) => Some(*value as f64),
        Value::F64(value) => Some(*value),
        Value::String(_) => None,
    }
}

struct Config {
    rounds: u64,
    seed: u64,
    benchmark_seconds: u64,
    benchmark_model: Option<PathBuf>,
    latency_factor: f64,
    throughput_floor: f64,
    /// 每轮向 /scan 发布的时长（秒）。
    round_duration_sec: f64,
    /// /scan 发布速率（Hz）。
    bridge_rate_hz: u32,
    /// TSAN 报告输出目录；None 表示不启用 TSAN_OPTIONS 透传。
    tsan_log_dir: Option<PathBuf>,
    /// 每轮 lcov 分支覆盖结果输出目录；None 表示不抓取覆盖。
    lcov_dir: Option<PathBuf>,
    /// LLVM SanitizerCoverage 输出目录；与 TSAN profile 同进程使用。
    sancov_dir: Option<PathBuf>,
    /// nav2-_fuzz 种子语料目录（scans/ 与 schedules/）；None 表示纯生成。
    seed_dir: Option<PathBuf>,
    /// 覆盖 nav2-_fuzz schedule 中的 LaserScan header stamp 模式。
    /// None 表示保持 seed 原样；Some("zero") 在 full-stack 模式下更稳定，
    /// 避免 costmap 因 scan stamp 与 TF cache 轻微错位而长时间等待。
    scan_stamp_mode_override: Option<String>,
    /// Runtime readiness/context sequence. 这些步骤从 YAML 读取，只用于
    /// lifecycle ready 后维持合法 ROS 状态；不得把 coverage-path scripting
    /// 写进这里。
    input_sequence: Option<PathBuf>,
    /// 每 N 轮强制重新从接口集合生成一次 payload；None/0 表示完全沿用
    /// R2D2 的 pool-first 变异循环。
    fresh_generation_period: Option<u64>,
    /// fresh generation 使用随机接口还是 round-robin 接口轮转。
    fresh_selection: FreshSelectionPolicy,
    /// full-stack fuzz 中，action 或 /goal_pose 可能让 Nav2 内部 goal 持续运行，
    /// 污染后续 round。默认每轮后取消当前 Nav2 goal；需要原始行为可传
    /// --no-action-cleanup。
    action_cleanup: bool,
    /// 每轮 payload 发送后继续收集 runtime trace 的最长等待时间。默认保持
    /// 原先 3s；full-stack action/BT 路径可通过 CLI 拉长观测窗口。
    round_settle_timeout: Duration,
    /// lcov 的 --gcov-tool 值（可含空格构成命令行，如
    /// "/usr/bin/llvm-cov-18 gcov"）；None 表示用 lcov 自动探测。
    gcov_tool: Option<String>,
}

impl Config {
    fn parse(args: impl Iterator<Item = String>) -> Result<Self, String> {
        let mut config = Self {
            rounds: 10,
            seed: 42,
            benchmark_seconds: 7_200,
            benchmark_model: None,
            latency_factor: 2.0,
            throughput_floor: 0.5,
            round_duration_sec: 2.0,
            bridge_rate_hz: 20,
            tsan_log_dir: None,
            lcov_dir: None,
            sancov_dir: None,
            seed_dir: None,
            scan_stamp_mode_override: None,
            input_sequence: None,
            fresh_generation_period: None,
            fresh_selection: FreshSelectionPolicy::Random,
            action_cleanup: true,
            round_settle_timeout: DEFAULT_ROUND_SETTLE_TIMEOUT,
            gcov_tool: None,
        };
        let mut args = args.peekable();
        while let Some(arg) = args.next() {
            let mut value = |flag: &str| -> Result<String, String> {
                args.next().ok_or_else(|| format!("{flag} needs a value"))
            };
            match arg.as_str() {
                "--rounds" => {
                    config.rounds = value("--rounds")?
                        .parse::<u64>()
                        .map_err(|e| e.to_string())?
                }
                "--seed" => {
                    config.seed = value("--seed")?.parse::<u64>().map_err(|e| e.to_string())?
                }
                "--benchmark-seconds" => {
                    config.benchmark_seconds = value("--benchmark-seconds")?
                        .parse::<u64>()
                        .map_err(|e| e.to_string())?
                }
                "--benchmark-model" => {
                    config.benchmark_model = Some(PathBuf::from(value("--benchmark-model")?))
                }
                "--latency-factor" => {
                    config.latency_factor = value("--latency-factor")?
                        .parse::<f64>()
                        .map_err(|e| e.to_string())?
                }
                "--throughput-floor" => {
                    config.throughput_floor = value("--throughput-floor")?
                        .parse::<f64>()
                        .map_err(|e| e.to_string())?
                }
                "--round-duration" => {
                    config.round_duration_sec = value("--round-duration")?
                        .parse::<f64>()
                        .map_err(|e| e.to_string())?
                }
                "--bridge-rate" => {
                    config.bridge_rate_hz = value("--bridge-rate")?
                        .parse::<u32>()
                        .map_err(|e| e.to_string())?
                }
                "--tsan-log-dir" => {
                    config.tsan_log_dir = Some(PathBuf::from(value("--tsan-log-dir")?))
                }
                "--lcov-dir" => config.lcov_dir = Some(PathBuf::from(value("--lcov-dir")?)),
                "--sancov-dir" => config.sancov_dir = Some(PathBuf::from(value("--sancov-dir")?)),
                "--seed-dir" => config.seed_dir = Some(PathBuf::from(value("--seed-dir")?)),
                "--scan-stamp-mode" => {
                    let mode = value("--scan-stamp-mode")?;
                    match mode.as_str() {
                        "now" | "zero" | "past" | "future" | "backward" => {
                            config.scan_stamp_mode_override = Some(mode)
                        }
                        other => {
                            return Err(format!(
                                "unknown --scan-stamp-mode {other}; expected now, zero, past, future, or backward"
                            ));
                        }
                    }
                }
                "--input-sequence" => {
                    config.input_sequence = Some(PathBuf::from(value("--input-sequence")?))
                }
                "--fresh-generation-period" => {
                    let period = value("--fresh-generation-period")?
                        .parse::<u64>()
                        .map_err(|e| e.to_string())?;
                    config.fresh_generation_period = (period > 0).then_some(period);
                }
                "--fresh-selection" => {
                    config.fresh_selection =
                        FreshSelectionPolicy::parse_cli(&value("--fresh-selection")?)?
                }
                "--no-action-cleanup" => config.action_cleanup = false,
                "--round-settle-seconds" => {
                    let seconds = value("--round-settle-seconds")?
                        .parse::<f64>()
                        .map_err(|e| e.to_string())?;
                    if !seconds.is_finite() || seconds <= 0.0 {
                        return Err(
                            "--round-settle-seconds must be a positive finite number".into()
                        );
                    }
                    config.round_settle_timeout = Duration::from_secs_f64(seconds);
                }
                "--gcov-tool" => config.gcov_tool = Some(value("--gcov-tool")?),
                other => return Err(format!("unknown flag {other}")),
            }
        }
        if config.tsan_log_dir.is_some() && config.lcov_dir.is_some() {
            return Err("--tsan-log-dir and --lcov-dir must be run in separate campaigns".into());
        }
        if config.sancov_dir.is_some() && config.lcov_dir.is_some() {
            return Err("--sancov-dir and --lcov-dir must be run in separate campaigns".into());
        }
        Ok(config)
    }

    fn generator_config(&self) -> GeneratorConfig {
        GeneratorConfig {
            fresh_generation_period: self.fresh_generation_period,
            fresh_selection: self.fresh_selection,
            ..GeneratorConfig::default()
        }
    }
}

#[path = "nav2_costmap_e2e/benchmark.rs"]
mod benchmark;
#[path = "nav2_costmap_e2e/bindings.rs"]
mod bindings;
#[path = "nav2_costmap_e2e/coverage.rs"]
mod coverage;
#[path = "nav2_costmap_e2e/stack.rs"]
mod stack;

use benchmark::{build_benchmark_model_live, write_benchmark_status};
use bindings::{extract_stack_bindings, find_binding, ros_share_root};
use coverage::{
    capture_round_coverage, capture_sancov_coverage, clear_stale_gcda, cumulative_branches,
    finalize_coverage, flush_stack_coverage, flush_stack_sancov, flush_stack_sancov_final,
    package_coverage_reports, package_sancov_reports, sancov_target_pc_count,
};
use stack::{
    drain_runtime_snapshot, ensure_full_stack_nodes_visible, execute_input_sequence_phase,
    execute_payload_round, restart_ready_stack, shutdown_stack, stack_alive, start_ready_stack,
};

fn write_json_report(path: &Path, value: impl serde::Serialize) {
    if let Some(parent) = path.parent() {
        let _ = fs::create_dir_all(parent);
    }
    if let Ok(bytes) = serde_json::to_vec_pretty(&value) {
        let _ = fs::write(path, bytes);
    }
}

fn record_post_coverage_stack_health(
    round_label: &str,
    ros_setup: &Path,
    install_setup: &Path,
    domain_id: &str,
    stack: &mut std::process::Child,
    crashed: &mut bool,
    stack_health_error: &mut Option<String>,
) -> bool {
    if *crashed {
        return false;
    }
    let label = format!("{round_label} post-coverage");
    match ensure_full_stack_nodes_visible(&label, ros_setup, install_setup, domain_id, stack) {
        Ok(()) => false,
        Err(error) => {
            *crashed = true;
            let message =
                format!("{round_label}: stack became unhealthy after coverage flush: {error}");
            if stack_health_error.is_none() {
                *stack_health_error = Some(message.clone());
            }
            eprintln!("{message}");
            true
        }
    }
}

fn write_payload_debug(path: &Path, payload: &Payload) {
    const HEX_PREVIEW_BYTES: usize = 4096;

    if let Some(parent) = path.parent() {
        let _ = fs::create_dir_all(parent);
    }

    let preview_len = payload.serialized.len().min(HEX_PREVIEW_BYTES);
    let mut hex_preview = String::with_capacity(preview_len * 2);
    for byte in &payload.serialized[..preview_len] {
        use std::fmt::Write as _;
        let _ = write!(&mut hex_preview, "{byte:02x}");
    }
    let truncated_bytes = payload.serialized.len().saturating_sub(preview_len);
    let report = format!(
        "interface_id: {}\nkind: {:?}\nrng_seed: {}\nserialized_len: {}\nserialized_hex_prefix: {}\nserialized_truncated_bytes: {}\nvalue:\n{:#?}\n",
        payload.interface_id,
        payload.kind,
        payload.rng_seed,
        payload.serialized.len(),
        hex_preview,
        truncated_bytes,
        payload.value
    );
    let _ = fs::write(path, report);
}

fn count_files_with_extension(root: &Path, extension: &str) -> usize {
    let Ok(entries) = fs::read_dir(root) else {
        return 0;
    };
    entries
        .flatten()
        .filter(|entry| entry.path().extension().is_some_and(|ext| ext == extension))
        .count()
}

fn filename_token(value: &str) -> String {
    value
        .chars()
        .map(|ch| {
            if ch.is_ascii_alphanumeric() || matches!(ch, '-' | '_') {
                ch
            } else {
                '_'
            }
        })
        .collect()
}

fn default_input_sequence_path(repo_root: &Path) -> PathBuf {
    repo_root.join(DEFAULT_INPUT_SEQUENCE)
}

fn trace_disposition_label(disposition: TraceDisposition) -> &'static str {
    match disposition {
        TraceDisposition::Analyzed => "analyzed",
        TraceDisposition::Empty => "empty",
        TraceDisposition::Invalid => "invalid",
    }
}

fn increment_counter(map: &mut BTreeMap<String, u64>, label: &str, amount: u64) {
    *map.entry(label.to_string()).or_default() += amount;
}

fn format_counts(counts: BTreeMap<String, u64>) -> String {
    counts
        .into_iter()
        .map(|(label, count)| format!("{label}:{count}"))
        .collect::<Vec<_>>()
        .join(",")
}

fn binding_kind_counts(bindings: &[InterfaceBinding]) -> BTreeMap<String, u64> {
    let mut counts = BTreeMap::new();
    for binding in bindings {
        increment_counter(&mut counts, binding.input_kind_label(), 1);
    }
    counts
}

fn binding_source_counts(bindings: &[InterfaceBinding]) -> BTreeMap<String, u64> {
    let mut counts = BTreeMap::new();
    for binding in bindings {
        increment_counter(&mut counts, binding.input_source_label(), 1);
    }
    counts
}

fn load_input_sequence(config: &Config, repo_root: &Path) -> Result<InputSequence, String> {
    let (path, source) = match &config.input_sequence {
        Some(path) => (path.clone(), "configured"),
        None => (default_input_sequence_path(repo_root), "default"),
    };
    let sequence = InputSequence::load(&path)?;
    audit_readiness_only_input_sequence(&sequence)?;
    println!(
        "input-sequence: loaded {} {} steps from {} ({})",
        source,
        sequence.steps.len(),
        sequence.path.display(),
        sequence.name
    );
    Ok(sequence)
}

fn audit_readiness_only_input_sequence(sequence: &InputSequence) -> Result<(), String> {
    for step in &sequence.steps {
        if !readiness_context_topic(&step.name) {
            return Err(format!(
                "input-sequence {} step {} publishes {}; warm-up is readiness-only and may only publish basic pose/map/scan/odom/tf context",
                sequence.path.display(),
                step.label_or_name(),
                step.name
            ));
        }
    }
    Ok(())
}

fn readiness_context_topic(name: &str) -> bool {
    matches!(
        name,
        "/initialpose" | "/map" | "/scan" | "/odom" | "/tf" | "/tf_static"
    )
}

fn main() {
    let config = match Config::parse(std::env::args().skip(1)) {
        Ok(config) => config,
        Err(message) => {
            eprintln!("nav2_costmap_e2e: {message}");
            std::process::exit(2);
        }
    };
    // 运行所需配置全部从 YAML 读取；缺哪个就直接报哪个键为空。
    let repo_root = Path::new(env!("CARGO_MANIFEST_DIR"));
    let yaml_env = match YamlEnv::load(repo_root) {
        Ok(env) => env,
        Err(message) => {
            eprintln!("nav2_costmap_e2e: {message}");
            std::process::exit(1);
        }
    };
    let nav2_ws = match yaml_env.require_path("R2D2_NAV2_WS") {
        Ok(path) => path,
        Err(message) => {
            eprintln!("nav2_costmap_e2e: {message}");
            std::process::exit(1);
        }
    };
    let nav2_source_root = match yaml_env.require_path("R2D2_NAV2_SOURCE_ROOT") {
        Ok(path) => path,
        Err(message) => {
            eprintln!("nav2_costmap_e2e: {message}");
            std::process::exit(1);
        }
    };
    let nav2_build_base = match yaml_env.require_path("R2D2_NAV2_BUILD_BASE") {
        Ok(path) => path,
        Err(message) => {
            eprintln!("nav2_costmap_e2e: {message}");
            std::process::exit(1);
        }
    };
    let install_setup = match yaml_env.require_path("R2D2_NAV2_INSTALL_SETUP") {
        Ok(path) => path,
        Err(message) => {
            eprintln!("nav2_costmap_e2e: {message}");
            std::process::exit(1);
        }
    };
    let ros_setup = match yaml_env.require_path("R2D2_ROS_SETUP") {
        Ok(path) => path,
        Err(message) => {
            eprintln!("nav2_costmap_e2e: {message}");
            std::process::exit(1);
        }
    };
    let trace_dir = match std::env::var_os("R2D2_SHM_PATH").filter(|path| !path.is_empty()) {
        Some(path) => PathBuf::from(path),
        None => match yaml_env.require_path("R2D2_SHM_PATH") {
            Ok(path) => path,
            Err(message) => {
                eprintln!("nav2_costmap_e2e: {message}");
                std::process::exit(1);
            }
        },
    };
    let domain_id = match std::env::var("ROS_DOMAIN_ID") {
        Ok(value) if !value.trim().is_empty() => value,
        _ => match yaml_env.require_string("ROS_DOMAIN_ID") {
            Ok(value) => value,
            Err(message) => {
                eprintln!("nav2_costmap_e2e: {message}");
                std::process::exit(1);
            }
        },
    };

    let payload_file = nav2_ws.join(format!(
        "payload_round_{}_{}.txt",
        filename_token(&domain_id),
        std::process::id()
    ));
    if !install_setup.exists() {
        eprintln!(
            "nav2_costmap_e2e: {} missing; build selected R2D2_PROFILE first",
            install_setup.display()
        );
        std::process::exit(1);
    }

    let share_root = match ros_share_root(&ros_setup) {
        Ok(path) => path,
        Err(message) => {
            eprintln!("nav2_costmap_e2e: {message}");
            std::process::exit(1);
        }
    };
    let interface_roots = vec![share_root, nav2_source_root];
    clear_stale_gcda(&nav2_build_base);

    // 1. 启动插桩 Nav2 栈，然后由 harness 接管 ready barrier。
    let (mut stack, mut session, mut reader, mut registry, registrations) = {
        let mut started = None;
        let mut last_error = String::new();
        for attempt in 1..=2 {
            match start_ready_stack(
                &nav2_ws,
                &trace_dir,
                &config,
                &ros_setup,
                &install_setup,
                &domain_id,
            ) {
                Ok(value) => {
                    started = Some(value);
                    break;
                }
                Err(message) => {
                    last_error = message;
                    if attempt < 2 {
                        eprintln!("startup: attempt {attempt} failed: {last_error}; retrying once");
                    }
                }
            }
        }
        match started {
            Some(value) => value,
            None => {
                eprintln!("nav2_costmap_e2e: startup barrier failed: {last_error}");
                std::process::exit(1);
            }
        }
    };
    let infos = registry.callback_infos();
    if let Some(root) = &config.lcov_dir {
        write_json_report(&root.join("callbacks.json"), &infos);
    }
    println!(
        "startup: ready barrier passed; {} registration records, {} complete callbacks: {}",
        registrations,
        infos.len(),
        infos
            .iter()
            .map(|i| format!("{} [{:?}]", i.name, i.callback_type))
            .collect::<Vec<_>>()
            .join(", ")
    );

    // 3. dry run：系统 ready 后再构建本轮 live 目标的接口规格。
    let bindings = match extract_stack_bindings(&interface_roots) {
        Ok(bindings) => bindings,
        Err(message) => {
            eprintln!("nav2_costmap_e2e: dry run failed: {message}");
            std::process::exit(1);
        }
    };
    let generator_bindings = bindings.iter().collect::<Vec<_>>();
    let interfaces = generator_bindings
        .iter()
        .map(|binding| binding.generator_interface())
        .collect::<Vec<_>>();
    if interfaces.is_empty() {
        eprintln!("nav2_costmap_e2e: no fuzzable interfaces after dry run");
        std::process::exit(1);
    }
    println!(
        "dry run: extracted {} full-stack interfaces: {}",
        bindings.len(),
        bindings
            .iter()
            .map(|binding| format!("{} -> {}", binding.interface_id, binding.endpoint_name()))
            .collect::<Vec<_>>()
            .join(", ")
    );
    println!(
        "dry run: input attribution kind={} source={}",
        format_counts(binding_kind_counts(&bindings)),
        format_counts(binding_source_counts(&bindings))
    );

    let input_sequence = match load_input_sequence(&config, repo_root) {
        Ok(sequence) => sequence,
        Err(message) => {
            eprintln!("nav2_costmap_e2e: input sequence failed: {message}");
            std::process::exit(1);
        }
    };

    if let Err(message) = execute_input_sequence_phase(
        Some(&input_sequence),
        my_r2d2::runtime::input_sequence::InputSequencePhase::Startup,
        "startup input-sequence",
        &bindings,
        &mut registry,
        &mut reader,
        &ros_setup,
        &install_setup,
        &domain_id,
        true,
    ) {
        eprintln!("nav2_costmap_e2e: {message}");
        shutdown_stack(&ros_setup, &install_setup, &domain_id, &mut stack);
        let _ = fs::remove_file(&payload_file);
        std::process::exit(1);
    }
    let _ = drain_runtime_snapshot("post-bootstrap runtime", &mut reader, &mut registry);
    let mut seen_sancov_pcs = BTreeSet::new();
    if let Some(sancov_root) = &config.sancov_dir {
        let _ = fs::create_dir_all(sancov_root);
        let startup_dir = sancov_root.join("startup");
        flush_stack_sancov(stack.id());
        if let Some(coverage) =
            capture_sancov_coverage(sancov_root, &startup_dir, &mut seen_sancov_pcs)
        {
            println!(
                "sancov: startup target pcs {} (+{}), all pcs {} (+{}) -> {}",
                coverage.target_covered_pcs,
                coverage.target_pc_increase,
                coverage.all_covered_pcs,
                coverage.all_pc_increase,
                startup_dir.display()
            );
            write_json_report(
                &sancov_root.join("coverage/status.json"),
                json!({
                    "phase":"startup",
                    "round":0,
                    "coverage_ok":true,
                    "sancov_covered_pcs":coverage.target_covered_pcs,
                    "sancov_covered_pc_increase":coverage.target_pc_increase,
                    "sancov_all_covered_pcs":coverage.all_covered_pcs,
                    "sancov_all_pc_increase":coverage.all_pc_increase,
                    "packages":coverage.packages
                }),
            );
        }
    }

    // 4. 主循环。仅保留时序 schedule 语料；payload pool 只接收真实触发
    // new state / crash 的输入，不再把单一 LaserScan seeds 预填进去。
    let mut schedules: Vec<(String, Schedule)> = Vec::new();
    if let Some(seed_dir) = &config.seed_dir {
        match load_schedules(&seed_dir.join("schedules")) {
            Ok(mut loaded) => {
                let mut rng = StdRng::seed_from_u64(config.seed ^ 0x9E37_79B9);
                loaded.shuffle(&mut rng);
                println!(
                    "seeds: {} schedules loaded from {}",
                    loaded.len(),
                    seed_dir.display()
                );
                schedules = loaded;
            }
            Err(error) => eprintln!("seeds: schedule loading failed: {error}"),
        }
    }
    let benchmark = match build_benchmark_model_live(
        &config,
        &bindings,
        &nav2_ws,
        &trace_dir,
        &ros_setup,
        &install_setup,
        &payload_file,
        &domain_id,
        &mut registry,
        &mut reader,
        &schedules,
        Some(&input_sequence),
        &mut stack,
        &mut session,
    ) {
        Ok(result) => result,
        Err(message) => {
            write_benchmark_status(
                config.lcov_dir.as_deref(),
                "failed",
                config.benchmark_seconds,
                Duration::ZERO,
                0,
                0,
                0,
                0,
                Some(&message),
            );
            if let Some(sancov_root) = &config.sancov_dir {
                flush_stack_sancov(stack.id());
                let failure_dir = sancov_root.join("benchmark_failed");
                if let Some(coverage) =
                    capture_sancov_coverage(sancov_root, &failure_dir, &mut seen_sancov_pcs)
                {
                    write_json_report(
                        &sancov_root.join("coverage/status.json"),
                        json!({
                            "phase":"benchmark_failed",
                            "round":0,
                            "coverage_ok":true,
                            "sancov_covered_pcs":coverage.target_covered_pcs,
                            "sancov_covered_pc_increase":coverage.target_pc_increase,
                            "sancov_all_covered_pcs":coverage.all_covered_pcs,
                            "sancov_all_pc_increase":coverage.all_pc_increase,
                            "packages":coverage.packages
                        }),
                    );
                }
            }
            eprintln!("nav2_costmap_e2e: benchmark failed: {message}");
            shutdown_stack(&ros_setup, &install_setup, &domain_id, &mut stack);
            let _ = fs::remove_file(&payload_file);
            std::process::exit(1);
        }
    };
    let mut generator = PayloadGenerator::new(interfaces, config.generator_config(), config.seed);
    let thresholds = DeviationThresholds::new(config.latency_factor, config.throughput_floor);
    let mut oracle = BenchmarkStateOracle::new(benchmark, thresholds);
    println!(
        "loop: rounds={} seed={} benchmark_seconds={} benchmark_traces={} oracle=callback-trace latency_factor={} throughput_floor={} fresh_generation_period={} fresh_selection={} scan_stamp_mode={} action_cleanup={} round_settle_seconds={:.3}",
        config.rounds,
        config.seed,
        config.benchmark_seconds,
        oracle.benchmark().analyzed_traces,
        config.latency_factor,
        config.throughput_floor,
        config
            .fresh_generation_period
            .map(|period| period.to_string())
            .unwrap_or_else(|| "off".to_string()),
        config.fresh_selection.as_str(),
        config.scan_stamp_mode_override.as_deref().unwrap_or("seed"),
        config.action_cleanup,
        config.round_settle_timeout.as_secs_f64()
    );
    if config.benchmark_seconds > 600 {
        eprintln!(
            "benchmark: warning --benchmark-seconds={} builds a long baseline before fuzzing; this can absorb callback/coverage states and make later fuzz rounds appear to plateau",
            config.benchmark_seconds
        );
    }
    if config.lcov_dir.is_none() && config.sancov_dir.is_none() {
        eprintln!("coverage: disabled; pass --lcov-dir <dir> or --sancov-dir <dir> for feedback");
    }
    let mut crashes = 0u64;
    let mut active_new_states = 0u64;
    let mut input_kind_rounds = BTreeMap::new();
    let mut input_source_rounds = BTreeMap::new();
    let mut input_kind_new_states = BTreeMap::new();
    let mut input_source_new_states = BTreeMap::new();
    let mut input_kind_branch_increase = BTreeMap::new();
    let mut input_source_branch_increase = BTreeMap::new();
    let mut input_kind_sancov_increase = BTreeMap::new();
    let mut input_source_sancov_increase = BTreeMap::new();
    let mut post_coverage_stack_health_failures = 0u64;
    let mut invalid = 0u64;
    let mut empty_rounds = 0u64;
    let mut prev_branches: u64 = 0;
    if let Some(lcov_root) = &config.lcov_dir {
        let _ = fs::create_dir_all(lcov_root);
        let benchmark_dir = lcov_root.join("benchmark");
        flush_stack_sancov(stack.id());
        if let Some((covered, total)) = capture_round_coverage(
            &nav2_build_base,
            &benchmark_dir,
            config.gcov_tool.as_deref(),
        ) {
            let packages = package_coverage_reports(
                &benchmark_dir.join("coverage.info"),
                &benchmark_dir.join("packages"),
            );
            prev_branches = covered;
            println!(
                "coverage: benchmark baseline branches {covered}/{total} -> {}",
                benchmark_dir.display()
            );
            write_json_report(
                &lcov_root.join("coverage/status.json"),
                json!({"phase":"benchmark","round":0,"coverage_ok":true,"branch_covered_total":covered,"branch_total":total,"branch_covered_increase":covered,"packages":packages}),
            );
        } else {
            eprintln!("coverage: benchmark baseline capture failed");
            write_json_report(
                &lcov_root.join("coverage/status.json"),
                json!({"phase":"benchmark","round":0,"coverage_ok":false}),
            );
        }
    }
    if let Some(sancov_root) = &config.sancov_dir {
        let _ = fs::create_dir_all(sancov_root);
        let benchmark_dir = sancov_root.join("benchmark");
        flush_stack_sancov(stack.id());
        if let Some(coverage) =
            capture_sancov_coverage(sancov_root, &benchmark_dir, &mut seen_sancov_pcs)
        {
            println!(
                "sancov: benchmark baseline target pcs {} (+{}), all pcs {} (+{}) -> {}",
                coverage.target_covered_pcs,
                coverage.target_pc_increase,
                coverage.all_covered_pcs,
                coverage.all_pc_increase,
                benchmark_dir.display()
            );
            write_json_report(
                &sancov_root.join("coverage/status.json"),
                json!({
                    "phase":"benchmark",
                    "round":0,
                    "coverage_ok":true,
                    "sancov_covered_pcs":coverage.target_covered_pcs,
                    "sancov_covered_pc_increase":coverage.target_pc_increase,
                    "sancov_all_covered_pcs":coverage.all_covered_pcs,
                    "sancov_all_pc_increase":coverage.all_pc_increase,
                    "packages":coverage.packages
                }),
            );
        } else {
            eprintln!("sancov: benchmark baseline capture failed");
            write_json_report(
                &sancov_root.join("coverage/status.json"),
                json!({"phase":"benchmark","round":0,"coverage_ok":false}),
            );
        }
    }

    for round in 1..=config.rounds {
        if !stack_alive(&mut stack) {
            println!("round {round:02}: costmap stack is dead, stopping");
            crashes += 1;
            break;
        }
        let payload = match generator.next_payload() {
            Ok(payload) => payload,
            Err(error) => {
                eprintln!("round {round}: generation failed: {error}");
                continue;
            }
        };
        let Some(binding) = find_binding(&bindings, &payload.interface_id) else {
            eprintln!("round {round}: unknown interface {}", payload.interface_id);
            continue;
        };
        let round_label = format!("round {round}");
        let mut execution = None;
        let mut abort_loop = false;
        for attempt in 1..=2 {
            match execute_payload_round(
                round,
                &round_label,
                &payload,
                binding,
                &bindings,
                &mut registry,
                &mut reader,
                &ros_setup,
                &install_setup,
                &payload_file,
                &domain_id,
                &schedules,
                &config,
                Some(&input_sequence),
                &mut stack,
                &mut session,
            ) {
                Ok(result) => {
                    execution = Some(result);
                    break;
                }
                Err(message) => {
                    eprintln!("round {round}: {message}");
                    if attempt == 2 {
                        eprintln!("round {round}: execution failed after retry; skipping payload");
                        break;
                    }
                    eprintln!(
                        "round {round}: restarting stack and retrying the same payload after failed execution"
                    );
                    if let Err(restart_error) = restart_ready_stack(
                        &nav2_ws,
                        &trace_dir,
                        &config,
                        &ros_setup,
                        &install_setup,
                        &domain_id,
                        &bindings,
                        Some(&input_sequence),
                        &mut stack,
                        &mut session,
                        &mut reader,
                        &mut registry,
                        "loop",
                    ) {
                        eprintln!("round {round}: restart failed: {restart_error}");
                        crashes += 1;
                        abort_loop = true;
                        break;
                    }
                }
            }
        }
        if abort_loop {
            break;
        }
        let Some(execution) = execution else {
            continue;
        };
        let trace = execution.trace;
        let mut crashed = execution.crashed;
        let mut stack_health_error = execution.stack_health_error.clone();
        let verdict = oracle.evaluate(&trace, crashed);
        let input_kind = binding.input_kind_label();
        let input_source = binding.input_source_label();
        increment_counter(&mut input_kind_rounds, input_kind, 1);
        increment_counter(&mut input_source_rounds, input_source, 1);
        let empty = verdict.trace == TraceDisposition::Empty;
        if empty && !crashed {
            empty_rounds += 1;
        } else if verdict.trace == TraceDisposition::Invalid {
            invalid += 1;
        }
        let new_state = verdict.new_state;
        let mut crash_counted = false;
        if crashed {
            crashes += 1;
            crash_counted = true;
        }
        if new_state {
            active_new_states += 1;
            increment_counter(&mut input_kind_new_states, input_kind, 1);
            increment_counter(&mut input_source_new_states, input_source, 1);
        }
        generator.retain_if_interesting(payload.clone(), &oracle);

        let mut decision = if crashed {
            if new_state {
                "crash+new-state"
            } else {
                "crash"
            }
        } else if empty {
            "empty"
        } else if new_state {
            "new-state"
        } else {
            "none"
        };
        let trace_disposition = trace_disposition_label(verdict.trace);
        let execs: Vec<String> = trace
            .call_trace
            .iter()
            .take(3)
            .map(|l| format!("{}ns", l.execution_latency))
            .collect();
        let throughputs: Vec<String> = trace
            .msg_trace
            .iter()
            .take(2)
            // throughput 内部单位是 bytes/ns（buffer_size / 耗时）；×1e3 换算
            // 为 MB/s，否则日志里几乎所有值都四舍五入成 0.00。
            .map(|m| format!("{:.2}", m.throughput * 1e3))
            .collect();
        let round_line = format!(
            "round {round:02} | iface={:<24} | ep={:<30} | kind={:<9} source={:<22} | len={:4} | calls={} msgs={} | trace={trace_disposition:<8} | exec=[{}] thr=[{}] MB/s | decision={decision:<14} | active={} | evidence=edge:{} cb:{} msg:{} lat:{} thr:{} | pool={} | sched={}",
            execution.interface_label,
            execution.endpoint_label,
            input_kind,
            input_source,
            payload.serialized.len(),
            trace.call_trace.len(),
            trace.msg_trace.len(),
            execs.join(","),
            throughputs.join(","),
            verdict.new_state,
            verdict.evidence.new_edge,
            verdict.evidence.new_callback,
            verdict.evidence.new_message,
            verdict.evidence.latency_deviation,
            verdict.evidence.throughput_deviation,
            generator.pool().len(),
            execution.sched_name.as_deref().unwrap_or("-"),
        );
        println!("{round_line}");
        if let Some(error) = &stack_health_error {
            println!("       stack-health: {error}");
        }

        if let Some(lcov_root) = &config.lcov_dir {
            let round_dir = lcov_root.join(format!("rounds/round_{round:06}"));
            write_json_report(&round_dir.join("trace.json"), &trace);
            flush_stack_coverage(stack.id());
            let stats =
                capture_round_coverage(&nav2_build_base, &round_dir, config.gcov_tool.as_deref());
            let coverage_ok = stats.is_some();
            let packages = coverage_ok.then(|| {
                package_coverage_reports(
                    &round_dir.join("coverage.info"),
                    &round_dir.join("packages"),
                )
            });
            if record_post_coverage_stack_health(
                &round_label,
                &ros_setup,
                &install_setup,
                &domain_id,
                &mut stack,
                &mut crashed,
                &mut stack_health_error,
            ) {
                post_coverage_stack_health_failures += 1;
                if !crash_counted {
                    crashes += 1;
                    crash_counted = true;
                    if empty {
                        empty_rounds = empty_rounds.saturating_sub(1);
                    }
                }
                decision = if new_state {
                    "crash+new-state"
                } else {
                    "crash"
                };
                if let Some(error) = &stack_health_error {
                    println!("       stack-health: {error}");
                    println!("       decision-corrected: {decision}");
                }
            }
            let (captured, total) = stats.unwrap_or((prev_branches, prev_branches));
            // 累计分支计数只会增长；SIGUSR1 dump 与进行中的回调并发时，快照
            // 会漏掉该回调尚未走到的分支，读数低于上一轮。以上一轮为下界。
            let (covered, dipped) = cumulative_branches(captured, prev_branches);
            let increase = covered - prev_branches;
            prev_branches = covered;
            increment_counter(&mut input_kind_branch_increase, input_kind, increase);
            increment_counter(&mut input_source_branch_increase, input_source, increase);
            let _ = fs::write(round_dir.join("payload.txt"), &payload.serialized);
            write_payload_debug(&round_dir.join("payload_debug.txt"), &payload);
            if let Some(preview) = &execution.send_preview {
                write_json_report(&round_dir.join("send_command.json"), preview);
                let _ = fs::write(
                    round_dir.join("send_command.sh"),
                    format!("{}\n", preview.command_line),
                );
            }
            let _ = fs::write(round_dir.join("round.txt"), format!("{round_line}\n"));
            write_json_report(
                &lcov_root.join("coverage/status.json"),
                json!({"phase":"fuzz","round":round,"coverage_ok":coverage_ok,"branch_covered_total":covered,"branch_total":total,"branch_covered_increase":increase,"packages":packages}),
            );
            write_json_report(
                &round_dir.join("summary.json"),
                json!({
                    "round": round,
                    "decision": decision,
                    "trace_disposition": trace_disposition,
                    "oracle": "callback-trace",
                    "active_new_state": verdict.new_state,
                    "input_kind": input_kind,
                    "input_source": input_source,
                    "evidence": verdict.evidence,
                    "calls": trace.call_trace.len(),
                    "msgs": trace.msg_trace.len(),
                    "pool_size": generator.pool().len(),
                    "crash_or_hang": crashed,
                    "stack_health_error": stack_health_error.as_deref(),
                    "coverage_ok": coverage_ok,
                    "branch_covered_total": covered,
                    "branch_covered_increase": increase,
                    "packages": packages,
                }),
            );
            println!(
                "       coverage: branches {covered}/{total} (+{increase}){} -> {}",
                if dipped { " [dip clamped]" } else { "" },
                round_dir.display()
            );
        }
        if let Some(sancov_root) = &config.sancov_dir {
            let round_dir = sancov_root.join(format!("rounds/round_{round:06}"));
            write_json_report(&round_dir.join("trace.json"), &trace);
            flush_stack_sancov(stack.id());
            let stats = capture_sancov_coverage(sancov_root, &round_dir, &mut seen_sancov_pcs);
            let coverage_ok = stats.is_some();
            let covered = stats
                .as_ref()
                .map(|coverage| coverage.target_covered_pcs)
                .unwrap_or_else(|| sancov_target_pc_count(&seen_sancov_pcs));
            let increase = stats
                .as_ref()
                .map(|coverage| coverage.target_pc_increase)
                .unwrap_or(0);
            let all_covered = stats
                .as_ref()
                .map(|coverage| coverage.all_covered_pcs)
                .unwrap_or(seen_sancov_pcs.len() as u64);
            let all_increase = stats
                .as_ref()
                .map(|coverage| coverage.all_pc_increase)
                .unwrap_or(0);
            let packages = stats.map(|coverage| coverage.packages).unwrap_or_default();
            if record_post_coverage_stack_health(
                &round_label,
                &ros_setup,
                &install_setup,
                &domain_id,
                &mut stack,
                &mut crashed,
                &mut stack_health_error,
            ) {
                post_coverage_stack_health_failures += 1;
                if !crash_counted {
                    crashes += 1;
                    if empty {
                        empty_rounds = empty_rounds.saturating_sub(1);
                    }
                }
                decision = if new_state {
                    "crash+new-state"
                } else {
                    "crash"
                };
                if let Some(error) = &stack_health_error {
                    println!("       stack-health: {error}");
                    println!("       decision-corrected: {decision}");
                }
            }
            increment_counter(&mut input_kind_sancov_increase, input_kind, increase);
            increment_counter(&mut input_source_sancov_increase, input_source, increase);
            let _ = fs::write(round_dir.join("payload.txt"), &payload.serialized);
            write_payload_debug(&round_dir.join("payload_debug.txt"), &payload);
            if let Some(preview) = &execution.send_preview {
                write_json_report(&round_dir.join("send_command.json"), preview);
                let _ = fs::write(
                    round_dir.join("send_command.sh"),
                    format!("{}\n", preview.command_line),
                );
            }
            let _ = fs::write(round_dir.join("round.txt"), format!("{round_line}\n"));
            write_json_report(
                &sancov_root.join("coverage/status.json"),
                json!({
                    "phase":"fuzz",
                    "round":round,
                    "coverage_ok":coverage_ok,
                    "sancov_covered_pcs":covered,
                    "sancov_covered_pc_increase":increase,
                    "sancov_all_covered_pcs":all_covered,
                    "sancov_all_pc_increase":all_increase,
                    "packages":packages
                }),
            );
            write_json_report(
                &round_dir.join("summary.json"),
                json!({
                    "round": round,
                    "decision": decision,
                    "trace_disposition": trace_disposition,
                    "oracle": "callback-trace",
                    "active_new_state": verdict.new_state,
                    "input_kind": input_kind,
                    "input_source": input_source,
                    "evidence": verdict.evidence,
                    "calls": trace.call_trace.len(),
                    "msgs": trace.msg_trace.len(),
                    "pool_size": generator.pool().len(),
                    "crash_or_hang": crashed,
                    "stack_health_error": stack_health_error.as_deref(),
                    "coverage_ok": coverage_ok,
                    "sancov_covered_pcs": covered,
                    "sancov_covered_pc_increase": increase,
                    "sancov_all_covered_pcs": all_covered,
                    "sancov_all_pc_increase": all_increase,
                    "packages": packages,
                }),
            );
            println!(
                "       sancov: target pcs {covered} (+{increase}), all pcs {all_covered} (+{all_increase}) -> {}",
                round_dir.display()
            );
        }
        if crashed && round < config.rounds {
            eprintln!("round {round}: restarting stack after recorded crash/unhealthy state");
            if let Err(restart_error) = restart_ready_stack(
                &nav2_ws,
                &trace_dir,
                &config,
                &ros_setup,
                &install_setup,
                &domain_id,
                &bindings,
                Some(&input_sequence),
                &mut stack,
                &mut session,
                &mut reader,
                &mut registry,
                "loop",
            ) {
                eprintln!("round {round}: restart failed after recorded crash: {restart_error}");
                crashes += 1;
                break;
            }
        }
    }

    // 4. 收尾：先在目标 ROS 进程还存活时做可选 sancov final dump；
    // 再整体终止 costmap 栈（进程组）并清理 trace/payload 临时文件。
    if config.sancov_dir.is_some() && stack_alive(&mut stack) {
        flush_stack_sancov_final(stack.id());
    }
    if let Some(sancov_root) = &config.sancov_dir {
        let final_dir = sancov_root.join("final");
        if let Some(coverage) =
            capture_sancov_coverage(sancov_root, &final_dir, &mut seen_sancov_pcs)
        {
            println!(
                "sancov: final target pcs {} (+{}), all pcs {} (+{}) -> {}",
                coverage.target_covered_pcs,
                coverage.target_pc_increase,
                coverage.all_covered_pcs,
                coverage.all_pc_increase,
                final_dir.display()
            );
            write_json_report(
                &sancov_root.join("coverage/status.json"),
                json!({
                    "phase":"final",
                    "round":config.rounds,
                    "coverage_ok":true,
                    "sancov_covered_pcs":coverage.target_covered_pcs,
                    "sancov_covered_pc_increase":coverage.target_pc_increase,
                    "sancov_all_covered_pcs":coverage.all_covered_pcs,
                    "sancov_all_pc_increase":coverage.all_pc_increase,
                    "packages":coverage.packages
                }),
            );
        }
    }
    shutdown_stack(&ros_setup, &install_setup, &domain_id, &mut stack);
    let _ = session.destroy();
    let _ = fs::remove_file(&payload_file);

    println!("\n=== summary ===");
    println!(
        "rounds={} crashes={} active_new_states={} invalid_traces={} empty_rounds={} pool_size={}",
        config.rounds,
        crashes,
        active_new_states,
        invalid,
        empty_rounds,
        generator.pool().len()
    );
    println!(
        "coverage-attribution: kind_rounds={} source_rounds={} kind_branch_increase={} source_branch_increase={} kind_sancov_increase={} source_sancov_increase={} post_coverage_stack_health_failures={}",
        format_counts(input_kind_rounds.clone()),
        format_counts(input_source_rounds.clone()),
        format_counts(input_kind_branch_increase.clone()),
        format_counts(input_source_branch_increase.clone()),
        format_counts(input_kind_sancov_increase.clone()),
        format_counts(input_source_sancov_increase.clone()),
        post_coverage_stack_health_failures
    );
    println!(
        "oracle=callback-trace callback_graph_edges={} distinct_callbacks={}",
        oracle.edge_count(),
        oracle.distinct_callbacks()
    );

    // 覆盖收尾：进程组已终止（exit 时 gcov 已做最终 dump），抓总覆盖并出 HTML。
    if let Some(lcov_root) = &config.lcov_dir {
        match finalize_coverage(&nav2_build_base, lcov_root, config.gcov_tool.as_deref()) {
            Some((covered, total)) => {
                let packages = package_coverage_reports(
                    &lcov_root.join("coverage_total.info"),
                    &lcov_root.join("packages"),
                );
                println!(
                    "coverage: final branches {covered}/{total} -> {}",
                    lcov_root.join("coverage_total.info").display()
                );
                write_json_report(
                    &lcov_root.join("summary.json"),
                    json!({
                        "rounds": config.rounds,
                        "seed": config.seed,
                        "oracle": "callback-trace",
                        "fresh_generation_period": config.fresh_generation_period,
                        "fresh_selection": config.fresh_selection.as_str(),
                        "scan_stamp_mode": config
                            .scan_stamp_mode_override
                            .as_deref()
                            .unwrap_or("seed"),
                        "action_cleanup": config.action_cleanup,
                        "round_settle_seconds": config.round_settle_timeout.as_secs_f64(),
                        "crashes": crashes,
                        "active_new_states": active_new_states,
                        "input_kind_rounds": input_kind_rounds.clone(),
                        "input_source_rounds": input_source_rounds.clone(),
                        "input_kind_new_states": input_kind_new_states.clone(),
                        "input_source_new_states": input_source_new_states.clone(),
                        "input_kind_branch_increase": input_kind_branch_increase.clone(),
                        "input_source_branch_increase": input_source_branch_increase.clone(),
                        "input_kind_sancov_increase": input_kind_sancov_increase.clone(),
                        "input_source_sancov_increase": input_source_sancov_increase.clone(),
                        "post_coverage_stack_health_failures": post_coverage_stack_health_failures,
                        "invalid_traces": invalid,
                        "empty_rounds": empty_rounds,
                        "pool_size": generator.pool().len(),
                        "callback_graph_edges": oracle.edge_count(),
                        "distinct_callbacks": oracle.distinct_callbacks(),
                        "coverage_ok": true,
                        "branch_covered_total": covered,
                        "branch_total": total,
                        "packages": packages,
                    }),
                );
                println!(
                    "coverage: html report -> {}",
                    lcov_root.join("lcov_html/index.html").display()
                );
            }
            None => eprintln!("coverage: final capture failed"),
        }
    }
    if let Some(sancov_root) = &config.sancov_dir {
        let packages = package_sancov_reports(&seen_sancov_pcs, &sancov_root.join("packages"));
        let target_covered = sancov_target_pc_count(&seen_sancov_pcs);
        let all_covered = seen_sancov_pcs.len() as u64;
        let data_files = count_files_with_extension(sancov_root, "sancov");
        write_json_report(
            &sancov_root.join("summary.json"),
            json!({
                "rounds": config.rounds,
                "seed": config.seed,
                "oracle": "callback-trace",
                "fresh_generation_period": config.fresh_generation_period,
                "fresh_selection": config.fresh_selection.as_str(),
                "scan_stamp_mode": config
                    .scan_stamp_mode_override
                    .as_deref()
                    .unwrap_or("seed"),
                "action_cleanup": config.action_cleanup,
                "round_settle_seconds": config.round_settle_timeout.as_secs_f64(),
                "crashes": crashes,
                "active_new_states": active_new_states,
                "input_kind_rounds": input_kind_rounds.clone(),
                "input_source_rounds": input_source_rounds.clone(),
                "input_kind_new_states": input_kind_new_states.clone(),
                "input_source_new_states": input_source_new_states.clone(),
                "input_kind_branch_increase": input_kind_branch_increase.clone(),
                "input_source_branch_increase": input_source_branch_increase.clone(),
                "input_kind_sancov_increase": input_kind_sancov_increase.clone(),
                "input_source_sancov_increase": input_source_sancov_increase.clone(),
                "post_coverage_stack_health_failures": post_coverage_stack_health_failures,
                "invalid_traces": invalid,
                "empty_rounds": empty_rounds,
                "pool_size": generator.pool().len(),
                "callback_graph_edges": oracle.edge_count(),
                "distinct_callbacks": oracle.distinct_callbacks(),
                "coverage_ok": !seen_sancov_pcs.is_empty(),
                "sancov_covered_pcs": target_covered,
                "sancov_all_covered_pcs": all_covered,
                "packages": packages,
                "sancov_data_files": data_files,
            }),
        );
        println!(
            "sancov: {} target pcs, {} all pcs, {} data files -> {}",
            target_covered,
            all_covered,
            data_files,
            sancov_root.join("summary.json").display()
        );
    }
}

#[cfg(test)]
mod tests {
    use super::coverage::{branch_totals_from_summary, cumulative_branches};

    #[test]
    fn branch_totals_parses_typical_summary() {
        let summary = "\
Summary coverage rate:
  lines......: 51.0% (1234 of 2420 lines)
  functions..: 60.0% (12 of 20 functions)
  branches......: 51.0% (1234 of 2420 branches)
";
        assert_eq!(branch_totals_from_summary(summary), Some((1234, 2420)));
    }

    #[test]
    fn branch_totals_rejects_missing_branch_line() {
        assert_eq!(
            branch_totals_from_summary("lines......: 1.0% (1 of 1 lines)"),
            None
        );
    }

    #[test]
    fn cumulative_branches_clamps_dips() {
        assert_eq!(cumulative_branches(100, 90), (100, false));
        assert_eq!(cumulative_branches(80, 90), (90, true));
        assert_eq!(cumulative_branches(90, 90), (90, false));
    }
}
