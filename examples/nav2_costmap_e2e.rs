//! R2D2 闭环 × 真实 Jazzy nav2（运行时插桩落在 rclcpp/rcl 层，见 docs/plan）。
//!
//! 流程（对应论文 Figure 3 的生成与反馈两侧）：
//!
//! 1. dry run：FileExtractor 解析 Jazzy 官方安装树中的 topic/service 接口，
//!    同时按 nav2_costmap_2d 源码声明 + 当前 costmap YAML 构建 parameter
//!    输入面：`/scan`、`/points`、`/map`、`/map_updates`、nav2 costmap
//!    services 与 `/costmap` 动态参数。
//! 2. 每轮：PayloadGenerator 在这些 topic/service/parameter 接口里选一个
//!    payload；LaserScan 仍走 `r2d2_scan_bridge`，其余 topic/service/
//!    parameter 走 `ros2` CLI。
//! 3. 插桩后的 nav2_costmap_2d（ObstacleLayer / StaticLayer / costmap services）
//!    把注册与运行时事件写进 /dev/shm/r2d2_nav2；本进程实时 drain、profile。
//!    每轮结束由
//!    `round_marker` 向 runtime ring 写 RoundBoundary 标记，reader 按标记把
//!    异步落盘的事件归到对应轮次（论文未披露轮次边界格式，此为
//!    reproduction choice）。
//! 4. 先执行独立 benchmark phase，建立 callback graph 与均值基线；随后 fuzz
//!    phase 判定新执行序列 / 延迟偏差 / 吞吐下降。costmap 进程组死亡记为
//!    crash；crash 或 new state 的 payload 入池。
//!
//! 需要先构建 nav2_ws（含 r2d2_tracer、r2d2_scan_bridge）：
//!   scripts/build_nav2_ws.sh
//!
//! 边界（防止把本 demo 误写成完整论文复现）：
//! - 目标仍仅为单个 nav2_costmap_2d 程序，但输入面扩到当前启用的
//!   topic/service/parameter 矩阵：LaserScan、PointCloud2、
//!   OccupancyGrid、OccupancyGridUpdate、GetCost、GetCostmap、
//!   ClearCostmap* 与 `/costmap` 动态参数
//! - live 插桩已切到 `rclcpp/rcl` 运行时层；剩余偏差见
//!   docs/plan/nav2_jazzy_instrumentation_plan.md 第 4 节
//! - state oracle 按论文两阶段结构先建 benchmark，再比较图边 + 延迟/吞吐；
//!   但显著偏离阈值公式未公开，仍用可配乘数近似
//! - 覆盖率为 gcc+gcov 近似，本 example 的覆盖与缺陷数字不能直接对齐论文
//!   的 SanitizerCoverage 口径与实验表格
//!
//! 快速冒烟：cargo run --example nav2_costmap_e2e -- --benchmark-seconds 5 --rounds 10 --seed 42

use my_r2d2::callback_profile::{CallbackRegistry, profile_trace};
use my_r2d2::interface_extractor::{
    Extractor, Field, FileExtractor, Interface, Kind, Primitive, TypeNode,
};
use my_r2d2::mutation::generate_value;
use my_r2d2::payload::{Payload, Value, ValueTree};
use my_r2d2::payload_generator::{GeneratorConfig, PayloadGenerator, Sender};
use my_r2d2::runtime::ros2_sender::{
    LaserScanSchedule, Ros2LaserScanSender, Ros2ParameterSender, Ros2ServiceSender,
    Ros2TopicOptions, Ros2TopicSender, ros2_cli_command,
};
use my_r2d2::runtime::state_oracle::{
    BenchmarkBuilder, BenchmarkModel, BenchmarkStateOracle, DeviationThresholds, OracleMode,
    TraceDisposition,
};
use my_r2d2::seed_corpus::{Schedule, load_scan_seeds, load_schedules};
use my_r2d2::trace_buffer::{RuntimeDrain, RuntimeEvent, RuntimeEventType, TraceReader};
use my_r2d2::utils::yaml_reader::YamlEnv;
use rand::SeedableRng;
use rand::rngs::StdRng;
use rand::seq::SliceRandom;
use serde_json::json;
use serde_yaml::Value as YamlValue;
use std::collections::BTreeMap;
use std::collections::BTreeSet;
use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::thread;
use std::time::{Duration, Instant};
/// 每轮最多起几次 scan bridge。bridge 每轮都是全新 DDS participant，发现
/// 偶尔会吃掉整轮或大部分消息；送达不足期望一半时重跑同一 payload/schedule，
/// 不把「没送到」当成「没触发」。
const BRIDGE_ATTEMPTS: u32 = 3;
/// 写 round marker 前等待异步回调落盘的时长。论文要求每个 payload 执行后
/// 分析「当前 callback trace」，但未披露轮次边界格式；marker + settle 是
/// 本仓库的轮次分段协议（reproduction choice）。
const MARKER_SETTLE: Duration = Duration::from_millis(300);
/// startup barrier 的总超时；系统在此之前没有进入可交互稳定态，就直接失败，
/// 不允许 benchmark 吞掉启动竞态。
const STARTUP_TIMEOUT: Duration = Duration::from_secs(60);
const STARTUP_POLL: Duration = Duration::from_millis(500);
const REGISTRATION_SETTLE_POLLS: usize = 3;
const COSTMAP_NODE: &str = "/costmap";
const REQUIRED_COSTMAP_SERVICES: [(&str, &str); 6] = [
    ("/get_cost_costmap", "nav2_msgs/srv/GetCost"),
    ("/get_costmap", "nav2_msgs/srv/GetCostmap"),
    (
        "/clear_except_costmap",
        "nav2_msgs/srv/ClearCostmapExceptRegion",
    ),
    (
        "/clear_around_costmap",
        "nav2_msgs/srv/ClearCostmapAroundRobot",
    ),
    (
        "/clear_around_pose_costmap",
        "nav2_msgs/srv/ClearCostmapAroundPose",
    ),
    (
        "/clear_entirely_costmap",
        "nav2_msgs/srv/ClearEntireCostmap",
    ),
];

/// 定位 round_marker 二进制：优先 standalone tracer 构建产物，其次 nav2_ws
/// 安装树。缺失时回退到纯 drain 游标分段（marker 之前的旧行为）。
fn resolve_round_marker(repo_root: &Path, nav2_ws: &Path) -> Option<PathBuf> {
    let candidates = [
        repo_root.join("tracer/build/round_marker"),
        nav2_ws.join("install/r2d2_tracer/lib/r2d2_tracer/round_marker"),
    ];
    candidates.into_iter().find(|path| path.exists())
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
    },
    Parameter {
        node_name: String,
        parameter_name: String,
        restore_value: ValueTree,
    },
}

#[derive(Clone)]
struct InterfaceBinding {
    interface: Interface,
    endpoint: EndpointBinding,
}

impl InterfaceBinding {
    fn endpoint_name(&self) -> &str {
        match &self.endpoint {
            EndpointBinding::LaserScan { topic_name } => topic_name,
            EndpointBinding::Topic { topic_name, .. } => topic_name,
            EndpointBinding::Service { service_name, .. } => service_name,
            EndpointBinding::Parameter { parameter_name, .. } => parameter_name,
        }
    }
}

struct Config {
    rounds: u64,
    seed: u64,
    benchmark_seconds: u64,
    /// benchmark 阶段允许 parameter 接口累计占用的总墙钟时间（秒）。
    benchmark_parameter_budget_seconds: u64,
    benchmark_model: Option<PathBuf>,
    oracle_mode: OracleMode,
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
    /// nav2-_fuzz 种子语料目录（scans/ 与 schedules/）；None 表示纯生成。
    seed_dir: Option<PathBuf>,
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
            benchmark_parameter_budget_seconds: 300,
            benchmark_model: None,
            oracle_mode: OracleMode::JazzyReproduction,
            latency_factor: 2.0,
            throughput_floor: 0.5,
            round_duration_sec: 2.0,
            bridge_rate_hz: 20,
            tsan_log_dir: None,
            lcov_dir: None,
            seed_dir: None,
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
                "--benchmark-parameter-budget-seconds" => {
                    config.benchmark_parameter_budget_seconds =
                        value("--benchmark-parameter-budget-seconds")?
                            .parse::<u64>()
                            .map_err(|e| e.to_string())?
                }
                "--benchmark-model" => {
                    config.benchmark_model = Some(PathBuf::from(value("--benchmark-model")?))
                }
                "--oracle-mode" => {
                    config.oracle_mode = OracleMode::parse_cli(&value("--oracle-mode")?)?
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
                "--seed-dir" => config.seed_dir = Some(PathBuf::from(value("--seed-dir")?)),
                "--gcov-tool" => config.gcov_tool = Some(value("--gcov-tool")?),
                other => return Err(format!("unknown flag {other}")),
            }
        }
        Ok(config)
    }
}

fn ros_share_root(ros_setup: &Path) -> Result<PathBuf, String> {
    let ros_root = ros_setup
        .parent()
        .ok_or_else(|| format!("cannot infer ROS root from {}", ros_setup.display()))?;
    Ok(ros_root.join("share"))
}

fn take_interface(
    by_name: &mut BTreeMap<String, Interface>,
    name: &str,
) -> Result<Interface, String> {
    by_name
        .remove(name)
        .ok_or_else(|| format!("{name} interface not extracted"))
}

#[derive(Clone)]
struct ParameterSpec {
    parameter_name: &'static str,
    ty: TypeNode,
    default_value: ValueTree,
}

impl ParameterSpec {
    fn new(parameter_name: &'static str, ty: TypeNode, default_value: ValueTree) -> Self {
        Self {
            parameter_name,
            ty,
            default_value,
        }
    }
}

fn bool_leaf(value: bool) -> ValueTree {
    ValueTree::Leaf(Value::Bool(value))
}

fn i64_leaf(value: i64) -> ValueTree {
    ValueTree::Leaf(Value::I64(value))
}

fn f64_leaf(value: f64) -> ValueTree {
    ValueTree::Leaf(Value::F64(value))
}

fn string_leaf(value: impl Into<String>) -> ValueTree {
    ValueTree::Leaf(Value::String(value.into()))
}

fn costmap_parameter_specs() -> Vec<ParameterSpec> {
    vec![
        ParameterSpec::new("robot_radius", Primitive::F64.into(), f64_leaf(0.1)),
        ParameterSpec::new("footprint_padding", Primitive::F64.into(), f64_leaf(0.01)),
        ParameterSpec::new("transform_tolerance", Primitive::F64.into(), f64_leaf(0.3)),
        ParameterSpec::new("publish_frequency", Primitive::F64.into(), f64_leaf(1.0)),
        ParameterSpec::new("resolution", Primitive::F64.into(), f64_leaf(0.1)),
        ParameterSpec::new("origin_x", Primitive::F64.into(), f64_leaf(0.0)),
        ParameterSpec::new("origin_y", Primitive::F64.into(), f64_leaf(0.0)),
        ParameterSpec::new("width", Primitive::I64.into(), i64_leaf(5)),
        ParameterSpec::new("height", Primitive::I64.into(), i64_leaf(5)),
        ParameterSpec::new("footprint", Primitive::String.into(), string_leaf("[]")),
        ParameterSpec::new(
            "robot_base_frame",
            Primitive::String.into(),
            string_leaf("base_link"),
        ),
        ParameterSpec::new(
            "obstacle_layer.enabled",
            Primitive::Bool.into(),
            bool_leaf(true),
        ),
        ParameterSpec::new(
            "obstacle_layer.footprint_clearing_enabled",
            Primitive::Bool.into(),
            bool_leaf(true),
        ),
        ParameterSpec::new(
            "obstacle_layer.min_obstacle_height",
            Primitive::F64.into(),
            f64_leaf(0.0),
        ),
        ParameterSpec::new(
            "obstacle_layer.max_obstacle_height",
            Primitive::F64.into(),
            f64_leaf(2.0),
        ),
        ParameterSpec::new(
            "obstacle_layer.combination_method",
            Primitive::I64.into(),
            i64_leaf(1),
        ),
        ParameterSpec::new(
            "static_layer.enabled",
            Primitive::Bool.into(),
            bool_leaf(true),
        ),
        ParameterSpec::new(
            "static_layer.footprint_clearing_enabled",
            Primitive::Bool.into(),
            bool_leaf(false),
        ),
        ParameterSpec::new(
            "static_layer.transform_tolerance",
            Primitive::F64.into(),
            f64_leaf(0.0),
        ),
        ParameterSpec::new(
            "inflation_layer.enabled",
            Primitive::Bool.into(),
            bool_leaf(true),
        ),
        ParameterSpec::new(
            "inflation_layer.inflation_radius",
            Primitive::F64.into(),
            f64_leaf(0.55),
        ),
        ParameterSpec::new(
            "inflation_layer.cost_scaling_factor",
            Primitive::F64.into(),
            f64_leaf(10.0),
        ),
        ParameterSpec::new(
            "inflation_layer.inflate_unknown",
            Primitive::Bool.into(),
            bool_leaf(false),
        ),
        ParameterSpec::new(
            "inflation_layer.inflate_around_unknown",
            Primitive::Bool.into(),
            bool_leaf(false),
        ),
    ]
}

fn load_costmap_parameter_root(costmap_params: &Path) -> Result<YamlValue, String> {
    let source = fs::read_to_string(costmap_params)
        .map_err(|error| format!("read {}: {error}", costmap_params.display()))?;
    let document: YamlValue = serde_yaml::from_str(&source)
        .map_err(|error| format!("parse {}: {error}", costmap_params.display()))?;
    let Some(root) = yaml_lookup_path(&document, &["costmap", "ros__parameters"]) else {
        return Err(format!(
            "{} missing costmap.ros__parameters",
            costmap_params.display()
        ));
    };
    Ok(root.clone())
}

fn yaml_lookup_path<'a>(value: &'a YamlValue, segments: &[&str]) -> Option<&'a YamlValue> {
    let mut current = value;
    for segment in segments {
        let YamlValue::Mapping(map) = current else {
            return None;
        };
        current = map.get(YamlValue::String((*segment).to_string()))?;
    }
    Some(current)
}

fn yaml_to_value_tree(value: &YamlValue, ty: &TypeNode) -> Result<ValueTree, String> {
    match ty {
        TypeNode::Constrained(inner, _) => yaml_to_value_tree(value, inner),
        TypeNode::Primitive(Primitive::Bool) => value
            .as_bool()
            .map(bool_leaf)
            .ok_or_else(|| format!("expected bool YAML scalar, found {value:?}")),
        TypeNode::Primitive(Primitive::I64) => value
            .as_i64()
            .or_else(|| value.as_u64().and_then(|v| i64::try_from(v).ok()))
            .map(i64_leaf)
            .ok_or_else(|| format!("expected int YAML scalar, found {value:?}")),
        TypeNode::Primitive(Primitive::F64) => value
            .as_f64()
            .map(f64_leaf)
            .ok_or_else(|| format!("expected float YAML scalar, found {value:?}")),
        TypeNode::Primitive(Primitive::String) => value
            .as_str()
            .map(string_leaf)
            .ok_or_else(|| format!("expected string YAML scalar, found {value:?}")),
        TypeNode::Primitive(Primitive::Bytes) => {
            let YamlValue::Sequence(items) = value else {
                return Err(format!(
                    "expected byte array YAML sequence, found {value:?}"
                ));
            };
            let mut out = Vec::with_capacity(items.len());
            for item in items {
                let Some(number) = item.as_i64().or_else(|| item.as_u64().map(|v| v as i64)) else {
                    return Err(format!("expected byte item, found {item:?}"));
                };
                let byte = u8::try_from(number)
                    .map_err(|_| format!("byte value out of range in {item:?}"))?;
                out.push(ValueTree::Leaf(Value::U8(byte)));
            }
            Ok(ValueTree::Array(out))
        }
        TypeNode::Array(element, fixed_len) => {
            let YamlValue::Sequence(items) = value else {
                return Err(format!("expected YAML sequence, found {value:?}"));
            };
            if let Some(expected) = fixed_len
                && items.len() != *expected
            {
                return Err(format!(
                    "expected fixed array of {expected} items, found {}",
                    items.len()
                ));
            }
            let mut out = Vec::with_capacity(items.len());
            for item in items {
                out.push(yaml_to_value_tree(item, element)?);
            }
            Ok(ValueTree::Array(out))
        }
        other => Err(format!(
            "unsupported parameter YAML conversion for {other:?}"
        )),
    }
}

fn extract_costmap_parameter_bindings(
    costmap_params: &Path,
) -> Result<Vec<InterfaceBinding>, String> {
    let yaml_root = load_costmap_parameter_root(costmap_params)?;
    let mut bindings = Vec::new();
    for spec in costmap_parameter_specs() {
        let segments = spec.parameter_name.split('.').collect::<Vec<_>>();
        let restore_value = yaml_lookup_path(&yaml_root, &segments)
            .map(|value| yaml_to_value_tree(value, &spec.ty))
            .transpose()?
            .unwrap_or_else(|| spec.default_value.clone());
        let interface = Interface::new(
            format!("param:/costmap/{}", spec.parameter_name),
            Kind::Parameter,
            vec![Field::new("value", spec.ty.clone())],
        );
        bindings.push(InterfaceBinding {
            interface,
            endpoint: EndpointBinding::Parameter {
                node_name: "/costmap".to_string(),
                parameter_name: spec.parameter_name.to_string(),
                restore_value,
            },
        });
    }
    Ok(bindings)
}

/// Dry run：从 Jazzy 官方安装树提取当前 costmap topic/service 接口，再按
/// nav2_costmap_2d 源码动态参数点 + 当前 costmap YAML 构建 parameter 输入面。
fn extract_costmap_bindings(
    share_root: &Path,
    costmap_params: &Path,
) -> Result<Vec<InterfaceBinding>, String> {
    let extractor = FileExtractor::new(
        vec![
            share_root.join("sensor_msgs/msg/LaserScan.msg"),
            share_root.join("sensor_msgs/msg/PointCloud2.msg"),
            share_root.join("nav_msgs/msg/OccupancyGrid.msg"),
            share_root.join("map_msgs/msg/OccupancyGridUpdate.msg"),
            share_root.join("nav2_msgs/srv/GetCost.srv"),
            share_root.join("nav2_msgs/srv/GetCostmap.srv"),
            share_root.join("nav2_msgs/srv/ClearCostmapExceptRegion.srv"),
            share_root.join("nav2_msgs/srv/ClearCostmapAroundRobot.srv"),
            share_root.join("nav2_msgs/srv/ClearCostmapAroundPose.srv"),
            share_root.join("nav2_msgs/srv/ClearEntireCostmap.srv"),
        ],
        vec![share_root.to_path_buf()],
    );
    let mut by_name = extractor
        .extract()
        .map_err(|e| e.to_string())?
        .into_iter()
        .map(|interface| (interface.name.clone(), interface))
        .collect::<BTreeMap<_, _>>();
    let mut bindings = vec![
        InterfaceBinding {
            interface: take_interface(&mut by_name, "LaserScan")?,
            endpoint: EndpointBinding::LaserScan {
                topic_name: "/scan".to_string(),
            },
        },
        InterfaceBinding {
            interface: take_interface(&mut by_name, "PointCloud2")?,
            endpoint: EndpointBinding::Topic {
                topic_name: "/points".to_string(),
                message_type: "sensor_msgs/msg/PointCloud2".to_string(),
                options: Ros2TopicOptions {
                    qos_profile: Some("sensor_data".to_string()),
                    qos_depth: Some(50),
                    keep_alive_sec: 0.5,
                    ..Ros2TopicOptions::default()
                },
            },
        },
        InterfaceBinding {
            interface: take_interface(&mut by_name, "OccupancyGrid")?,
            endpoint: EndpointBinding::Topic {
                topic_name: "/map".to_string(),
                message_type: "nav_msgs/msg/OccupancyGrid".to_string(),
                options: Ros2TopicOptions {
                    qos_depth: Some(1),
                    qos_reliability: Some("reliable".to_string()),
                    qos_durability: Some("transient_local".to_string()),
                    keep_alive_sec: 0.5,
                    ..Ros2TopicOptions::default()
                },
            },
        },
        InterfaceBinding {
            interface: take_interface(&mut by_name, "OccupancyGridUpdate")?,
            endpoint: EndpointBinding::Topic {
                topic_name: "/map_updates".to_string(),
                message_type: "map_msgs/msg/OccupancyGridUpdate".to_string(),
                options: Ros2TopicOptions::default(),
            },
        },
        InterfaceBinding {
            interface: take_interface(&mut by_name, "GetCost")?,
            endpoint: EndpointBinding::Service {
                service_name: "/get_cost_costmap".to_string(),
                service_type: "nav2_msgs/srv/GetCost".to_string(),
            },
        },
        InterfaceBinding {
            interface: take_interface(&mut by_name, "GetCostmap")?,
            endpoint: EndpointBinding::Service {
                service_name: "/get_costmap".to_string(),
                service_type: "nav2_msgs/srv/GetCostmap".to_string(),
            },
        },
        InterfaceBinding {
            interface: take_interface(&mut by_name, "ClearCostmapExceptRegion")?,
            endpoint: EndpointBinding::Service {
                service_name: "/clear_except_costmap".to_string(),
                service_type: "nav2_msgs/srv/ClearCostmapExceptRegion".to_string(),
            },
        },
        InterfaceBinding {
            interface: take_interface(&mut by_name, "ClearCostmapAroundRobot")?,
            endpoint: EndpointBinding::Service {
                service_name: "/clear_around_costmap".to_string(),
                service_type: "nav2_msgs/srv/ClearCostmapAroundRobot".to_string(),
            },
        },
        InterfaceBinding {
            interface: take_interface(&mut by_name, "ClearCostmapAroundPose")?,
            endpoint: EndpointBinding::Service {
                service_name: "/clear_around_pose_costmap".to_string(),
                service_type: "nav2_msgs/srv/ClearCostmapAroundPose".to_string(),
            },
        },
        InterfaceBinding {
            interface: take_interface(&mut by_name, "ClearEntireCostmap")?,
            endpoint: EndpointBinding::Service {
                service_name: "/clear_entirely_costmap".to_string(),
                service_type: "nav2_msgs/srv/ClearEntireCostmap".to_string(),
            },
        },
    ];
    bindings.extend(extract_costmap_parameter_bindings(costmap_params)?);
    Ok(bindings)
}

fn find_binding<'a>(
    bindings: &'a [InterfaceBinding],
    interface_id: &str,
) -> Option<&'a InterfaceBinding> {
    bindings
        .iter()
        .find(|binding| binding.interface.name == interface_id)
}

fn stack_alive(stack: &mut std::process::Child) -> bool {
    // try_wait 会回收僵尸进程；kill -0 对僵尸返回成功，不能用于存活判定。
    matches!(stack.try_wait(), Ok(None))
}

fn cleanup_stale_stack_groups(stack_script: &Path, costmap_params: &Path) {
    let Ok(output) = Command::new("ps")
        .args(["-eo", "pid=,pgid=,args="])
        .output()
    else {
        eprintln!("startup: failed to inspect stale stack processes");
        return;
    };
    if !output.status.success() {
        eprintln!("startup: ps failed while inspecting stale stack processes");
        return;
    }

    let script = stack_script.display().to_string();
    let params = costmap_params.display().to_string();
    let text = String::from_utf8_lossy(&output.stdout);
    let mut stale_pgids = BTreeSet::new();
    for line in text.lines() {
        let mut parts = line.trim().splitn(3, char::is_whitespace);
        let _pid = parts.next();
        let pgid = parts.next().and_then(|value| value.parse::<i32>().ok());
        let cmd = parts.next().unwrap_or_default();
        let matches_stack = cmd.contains(&script);
        let matches_costmap = cmd.contains("nav2_costmap_2d")
            && cmd.contains("--params-file")
            && cmd.contains(&params);
        if (matches_stack || matches_costmap)
            && let Some(pgid) = pgid
            && pgid > 0
        {
            stale_pgids.insert(pgid);
        }
    }

    if stale_pgids.is_empty() {
        return;
    }

    let groups = stale_pgids
        .iter()
        .map(|pgid| pgid.to_string())
        .collect::<Vec<_>>()
        .join(", ");
    eprintln!("startup: cleaning stale stack process groups: {groups}");
    for signal in ["-TERM", "-KILL"] {
        for pgid in &stale_pgids {
            let _ = Command::new("kill")
                .args([signal, &format!("-{pgid}")])
                .status();
        }
        if signal == "-TERM" {
            thread::sleep(Duration::from_secs(2));
        }
    }
}

fn terminate_stack_process_group(stack: &mut std::process::Child) {
    let stack_pid = stack.id();
    let _ = Command::new("kill")
        .args(["--", "-TERM", &format!("-{stack_pid}")])
        .status();
    thread::sleep(Duration::from_secs(2));
    let _ = Command::new("kill")
        .args(["--", "-KILL", &format!("-{stack_pid}")])
        .status();
}

fn spawn_instrumented_stack(
    nav2_ws: &Path,
    shm_path: &Path,
    config: &Config,
    costmap_params: &Path,
) -> Result<std::process::Child, String> {
    let stack_script = nav2_ws.join("launch_stack.sh");
    cleanup_stale_stack_groups(&stack_script, costmap_params);
    let _ = fs::remove_file(shm_path);
    let _ = fs::remove_file(shm_path.with_extension("pid"));

    let mut stack_command = Command::new("setsid");
    stack_command
        .arg("bash")
        .arg(&stack_script)
        .env("R2D2_SHM_PATH", shm_path)
        .env_remove("LD_PRELOAD")
        .env_remove("ASAN_OPTIONS")
        .env_remove("COLCON_CURRENT_PREFIX");
    if let Some(log_dir) = &config.tsan_log_dir {
        let _ = fs::create_dir_all(log_dir);
        stack_command.env(
            "TSAN_OPTIONS",
            format!(
                "log_path={}/tsan:halt_on_error=0:history_size=7:second_deadlock_stack=1:report_signal_unsafe=0",
                log_dir.display()
            ),
        );
        println!("tsan: reports will be written to {}", log_dir.display());
    } else {
        stack_command.env_remove("TSAN_OPTIONS");
    }
    let stack = stack_command
        .spawn()
        .map_err(|error| format!("spawn costmap stack: {error}"))?;
    println!("stack leader pid = {}", stack.id());
    Ok(stack)
}

fn open_trace_reader_after_stack_start(
    shm_path: &Path,
    stack: &mut std::process::Child,
) -> Result<TraceReader, String> {
    loop {
        if let Ok(reader) = TraceReader::open(shm_path) {
            return Ok(reader);
        }
        if !stack_alive(stack) {
            return Err("costmap stack died during startup".to_string());
        }
        thread::sleep(Duration::from_millis(500));
    }
}

fn start_ready_stack(
    nav2_ws: &Path,
    shm_path: &Path,
    config: &Config,
    costmap_params: &Path,
    ros_setup: &Path,
    install_setup: &Path,
    domain_id: &str,
) -> Result<
    (
        std::process::Child,
        TraceReader,
        CallbackRegistry,
        usize,
    ),
    String,
> {
    let mut stack = spawn_instrumented_stack(nav2_ws, shm_path, config, costmap_params)?;
    let mut reader = match open_trace_reader_after_stack_start(shm_path, &mut stack) {
        Ok(reader) => reader,
        Err(error) => {
            terminate_stack_process_group(&mut stack);
            return Err(error);
        }
    };
    let mut registry = CallbackRegistry::new();
    let registrations = match wait_costmap_ready(
        ros_setup,
        install_setup,
        domain_id,
        &mut reader,
        &mut registry,
        &mut stack,
    ) {
        Ok(registrations) => registrations,
        Err(error) => {
            terminate_stack_process_group(&mut stack);
            return Err(error);
        }
    };
    Ok((stack, reader, registry, registrations))
}

fn bootstrap_map_if_available(
    bindings: &[InterfaceBinding],
    config: &Config,
    ros_setup: &Path,
    install_setup: &Path,
    domain_id: &str,
) {
    if let Some(map_binding) = find_binding(bindings, "OccupancyGrid")
        && let EndpointBinding::Topic {
            topic_name,
            message_type,
            options,
        } = &map_binding.endpoint
    {
        let mut rng = StdRng::seed_from_u64(config.seed ^ 0xC0DE_CAFE);
        let bootstrap = Payload::new(
            map_binding.interface.name.clone(),
            map_binding.interface.kind,
            generate_value(
                &TypeNode::Nested(map_binding.interface.fields.clone()),
                &mut rng,
                &GeneratorConfig::default(),
            ),
            config.seed,
        );
        let sender = Ros2TopicSender::new(
            ros_setup,
            install_setup,
            domain_id,
            topic_name.clone(),
            message_type.clone(),
            map_binding.interface.clone(),
            options.clone(),
        );
        match sender.send(&bootstrap) {
            Ok(()) => println!("startup: bootstrapped {}", topic_name),
            Err(error) => eprintln!("startup: bootstrap {} failed: {}", topic_name, error),
        }
        thread::sleep(Duration::from_millis(200));
    }
}

fn ingest_registration_updates(
    round_label: &str,
    reader: &mut TraceReader,
    registry: &mut CallbackRegistry,
) -> Result<usize, String> {
    let drain = reader
        .drain_registration()
        .map_err(|error| format!("{round_label}: registration drain failed: {error}"))?;
    let count = drain.events.len();
    registry.ingest(&drain);
    Ok(count)
}

fn drain_runtime_with_live_registry(
    round_label: &str,
    reader: &mut TraceReader,
    registry: &mut CallbackRegistry,
) -> Result<RuntimeDrain, String> {
    let _ = ingest_registration_updates(round_label, reader, registry)?;
    let runtime = reader
        .drain_runtime()
        .map_err(|error| format!("{round_label}: runtime drain failed: {error}"))?;
    let _ = ingest_registration_updates(round_label, reader, registry)?;
    Ok(runtime)
}

struct Ros2CliOutput {
    stdout: String,
    stderr: String,
    success: bool,
}

fn ros2_cli_output(
    ros_setup: &Path,
    install_setup: &Path,
    domain_id: &str,
    args: &[&str],
) -> Result<Ros2CliOutput, String> {
    let mut cli_args = vec!["ros2".to_string()];
    cli_args.extend(
        args.iter()
            .map(|arg| (*arg).to_string())
            .collect::<Vec<_>>(),
    );
    let output = ros2_cli_command(ros_setup, install_setup, domain_id, &cli_args)
        .output()
        .map_err(|error| format!("failed to run ros2 {}: {error}", cli_args[1..].join(" ")))?;
    Ok(Ros2CliOutput {
        stdout: String::from_utf8_lossy(&output.stdout).into_owned(),
        stderr: String::from_utf8_lossy(&output.stderr).into_owned(),
        success: output.status.success(),
    })
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum LifecycleState {
    Unknown,
    Unconfigured,
    Inactive,
    Active,
    Finalized,
}

fn parse_lifecycle_state(stdout: &str, stderr: &str) -> LifecycleState {
    let text = format!("{stdout}\n{stderr}").to_ascii_lowercase();
    if text.contains("unconfigured") {
        LifecycleState::Unconfigured
    } else if text.contains("inactive") {
        LifecycleState::Inactive
    } else if text.contains("active") {
        LifecycleState::Active
    } else if text.contains("finalized") {
        LifecycleState::Finalized
    } else {
        LifecycleState::Unknown
    }
}

fn wait_for_node_visible(
    ros_setup: &Path,
    install_setup: &Path,
    domain_id: &str,
    stack: &mut std::process::Child,
    deadline: Instant,
) -> Result<(), String> {
    while Instant::now() < deadline {
        if !stack_alive(stack) {
            return Err("costmap stack died before node became visible".to_string());
        }
        let output = ros2_cli_output(ros_setup, install_setup, domain_id, &["node", "list"])?;
        if output.success
            && output
                .stdout
                .lines()
                .map(str::trim)
                .any(|line| line == COSTMAP_NODE)
        {
            return Ok(());
        }
        thread::sleep(STARTUP_POLL);
    }
    Err(format!(
        "timeout waiting for {COSTMAP_NODE} to appear in ros2 node list"
    ))
}

fn lifecycle_state(
    ros_setup: &Path,
    install_setup: &Path,
    domain_id: &str,
) -> Result<LifecycleState, String> {
    let output = ros2_cli_output(
        ros_setup,
        install_setup,
        domain_id,
        &["lifecycle", "get", COSTMAP_NODE],
    )?;
    if !output.success {
        return Ok(LifecycleState::Unknown);
    }
    Ok(parse_lifecycle_state(&output.stdout, &output.stderr))
}

fn set_lifecycle_transition(
    ros_setup: &Path,
    install_setup: &Path,
    domain_id: &str,
    transition: &str,
) -> Result<(), String> {
    let output = ros2_cli_output(
        ros_setup,
        install_setup,
        domain_id,
        &["lifecycle", "set", COSTMAP_NODE, transition],
    )?;
    if output.success {
        Ok(())
    } else {
        Err(format!(
            "ros2 lifecycle set {COSTMAP_NODE} {transition} failed: {}",
            output.stderr.trim()
        ))
    }
}

fn wait_for_lifecycle_state(
    ros_setup: &Path,
    install_setup: &Path,
    domain_id: &str,
    stack: &mut std::process::Child,
    expected: LifecycleState,
    deadline: Instant,
) -> Result<(), String> {
    while Instant::now() < deadline {
        if !stack_alive(stack) {
            return Err(format!(
                "costmap stack died while waiting for lifecycle state {expected:?}"
            ));
        }
        if lifecycle_state(ros_setup, install_setup, domain_id)? == expected {
            return Ok(());
        }
        thread::sleep(STARTUP_POLL);
    }
    Err(format!(
        "timeout waiting for lifecycle state {expected:?} on {COSTMAP_NODE}"
    ))
}

fn wait_for_lifecycle_queryable(
    ros_setup: &Path,
    install_setup: &Path,
    domain_id: &str,
    stack: &mut std::process::Child,
    deadline: Instant,
) -> Result<LifecycleState, String> {
    while Instant::now() < deadline {
        if !stack_alive(stack) {
            return Err("costmap stack died before lifecycle service became queryable".to_string());
        }
        let state = lifecycle_state(ros_setup, install_setup, domain_id)?;
        if state != LifecycleState::Unknown {
            return Ok(state);
        }
        thread::sleep(STARTUP_POLL);
    }
    Err(format!(
        "timeout waiting for lifecycle service on {COSTMAP_NODE} to become queryable"
    ))
}

fn wait_for_registration_settled(
    reader: &mut TraceReader,
    registry: &mut CallbackRegistry,
    deadline: Instant,
) -> Result<usize, String> {
    let mut total = 0usize;
    let mut stable_polls = 0usize;
    while Instant::now() < deadline {
        let new = ingest_registration_updates("startup registration", reader, registry)?;
        total += new;
        if total > 0 && !registry.callback_infos().is_empty() && new == 0 {
            stable_polls += 1;
            if stable_polls >= REGISTRATION_SETTLE_POLLS {
                return Ok(total);
            }
        } else {
            stable_polls = 0;
        }
        thread::sleep(STARTUP_POLL);
    }
    Err(format!(
        "timeout waiting for registration to settle: total_records={total}, complete_callbacks={}",
        registry.callback_infos().len()
    ))
}

fn wait_for_service_type(
    ros_setup: &Path,
    install_setup: &Path,
    domain_id: &str,
    stack: &mut std::process::Child,
    service_name: &str,
    expected_type: &str,
    deadline: Instant,
) -> Result<(), String> {
    while Instant::now() < deadline {
        if !stack_alive(stack) {
            return Err(format!(
                "costmap stack died while waiting for service {service_name}"
            ));
        }
        let output = ros2_cli_output(
            ros_setup,
            install_setup,
            domain_id,
            &["service", "type", service_name],
        )?;
        if output.success && output.stdout.trim() == expected_type {
            return Ok(());
        }
        thread::sleep(STARTUP_POLL);
    }
    Err(format!(
        "timeout waiting for service {service_name} to expose type {expected_type}"
    ))
}

fn wait_for_parameter_service(
    ros_setup: &Path,
    install_setup: &Path,
    domain_id: &str,
    stack: &mut std::process::Child,
    deadline: Instant,
) -> Result<(), String> {
    while Instant::now() < deadline {
        if !stack_alive(stack) {
            return Err("costmap stack died while waiting for parameter service".to_string());
        }
        let output = ros2_cli_output(
            ros_setup,
            install_setup,
            domain_id,
            &["param", "list", COSTMAP_NODE],
        )?;
        if output.success && output.stdout.contains("robot_radius") {
            return Ok(());
        }
        thread::sleep(STARTUP_POLL);
    }
    Err(format!(
        "timeout waiting for ros2 param list {COSTMAP_NODE} to succeed"
    ))
}

fn wait_costmap_ready(
    ros_setup: &Path,
    install_setup: &Path,
    domain_id: &str,
    reader: &mut TraceReader,
    registry: &mut CallbackRegistry,
    stack: &mut std::process::Child,
) -> Result<usize, String> {
    wait_for_node_visible(
        ros_setup,
        install_setup,
        domain_id,
        stack,
        Instant::now() + STARTUP_TIMEOUT,
    )?;
    match wait_for_lifecycle_queryable(
        ros_setup,
        install_setup,
        domain_id,
        stack,
        Instant::now() + STARTUP_TIMEOUT,
    )? {
        LifecycleState::Active => {}
        LifecycleState::Inactive => {
            set_lifecycle_transition(ros_setup, install_setup, domain_id, "activate")?;
            wait_for_lifecycle_state(
                ros_setup,
                install_setup,
                domain_id,
                stack,
                LifecycleState::Active,
                Instant::now() + STARTUP_TIMEOUT,
            )?;
        }
        LifecycleState::Unconfigured => {
            set_lifecycle_transition(ros_setup, install_setup, domain_id, "configure")?;
            wait_for_lifecycle_state(
                ros_setup,
                install_setup,
                domain_id,
                stack,
                LifecycleState::Inactive,
                Instant::now() + STARTUP_TIMEOUT,
            )?;
            set_lifecycle_transition(ros_setup, install_setup, domain_id, "activate")?;
            wait_for_lifecycle_state(
                ros_setup,
                install_setup,
                domain_id,
                stack,
                LifecycleState::Active,
                Instant::now() + STARTUP_TIMEOUT,
            )?;
        }
        LifecycleState::Finalized => {
            return Err("costmap node reached finalized during startup".to_string());
        }
        LifecycleState::Unknown => unreachable!("queryable state already filtered"),
    }
    let registrations =
        wait_for_registration_settled(reader, registry, Instant::now() + STARTUP_TIMEOUT)?;
    for (service_name, expected_type) in REQUIRED_COSTMAP_SERVICES {
        wait_for_service_type(
            ros_setup,
            install_setup,
            domain_id,
            stack,
            service_name,
            expected_type,
            Instant::now() + STARTUP_TIMEOUT,
        )?;
    }
    wait_for_parameter_service(
        ros_setup,
        install_setup,
        domain_id,
        stack,
        Instant::now() + STARTUP_TIMEOUT,
    )?;
    Ok(registrations)
}

struct RoundExecution {
    trace: my_r2d2::callback_profile::CallbackTrace,
    crashed: bool,
    sched_name: Option<String>,
    interface_label: String,
    endpoint_label: String,
}

#[allow(clippy::too_many_arguments)]
fn execute_payload_round(
    round_index: u64,
    round_label: &str,
    marker_round: u64,
    payload: &Payload,
    binding: &InterfaceBinding,
    registry: &mut CallbackRegistry,
    reader: &mut TraceReader,
    pending_events: &mut Vec<RuntimeEvent>,
    round_marker: &Option<(PathBuf, String)>,
    ros_setup: &Path,
    install_setup: &Path,
    payload_file: &Path,
    domain_id: &str,
    schedules: &[(String, Schedule)],
    config: &Config,
    stack: &mut std::process::Child,
) -> Result<RoundExecution, String> {
    let interface_label = binding.interface.name.clone();
    let endpoint_label = binding.endpoint_name().to_string();
    if !pending_events.is_empty() {
        eprintln!(
            "{round_label}: dropping {} carry-over events after previous boundary",
            pending_events.len()
        );
        pending_events.clear();
    }
    let stale =
        drain_runtime_with_live_registry(&format!("{round_label} preflight"), reader, registry)?;
    if !stale.events.is_empty() || stale.missed != 0 {
        eprintln!(
            "{round_label}: dropping {} stale runtime events (missed={}) before payload send",
            stale.events.len(),
            stale.missed
        );
    }
    let mut round_events: Vec<RuntimeEvent> = Vec::new();
    let mut round_missed: u64 = 0;
    let mut last_delivered = 0u64;
    let mut expected_msgs = 0u64;
    let mut sched_name: Option<String> = None;
    let mut parameter_restore: Option<(Ros2ParameterSender, ValueTree)> = None;

    match &binding.endpoint {
        EndpointBinding::LaserScan { .. } => {
            let schedule_slot = schedules.get((round_index as usize - 1) % schedules.len().max(1));
            let (rate_hz, duration_sec, burst, burst_gap, max_publishes, stamp_mode) =
                match schedule_slot {
                    Some((name, schedule)) => {
                        sched_name = Some(name.clone());
                        (
                            1000.0 / schedule.period_ms.max(1) as f64,
                            schedule.duration_sec.clamp(0.1, 300.0),
                            schedule.burst_count.max(1),
                            schedule.burst_gap_ms,
                            schedule.max_publishes,
                            schedule.stamp_mode.clone(),
                        )
                    }
                    None => (
                        config.bridge_rate_hz as f64,
                        config.round_duration_sec,
                        1,
                        0,
                        0,
                        "now".to_string(),
                    ),
                };
            let sender = Ros2LaserScanSender::new(
                ros_setup,
                install_setup,
                payload_file,
                domain_id,
                LaserScanSchedule {
                    rate_hz,
                    duration_sec,
                    burst_count: burst,
                    burst_gap_ms: burst_gap,
                    max_publishes,
                    stamp_mode,
                },
            );
            expected_msgs = sender.expected_messages();
            for attempt in 0..BRIDGE_ATTEMPTS {
                if let Err(error) = sender.send(payload) {
                    eprintln!("{round_label}: {error}");
                }
                let drained = drain_runtime_with_live_registry(&round_label, reader, registry)?;
                let attempt_trace = profile_trace(registry, &drained);
                let delivered = attempt_trace.msg_trace.len() as u64;
                last_delivered = delivered;
                round_missed += drained.missed;
                round_events.extend(drained.events);
                if expected_msgs > 0
                    && delivered * 2 < expected_msgs
                    && stack_alive(stack)
                    && attempt + 1 < BRIDGE_ATTEMPTS
                {
                    eprintln!(
                        "{round_label}: {delivered}/{} messages delivered (attempt {}), respawning bridge",
                        expected_msgs,
                        attempt + 1
                    );
                    continue;
                }
                break;
            }
        }
        EndpointBinding::Topic {
            topic_name,
            message_type,
            options,
        } => {
            expected_msgs = 1;
            let sender = Ros2TopicSender::new(
                ros_setup,
                install_setup,
                domain_id,
                topic_name.clone(),
                message_type.clone(),
                binding.interface.clone(),
                options.clone(),
            );
            if let Err(error) = sender.send(payload) {
                eprintln!("{round_label}: {error}");
            }
            let drained = drain_runtime_with_live_registry(&round_label, reader, registry)?;
            let attempt_trace = profile_trace(registry, &drained);
            last_delivered = attempt_trace.msg_trace.len() as u64;
            round_missed += drained.missed;
            round_events.extend(drained.events);
        }
        EndpointBinding::Service {
            service_name,
            service_type,
        } => {
            let sender = Ros2ServiceSender::new(
                ros_setup,
                install_setup,
                domain_id,
                service_name.clone(),
                service_type.clone(),
                binding.interface.clone(),
            );
            if let Err(error) = sender.send(payload) {
                eprintln!("{round_label}: {error}");
            }
            let drained = drain_runtime_with_live_registry(&round_label, reader, registry)?;
            let attempt_trace = profile_trace(registry, &drained);
            last_delivered = attempt_trace.msg_trace.len() as u64;
            round_missed += drained.missed;
            round_events.extend(drained.events);
        }
        EndpointBinding::Parameter {
            node_name,
            parameter_name,
            restore_value,
        } => {
            let sender = Ros2ParameterSender::new(
                ros_setup,
                install_setup,
                domain_id,
                node_name.clone(),
                parameter_name.clone(),
                binding.interface.clone(),
            );
            if let Err(error) = sender.send(payload) {
                eprintln!("{round_label}: {error}");
            }
            let drained = drain_runtime_with_live_registry(&round_label, reader, registry)?;
            let attempt_trace = profile_trace(registry, &drained);
            last_delivered = attempt_trace.msg_trace.len() as u64;
            round_missed += drained.missed;
            round_events.extend(drained.events);
            parameter_restore = Some((sender, restore_value.clone()));
        }
    }

    if let Some((marker_binary, shm_name)) = round_marker {
        thread::sleep(MARKER_SETTLE);
        if let Err(error) = Command::new(marker_binary)
            .arg(shm_name)
            .arg(marker_round.to_string())
            .status()
        {
            eprintln!("{round_label}: round_marker spawn failed: {error}");
        }
        let drained = drain_runtime_with_live_registry(&round_label, reader, registry)
            .map_err(|error| format!("{error}; final marker drain"))?;
        round_missed += drained.missed;
        round_events.extend(drained.events);
        if let Some(pos) = round_events.iter().rposition(|event| {
            event.event_type == RuntimeEventType::RoundBoundary
                && event.round_id == marker_round as u32
        }) {
            *pending_events = round_events.split_off(pos + 1);
            round_events.pop();
        }
    }

    let _ = ingest_registration_updates(&round_label, reader, registry)?;
    let trace = profile_trace(
        registry,
        &RuntimeDrain {
            events: round_events,
            missed: round_missed,
        },
    );
    if let Some((sender, restore_value)) = parameter_restore {
        if !pending_events.is_empty() {
            eprintln!(
                "{round_label}: dropping {} post-marker events before parameter restore",
                pending_events.len()
            );
            pending_events.clear();
        }
        sender.send_value(&restore_value).map_err(|error| {
            format!(
                "{round_label}: failed to restore parameter {} to startup value: {error}",
                binding.endpoint_name()
            )
        })?;
        thread::sleep(MARKER_SETTLE);
        for _ in 0..2 {
            let drained = drain_runtime_with_live_registry(
                &format!("{round_label} restore"),
                reader,
                registry,
            )?;
            if drained.events.is_empty() && drained.missed == 0 {
                break;
            }
        }
    }
    if expected_msgs > 0 && last_delivered * 2 < expected_msgs && !trace.msg_trace.is_empty() {
        eprintln!(
            "{round_label}: only {last_delivered}/{} messages delivered after {} attempts",
            expected_msgs, BRIDGE_ATTEMPTS
        );
    }

    Ok(RoundExecution {
        trace,
        crashed: !stack_alive(stack),
        sched_name,
        interface_label,
        endpoint_label,
    })
}

#[allow(clippy::too_many_arguments)]
fn build_benchmark_model_live(
    config: &Config,
    bindings: &[InterfaceBinding],
    nav2_ws: &Path,
    shm_path: &Path,
    costmap_params: &Path,
    ros_setup: &Path,
    install_setup: &Path,
    payload_file: &Path,
    domain_id: &str,
    registry: &mut CallbackRegistry,
    reader: &mut TraceReader,
    pending_events: &mut Vec<RuntimeEvent>,
    round_marker: &Option<(PathBuf, String)>,
    schedules: &[(String, Schedule)],
    stack: &mut std::process::Child,
) -> Result<(BenchmarkModel, u64), String> {
    let lcov_root = config.lcov_dir.as_deref();
    let parameter_budget = Duration::from_secs(config.benchmark_parameter_budget_seconds);
    let mut parameter_spent = Duration::ZERO;
    let mut parameter_skips = 0u64;
    let mut parameter_budget_announced = false;
    write_benchmark_status(
        lcov_root,
        "sampling",
        config.benchmark_seconds,
        Duration::ZERO,
        0,
        0,
        0,
        0,
        parameter_spent,
        parameter_budget,
        parameter_skips,
        Some("benchmark bootstrap"),
    );
    if let Some(path) = &config.benchmark_model
        && path.exists()
    {
        println!("benchmark: loading model from {}", path.display());
        let model = BenchmarkModel::load_json(path)?;
        write_benchmark_status(
            lcov_root,
            "complete",
            config.benchmark_seconds,
            Duration::ZERO,
            0,
            model.analyzed_traces,
            0,
            0,
            parameter_spent,
            parameter_budget,
            parameter_skips,
            Some("loaded precomputed benchmark model"),
        );
        write_benchmark_summary(
            lcov_root,
            "precomputed-model",
            config.benchmark_seconds,
            Duration::ZERO,
            0,
            model.analyzed_traces,
            0,
            0,
            parameter_spent,
            parameter_budget,
            parameter_skips,
            model.edge_count(),
            model.distinct_callbacks(),
        );
        return Ok((model, 0));
    }
    if config.benchmark_seconds == 0 {
        return Err(
            "benchmark-seconds must be > 0 when no precomputed benchmark model is supplied"
                .to_string(),
        );
    }

    let interfaces = bindings
        .iter()
        .map(|binding| binding.interface.clone())
        .collect::<Vec<_>>();
    let mut generator = PayloadGenerator::new(
        interfaces,
        GeneratorConfig::default(),
        config.seed ^ 0xB3A5_E1A0,
    );
    let mut builder = BenchmarkBuilder::default();
    let start = Instant::now();
    let deadline = start + Duration::from_secs(config.benchmark_seconds);
    let mut next_log = start + Duration::from_secs(30);
    let mut round = 0u64;

    println!(
        "benchmark: sampling for {}s to build callback graph and average benchmarks (parameter budget={}s)",
        config.benchmark_seconds,
        config.benchmark_parameter_budget_seconds
    );
    while Instant::now() < deadline {
        round += 1;
        if !stack_alive(stack) {
            return Err(format!("benchmark round {round}: costmap stack died"));
        }
        let payload = generator
            .next_payload()
            .map_err(|error| error.to_string())?;
        let binding = find_binding(bindings, &payload.interface_id).ok_or_else(|| {
            format!(
                "benchmark round {round}: unknown interface {}",
                payload.interface_id
            )
        })?;
        let round_label = format!("benchmark round {round}");
        let parameter_binding = matches!(binding.endpoint, EndpointBinding::Parameter { .. });
        if parameter_binding && parameter_spent >= parameter_budget {
            parameter_skips += 1;
            if !parameter_budget_announced {
                eprintln!(
                    "benchmark: parameter budget exhausted at {}s/{}s; skipping parameter interfaces for the remaining benchmark window",
                    parameter_spent.as_secs(),
                    parameter_budget.as_secs()
                );
                parameter_budget_announced = true;
            }
            write_benchmark_status(
                lcov_root,
                "sampling",
                config.benchmark_seconds,
                start.elapsed(),
                round,
                builder.analyzed_traces(),
                builder.empty_traces(),
                builder.invalid_traces(),
                parameter_spent,
                parameter_budget,
                parameter_skips,
                Some("parameter interface skipped after budget exhaustion"),
            );
            continue;
        }
        let execution_started = Instant::now();
        let execution = match execute_payload_round(
            round,
            &round_label,
            round,
            &payload,
            binding,
            registry,
            reader,
            pending_events,
            round_marker,
            ros_setup,
            install_setup,
            payload_file,
            domain_id,
            schedules,
            config,
            stack,
        ) {
            Ok(execution) => {
                if parameter_binding {
                    parameter_spent += execution_started.elapsed();
                }
                execution
            }
            Err(message) => {
                if parameter_binding {
                    parameter_spent += execution_started.elapsed();
                }
                builder.record_invalid_round();
                eprintln!("benchmark round {round}: {message}");
                if !stack_alive(stack) {
                    return Err(format!(
                        "benchmark round {round}: costmap stack died after execution failure"
                    ));
                }
                eprintln!("benchmark round {round}: restarting stack after failed execution");
                terminate_stack_process_group(stack);
                pending_events.clear();
                let (new_stack, new_reader, new_registry, registrations) = start_ready_stack(
                    nav2_ws,
                    shm_path,
                    config,
                    costmap_params,
                    ros_setup,
                    install_setup,
                    domain_id,
                )?;
                *stack = new_stack;
                *reader = new_reader;
                *registry = new_registry;
                println!(
                    "benchmark: stack restarted; ready barrier passed with {} registration records and {} complete callbacks",
                    registrations,
                    registry.callback_infos().len()
                );
                bootstrap_map_if_available(bindings, config, ros_setup, install_setup, domain_id);
                write_benchmark_status(
                    lcov_root,
                    "sampling",
                    config.benchmark_seconds,
                    start.elapsed(),
                    round,
                    builder.analyzed_traces(),
                    builder.empty_traces(),
                    builder.invalid_traces(),
                    parameter_spent,
                    parameter_budget,
                    parameter_skips,
                    Some("stack restarted after benchmark execution failure"),
                );
                continue;
            }
        };
        if execution.crashed {
            return Err(format!("benchmark round {round}: costmap stack crashed"));
        }
        builder.observe(&execution.trace);

        if Instant::now() >= next_log {
            println!(
                "benchmark: elapsed={}s rounds={} analyzed={} empty={} invalid={} param_spent={}s/{}s param_skips={}",
                start.elapsed().as_secs(),
                round,
                builder.analyzed_traces(),
                builder.empty_traces(),
                builder.invalid_traces(),
                parameter_spent.as_secs(),
                parameter_budget.as_secs(),
                parameter_skips,
            );
            next_log = Instant::now() + Duration::from_secs(30);
        }
        write_benchmark_status(
            lcov_root,
            "sampling",
            config.benchmark_seconds,
            start.elapsed(),
            round,
            builder.analyzed_traces(),
            builder.empty_traces(),
            builder.invalid_traces(),
            parameter_spent,
            parameter_budget,
            parameter_skips,
            Some("sampling"),
        );
    }

    if builder.analyzed_traces() == 0 {
        return Err("benchmark phase collected zero analyzed traces".to_string());
    }
    let model = builder.build();
    println!(
        "benchmark: rounds={} analyzed={} empty={} invalid={} edges={} callbacks={}",
        round,
        builder.analyzed_traces(),
        builder.empty_traces(),
        builder.invalid_traces(),
        model.edge_count(),
        model.distinct_callbacks()
    );
    write_benchmark_status(
        lcov_root,
        "complete",
        config.benchmark_seconds,
        start.elapsed(),
        round,
        builder.analyzed_traces(),
        builder.empty_traces(),
        builder.invalid_traces(),
        parameter_spent,
        parameter_budget,
        parameter_skips,
        Some("benchmark sampling complete"),
    );
    write_benchmark_summary(
        lcov_root,
        "live-sampling",
        config.benchmark_seconds,
        start.elapsed(),
        round,
        builder.analyzed_traces(),
        builder.empty_traces(),
        builder.invalid_traces(),
        parameter_spent,
        parameter_budget,
        parameter_skips,
        model.edge_count(),
        model.distinct_callbacks(),
    );
    if let Some(path) = &config.benchmark_model {
        model.save_json(path)?;
        println!("benchmark: saved model to {}", path.display());
    }
    Ok((model, round))
}

fn write_json_report(path: &Path, value: serde_json::Value) {
    if let Some(parent) = path.parent() {
        let _ = fs::create_dir_all(parent);
    }
    if let Ok(bytes) = serde_json::to_vec_pretty(&value) {
        let _ = fs::write(path, bytes);
    }
}

#[allow(clippy::too_many_arguments)]
fn write_benchmark_status(
    lcov_root: Option<&Path>,
    phase: &str,
    configured_seconds: u64,
    elapsed: Duration,
    round: u64,
    analyzed_traces: u64,
    empty_traces: u64,
    invalid_traces: u64,
    parameter_spent: Duration,
    parameter_budget: Duration,
    parameter_skips: u64,
    note: Option<&str>,
) {
    let Some(lcov_root) = lcov_root else {
        return;
    };
    let elapsed_secs = elapsed.as_secs();
    let remaining_secs = configured_seconds.saturating_sub(elapsed_secs);
    write_json_report(
        &lcov_root.join("benchmark/status.json"),
        json!({
            "phase": phase,
            "configured_seconds": configured_seconds,
            "elapsed_seconds": elapsed_secs,
            "remaining_seconds": remaining_secs,
            "round": round,
            "analyzed_traces": analyzed_traces,
            "empty_traces": empty_traces,
            "invalid_traces": invalid_traces,
            "parameter_spent_seconds": parameter_spent.as_secs(),
            "parameter_budget_seconds": parameter_budget.as_secs(),
            "parameter_skips": parameter_skips,
            "note": note,
        }),
    );
}

#[allow(clippy::too_many_arguments)]
fn write_benchmark_summary(
    lcov_root: Option<&Path>,
    source: &str,
    configured_seconds: u64,
    elapsed: Duration,
    rounds: u64,
    analyzed_traces: u64,
    empty_traces: u64,
    invalid_traces: u64,
    parameter_spent: Duration,
    parameter_budget: Duration,
    parameter_skips: u64,
    edge_count: usize,
    distinct_callbacks: usize,
) {
    let Some(lcov_root) = lcov_root else {
        return;
    };
    write_json_report(
        &lcov_root.join("benchmark/summary.json"),
        json!({
            "phase": "complete",
            "source": source,
            "configured_seconds": configured_seconds,
            "elapsed_seconds": elapsed.as_secs(),
            "rounds": rounds,
            "analyzed_traces": analyzed_traces,
            "empty_traces": empty_traces,
            "invalid_traces": invalid_traces,
            "parameter_spent_seconds": parameter_spent.as_secs(),
            "parameter_budget_seconds": parameter_budget.as_secs(),
            "parameter_skips": parameter_skips,
            "callback_graph_edges": edge_count,
            "distinct_callbacks": distinct_callbacks,
        }),
    );
}

/// gcov 计数只在进程退出或 __gcov_dump() 时落盘；tracer 在 COVERAGE_RUN
/// 构建下为 SIGUSR1 安装 dump handler。每轮结束由本进程发信号，随后 lcov
/// 即可读到本轮到当前为止的累计覆盖。
fn flush_costmap_coverage(shm: &Path) {
    let pid_path = shm.with_extension("pid");
    let Ok(pid_text) = fs::read_to_string(&pid_path) else {
        eprintln!("coverage: missing pid file {}", pid_path.display());
        return;
    };
    let Ok(pid) = pid_text.trim().parse::<i32>() else {
        eprintln!("coverage: unparsable pid file {}", pid_path.display());
        return;
    };
    if !Command::new("kill")
        .args(["-USR1", &pid.to_string()])
        .status()
        .is_ok_and(|status| status.success())
    {
        eprintln!("coverage: SIGUSR1 to costmap pid {pid} failed");
    }
    thread::sleep(Duration::from_millis(500));
}

/// lcov --summary 的 branches 行（"branches......: 51.0% (1234 of 2420 branches)"）
/// 解析为 (covered, total)。
fn branch_totals_from_summary(summary_text: &str) -> Option<(u64, u64)> {
    let branch_line = summary_text
        .lines()
        .find(|line| line.trim_start().starts_with("branches"))?;
    let open = branch_line.find('(')?;
    let close = branch_line.find(')')?;
    let mut parts = branch_line[open + 1..close].split_whitespace();
    let covered: u64 = parts.next()?.parse().ok()?;
    if parts.next()? != "of" {
        return None;
    }
    let total: u64 = parts.next()?.parse().ok()?;
    Some((covered, total))
}

/// lcov 的 `--gcov-tool` 可多次给出以构成完整命令行（例如 llvm-cov 需要
/// `--gcov-tool /usr/bin/llvm-cov-18 --gcov-tool gcov`）；`gcov_tool` 的 token
/// 按空白拆分逐个追加，None 表示交给 lcov 自动探测。
fn lcov_capture_command(gcov_tool: Option<&str>) -> Command {
    let mut command = Command::new("lcov");
    command.arg("--capture");
    if let Some(tool) = gcov_tool {
        for token in tool.split_whitespace() {
            command.args(["--gcov-tool", token]);
        }
    }
    command
}

/// 累计分支计数只增不减。SIGUSR1 触发的 `__gcov_dump()` 与进行中的回调
/// 并发执行时，快照会漏掉该回调尚未走到的分支，导致读数低于上一轮；此时
/// 以上一轮读数作为累计下界，并标记本轮捕获为 dip。
fn cumulative_branches(captured: u64, previous: u64) -> (u64, bool) {
    if captured < previous {
        (previous, true)
    } else {
        (captured, false)
    }
}

/// 抓取一轮累计覆盖：lcov --capture 整个 workspace build 目录，返回
/// branches (covered, total)；失败返回 None（不影响 fuzzing 主循环）。
fn capture_round_coverage(
    ws: &Path,
    round_dir: &Path,
    gcov_tool: Option<&str>,
) -> Option<(u64, u64)> {
    let build_dir = ws.join("build");
    fs::create_dir_all(round_dir).ok()?;
    let info_path = round_dir.join("coverage.info");
    let status = lcov_capture_command(gcov_tool)
        .args([
            "--directory",
            build_dir.to_str()?,
            "--rc",
            "branch_coverage=1",
            "--ignore-errors",
            "mismatch,empty,gcov,negative",
            "--output-file",
            info_path.to_str()?,
            "--quiet",
        ])
        .status()
        .ok()?;
    if !status.success() {
        eprintln!("coverage: lcov capture failed for {}", round_dir.display());
        return None;
    }
    let summary = Command::new("lcov")
        .args([
            "--summary",
            info_path.to_str()?,
            "--rc",
            "branch_coverage=1",
        ])
        .output()
        .ok()?;
    if !summary.status.success() {
        return None;
    }
    branch_totals_from_summary(&String::from_utf8_lossy(&summary.stdout))
}

/// 收尾：costmap 进程组终止（exit 时 gcov 做最终 dump）后，抓取总覆盖、
/// 生成 genhtml HTML 报告，返回最终 branches (covered, total)。
fn finalize_coverage(ws: &Path, lcov_root: &Path, gcov_tool: Option<&str>) -> Option<(u64, u64)> {
    let build_dir = ws.join("build");
    let total_info = lcov_root.join("coverage_total.info");
    let status = lcov_capture_command(gcov_tool)
        .args([
            "--directory",
            build_dir.to_str()?,
            "--rc",
            "branch_coverage=1",
            "--ignore-errors",
            "mismatch,empty,gcov,negative",
            "--output-file",
            total_info.to_str()?,
            "--quiet",
        ])
        .status()
        .ok()?;
    if !status.success() {
        return None;
    }
    let html_dir = lcov_root.join("lcov_html");
    let _ = fs::remove_dir_all(&html_dir);
    fs::create_dir_all(&html_dir).ok()?;
    let _ = Command::new("genhtml")
        .args([
            total_info.to_str()?,
            "--rc",
            "branch_coverage=1",
            "--branch-coverage",
            "--output-directory",
            html_dir.to_str()?,
            "--quiet",
        ])
        .status();
    let summary = Command::new("lcov")
        .args([
            "--summary",
            total_info.to_str()?,
            "--rc",
            "branch_coverage=1",
        ])
        .output()
        .ok()?;
    if !summary.status.success() {
        return None;
    }
    branch_totals_from_summary(&String::from_utf8_lossy(&summary.stdout))
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
    let ros_setup = match yaml_env.require_path("R2D2_ROS_SETUP") {
        Ok(path) => path,
        Err(message) => {
            eprintln!("nav2_costmap_e2e: {message}");
            std::process::exit(1);
        }
    };
    let costmap_params = match yaml_env.require_path("R2D2_COSTMAP_PARAMS") {
        Ok(path) => path,
        Err(message) => {
            eprintln!("nav2_costmap_e2e: {message}");
            std::process::exit(1);
        }
    };
    let shm_path = match yaml_env.require_path("R2D2_SHM_PATH") {
        Ok(path) => path,
        Err(message) => {
            eprintln!("nav2_costmap_e2e: {message}");
            std::process::exit(1);
        }
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

    let payload_file = nav2_ws.join("payload_round.txt");
    let install_setup = nav2_ws.join("install/setup.bash");
    if !install_setup.exists() {
        eprintln!(
            "nav2_costmap_e2e: {} missing; build nav2_ws first",
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

    // 1. 启动插桩 costmap 栈，然后由 harness 接管 lifecycle + ready barrier。
    let (mut stack, mut reader, mut registry, registrations) = match start_ready_stack(
        &nav2_ws,
        &shm_path,
        &config,
        &costmap_params,
        &ros_setup,
        &install_setup,
        &domain_id,
    ) {
        Ok(registrations) => registrations,
        Err(message) => {
            eprintln!("nav2_costmap_e2e: startup barrier failed: {message}");
            let _ = fs::remove_file(&shm_path);
            let _ = fs::remove_file(shm_path.with_extension("pid"));
            std::process::exit(1);
        }
    };
    let infos = registry.callback_infos();
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
    let bindings = match extract_costmap_bindings(&share_root, &costmap_params) {
        Ok(bindings) => bindings,
        Err(message) => {
            eprintln!("nav2_costmap_e2e: dry run failed: {message}");
            std::process::exit(1);
        }
    };
    let interfaces = bindings
        .iter()
        .map(|binding| binding.interface.clone())
        .collect::<Vec<_>>();
    println!(
        "dry run: extracted {} costmap interfaces: {}",
        interfaces.len(),
        bindings
            .iter()
            .map(|binding| format!("{} -> {}", binding.interface.name, binding.endpoint_name()))
            .collect::<Vec<_>>()
            .join(", ")
    );

    bootstrap_map_if_available(&bindings, &config, &ros_setup, &install_setup, &domain_id);

    // 4. 主循环。scan seeds 仍只预填到 LaserScan 接口，其余接口纯按结构生成。
    let mut seed_preload: Vec<Payload> = Vec::new();
    let mut schedules: Vec<(String, Schedule)> = Vec::new();
    let scan_interface =
        find_binding(&bindings, "LaserScan").map(|binding| binding.interface.clone());
    if let Some(seed_dir) = &config.seed_dir {
        if let Some(interface) = &scan_interface {
            match load_scan_seeds(&seed_dir.join("scans"), interface) {
                Ok(seeds) => {
                    println!(
                        "seeds: {} scan seeds loaded from {}",
                        seeds.len(),
                        seed_dir.display()
                    );
                    seed_preload = seeds;
                }
                Err(error) => eprintln!("seeds: scan loading failed: {error}"),
            }
        }
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
    // 轮次分界：每轮结束向 runtime ring 写一条 RoundBoundary marker，reader
    // 按 marker 把异步落盘的事件归到对应轮次（论文未披露边界格式，此为
    // reproduction choice）。round_marker 缺失时回退纯游标分段。
    let shm_name = shm_path
        .file_name()
        .map(|name| name.to_string_lossy().into_owned());
    let round_marker = resolve_round_marker(repo_root, &nav2_ws).zip(shm_name);
    match &round_marker {
        Some((binary, _)) => println!("rounds: delimiting payload rounds via {}", binary.display()),
        None => {
            eprintln!("rounds: round_marker not found; falling back to cursor-only segmentation")
        }
    }
    // 上一轮 marker 之后才读到的事件（异步写延迟）带入下一轮。
    let mut pending_events: Vec<RuntimeEvent> = Vec::new();
    let (benchmark, benchmark_rounds) = match build_benchmark_model_live(
        &config,
        &bindings,
        &nav2_ws,
        &shm_path,
        &costmap_params,
        &ros_setup,
        &install_setup,
        &payload_file,
        &domain_id,
        &mut registry,
        &mut reader,
        &mut pending_events,
        &round_marker,
        &schedules,
        &mut stack,
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
                Duration::ZERO,
                Duration::from_secs(config.benchmark_parameter_budget_seconds),
                0,
                Some(&message),
            );
            eprintln!("nav2_costmap_e2e: benchmark failed: {message}");
            terminate_stack_process_group(&mut stack);
            let _ = fs::remove_file(&shm_path);
            let _ = fs::remove_file(shm_path.with_extension("pid"));
            let _ = fs::remove_file(&payload_file);
            std::process::exit(1);
        }
    };
    let mut generator = PayloadGenerator::new(interfaces, GeneratorConfig::default(), config.seed);
    for seed in seed_preload {
        generator.pool_mut().push(seed);
    }
    let thresholds = DeviationThresholds::new(config.latency_factor, config.throughput_floor);
    let mut oracle = BenchmarkStateOracle::with_mode(benchmark, thresholds, config.oracle_mode);
    println!(
        "loop: rounds={} seed={} benchmark_traces={} oracle_mode={} latency_factor={} throughput_floor={}",
        config.rounds,
        config.seed,
        oracle.benchmark().analyzed_traces,
        oracle.mode().as_str(),
        config.latency_factor,
        config.throughput_floor
    );
    if config.lcov_dir.is_none() {
        eprintln!("coverage: disabled; pass --lcov-dir <dir> if you expect nav2_ws/results output");
    }
    let mut crashes = 0u64;
    let mut active_new_states = 0u64;
    let mut paper_supported_new_states = 0u64;
    let mut jazzy_reproduction_new_states = 0u64;
    let mut invalid = 0u64;
    let mut empty_rounds = 0u64;
    let mut prev_branches: u64 = 0;
    if let Some(lcov_root) = &config.lcov_dir {
        let _ = fs::create_dir_all(lcov_root);
        let benchmark_dir = lcov_root.join("benchmark");
        flush_costmap_coverage(&shm_path);
        if let Some((covered, total)) =
            capture_round_coverage(&nav2_ws, &benchmark_dir, config.gcov_tool.as_deref())
        {
            prev_branches = covered;
            println!(
                "coverage: benchmark baseline branches {covered}/{total} -> {}",
                benchmark_dir.display()
            );
        } else {
            eprintln!("coverage: benchmark baseline capture failed");
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
        let marker_round = benchmark_rounds + round;
        let round_label = format!("round {round}");
        let execution = match execute_payload_round(
            round,
            &round_label,
            marker_round,
            &payload,
            binding,
            &mut registry,
            &mut reader,
            &mut pending_events,
            &round_marker,
            &ros_setup,
            &install_setup,
            &payload_file,
            &domain_id,
            &schedules,
            &config,
            &mut stack,
        ) {
            Ok(execution) => execution,
            Err(message) => {
                eprintln!("round {round}: {message}");
                continue;
            }
        };
        let trace = execution.trace;
        let crashed = execution.crashed;
        let verdict = oracle.evaluate(&trace, crashed);
        let empty = verdict.trace == TraceDisposition::Empty;
        if empty && !crashed {
            empty_rounds += 1;
        } else if verdict.trace == TraceDisposition::Invalid {
            invalid += 1;
        }
        if verdict.paper_supported_new_state {
            paper_supported_new_states += 1;
        }
        if verdict.jazzy_reproduction_new_state {
            jazzy_reproduction_new_states += 1;
        }
        let new_state = verdict.new_state;
        if crashed {
            crashes += 1;
        }
        if new_state {
            active_new_states += 1;
        }
        generator.retain_if_interesting(payload.clone(), &oracle);

        let decision = if crashed {
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
            "round {round:02} | iface={:<24} | ep={:<30} | len={:4} | calls={} msgs={} | exec=[{}] thr=[{}] MB/s | decision={decision:<14} | mode={} active={} paper={} jazzy={} | evidence=edge:{} cb:{} msg:{} lat:{} thr:{} | pool={} | sched={}",
            execution.interface_label,
            execution.endpoint_label,
            payload.serialized.len(),
            trace.call_trace.len(),
            trace.msg_trace.len(),
            execs.join(","),
            throughputs.join(","),
            oracle.mode().as_str(),
            verdict.new_state,
            verdict.paper_supported_new_state,
            verdict.jazzy_reproduction_new_state,
            verdict.evidence.new_edge,
            verdict.evidence.new_callback,
            verdict.evidence.new_message,
            verdict.evidence.latency_deviation,
            verdict.evidence.throughput_deviation,
            generator.pool().len(),
            execution.sched_name.as_deref().unwrap_or("-"),
        );
        println!("{round_line}");

        if let Some(lcov_root) = &config.lcov_dir {
            let round_dir = lcov_root.join(format!("rounds/round_{round:06}"));
            flush_costmap_coverage(&shm_path);
            let stats = capture_round_coverage(&nav2_ws, &round_dir, config.gcov_tool.as_deref());
            let coverage_ok = stats.is_some();
            let (captured, total) = stats.unwrap_or((prev_branches, prev_branches));
            // 累计分支计数只会增长；SIGUSR1 dump 与进行中的回调并发时，快照
            // 会漏掉该回调尚未走到的分支，读数低于上一轮。以上一轮为下界。
            let (covered, dipped) = cumulative_branches(captured, prev_branches);
            let increase = covered - prev_branches;
            prev_branches = covered;
            let _ = fs::copy(&payload_file, round_dir.join("payload.txt"));
            let _ = fs::write(round_dir.join("round.txt"), format!("{round_line}\n"));
            let round_summary = format!(
                "{{\n  \"round\": {round},\n  \"decision\": \"{decision}\",\n  \"oracle_mode\": \"{}\",\n  \"active_new_state\": {},\n  \"paper_supported_new_state\": {},\n  \"jazzy_reproduction_new_state\": {},\n  \"evidence\": {{ \"new_edge\": {}, \"new_callback\": {}, \"new_message\": {}, \"latency_deviation\": {}, \"throughput_deviation\": {} }},\n  \"calls\": {},\n  \"msgs\": {},\n  \"pool_size\": {},\n  \"crash_or_hang\": {},\n  \"coverage_ok\": {coverage_ok},\n  \"branch_covered_total\": {covered},\n  \"branch_covered_increase\": {increase}\n}}\n",
                oracle.mode().as_str(),
                verdict.new_state,
                verdict.paper_supported_new_state,
                verdict.jazzy_reproduction_new_state,
                verdict.evidence.new_edge,
                verdict.evidence.new_callback,
                verdict.evidence.new_message,
                verdict.evidence.latency_deviation,
                verdict.evidence.throughput_deviation,
                trace.call_trace.len(),
                trace.msg_trace.len(),
                generator.pool().len(),
                crashed,
            );
            let _ = fs::write(round_dir.join("summary.json"), round_summary);
            println!(
                "       coverage: branches {covered}/{total} (+{increase}){} -> {}",
                if dipped { " [dip clamped]" } else { "" },
                round_dir.display()
            );
        }
    }

    // 4. 收尾：整体终止 costmap 栈（进程组）并清理 shm。
    terminate_stack_process_group(&mut stack);
    let _ = fs::remove_file(&shm_path);
    let _ = fs::remove_file(shm_path.with_extension("pid"));
    let _ = fs::remove_file(&payload_file);

    println!("\n=== summary ===");
    println!(
        "rounds={} crashes={} active_new_states={} paper_supported_new_states={} jazzy_reproduction_new_states={} invalid_traces={} empty_rounds={} pool_size={}",
        config.rounds,
        crashes,
        active_new_states,
        paper_supported_new_states,
        jazzy_reproduction_new_states,
        invalid,
        empty_rounds,
        generator.pool().len()
    );
    println!(
        "oracle_mode={} callback_graph_edges={} distinct_callbacks={}",
        oracle.mode().as_str(),
        oracle.edge_count(),
        oracle.distinct_callbacks()
    );

    // 覆盖收尾：进程组已终止（exit 时 gcov 已做最终 dump），抓总覆盖并出 HTML。
    if let Some(lcov_root) = &config.lcov_dir {
        match finalize_coverage(&nav2_ws, lcov_root, config.gcov_tool.as_deref()) {
            Some((covered, total)) => {
                println!(
                    "coverage: final branches {covered}/{total} -> {}",
                    lcov_root.join("coverage_total.info").display()
                );
                let campaign_summary = format!(
                    "{{\n  \"rounds\": {},\n  \"seed\": {},\n  \"oracle_mode\": \"{}\",\n  \"crashes\": {},\n  \"active_new_states\": {},\n  \"paper_supported_new_states\": {},\n  \"jazzy_reproduction_new_states\": {},\n  \"invalid_traces\": {},\n  \"empty_rounds\": {},\n  \"pool_size\": {},\n  \"callback_graph_edges\": {},\n  \"distinct_callbacks\": {},\n  \"coverage_ok\": true,\n  \"branch_covered_total\": {covered}\n}}\n",
                    config.rounds,
                    config.seed,
                    oracle.mode().as_str(),
                    crashes,
                    active_new_states,
                    paper_supported_new_states,
                    jazzy_reproduction_new_states,
                    invalid,
                    empty_rounds,
                    generator.pool().len(),
                    oracle.edge_count(),
                    oracle.distinct_callbacks(),
                );
                let _ = fs::write(lcov_root.join("summary.json"), campaign_summary);
                println!(
                    "coverage: html report -> {}",
                    lcov_root.join("lcov_html/index.html").display()
                );
            }
            None => eprintln!("coverage: final capture failed"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{branch_totals_from_summary, cumulative_branches};

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
