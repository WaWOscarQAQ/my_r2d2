//! R2D2 端到端冒烟闭环（mock 系统执行，无真实 ROS）。
//!
//! 把现有模块按论文 Figure 3 串成一个可运行的 fuzzing 循环：
//!
//! 1. dry run：从 `tests/fixtures/ros_interfaces` 用 `FileExtractor` 提取真实接口。
//! 2. 每轮：`PayloadGenerator` 生成 payload（空池按接口生成 / 非空池变异）。
//! 3. 系统执行（mock）：spawn `mock_writer --live`，回调延迟、消息大小、
//!    timer 是否执行、是否 crash 都由 payload 序列化长度确定性推导，
//!    事件写入 `/dev/shm`。
//! 4. `TraceReader` 实时 drain；`CallbackRegistry` + `profile_trace` 组装
//!    `CallbackTrace`。
//! 5. 新状态判定（阶段 F 雏形）：新执行序列 edge、执行延迟显著偏离基线、
//!    吞吐低于基线；mock 进程以非零退出码模拟 crash。
//! 6. crash 或 new state 的 payload 入 pool，供后续轮变异。
//!
//! 论文未披露"显著偏离"阈值与采样期长度，本示例的 `--latency-factor`、
//! `--throughput-floor`、`--baseline` 均为 reproduction choice，不代表论文配置。
//!
//! 运行：`cargo run --example end_to_end -- --rounds 15 --seed 42`

use my_r2d2::callback_profile::{CallbackRegistry, CallbackTrace, profile_trace};
use my_r2d2::interface_extractor::{Extractor, FileExtractor, Interface, Kind};
use my_r2d2::payload::Payload;
use my_r2d2::payload_generator::{GeneratorConfig, PayloadGenerator};
use my_r2d2::trace_buffer::TraceReader;
use std::collections::{HashMap, HashSet};
use std::path::{Path, PathBuf};
use std::process::Command;

/// 阶段 F 雏形：维护论文 §4.2.1 的 Global Callback Graph 与 Global
/// Callback Latency 基线，判定当前 trace 是否引入新状态。
struct BaselineOracle {
    /// Global Callback Graph 的边：回调执行序列中的相邻对。
    graph_edges: HashSet<(u64, u64)>,
    latency_sum: HashMap<u64, u64>,
    latency_count: HashMap<u64, u64>,
    throughput_sum: HashMap<u64, f64>,
    throughput_count: HashMap<u64, u64>,
    /// execution latency 显著偏离均值的倍数（reproduction choice）。
    latency_factor: f64,
    /// throughput 显著低于均值的比例上限（reproduction choice）。
    throughput_floor: f64,
}

impl BaselineOracle {
    fn new(latency_factor: f64, throughput_floor: f64) -> Self {
        Self {
            graph_edges: HashSet::new(),
            latency_sum: HashMap::new(),
            latency_count: HashMap::new(),
            throughput_sum: HashMap::new(),
            throughput_count: HashMap::new(),
            latency_factor,
            throughput_floor,
        }
    }

    fn observe(&mut self, trace: &CallbackTrace) {
        for latency in &trace.call_trace {
            *self.latency_sum.entry(latency.callback_id).or_default() += latency.execution_latency;
            *self.latency_count.entry(latency.callback_id).or_default() += 1;
        }
        for msg in &trace.msg_trace {
            *self.throughput_sum.entry(msg.callback_id).or_default() += msg.throughput;
            *self.throughput_count.entry(msg.callback_id).or_default() += 1;
        }
    }

    fn mean_latency(&self, id: u64) -> Option<f64> {
        let count = *self.latency_count.get(&id)?;
        if count == 0 {
            return None;
        }
        Some(*self.latency_sum.get(&id).unwrap_or(&0) as f64 / count as f64)
    }

    fn mean_throughput(&self, id: u64) -> Option<f64> {
        let count = *self.throughput_count.get(&id)?;
        if count == 0 {
            return None;
        }
        Some(*self.throughput_sum.get(&id).unwrap_or(&0.0) / count as f64)
    }

    /// 论文 §4.2.1 的三个判定指标。baseline（采样）阶段只积累基准不判定。
    fn decide(&mut self, trace: &CallbackTrace, baseline: bool) -> bool {
        let mut new_state = false;

        // (1) 新 execution sequence：Global Callback Graph 出现新边。
        let mut previous: Option<u64> = None;
        for latency in &trace.call_trace {
            if let Some(prev) = previous
                && !baseline
                && self.graph_edges.insert((prev, latency.callback_id))
            {
                new_state = true;
            }
            previous = Some(latency.callback_id);
        }

        // (2) callback latency 相对均值出现显著偏差（先比后更新）。
        if !baseline {
            for latency in &trace.call_trace {
                if let Some(mean) = self.mean_latency(latency.callback_id)
                    && mean > 0.0
                    && latency.execution_latency as f64 > mean * self.latency_factor
                {
                    new_state = true;
                }
            }
        }

        // (3) message throughput 显著低于均值。
        if !baseline {
            for msg in &trace.msg_trace {
                if let Some(mean) = self.mean_throughput(msg.callback_id)
                    && msg.throughput < mean * self.throughput_floor
                {
                    new_state = true;
                }
            }
        }

        self.observe(trace);
        new_state
    }

    fn edge_count(&self) -> usize {
        self.graph_edges.len()
    }

    fn distinct_callbacks(&self) -> usize {
        self.latency_sum.len()
    }
}

struct Config {
    rounds: u64,
    seed: u64,
    baseline: u64,
    latency_factor: f64,
    throughput_floor: f64,
    mock_writer: PathBuf,
}

impl Config {
    fn parse(args: impl Iterator<Item = String>) -> Result<Self, String> {
        let mut config = Self {
            rounds: 15,
            seed: 42,
            baseline: 3,
            latency_factor: 2.0,
            throughput_floor: 0.5,
            mock_writer: std::env::var_os("TRACER_MOCK_WRITER")
                .map(PathBuf::from)
                .unwrap_or_else(|| {
                    Path::new(env!("CARGO_MANIFEST_DIR")).join("tracer/build/mock_writer")
                }),
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
                "--baseline" => {
                    config.baseline = value("--baseline")?
                        .parse::<u64>()
                        .map_err(|e| e.to_string())?
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
                "--mock-writer" => config.mock_writer = PathBuf::from(value("--mock-writer")?),
                other => return Err(format!("unknown flag {other}")),
            }
        }
        Ok(config)
    }
}

/// Dry run：提取真实接口文件（论文 §4.2.2 的 interface specification 来源）。
fn extract_interfaces(fixtures_root: &Path) -> Result<Vec<Interface>, String> {
    let files = [
        "geometry_msgs/msg/Twist.msg",
        "sensor_msgs/msg/LaserScan.msg",
        "nav2_msgs/srv/LoadMap.srv",
        "example_interfaces/srv/AddTwoInts.srv",
    ];
    let extractor = FileExtractor::new(
        files
            .iter()
            .map(|f| fixtures_root.join(f))
            .collect::<Vec<_>>(),
        vec![fixtures_root.to_path_buf()],
    );
    extractor.extract().map_err(|e| e.to_string())
}

/// 由 payload 序列化结果确定性推导 mock 系统行为，保证同一 seed 可复现。
fn mock_params(payload: &Payload) -> (u64, u64, bool, u64, u64, u64) {
    let len = payload.serialized.len() as u64;
    let exec_sub = 100 + len % 400;
    let sched_sub = 10 + len % 90;
    let skip_timer = len.is_multiple_of(3);
    let size = 64 + len;
    let pub_ts = 50 + len % 20;
    let sub_ts = pub_ts + 40 + len % 40;
    (exec_sub, sched_sub, skip_timer, size, pub_ts, sub_ts)
}

fn crashes(payload: &Payload) -> bool {
    (payload.serialized.len() as u64).is_multiple_of(7)
}

struct Stats {
    crashes: u64,
    new_states: u64,
    invalid: u64,
}

fn run_round(
    generator: &mut PayloadGenerator,
    registry: &mut CallbackRegistry,
    oracle: &mut BaselineOracle,
    baseline: bool,
    mock_writer: &Path,
    round: u64,
    stats: &mut Stats,
) -> Result<(), String> {
    let payload = generator.next_payload().map_err(|e| e.to_string())?;
    let (exec_sub, sched_sub, skip_timer, size, pub_ts, sub_ts) = mock_params(&payload);
    let crash = crashes(&payload);

    // Mock 系统执行：事件写进 /dev/shm，然后由 Rust 侧实时读取。
    let shm_name = format!("my_r2d2_e2e_{}", std::process::id());
    let mut command = Command::new(mock_writer);
    command
        .arg(&shm_name)
        .arg("--live")
        .arg("--exec-sub")
        .arg(exec_sub.to_string())
        .arg("--sched-sub")
        .arg(sched_sub.to_string())
        .arg("--size")
        .arg(size.to_string())
        .arg("--pub")
        .arg(pub_ts.to_string())
        .arg("--sub")
        .arg(sub_ts.to_string());
    if skip_timer {
        command.arg("--no-timer");
    }
    if crash {
        command.arg("--crash");
    }
    let status = command
        .status()
        .map_err(|e| format!("spawn {mock_writer:?}: {e}"))?;

    let shm_path = format!("/dev/shm/{shm_name}");
    let mut reader = TraceReader::open(&shm_path).map_err(|e| e.to_string())?;
    registry.ingest(&reader.drain_registration().map_err(|e| e.to_string())?);
    let trace = profile_trace(
        registry,
        &reader.drain_runtime().map_err(|e| e.to_string())?,
    );
    drop(reader);
    let _ = std::fs::remove_file(&shm_path);

    let crashed = !status.success();
    let mut new_state = false;
    if trace.valid_for_state_analysis() {
        new_state = oracle.decide(&trace, baseline);
    } else {
        stats.invalid += 1;
    }
    if crashed {
        stats.crashes += 1;
    }
    if new_state {
        stats.new_states += 1;
    }
    if crashed || new_state {
        generator.pool_mut().push(payload.clone());
    }

    let decision = match (crashed, new_state) {
        (true, true) => "crash+new-state",
        (true, false) => "crash",
        (false, true) => "new-state",
        (false, false) => "none",
    };
    let kind = match payload.kind {
        Kind::Topic => "topic",
        Kind::Service => "service",
    };
    println!(
        "round {round:02} | iface={:>28} {kind:<7} | len={:4} | exec_sub={:3} sched_sub={:2} timer={:<3} crash={:<3} | calls={} msgs={} | decision={decision:<14} | pool={}",
        payload.interface_id,
        payload.serialized.len(),
        exec_sub,
        sched_sub,
        if skip_timer { "no" } else { "yes" },
        if crash { "yes" } else { "no" },
        trace.call_trace.len(),
        trace.msg_trace.len(),
        generator.pool().len(),
    );
    Ok(())
}

fn main() {
    let config = match Config::parse(std::env::args().skip(1)) {
        Ok(config) => config,
        Err(message) => {
            eprintln!("end_to_end: {message}");
            std::process::exit(2);
        }
    };
    if !config.mock_writer.exists() {
        eprintln!(
            "end_to_end: {} not found; build tracer/ with cmake first \
             (or set TRACER_MOCK_WRITER)",
            config.mock_writer.display()
        );
        std::process::exit(1);
    }

    let fixtures_root = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/ros_interfaces");
    let interfaces = match extract_interfaces(&fixtures_root) {
        Ok(interfaces) => interfaces,
        Err(message) => {
            eprintln!("end_to_end: dry run failed: {message}");
            std::process::exit(1);
        }
    };
    println!(
        "dry run: extracted {} interfaces: {}",
        interfaces.len(),
        interfaces
            .iter()
            .map(|i| format!(
                "{} ({})",
                i.name,
                if i.kind == Kind::Topic {
                    "topic"
                } else {
                    "service"
                }
            ))
            .collect::<Vec<_>>()
            .join(", ")
    );

    let mut generator = PayloadGenerator::new(interfaces, GeneratorConfig::default(), config.seed);
    let mut registry = CallbackRegistry::new();
    let mut oracle = BaselineOracle::new(config.latency_factor, config.throughput_floor);
    let mut stats = Stats {
        crashes: 0,
        new_states: 0,
        invalid: 0,
    };

    println!(
        "loop: rounds={} seed={} baseline_rounds={} latency_factor={} throughput_floor={}",
        config.rounds, config.seed, config.baseline, config.latency_factor, config.throughput_floor
    );
    for round in 1..=config.rounds {
        let baseline = round <= config.baseline;
        if let Err(message) = run_round(
            &mut generator,
            &mut registry,
            &mut oracle,
            baseline,
            &config.mock_writer,
            round,
            &mut stats,
        ) {
            eprintln!("end_to_end: round {round} failed: {message}");
            std::process::exit(1);
        }
    }

    println!("\n=== summary ===");
    println!(
        "rounds={} crashes={} new_states={} invalid_traces={} pool_size={}",
        config.rounds,
        stats.crashes,
        stats.new_states,
        stats.invalid,
        generator.pool().len()
    );
    println!(
        "callback_graph_edges={} distinct_callbacks={}",
        oracle.edge_count(),
        oracle.distinct_callbacks()
    );
}
