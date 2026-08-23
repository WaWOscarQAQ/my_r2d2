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
//! 5. 状态基线先通过独立 benchmark phase 建模，再在 fuzz phase 中判断：
//!    新执行序列 edge、执行延迟显著偏离 benchmark、吞吐低于 benchmark；
//!    mock 进程以非零退出码模拟 crash。
//! 6. crash 或 new state 的 payload 入 pool，供后续轮变异。
//!
//! 论文未披露"显著偏离"阈值公式；本示例按论文的两阶段结构实现 2 小时
//! benchmark（可通过 CLI 覆盖秒数，或加载已保存模型），而
//! `--latency-factor`、`--throughput-floor` 仍是 reproduction choice。
//!
//! 快速冒烟：`cargo run --example end_to_end -- --benchmark-seconds 5 --rounds 15 --seed 42`

use my_r2d2::callback_profile::{CallbackRegistry, profile_trace};
use my_r2d2::interface_extractor::{Extractor, FileExtractor, Interface, Kind};
use my_r2d2::payload::Payload;
use my_r2d2::payload_generator::{GeneratorConfig, PayloadGenerator};
use my_r2d2::runtime::state_oracle::{
    BenchmarkBuilder, BenchmarkModel, BenchmarkStateOracle, DeviationThresholds, OracleMode,
    TraceDisposition,
};
use my_r2d2::trace_buffer::TraceReader;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::time::{Duration, Instant};

struct Config {
    rounds: u64,
    seed: u64,
    benchmark_seconds: u64,
    benchmark_model: Option<PathBuf>,
    oracle_mode: OracleMode,
    latency_factor: f64,
    throughput_floor: f64,
    mock_writer: PathBuf,
}

impl Config {
    fn parse(args: impl Iterator<Item = String>) -> Result<Self, String> {
        let mut config = Self {
            rounds: 15,
            seed: 42,
            benchmark_seconds: 7_200,
            benchmark_model: None,
            oracle_mode: OracleMode::JazzyReproduction,
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
                "--benchmark-seconds" => {
                    config.benchmark_seconds = value("--benchmark-seconds")?
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
    active_new_states: u64,
    paper_supported_new_states: u64,
    jazzy_reproduction_new_states: u64,
    invalid: u64,
}

fn execute_payload_round(
    registry: &mut CallbackRegistry,
    mock_writer: &Path,
    payload: &Payload,
    round: u64,
) -> Result<(my_r2d2::callback_profile::CallbackTrace, bool), String> {
    let (exec_sub, sched_sub, skip_timer, size, pub_ts, sub_ts) = mock_params(payload);
    let crash = crashes(payload);

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
    // 轮次分界 marker：mock 每轮独立 shm，marker 主要用于演练 reader 的
    // marker 解析路径（profile 层会跳过该 framing 事件）。
    command.arg("--mark-round").arg(round.to_string());
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

    Ok((trace, !status.success()))
}

fn build_benchmark_model(
    config: &Config,
    interfaces: &[Interface],
) -> Result<BenchmarkModel, String> {
    if let Some(path) = &config.benchmark_model
        && path.exists()
    {
        println!("benchmark: loading model from {}", path.display());
        return BenchmarkModel::load_json(path);
    }
    if config.benchmark_seconds == 0 {
        return Err(
            "benchmark-seconds must be > 0 when no precomputed benchmark model is supplied"
                .to_string(),
        );
    }

    let deadline = Instant::now() + Duration::from_secs(config.benchmark_seconds);
    let mut generator = PayloadGenerator::new(
        interfaces.to_vec(),
        GeneratorConfig::default(),
        config.seed ^ 0xB3A5_E1A0,
    );
    let mut registry = CallbackRegistry::new();
    let mut builder = BenchmarkBuilder::default();
    let mut crashed_samples = 0u64;
    let mut round = 0u64;

    println!(
        "benchmark: sampling for {}s to build callback graph and average benchmarks",
        config.benchmark_seconds
    );
    while Instant::now() < deadline {
        round += 1;
        let payload = generator.next_payload().map_err(|e| e.to_string())?;
        let (trace, crashed) =
            execute_payload_round(&mut registry, &config.mock_writer, &payload, round)?;
        if crashed {
            crashed_samples += 1;
            continue;
        }
        builder.observe(&trace);
    }

    if builder.analyzed_traces() == 0 {
        return Err("benchmark phase collected zero analyzed traces".to_string());
    }
    let model = builder.build();
    println!(
        "benchmark: rounds={} analyzed={} empty={} invalid={} crashed={} edges={} callbacks={}",
        round,
        builder.analyzed_traces(),
        builder.empty_traces(),
        builder.invalid_traces(),
        crashed_samples,
        model.edge_count(),
        model.distinct_callbacks()
    );
    if let Some(path) = &config.benchmark_model {
        model.save_json(path)?;
        println!("benchmark: saved model to {}", path.display());
    }
    Ok(model)
}

fn run_round(
    generator: &mut PayloadGenerator,
    registry: &mut CallbackRegistry,
    oracle: &mut BenchmarkStateOracle,
    mock_writer: &Path,
    round: u64,
    stats: &mut Stats,
) -> Result<(), String> {
    let payload = generator.next_payload().map_err(|e| e.to_string())?;
    let (trace, crashed) = execute_payload_round(registry, mock_writer, &payload, round)?;
    let (exec_sub, sched_sub, skip_timer, _, _, _) = mock_params(&payload);
    let crash = crashes(&payload);
    let verdict = oracle.evaluate(&trace, crashed);
    if verdict.trace == TraceDisposition::Invalid {
        stats.invalid += 1;
    }
    if verdict.paper_supported_new_state {
        stats.paper_supported_new_states += 1;
    }
    if verdict.jazzy_reproduction_new_state {
        stats.jazzy_reproduction_new_states += 1;
    }
    let new_state = verdict.new_state;
    if crashed {
        stats.crashes += 1;
    }
    if new_state {
        stats.active_new_states += 1;
    }
    generator.retain_if_interesting(payload.clone(), oracle);

    let decision = match (crashed, new_state) {
        (true, true) => "crash+new-state",
        (true, false) => "crash",
        (false, true) => "new-state",
        (false, false) => "none",
    };
    let kind = match payload.kind {
        Kind::Topic => "topic",
        Kind::Service => "service",
        Kind::Parameter => "parameter",
    };
    println!(
        "round {round:02} | iface={:>28} {kind:<7} | len={:4} | exec_sub={:3} sched_sub={:2} timer={:<3} crash={:<3} | calls={} msgs={} | decision={decision:<14} | mode={} active={} paper={} jazzy={} | evidence=edge:{} cb:{} msg:{} lat:{} thr:{} | pool={}",
        payload.interface_id,
        payload.serialized.len(),
        exec_sub,
        sched_sub,
        if skip_timer { "no" } else { "yes" },
        if crash { "yes" } else { "no" },
        trace.call_trace.len(),
        trace.msg_trace.len(),
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
                match i.kind {
                    Kind::Topic => "topic",
                    Kind::Service => "service",
                    Kind::Parameter => "parameter",
                }
            ))
            .collect::<Vec<_>>()
            .join(", ")
    );

    let benchmark = match build_benchmark_model(&config, &interfaces) {
        Ok(model) => model,
        Err(message) => {
            eprintln!("end_to_end: benchmark failed: {message}");
            std::process::exit(1);
        }
    };
    let mut generator = PayloadGenerator::new(interfaces, GeneratorConfig::default(), config.seed);
    let mut registry = CallbackRegistry::new();
    let thresholds = DeviationThresholds::new(config.latency_factor, config.throughput_floor);
    let mut oracle = BenchmarkStateOracle::with_mode(benchmark, thresholds, config.oracle_mode);
    let mut stats = Stats {
        crashes: 0,
        active_new_states: 0,
        paper_supported_new_states: 0,
        jazzy_reproduction_new_states: 0,
        invalid: 0,
    };

    println!(
        "loop: rounds={} seed={} benchmark_traces={} oracle_mode={} latency_factor={} throughput_floor={}",
        config.rounds,
        config.seed,
        oracle.benchmark().analyzed_traces,
        oracle.mode().as_str(),
        config.latency_factor,
        config.throughput_floor
    );
    for round in 1..=config.rounds {
        if let Err(message) = run_round(
            &mut generator,
            &mut registry,
            &mut oracle,
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
        "rounds={} crashes={} active_new_states={} paper_supported_new_states={} jazzy_reproduction_new_states={} invalid_traces={} pool_size={}",
        config.rounds,
        stats.crashes,
        stats.active_new_states,
        stats.paper_supported_new_states,
        stats.jazzy_reproduction_new_states,
        stats.invalid,
        generator.pool().len()
    );
    println!(
        "oracle_mode={} callback_graph_edges={} distinct_callbacks={}",
        oracle.mode().as_str(),
        oracle.edge_count(),
        oracle.distinct_callbacks()
    );
}
