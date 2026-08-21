//! R2D2 闭环 × 真实 Jazzy nav2（只插桩 nav2 应用层，见 docs/plan）。
//!
//! 流程（对应论文 Figure 3 的生成与反馈两侧）：
//!
//! 1. dry run：FileExtractor 解析仓库内真实 `sensor_msgs/LaserScan.msg`。
//! 2. 每轮：PayloadGenerator 生成 LaserScan payload → 写文本 payload 文件 →
//!    spawn `r2d2_scan_bridge` 以 20 Hz 向 /scan 发布 2 秒。
//! 3. 插桩后的 nav2_costmap_2d（ObstacleLayer LaserScan 回调）把注册与运行时
//!    事件写进 /dev/shm/r2d2_nav2；本进程实时 drain、profile。
//! 4. BaselineOracle（阶段 F 雏形）判定新执行序列 / 延迟偏差 / 吞吐下降；
//!    costmap 进程组死亡记为 crash；crash 或 new state 的 payload 入池。
//!
//! 需要先构建 nav2_ws（含 r2d2_tracer、r2d2_scan_bridge）：
//!   bash nav2_ws 下 colcon build --packages-select r2d2_tracer r2d2_scan_bridge ...
//!
//! 运行：cargo run --example nav2_costmap_e2e -- --rounds 10 --seed 42

use my_r2d2::callback_profile::{CallbackRegistry, CallbackTrace, profile_trace};
use my_r2d2::interface_extractor::{Extractor, FileExtractor, Interface, Kind};
use my_r2d2::payload::{Payload, Value, ValueTree};
use my_r2d2::payload_generator::{GeneratorConfig, PayloadGenerator};
use my_r2d2::seed_corpus::{Schedule, load_scan_seeds, load_schedules};
use my_r2d2::trace_buffer::TraceReader;
use rand::SeedableRng;
use rand::rngs::StdRng;
use rand::seq::SliceRandom;
use std::collections::{HashMap, HashSet};
use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::thread;
use std::time::{Duration, Instant};

/// 读取环境变量，空值视为未设置，返回默认值。
fn env_or(key: &str, default: &str) -> String {
    std::env::var(key)
        .ok()
        .filter(|v| !v.is_empty())
        .unwrap_or_else(|| default.to_string())
}

/// 读取路径环境变量（空值视为未设置，默认值兜底）。
fn env_path(key: &str, default: &Path) -> PathBuf {
    std::env::var(key)
        .ok()
        .filter(|v| !v.is_empty())
        .map(PathBuf::from)
        .unwrap_or_else(|| default.to_path_buf())
}
const RANGES_PER_SCAN: usize = 180;
/// 每轮最多起几次 scan bridge。bridge 每轮都是全新 DDS participant，发现
/// 偶尔会吃掉整轮或大部分消息；送达不足期望一半时重跑同一 payload/schedule，
/// 不把「没送到」当成「没触发」。
const BRIDGE_ATTEMPTS: u32 = 3;

/// 阶段 F 雏形（与 end_to_end example 相同的三指标判定）。
struct BaselineOracle {
    graph_edges: HashSet<(u64, u64)>,
    latency_sum: HashMap<u64, u64>,
    latency_count: HashMap<u64, u64>,
    throughput_sum: HashMap<u64, f64>,
    throughput_count: HashMap<u64, u64>,
    latency_factor: f64,
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

    fn decide(&mut self, trace: &CallbackTrace, baseline: bool) -> bool {
        let mut new_state = false;
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
        if !baseline {
            for latency in &trace.call_trace {
                if let Some(mean) = self.mean_latency(latency.callback_id)
                    && mean > 0.0
                    && latency.execution_latency as f64 > mean * self.latency_factor
                {
                    new_state = true;
                }
            }
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
            baseline: 2,
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

fn ws_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("nav2_ws")
}

/// Dry run：从真实 LaserScan.msg 提取接口规范。
fn extract_laser_scan(fixtures_root: &Path) -> Result<Interface, String> {
    let extractor = FileExtractor::new(
        vec![fixtures_root.join("sensor_msgs/msg/LaserScan.msg")],
        vec![fixtures_root.to_path_buf()],
    );
    let mut interfaces = extractor.extract().map_err(|e| e.to_string())?;
    interfaces
        .drain(..)
        .find(|i| i.name == "LaserScan")
        .ok_or_else(|| "LaserScan interface not extracted".to_string())
}

fn field_f32(fields: &[ValueTree], index: usize) -> f32 {
    match fields.get(index) {
        Some(ValueTree::Leaf(Value::F32(v))) => *v,
        _ => 0.0,
    }
}

/// payload 值树 → 物理上合法的 LaserScan 参数 + ranges（夹取范围是测试
/// harness 的 reproduction choice，保证 nav2 收到可投影的扫描）。
fn scan_params(payload: &Payload) -> (f32, f32, f32, f32, f32, f32, f32, Vec<f32>) {
    let ValueTree::Nested(fields) = &payload.value else {
        return (0.0, 0.0, 0.01, 0.0, 0.05, 0.01, 12.0, vec![]);
    };
    let angle_min = field_f32(fields, 1).clamp(-std::f32::consts::PI, std::f32::consts::PI);
    let angle_max = field_f32(fields, 2).clamp(angle_min + 0.01, angle_min + std::f32::consts::PI);
    let angle_increment = field_f32(fields, 3).abs().clamp(0.001, 0.35);
    let time_increment = field_f32(fields, 4).abs().min(0.1);
    let scan_time = field_f32(fields, 5).abs().min(0.5);
    let range_min = field_f32(fields, 6).clamp(0.01, 5.0);
    let range_max = field_f32(fields, 7).clamp(range_min + 0.1, 20.0);

    let mut ranges = Vec::new();
    if let Some(ValueTree::Array(items)) = fields.get(8) {
        for item in items {
            if let ValueTree::Leaf(Value::F32(v)) = item {
                ranges.push(*v);
            }
        }
    }
    let cycled: Vec<f32> = if ranges.is_empty() {
        vec![range_max; RANGES_PER_SCAN]
    } else {
        ranges
            .iter()
            .cycle()
            .take(RANGES_PER_SCAN)
            .copied()
            .collect()
    };
    let clamped = cycled
        .iter()
        .map(|r| r.clamp(range_min, range_max))
        .collect();
    (
        angle_min,
        angle_max,
        angle_increment,
        time_increment,
        scan_time,
        range_min,
        range_max,
        clamped,
    )
}

fn write_payload_file(path: &Path, payload: &Payload) {
    let (amin, amax, ainc, tinc, stime, rmin, rmax, ranges) = scan_params(payload);
    let mut text = format!("{amin} {amax} {ainc} {tinc} {stime} {rmin} {rmax}\n");
    for (i, r) in ranges.iter().enumerate() {
        if i > 0 {
            text.push(' ');
        }
        text.push_str(&format!("{r}"));
    }
    text.push('\n');
    fs::write(path, text).expect("write payload file");
}

fn clean_env(mut command: Command, domain_id: &str) -> Command {
    command
        .env("ROS_DOMAIN_ID", domain_id)
        .env_remove("LD_PRELOAD")
        .env_remove("ASAN_OPTIONS")
        .env_remove("TSAN_OPTIONS")
        .env_remove("COLCON_CURRENT_PREFIX");
    command
}

fn stack_alive(stack: &mut std::process::Child) -> bool {
    // try_wait 会回收僵尸进程；kill -0 对僵尸返回成功，不能用于存活判定。
    matches!(stack.try_wait(), Ok(None))
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

/// 按 bridge 的发布循环推算一轮期望消息数：tick 数 × burst，受
/// `max_publishes` 封顶（0 表示不封顶），与 r2d2_scan_bridge 的
/// `total_ticks`/`published` 逻辑保持一致。
fn expected_publishes(rate_hz: f64, duration_sec: f64, burst: u32, max_publishes: u64) -> u64 {
    let ticks = (rate_hz * duration_sec) as u64;
    let uncapped = ticks.saturating_mul(burst as u64);
    if max_publishes > 0 {
        uncapped.min(max_publishes)
    } else {
        uncapped
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
    // 路径全部可由环境变量覆盖，默认取编译期仓库根 + 本机 ROS 安装；
    // 仓库挪位置后重编译即可，或显式设置 R2D2_NAV2_WS / R2D2_WS_ROOT。
    let ws_root = std::env::var("R2D2_WS_ROOT")
        .ok()
        .filter(|v| !v.is_empty())
        .map(PathBuf::from);
    let nav2_ws = env_path(
        "R2D2_NAV2_WS",
        &ws_root
            .map(|root| root.join("nav2_ws"))
            .unwrap_or_else(ws_dir),
    );
    let ros_setup = env_path("R2D2_ROS_SETUP", Path::new("/opt/ros/jazzy/setup.bash"));
    let costmap_params = env_path("R2D2_COSTMAP_PARAMS", &nav2_ws.join("costmap_params.yaml"));
    let shm_path = env_path("R2D2_SHM_PATH", Path::new("/dev/shm/r2d2_nav2"));
    let domain_id = env_or("ROS_DOMAIN_ID", "190");

    let stack_script = nav2_ws.join("launch_stack.sh");
    let payload_file = nav2_ws.join("payload_round.txt");
    let install_setup = nav2_ws.join("install/setup.bash");
    if !install_setup.exists() {
        eprintln!(
            "nav2_costmap_e2e: {} missing; build nav2_ws first",
            install_setup.display()
        );
        std::process::exit(1);
    }

    let fixtures_root = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/ros_interfaces");
    let interface = match extract_laser_scan(&fixtures_root) {
        Ok(interface) => interface,
        Err(message) => {
            eprintln!("nav2_costmap_e2e: dry run failed: {message}");
            std::process::exit(1);
        }
    };
    println!(
        "dry run: extracted interface {} ({}) with {} top-level fields",
        interface.name,
        if interface.kind == Kind::Topic {
            "topic"
        } else {
            "service"
        },
        interface.fields.len()
    );

    // 1. 启动插桩 costmap 栈（setsid 使其自成一个进程组，便于整体终止）。
    let mut stack_command = Command::new("setsid");
    stack_command
        .arg("bash")
        .arg(&stack_script)
        .env("ROS_DOMAIN_ID", &domain_id)
        .env("R2D2_ROS_SETUP", &ros_setup)
        .env("R2D2_NAV2_WS", &nav2_ws)
        .env("R2D2_COSTMAP_PARAMS", &costmap_params)
        .env("R2D2_SHM_PATH", &shm_path)
        .env_remove("LD_PRELOAD")
        .env_remove("ASAN_OPTIONS")
        .env_remove("COLCON_CURRENT_PREFIX");
    if let Some(log_dir) = &config.tsan_log_dir {
        let _ = fs::create_dir_all(log_dir);
        // report_signal_unsafe=0：本 harness 有意在轮间用 SIGUSR1 触发
        // __gcov_dump()（见 r2d2_tracer），该类别会在每轮产生海量保守告警。
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
    let mut stack = stack_command.spawn().expect("spawn costmap stack");
    let stack_pid = stack.id();
    println!("stack leader pid = {stack_pid}");

    // 2. 等待 costmap 注册回调并 drain 注册记录（超时 30s）。
    let _ = fs::remove_file(&shm_path);
    let mut reader = loop {
        if let Ok(reader) = TraceReader::open(&shm_path) {
            break reader;
        }
        if !stack_alive(&mut stack) {
            eprintln!("nav2_costmap_e2e: costmap stack died during startup");
            std::process::exit(1);
        }
        thread::sleep(Duration::from_millis(500));
    };
    let mut registry = CallbackRegistry::new();
    let mut registrations = 0usize;
    let deadline = Instant::now() + Duration::from_secs(30);
    while registrations == 0 && Instant::now() < deadline {
        match reader.drain_registration() {
            Ok(drain) => {
                registrations = drain.events.len();
                registry.ingest(&drain);
            }
            Err(error) => eprintln!("registration drain error: {error}"),
        }
        if registrations == 0 {
            thread::sleep(Duration::from_secs(1));
        }
    }
    let infos = registry.callback_infos();
    println!(
        "startup: {} registration records, {} complete callbacks: {}",
        registrations,
        infos.len(),
        infos
            .iter()
            .map(|i| format!("{} [{:?}]", i.name, i.callback_type))
            .collect::<Vec<_>>()
            .join(", ")
    );

    // 3. 主循环。先在生成器构造前载入种子语料（可选），再用预填后的 pool
    // 从真实扫描开始变异；发布时序种子决定每轮的 rate/duration/burst。
    let mut seed_preload: Vec<Payload> = Vec::new();
    let mut schedules: Vec<(String, Schedule)> = Vec::new();
    if let Some(seed_dir) = &config.seed_dir {
        match load_scan_seeds(&seed_dir.join("scans"), &interface) {
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
    let mut generator =
        PayloadGenerator::new(vec![interface], GeneratorConfig::default(), config.seed);
    for seed in seed_preload {
        generator.pool_mut().push(seed);
    }
    let mut oracle = BaselineOracle::new(config.latency_factor, config.throughput_floor);
    println!(
        "loop: rounds={} seed={} baseline_rounds={} latency_factor={} throughput_floor={}",
        config.rounds, config.seed, config.baseline, config.latency_factor, config.throughput_floor
    );
    let mut crashes = 0u64;
    let mut new_states = 0u64;
    let mut invalid = 0u64;
    let mut empty_rounds = 0u64;
    let mut prev_branches: u64 = 0;
    if let Some(lcov_root) = &config.lcov_dir {
        let _ = fs::create_dir_all(lcov_root);
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
        write_payload_file(&payload_file, &payload);

        let schedule_slot = schedules.get((round as usize - 1) % schedules.len().max(1));
        let (rate_hz, duration_sec, burst, burst_gap, max_publishes, stamp_mode, sched_name) =
            match schedule_slot {
                Some((name, schedule)) => (
                    1000.0 / schedule.period_ms.max(1) as f64,
                    schedule.duration_sec.clamp(0.1, 300.0),
                    schedule.burst_count.max(1),
                    schedule.burst_gap_ms,
                    schedule.max_publishes,
                    schedule.stamp_mode.clone(),
                    Some(name.clone()),
                ),
                None => (
                    config.bridge_rate_hz as f64,
                    config.round_duration_sec,
                    1,
                    0,
                    0,
                    "now".to_string(),
                    None,
                ),
            };

        // setarch -R 关闭 ASLR：TSAN 构建下不关会在内核 6.x 高熵 ASLR 上
        // FATAL（gcc）或静默漏报（clang），与 launch_stack.sh 的处理一致。
        let bridge_command = format!(
            "source {} && source {} && \
             export ROS_DOMAIN_ID={domain_id} && \
             setarch x86_64 -R ros2 run r2d2_scan_bridge r2d2_scan_bridge {} {rate_hz} {duration_sec} \
             {burst} {burst_gap} {max_publishes} {stamp_mode}",
            ros_setup.display(),
            install_setup.display(),
            payload_file.display(),
        );
        // 每轮 bridge 都是全新的 DDS participant，发现偶尔会吃掉整轮消息
        // （300 轮战役里有 2 轮 0 消息、16 轮不足一半）。送达不足期望一半
        // 时重跑同一 payload/schedule，避免把「没送到」当成「没触发」。
        let expected_msgs = expected_publishes(rate_hz, duration_sec, burst, max_publishes);
        let mut trace = None;
        let mut last_delivered = 0u64;
        for attempt in 0..BRIDGE_ATTEMPTS {
            let bridge_status = clean_env(Command::new("bash"), &domain_id)
                .args(["-c", &bridge_command])
                .status();
            match bridge_status {
                Ok(status) if !status.success() => {
                    eprintln!("round {round}: scan bridge exited with {status}");
                }
                Err(error) => eprintln!("round {round}: spawn scan bridge failed: {error}"),
                Ok(_) => {}
            }
            let drained = match reader.drain_runtime() {
                Ok(drain) => profile_trace(&registry, &drain),
                Err(error) => {
                    eprintln!("round {round}: runtime drain failed: {error}");
                    break;
                }
            };
            let delivered = drained.msg_trace.len() as u64;
            last_delivered = delivered;
            if expected_msgs > 0
                && delivered * 2 < expected_msgs
                && stack_alive(&mut stack)
                && attempt + 1 < BRIDGE_ATTEMPTS
            {
                eprintln!(
                    "round {round}: {delivered}/{} messages delivered (attempt {}), respawning bridge",
                    expected_msgs,
                    attempt + 1
                );
                continue;
            }
            trace = Some(drained);
            break;
        }
        let Some(trace) = trace else {
            continue;
        };
        if expected_msgs > 0 && last_delivered * 2 < expected_msgs && !trace.msg_trace.is_empty() {
            eprintln!(
                "round {round}: only {last_delivered}/{} messages delivered after {} attempts",
                expected_msgs, BRIDGE_ATTEMPTS
            );
        }

        let crashed = !stack_alive(&mut stack);
        let empty = trace.call_trace.is_empty() && trace.msg_trace.is_empty();
        let mut new_state = false;
        if empty && !crashed {
            empty_rounds += 1;
        } else if trace.valid_for_state_analysis() {
            new_state = oracle.decide(&trace, round <= config.baseline);
        } else {
            invalid += 1;
        }
        if crashed {
            crashes += 1;
        }
        if new_state {
            new_states += 1;
        }
        if crashed || new_state {
            generator.pool_mut().push(payload.clone());
        }

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
            "round {round:02} | len={:4} | calls={} msgs={} | exec=[{}] thr=[{}] MB/s | decision={decision:<14} | pool={} | sched={}",
            payload.serialized.len(),
            trace.call_trace.len(),
            trace.msg_trace.len(),
            execs.join(","),
            throughputs.join(","),
            generator.pool().len(),
            sched_name.as_deref().unwrap_or("-"),
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
                "{{\n  \"round\": {round},\n  \"decision\": \"{decision}\",\n  \"calls\": {},\n  \"msgs\": {},\n  \"pool_size\": {},\n  \"crash_or_hang\": {},\n  \"coverage_ok\": {coverage_ok},\n  \"branch_covered_total\": {covered},\n  \"branch_covered_increase\": {increase}\n}}\n",
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
    let _ = Command::new("kill")
        .args(["--", "-TERM", &format!("-{stack_pid}")])
        .status();
    thread::sleep(Duration::from_secs(2));
    let _ = Command::new("kill")
        .args(["--", "-KILL", &format!("-{stack_pid}")])
        .status();
    let _ = fs::remove_file(&shm_path);
    let _ = fs::remove_file(shm_path.with_extension("pid"));
    let _ = fs::remove_file(&payload_file);

    println!("\n=== summary ===");
    println!(
        "rounds={} crashes={} new_states={} invalid_traces={} empty_rounds={} pool_size={}",
        config.rounds,
        crashes,
        new_states,
        invalid,
        empty_rounds,
        generator.pool().len()
    );
    println!(
        "callback_graph_edges={} distinct_callbacks={}",
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
                    "{{\n  \"rounds\": {},\n  \"seed\": {},\n  \"crashes\": {},\n  \"new_states\": {},\n  \"invalid_traces\": {},\n  \"empty_rounds\": {},\n  \"pool_size\": {},\n  \"callback_graph_edges\": {},\n  \"distinct_callbacks\": {},\n  \"coverage_ok\": true,\n  \"branch_covered_total\": {covered}\n}}\n",
                    config.rounds,
                    config.seed,
                    crashes,
                    new_states,
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
    use super::{branch_totals_from_summary, cumulative_branches, expected_publishes};

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

    #[test]
    fn expected_publishes_matches_bridge_logic() {
        // 20 Hz × 20 s × burst 2 = 800，受 max_publishes=120 封顶。
        assert_eq!(expected_publishes(20.0, 20.0, 2, 120), 120);
        // 55.6 Hz × 13 s ≈ 722 tick × burst 2，受封顶。
        assert_eq!(expected_publishes(1000.0 / 18.0, 13.0, 2, 120), 120);
        // max_publishes=0 不封顶：20 Hz × 2 s × burst 1 = 40。
        assert_eq!(expected_publishes(20.0, 2.0, 1, 0), 40);
        // 封顶小于 tick 数时不溢出（burst 上限先取 min）。
        assert_eq!(expected_publishes(20.0, 2.0, 2, 40), 40);
    }
}
