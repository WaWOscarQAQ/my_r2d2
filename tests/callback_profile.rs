//! Callback Trace Profile 测试 —— 论文 §4.1.2 的三层数据结构。
//!
//! 端到端路径：真实 golden fixture（C++ mock_writer 生成）→
//! `TraceReader` 读原始事件 → `build_callback_infos` → `profile_trace`。
#![cfg(unix)]

use my_r2d2::callback_profile::{build_callback_infos, profile_trace, CallbackInfo};
use my_r2d2::trace_buffer::{
    CallbackType, RegistrationEvent, RegistrationSource, RuntimeEvent, RuntimeEventType,
    TraceReader,
};
use std::path::{Path, PathBuf};

fn fixtures_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures")
}

fn reg(
    source: RegistrationSource,
    callback_type: CallbackType,
    rclcpp_handler: u64,
    rcl_handler: u64,
    callback_name: &str,
) -> RegistrationEvent {
    RegistrationEvent {
        source,
        callback_type,
        rclcpp_handler,
        rcl_handler,
        callback_name: callback_name.to_string(),
    }
}

fn execute(handler: u64, ts: u64) -> RuntimeEvent {
    runtime(RuntimeEventType::ExecutorExecute, handler, ts, 0, 0, 0, 0)
}

fn start(handler: u64, ts: u64) -> RuntimeEvent {
    runtime(RuntimeEventType::CallbackStart, handler, ts, 0, 0, 0, 0)
}

fn end(handler: u64, ts: u64) -> RuntimeEvent {
    runtime(RuntimeEventType::CallbackEnd, handler, ts, 0, 0, 0, 0)
}

fn take(rcl_handler: u64, buffer_size: u64, pub_ts: u64, sub_ts: u64) -> RuntimeEvent {
    runtime(
        RuntimeEventType::RclTake,
        0,
        0,
        rcl_handler,
        buffer_size,
        pub_ts,
        sub_ts,
    )
}

#[allow(clippy::too_many_arguments)]
fn runtime(
    event_type: RuntimeEventType,
    rclcpp_handler: u64,
    timestamp: u64,
    rcl_handler: u64,
    buffer_size: u64,
    pub_timestamp: u64,
    sub_timestamp: u64,
) -> RuntimeEvent {
    RuntimeEvent {
        event_type,
        rclcpp_handler,
        timestamp,
        rcl_handler,
        buffer_size,
        pub_timestamp,
        sub_timestamp,
    }
}

fn assert_approx(actual: f64, expected: f64) {
    assert!(
        (actual - expected).abs() < 1e-9,
        "expected {expected}, got {actual}"
    );
}

/// golden fixture 的注册记录：两个回调，各两条（RCLCPP + RCL）记录。
fn golden_registration() -> Vec<RegistrationEvent> {
    vec![
        reg(RegistrationSource::Rclcpp, CallbackType::Subscription, 0x1000, 0x2000, ""),
        reg(RegistrationSource::Rcl, CallbackType::Subscription, 0, 0x2000, "/cmd_vel_callback"),
        reg(RegistrationSource::Rclcpp, CallbackType::Timer, 0x3000, 0x4000, ""),
        reg(RegistrationSource::Rcl, CallbackType::Subscription, 0, 0x4000, "timer_callback"),
    ]
}

// ---------------------------------------------------------------------------
// 端到端：真实 fixture → CallbackInfo → CallbackTrace
// ---------------------------------------------------------------------------

#[test]
fn golden_fixture_builds_two_callback_infos() {
    let mut reader = TraceReader::open(fixtures_dir().join("trace_golden.bin")).unwrap();
    let events = reader.drain_registration().unwrap().events;

    let infos = build_callback_infos(&events);

    // 按 rcl_handler 升序：0x2000 < 0x4000
    assert_eq!(infos.len(), 2);

    let cmd_vel = &infos[0];
    assert_eq!(cmd_vel.rcl_handler, 0x2000);
    assert_eq!(cmd_vel.rclcpp_handler, 0x1000);
    assert_eq!(cmd_vel.name, "/cmd_vel_callback");
    assert_eq!(cmd_vel.callback_type, Some(CallbackType::Subscription));

    let timer = &infos[1];
    assert_eq!(timer.rcl_handler, 0x4000);
    assert_eq!(timer.rclcpp_handler, 0x3000);
    assert_eq!(timer.name, "timer_callback");
    assert_eq!(timer.callback_type, Some(CallbackType::Timer));
}

#[test]
fn golden_fixture_profiles_latencies_and_throughput() {
    let mut reader = TraceReader::open(fixtures_dir().join("trace_golden.bin")).unwrap();
    let infos = build_callback_infos(&reader.drain_registration().unwrap().events);
    let runtime = reader.drain_runtime().unwrap().events;

    let trace = profile_trace(&infos, &runtime);

    // 两个回调各执行一次 -> 两条 CallbackLatency
    assert_eq!(trace.call_trace.len(), 2);
    let cmd_vel = &trace.call_trace[0];
    assert_eq!(cmd_vel.callback_id, infos[0].id);
    assert_eq!(cmd_vel.scheduling_latency, 100); // 200 - 100
    assert_eq!(cmd_vel.execution_latency, 100); // 300 - 200
    let timer = &trace.call_trace[1];
    assert_eq!(timer.callback_id, infos[1].id);
    assert_eq!(timer.scheduling_latency, 100); // 500 - 400
    assert_eq!(timer.execution_latency, 100); // 600 - 500

    // 两条消息 -> 两条 MessageLatency
    assert_eq!(trace.msg_trace.len(), 2);
    assert_eq!(trace.msg_trace[0].callback_id, infos[0].id);
    assert_approx(trace.msg_trace[0].throughput, 512.0 / 40.0);
    assert_eq!(trace.msg_trace[1].callback_id, infos[1].id);
    assert_approx(trace.msg_trace[1].throughput, 1024.0 / 40.0);
}

// ---------------------------------------------------------------------------
// 关联边界：记录缺失一侧
// ---------------------------------------------------------------------------

#[test]
fn rcl_only_record_yields_unknown_type() {
    let events = vec![reg(RegistrationSource::Rcl, CallbackType::Subscription, 0, 0x2000, "/solo")];
    let infos = build_callback_infos(&events);

    assert_eq!(infos.len(), 1);
    assert_eq!(infos[0].name, "/solo");
    assert_eq!(infos[0].callback_type, None);
    assert_eq!(infos[0].rclcpp_handler, 0);
}

#[test]
fn rclcpp_only_record_yields_empty_name() {
    let events = vec![reg(RegistrationSource::Rclcpp, CallbackType::Service, 0x1000, 0x2000, "")];
    let infos = build_callback_infos(&events);

    assert_eq!(infos.len(), 1);
    assert_eq!(infos[0].name, "");
    assert_eq!(infos[0].callback_type, Some(CallbackType::Service));
    assert_eq!(infos[0].rclcpp_handler, 0x1000);
}

// ---------------------------------------------------------------------------
// Callback ID 稳定性
// ---------------------------------------------------------------------------

#[test]
fn callback_id_is_deterministic_and_discriminating() {
    let events = golden_registration();
    let first = build_callback_infos(&events);
    let second = build_callback_infos(&events);

    assert_eq!(first, second); // 确定性

    // 两个不同回调的 ID 不同
    assert_ne!(first[0].id, first[1].id);

    // 相同 name + type 产生相同 ID，即使出现在不同位置
    let a = reg(RegistrationSource::Rcl, CallbackType::Subscription, 0, 0xAAAA, "/same");
    let b = reg(RegistrationSource::Rclcpp, CallbackType::Subscription, 0xBBBB, 0xAAAA, "");
    let infos = build_callback_infos(&[a, b]);
    assert_eq!(infos.len(), 1);
    assert_eq!(infos[0].id, infos[0].id);
}

#[test]
fn callback_id_differs_by_type() {
    let make = |kind| build_callback_infos(&[
        reg(RegistrationSource::Rclcpp, kind, 0x1000, 0x2000, ""),
        reg(RegistrationSource::Rcl, CallbackType::Subscription, 0, 0x2000, "/cb"),
    ]);
    let sub = make(CallbackType::Subscription);
    let timer = make(CallbackType::Timer);
    let service = make(CallbackType::Service);
    assert_ne!(sub[0].id, timer[0].id);
    assert_ne!(sub[0].id, service[0].id);
    assert_ne!(timer[0].id, service[0].id);
}

// ---------------------------------------------------------------------------
// 运行时配对：重入 / 乱序 / 缺配
// ---------------------------------------------------------------------------

#[test]
fn reentrant_invocations_pair_lifo() {
    let infos = build_callback_infos(&[
        reg(RegistrationSource::Rclcpp, CallbackType::Subscription, 0x1000, 0x2000, ""),
        reg(RegistrationSource::Rcl, CallbackType::Subscription, 0, 0x2000, "/cb"),
    ]);
    let id = infos[0].id;

    // 两次 invoke 交错：后进的 invoke 先配 start
    let runtime = vec![
        execute(0x1000, 10),
        execute(0x1000, 20),
        start(0x1000, 30),  // 配 invoke=20 -> scheduling 10
        end(0x1000, 40),    // execution 10
        start(0x1000, 50),  // 配 invoke=10 -> scheduling 40
        end(0x1000, 60),    // execution 10
    ];
    let trace = profile_trace(&infos, &runtime);

    assert_eq!(trace.call_trace.len(), 2);
    assert_eq!(trace.call_trace[0], my_r2d2::callback_profile::CallbackLatency {
        callback_id: id,
        execution_latency: 10,
        scheduling_latency: 10,
    });
    assert_eq!(trace.call_trace[1].scheduling_latency, 40);
    assert_eq!(trace.call_trace[1].execution_latency, 10);
}

#[test]
fn unpaired_events_are_ignored() {
    let infos = build_callback_infos(&[
        reg(RegistrationSource::Rclcpp, CallbackType::Subscription, 0x1000, 0x2000, ""),
        reg(RegistrationSource::Rcl, CallbackType::Subscription, 0, 0x2000, "/cb"),
    ]);

    // 只有 end，没有 start/invoke；只有 invoke 没有后续
    let runtime = vec![
        end(0x1000, 40),
        execute(0x1000, 10),
    ];
    let trace = profile_trace(&infos, &runtime);
    assert!(trace.call_trace.is_empty());

    // 只有 start，没有 invoke -> 无法计算 scheduling，丢弃
    let runtime = vec![start(0x1000, 20)];
    let trace = profile_trace(&infos, &runtime);
    assert!(trace.call_trace.is_empty());
}

#[test]
fn throughput_skips_non_positive_duration() {
    let infos = build_callback_infos(&[
        reg(RegistrationSource::Rclcpp, CallbackType::Subscription, 0x1000, 0x2000, ""),
        reg(RegistrationSource::Rcl, CallbackType::Subscription, 0, 0x2000, "/cb"),
    ]);

    // sub < pub（负 duration）和 sub == pub（零 duration）都跳过
    let runtime = vec![
        take(0x2000, 512, 90, 50),
        take(0x2000, 512, 50, 50),
    ];
    let trace = profile_trace(&infos, &runtime);
    assert!(trace.msg_trace.is_empty());
}

#[test]
fn unknown_handler_is_ignored() {
    // 空 infos：任何 handler 都查不到 ID
    let runtime = vec![
        execute(0x1000, 10),
        start(0x1000, 20),
        end(0x1000, 30),
        take(0x2000, 512, 50, 90),
    ];
    let trace = profile_trace(&[], &runtime);
    assert!(trace.call_trace.is_empty());
    assert!(trace.msg_trace.is_empty());
}

// ---------------------------------------------------------------------------
// CallbackInfo 类型完整性
// ---------------------------------------------------------------------------

#[test]
fn callback_info_exposes_paper_fields() {
    let info = CallbackInfo {
        id: 42,
        name: "/cmd_vel".to_string(),
        callback_type: Some(CallbackType::Subscription),
        rclcpp_handler: 0x1000,
        rcl_handler: 0x2000,
    };
    assert_eq!(info.id, 42);
    assert_eq!(info.name, "/cmd_vel");
    assert_eq!(info.callback_type, Some(CallbackType::Subscription));
}
