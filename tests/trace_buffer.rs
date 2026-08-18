//! Reader tests over the C++ tracer layout: golden fixtures produced by
//! `tracer/build/mock_writer`, plus a live round trip when the binary
//! exists. The fixtures are generated with:
//!
//! ```text
//! cmake -S tracer -B tracer/build && cmake --build tracer/build
//! tracer/build/mock_writer fixture_golden --fixture tests/fixtures/trace_golden.bin --reg-capacity 8 --runtime-capacity 8 --cleanup
//! tracer/build/mock_writer fixture_overflow --fixture tests/fixtures/trace_overflow.bin --reg-capacity 4 --runtime-capacity 4 --overflow --cleanup
//! ```
#![cfg(unix)]

use my_r2d2::trace_buffer::{
    CallbackType, RegistrationEvent, RegistrationSource, RuntimeEvent, RuntimeEventType,
    TraceReader,
};
use std::path::{Path, PathBuf};

fn fixtures_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures")
}

fn registration(
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

/// The canonical deterministic sequence written by `write_sequence()` in
/// mock_writer.cpp. RCL records leave `callback_type` zero because the RCL
/// registration tracer does not record a type.
fn expected_registration() -> Vec<RegistrationEvent> {
    vec![
        registration(
            RegistrationSource::Rclcpp,
            CallbackType::Subscription,
            0x1000,
            0x2000,
            "",
        ),
        registration(RegistrationSource::Rcl, CallbackType::Subscription, 0, 0x2000, "/cmd_vel_callback"),
        registration(RegistrationSource::Rclcpp, CallbackType::Timer, 0x3000, 0x4000, ""),
        registration(RegistrationSource::Rcl, CallbackType::Subscription, 0, 0x4000, "timer_callback"),
    ]
}

fn expected_runtime() -> Vec<RuntimeEvent> {
    let execute = |handler: u64, timestamp: u64| RuntimeEvent {
        event_type: RuntimeEventType::ExecutorExecute,
        rclcpp_handler: handler,
        timestamp,
        rcl_handler: 0,
        buffer_size: 0,
        pub_timestamp: 0,
        sub_timestamp: 0,
    };
    let start = |handler: u64, timestamp: u64| RuntimeEvent {
        event_type: RuntimeEventType::CallbackStart,
        rclcpp_handler: handler,
        timestamp,
        rcl_handler: 0,
        buffer_size: 0,
        pub_timestamp: 0,
        sub_timestamp: 0,
    };
    let end = |handler: u64, timestamp: u64| RuntimeEvent {
        event_type: RuntimeEventType::CallbackEnd,
        rclcpp_handler: handler,
        timestamp,
        rcl_handler: 0,
        buffer_size: 0,
        pub_timestamp: 0,
        sub_timestamp: 0,
    };
    let take = |rcl_handler: u64, buffer_size: u64, pub_ts: u64, sub_ts: u64| RuntimeEvent {
        event_type: RuntimeEventType::RclTake,
        rclcpp_handler: 0,
        timestamp: 0,
        rcl_handler,
        buffer_size,
        pub_timestamp: pub_ts,
        sub_timestamp: sub_ts,
    };
    vec![
        execute(0x1000, 100),
        start(0x1000, 200),
        end(0x1000, 300),
        take(0x2000, 512, 50, 90),
        execute(0x3000, 400),
        start(0x3000, 500),
        end(0x3000, 600),
        take(0x4000, 1024, 350, 390),
    ]
}

#[test]
fn golden_fixture_parses_registration_events() {
    let mut reader = TraceReader::open(fixtures_dir().join("trace_golden.bin")).unwrap();
    let drain = reader.drain_registration().unwrap();

    assert_eq!(drain.missed, 0);
    assert_eq!(drain.events, expected_registration());
    assert_eq!(reader.registration_overflow().unwrap(), 0);

    // The RCLCPP and RCL records of one callback share rcl_handler, which
    // is the association required by the stage C acceptance criteria.
    assert_eq!(drain.events[0].rcl_handler, drain.events[1].rcl_handler);
    assert_eq!(drain.events[1].callback_name, "/cmd_vel_callback");
    assert_eq!(drain.events[1].callback_type, CallbackType::Subscription);
}

#[test]
fn golden_fixture_parses_runtime_events() {
    let mut reader = TraceReader::open(fixtures_dir().join("trace_golden.bin")).unwrap();
    let drain = reader.drain_runtime().unwrap();

    assert_eq!(drain.missed, 0);
    assert_eq!(drain.events, expected_runtime());
    assert_eq!(reader.runtime_overflow().unwrap(), 0);

    // Invoke -> start -> end ordering is preserved with fixed timestamps.
    let invoke = &drain.events[0];
    let start = &drain.events[1];
    let end = &drain.events[2];
    assert_eq!(invoke.event_type, RuntimeEventType::ExecutorExecute);
    assert_eq!(start.event_type, RuntimeEventType::CallbackStart);
    assert_eq!(end.event_type, RuntimeEventType::CallbackEnd);
    assert!(invoke.timestamp < start.timestamp && start.timestamp < end.timestamp);

    // A message keeps its buffer size and pub/sub timestamps together.
    let take = &drain.events[3];
    assert_eq!(take.event_type, RuntimeEventType::RclTake);
    assert_eq!(take.buffer_size, 512);
    assert_eq!(take.pub_timestamp, 50);
    assert_eq!(take.sub_timestamp, 90);
}

#[test]
fn overflow_fixture_reports_overwritten_records() {
    let mut reader = TraceReader::open(fixtures_dir().join("trace_overflow.bin")).unwrap();

    let reg = reader.drain_registration().unwrap();
    // Six callbacks x two records into a four-record ring.
    assert_eq!(reg.events.len(), 4);
    assert_eq!(reg.missed, 8);
    assert_eq!(reader.registration_overflow().unwrap(), 8);
    assert_eq!(reg.events[0].source, RegistrationSource::Rclcpp);
    assert_eq!(reg.events[0].rclcpp_handler, 0xa004);
    assert_eq!(reg.events[0].rcl_handler, 0xa104);

    let run = reader.drain_runtime().unwrap();
    assert_eq!(run.events.len(), 4);
    assert_eq!(run.missed, 2);
    assert_eq!(reader.runtime_overflow().unwrap(), 2);
    assert_eq!(run.events[0].event_type, RuntimeEventType::ExecutorExecute);
    assert_eq!(run.events[0].rclcpp_handler, 0xa002);
    assert_eq!(run.events[0].timestamp, 1020);
}

#[test]
fn live_round_trip_with_mock_writer() {
    let binary = std::env::var_os("TRACER_MOCK_WRITER")
        .map(PathBuf::from)
        .unwrap_or_else(|| {
            Path::new(env!("CARGO_MANIFEST_DIR")).join("tracer/build/mock_writer")
        });
    if !binary.exists() {
        eprintln!(
            "skipping live round trip: {} not found (build tracer/ with cmake first)",
            binary.display()
        );
        return;
    }

    let name = format!("my_r2d2_mock_{}", std::process::id());
    let status = std::process::Command::new(&binary)
        .arg(&name)
        .status()
        .expect("failed to run mock_writer");
    assert!(status.success(), "mock_writer exited with {status}");

    let shm_path = format!("/dev/shm/{name}");
    let mut reader = TraceReader::open(&shm_path).unwrap();
    assert_eq!(
        reader.drain_registration().unwrap().events,
        expected_registration()
    );
    assert_eq!(reader.drain_runtime().unwrap().events, expected_runtime());
    let _ = std::fs::remove_file(&shm_path);
}
