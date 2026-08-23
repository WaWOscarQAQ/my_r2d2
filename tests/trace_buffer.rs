//! Reader tests over the C++ tracer layout: golden fixtures produced by
//! `tracer/build/mock_writer`, plus a live round trip when the binary
//! exists. The fixtures are generated with:
//!
//! ```text
//! cmake -S tracer -B tracer/build && cmake --build tracer/build
//! tracer/build/mock_writer fixture_golden --fixture tests/fixtures/trace_golden.bin --reg-capacity 8 --runtime-capacity 16 --cleanup
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

fn mock_writer_binary() -> PathBuf {
    std::env::var_os("TRACER_MOCK_WRITER")
        .map(PathBuf::from)
        .unwrap_or_else(|| Path::new(env!("CARGO_MANIFEST_DIR")).join("tracer/build/mock_writer"))
}

fn registration(
    source: RegistrationSource,
    callback_type: CallbackType,
    rclcpp_handler: u64,
    rcl_handler: u64,
    callback_name: &str,
    callback_namespace: &str,
) -> RegistrationEvent {
    RegistrationEvent {
        source,
        callback_type,
        rclcpp_handler,
        rcl_handler,
        callback_name: callback_name.to_string(),
        callback_name_truncated: false,
        callback_namespace: callback_namespace.to_string(),
        callback_namespace_truncated: false,
    }
}

/// The canonical deterministic sequence written by `write_sequence()` in
/// mock_writer.cpp. RCL records leave `callback_type` zero because the RCL
/// registration tracer does not record a type; only the RCL records carry
/// the callback namespace (paper §4.1.1 registration attribute).
fn expected_registration() -> Vec<RegistrationEvent> {
    vec![
        registration(
            RegistrationSource::Rclcpp,
            CallbackType::Subscription,
            0x1000,
            0x2000,
            "",
            "",
        ),
        registration(
            RegistrationSource::Rcl,
            CallbackType::Subscription,
            0,
            0x2000,
            "/cmd_vel_callback",
            "/robot",
        ),
        registration(
            RegistrationSource::Rclcpp,
            CallbackType::Timer,
            0x3000,
            0x4000,
            "",
            "",
        ),
        registration(
            RegistrationSource::Rcl,
            CallbackType::Subscription,
            0,
            0x4000,
            "timer_callback",
            "/robot",
        ),
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
        round_id: 0,
    };
    let start = |handler: u64, timestamp: u64| RuntimeEvent {
        event_type: RuntimeEventType::CallbackStart,
        rclcpp_handler: handler,
        timestamp,
        rcl_handler: 0,
        buffer_size: 0,
        pub_timestamp: 0,
        sub_timestamp: 0,
        round_id: 0,
    };
    let end = |handler: u64, timestamp: u64| RuntimeEvent {
        event_type: RuntimeEventType::CallbackEnd,
        rclcpp_handler: handler,
        timestamp,
        rcl_handler: 0,
        buffer_size: 0,
        pub_timestamp: 0,
        sub_timestamp: 0,
        round_id: 0,
    };
    let take = |rcl_handler: u64, buffer_size: u64, pub_ts: u64, sub_ts: u64| RuntimeEvent {
        event_type: RuntimeEventType::RclTake,
        rclcpp_handler: 0,
        timestamp: 0,
        rcl_handler,
        buffer_size,
        pub_timestamp: pub_ts,
        sub_timestamp: sub_ts,
        round_id: 0,
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
        RuntimeEvent {
            event_type: RuntimeEventType::RoundBoundary,
            rclcpp_handler: 0,
            timestamp: 700,
            rcl_handler: 0,
            buffer_size: 0,
            pub_timestamp: 0,
            sub_timestamp: 0,
            round_id: 1,
        },
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

    // The trailing round boundary marker carries its round id.
    let marker = drain.events.last().unwrap();
    assert_eq!(marker.event_type, RuntimeEventType::RoundBoundary);
    assert_eq!(marker.round_id, 1);
    assert_eq!(marker.timestamp, 700);
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
    let binary = mock_writer_binary();
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

/// Concurrent read/write verification (paper §4.3: the mutex guards the
/// circular buffer; the reader never locks). A stress writer overflows small
/// rings at full speed from two threads while this reader drains
/// concurrently. The reader must observe losses, but every record it does
/// observe must parse and be structurally sane — never a torn record.
#[test]
fn concurrent_reader_never_sees_torn_records() {
    let binary = mock_writer_binary();
    if !binary.exists() {
        eprintln!(
            "skipping concurrent stress: {} not found (build tracer/ with cmake first)",
            binary.display()
        );
        return;
    }

    let name = format!("my_r2d2_stress_{}", std::process::id());
    let shm_path = format!("/dev/shm/{name}");
    let _ = std::fs::remove_file(&shm_path);

    // Small rings force continuous overflow: 2 threads x 3000 rounds x 4
    // runtime events against a 256-record ring.
    let mut writer = std::process::Command::new(&binary)
        .arg(&name)
        .arg("--stress")
        .arg("--stress-rounds")
        .arg("3000")
        .arg("--threads")
        .arg("2")
        .arg("--reg-capacity")
        .arg("16")
        .arg("--runtime-capacity")
        .arg("256")
        .spawn()
        .expect("failed to spawn stress mock_writer");

    // Wait for the writer to create and initialize the shared memory; the
    // header is only valid after init() finishes, so retry until the magic
    // and version check pass.
    let run_start = std::time::Instant::now();
    let mut reader = loop {
        match TraceReader::open(&shm_path) {
            Ok(reader) => break reader,
            Err(_) => {
                assert!(
                    run_start.elapsed() < std::time::Duration::from_secs(10),
                    "stress writer never initialized {shm_path}"
                );
                std::thread::sleep(std::time::Duration::from_millis(5));
            }
        }
    };
    let mut total_events = 0u64;
    let mut total_missed = 0u64;
    let mut handlers_seen = std::collections::HashSet::new();
    let mut writer_done = false;
    loop {
        if !writer_done {
            match writer.try_wait() {
                Ok(Some(status)) => {
                    assert!(status.success(), "stress writer exited with {status}");
                    writer_done = true;
                }
                Ok(None) => {}
                Err(error) => panic!("failed to wait on stress writer: {error}"),
            }
        }
        let drain = reader.drain_runtime().unwrap();
        total_missed += drain.missed;
        for event in &drain.events {
            // Parsing already rejected unknown event kinds; here assert the
            // payload is one the stress writer could have produced.
            assert_eq!(event.round_id, 0, "no markers in stress mode");
            if event.rclcpp_handler != 0 {
                handlers_seen.insert(event.rclcpp_handler);
            }
            if event.rcl_handler != 0 {
                handlers_seen.insert(event.rcl_handler);
            }
        }
        total_events += drain.events.len() as u64;
        if writer_done && drain.events.is_empty() && drain.missed == 0 {
            break;
        }
        assert!(
            run_start.elapsed() < std::time::Duration::from_secs(60),
            "stress drain loop did not converge"
        );
    }

    let observed = total_events + total_missed;
    let expected = 2 * 3000 * 4;
    assert_eq!(
        observed, expected,
        "events + missed must account for every written record"
    );
    assert!(total_missed > 0, "small rings must have overflowed");
    // Only thread-derived handler values may appear: base 0x10000 + t*0x1000
    // (rclcpp) and +0x100 (rcl), for t in {0, 1}.
    for handler in handlers_seen {
        assert!(
            (0x10000..0x10000 + 2 * 0x1000).contains(&handler),
            "unexpected handler 0x{handler:x}"
        );
    }

    let _ = writer.wait();
    let _ = std::fs::remove_file(&shm_path);
}
