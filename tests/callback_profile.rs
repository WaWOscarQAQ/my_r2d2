//! Callback Trace Profile 回归测试：覆盖论文 Figure 5 数据结构、实时注册合并
//! 与 trace 数据质量边界。
#![cfg(unix)]

use my_r2d2::callback_profile::{
    CallbackInfo, CallbackLatency, CallbackRegistry, build_callback_infos, profile_trace,
};
use my_r2d2::trace_buffer::{
    CallbackType, RegistrationDrain, RegistrationEvent, RegistrationSource, RuntimeDrain,
    RuntimeEvent, RuntimeEventType,
};

fn reg(
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

fn registration_drain(events: Vec<RegistrationEvent>, missed: u64) -> RegistrationDrain {
    RegistrationDrain { events, missed }
}

fn registry(events: Vec<RegistrationEvent>) -> CallbackRegistry {
    let mut registry = CallbackRegistry::new();
    registry.ingest(&registration_drain(events, 0));
    registry
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

fn boundary(round_id: u32, ts: u64) -> RuntimeEvent {
    RuntimeEvent {
        event_type: RuntimeEventType::RoundBoundary,
        rclcpp_handler: 0,
        timestamp: ts,
        rcl_handler: 0,
        buffer_size: 0,
        pub_timestamp: 0,
        sub_timestamp: 0,
        round_id,
    }
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
        round_id: 0,
    }
}

fn runtime_drain(events: Vec<RuntimeEvent>, missed: u64) -> RuntimeDrain {
    RuntimeDrain { events, missed }
}

fn complete_registration() -> Vec<RegistrationEvent> {
    vec![
        reg(
            RegistrationSource::Rclcpp,
            CallbackType::Subscription,
            0x1000,
            0x2000,
            "",
            "",
        ),
        reg(
            RegistrationSource::Rcl,
            CallbackType::Subscription,
            0,
            0x2000,
            "/cb",
            "/ns",
        ),
    ]
}

fn golden_registration() -> Vec<RegistrationEvent> {
    vec![
        reg(
            RegistrationSource::Rclcpp,
            CallbackType::Subscription,
            0x1000,
            0x2000,
            "",
            "",
        ),
        reg(
            RegistrationSource::Rcl,
            CallbackType::Subscription,
            0,
            0x2000,
            "/cmd_vel_callback",
            "/ns",
        ),
        reg(
            RegistrationSource::Rclcpp,
            CallbackType::Timer,
            0x3000,
            0x4000,
            "",
            "",
        ),
        reg(
            RegistrationSource::Rcl,
            CallbackType::Subscription,
            0,
            0x4000,
            "timer_callback",
            "/ns",
        ),
    ]
}

#[test]
fn incomplete_registration_never_produces_temporary_callback_id() {
    let rcl_only = vec![reg(
        RegistrationSource::Rcl,
        CallbackType::Subscription,
        0,
        0x2000,
        "/solo",
        "/ns",
    )];
    let rclcpp_only = vec![reg(
        RegistrationSource::Rclcpp,
        CallbackType::Service,
        0x1000,
        0x2000,
        "",
        "",
    )];

    assert!(build_callback_infos(&rcl_only).is_empty());
    assert!(build_callback_infos(&rclcpp_only).is_empty());
}

#[test]
fn callback_id_appears_once_and_stays_stable_across_drains() {
    let mut registry = CallbackRegistry::new();
    registry.ingest(&registration_drain(
        vec![reg(
            RegistrationSource::Rclcpp,
            CallbackType::Service,
            0x1000,
            0x2000,
            "",
            "",
        )],
        0,
    ));
    assert!(registry.callback_infos().is_empty());

    registry.ingest(&registration_drain(
        vec![reg(
            RegistrationSource::Rcl,
            CallbackType::Subscription,
            0,
            0x2000,
            "/service",
            "/ns",
        )],
        0,
    ));
    let id = registry.callback_infos()[0].id;

    registry.ingest(&registration_drain(Vec::new(), 0));
    assert_eq!(registry.callback_infos()[0].id, id);
}

#[test]
fn callback_id_is_deterministic_and_differs_by_type() {
    let first = build_callback_infos(&golden_registration());
    let second = build_callback_infos(&golden_registration());
    assert_eq!(first, second);
    assert_ne!(first[0].id, first[1].id);

    let make = |kind| {
        build_callback_infos(&[
            reg(RegistrationSource::Rclcpp, kind, 0x1000, 0x2000, "", ""),
            reg(
                RegistrationSource::Rcl,
                CallbackType::Subscription,
                0,
                0x2000,
                "/same",
                "/ns",
            ),
        ])
    };
    assert_ne!(
        make(CallbackType::Subscription)[0].id,
        make(CallbackType::Timer)[0].id
    );
    assert_ne!(
        make(CallbackType::Timer)[0].id,
        make(CallbackType::Service)[0].id
    );
}

#[test]
fn conflicting_registration_is_latest_wins_without_poisoning_feedback() {
    let mut registry = registry(complete_registration());
    registry.ingest(&registration_drain(
        vec![reg(
            RegistrationSource::Rcl,
            CallbackType::Subscription,
            0,
            0x2000,
            "/different",
            "/ns",
        )],
        0,
    ));

    assert_eq!(registry.callback_infos()[0].name, "/different");
    assert_eq!(registry.registration_conflicts(), 1);
    let trace = profile_trace(&registry, &runtime_drain(Vec::new(), 0));
    assert_eq!(trace.diagnostics.registration_conflicts, 1);
    assert!(trace.valid_for_state_analysis());
}

#[test]
fn reentrant_invocations_pair_lifo() {
    let registry = registry(complete_registration());
    let id = registry.callback_infos()[0].id;
    let trace = profile_trace(
        &registry,
        &runtime_drain(
            vec![
                execute(0x1000, 10),
                execute(0x1000, 20),
                start(0x1000, 30),
                end(0x1000, 40),
                start(0x1000, 50),
                end(0x1000, 60),
            ],
            0,
        ),
    );

    assert_eq!(
        trace.call_trace[0],
        CallbackLatency {
            callback_id: id,
            execution_latency: 10,
            scheduling_latency: Some(10),
        }
    );
    assert_eq!(trace.call_trace[1].scheduling_latency, Some(40));
    assert!(trace.valid_for_state_analysis());
}

#[test]
fn missing_invoke_keeps_execution_latency_but_invalidates_feedback() {
    let registry = registry(complete_registration());
    let trace = profile_trace(
        &registry,
        &runtime_drain(vec![start(0x1000, 20), end(0x1000, 30)], 1),
    );

    assert_eq!(trace.call_trace.len(), 1);
    assert_eq!(trace.call_trace[0].execution_latency, 10);
    assert_eq!(trace.call_trace[0].scheduling_latency, None);
    assert!(trace.lossy);
    assert_eq!(trace.diagnostics.runtime_records_missed, 1);
    assert!(!trace.valid_for_state_analysis());
}

#[test]
fn missing_invoke_alone_does_not_invalidate_feedback() {
    // live 路径应由 runtime interposer 提供 executor_execute；这里保留
    // 的是降级/历史 trace 语义：只要没有记录丢失，invoke 缺失本身不拦截
    // 状态分析。
    let registry = registry(complete_registration());
    let trace = profile_trace(
        &registry,
        &runtime_drain(vec![start(0x1000, 20), end(0x1000, 30)], 0),
    );

    assert_eq!(trace.call_trace.len(), 1);
    assert_eq!(trace.call_trace[0].execution_latency, 10);
    assert_eq!(trace.call_trace[0].scheduling_latency, None);
    assert_eq!(trace.diagnostics.missing_invokes, 1);
    assert_eq!(trace.diagnostics.unmatched_runtime_events, 0);
    assert!(trace.valid_for_state_analysis());
}

#[test]
fn leftover_executor_invoke_alone_does_not_invalidate_feedback() {
    let registry = registry(complete_registration());
    let trace = profile_trace(
        &registry,
        &runtime_drain(
            vec![
                execute(0x1000, 10),
                start(0x1000, 20),
                end(0x1000, 30),
                execute(0x1000, 40),
            ],
            0,
        ),
    );

    assert_eq!(trace.call_trace.len(), 1);
    assert_eq!(trace.diagnostics.unmatched_runtime_events, 1);
    assert!(trace.valid_for_state_analysis());
}

#[test]
fn overflow_mispair_never_becomes_valid_zero_latency() {
    let registry = registry(complete_registration());
    let trace = profile_trace(
        &registry,
        &runtime_drain(
            vec![
                start(0x1000, 200),
                execute(0x1000, 1000),
                start(0x1000, 1100),
                end(0x1000, 300),
                end(0x1000, 1200),
            ],
            1,
        ),
    );

    assert!(trace.lossy);
    assert!(
        trace
            .call_trace
            .iter()
            .all(|latency| latency.execution_latency != 0)
    );
    assert_eq!(trace.diagnostics.invalid_timestamp_order, 1);
    assert!(!trace.valid_for_state_analysis());
}

#[test]
fn invalid_scheduling_order_is_unknown_not_zero() {
    let registry = registry(complete_registration());
    let trace = profile_trace(
        &registry,
        &runtime_drain(
            vec![execute(0x1000, 30), start(0x1000, 20), end(0x1000, 40)],
            0,
        ),
    );

    assert_eq!(trace.call_trace[0].scheduling_latency, None);
    assert_eq!(trace.diagnostics.invalid_timestamp_order, 1);
    assert!(!trace.valid_for_state_analysis());
}

#[test]
fn unpaired_and_unknown_events_are_reported() {
    let registry = registry(complete_registration());
    let trace = profile_trace(
        &registry,
        &runtime_drain(
            vec![end(0x1000, 40), execute(0x1000, 10), take(0x9999, 1, 1, 2)],
            0,
        ),
    );

    assert_eq!(trace.diagnostics.unmatched_runtime_events, 2);
    assert_eq!(trace.diagnostics.unknown_handlers, 1);
    assert_eq!(trace.diagnostics.unknown_rcl_take_handlers, 1);
    assert!(trace.valid_for_state_analysis());
}

#[test]
fn unknown_callback_handler_still_invalidates_feedback() {
    let registry = registry(complete_registration());
    let trace = profile_trace(
        &registry,
        &runtime_drain(vec![execute(0x9999, 10), take(0x9998, 1, 1, 2)], 0),
    );

    assert_eq!(trace.diagnostics.unknown_handlers, 2);
    assert_eq!(trace.diagnostics.unknown_rcl_take_handlers, 1);
    assert!(!trace.valid_for_state_analysis());
}

#[test]
fn unknown_rcl_take_does_not_invalidate_known_trace() {
    let registry = registry(complete_registration());
    let trace = profile_trace(
        &registry,
        &runtime_drain(
            vec![
                execute(0x1000, 10),
                start(0x1000, 20),
                take(0x9999, 128, 25, 30),
                take(0x2000, 128, 30, 40),
                end(0x1000, 50),
            ],
            0,
        ),
    );

    assert_eq!(trace.call_trace.len(), 1);
    assert_eq!(trace.msg_trace.len(), 1);
    assert_eq!(trace.diagnostics.unknown_handlers, 1);
    assert_eq!(trace.diagnostics.unknown_rcl_take_handlers, 1);
    assert!(trace.valid_for_state_analysis());
}

#[test]
fn non_positive_message_duration_is_reported() {
    let registry = registry(complete_registration());
    let trace = profile_trace(
        &registry,
        &runtime_drain(
            vec![take(0x2000, 512, 90, 50), take(0x2000, 512, 50, 50)],
            0,
        ),
    );

    assert!(trace.msg_trace.is_empty());
    assert_eq!(trace.diagnostics.invalid_message_durations, 2);
    assert!(!trace.valid_for_state_analysis());
}

#[test]
fn round_boundary_events_do_not_affect_profile_metrics() {
    let registry = registry(complete_registration());
    let trace = profile_trace(
        &registry,
        &runtime_drain(
            vec![
                boundary(7, 1),
                execute(0x1000, 10),
                start(0x1000, 20),
                take(0x2000, 128, 30, 40),
                end(0x1000, 50),
                boundary(8, 60),
            ],
            0,
        ),
    );

    assert_eq!(trace.call_trace.len(), 1);
    assert_eq!(trace.msg_trace.len(), 1);
    assert_eq!(trace.diagnostics.unknown_handlers, 0);
    assert_eq!(trace.diagnostics.unknown_rcl_take_handlers, 0);
    assert_eq!(trace.diagnostics.unmatched_runtime_events, 0);
    assert!(trace.valid_for_state_analysis());
}

#[test]
fn truncated_name_never_generates_callback_id() {
    let mut truncated = reg(
        RegistrationSource::Rcl,
        CallbackType::Subscription,
        0,
        0x2000,
        "partial",
        "/ns",
    );
    truncated.callback_name_truncated = true;
    let mut registry = CallbackRegistry::new();
    registry.ingest(&registration_drain(
        vec![
            reg(
                RegistrationSource::Rclcpp,
                CallbackType::Subscription,
                0x1000,
                0x2000,
                "",
                "",
            ),
            truncated,
        ],
        0,
    ));

    assert!(registry.callback_infos().is_empty());
    assert_eq!(registry.truncated_callback_names(), 1);
    assert_eq!(registry.incomplete_registrations(), 1);
}

#[test]
fn callback_info_exposes_paper_fields() {
    let info = CallbackInfo {
        id: 42,
        name: "/cmd_vel".to_string(),
        callback_type: CallbackType::Subscription,
        namespace: "/robot".to_string(),
        rclcpp_handler: 0x1000,
        rcl_handler: 0x2000,
    };
    assert_eq!(info.callback_type, CallbackType::Subscription);
}

#[test]
fn truncated_namespace_never_generates_callback_id() {
    let mut truncated = reg(
        RegistrationSource::Rcl,
        CallbackType::Subscription,
        0,
        0x2000,
        "/cb",
        "/very/long/namespace",
    );
    truncated.callback_namespace_truncated = true;
    let mut registry = CallbackRegistry::new();
    registry.ingest(&registration_drain(
        vec![
            reg(
                RegistrationSource::Rclcpp,
                CallbackType::Subscription,
                0x1000,
                0x2000,
                "",
                "",
            ),
            truncated,
        ],
        0,
    ));

    assert!(registry.callback_infos().is_empty());
    assert_eq!(registry.truncated_callback_namespaces(), 1);
    assert_eq!(registry.incomplete_registrations(), 1);
    let trace = profile_trace(&registry, &runtime_drain(Vec::new(), 0));
    assert!(!trace.valid_for_state_analysis());
}

#[test]
fn same_name_and_type_across_namespaces_is_a_collision() {
    // Figure 5 derives the ID from (name, type) only: two callbacks sharing
    // both but living in different namespaces would collapse into one ID.
    // The profile must detect the ambiguity and refuse feedback, without
    // changing the paper-defined hash.
    let events = vec![
        reg(
            RegistrationSource::Rclcpp,
            CallbackType::Subscription,
            0x1000,
            0x2000,
            "",
            "",
        ),
        reg(
            RegistrationSource::Rcl,
            CallbackType::Subscription,
            0,
            0x2000,
            "/cb",
            "/ns_a",
        ),
        reg(
            RegistrationSource::Rclcpp,
            CallbackType::Subscription,
            0x3000,
            0x4000,
            "",
            "",
        ),
        reg(
            RegistrationSource::Rcl,
            CallbackType::Subscription,
            0,
            0x4000,
            "/cb",
            "/ns_b",
        ),
    ];
    let registry = registry(events);
    let infos = registry.callback_infos();
    assert_eq!(infos.len(), 2);
    assert_eq!(infos[0].id, infos[1].id, "paper ID ignores the namespace");

    let trace = profile_trace(&registry, &runtime_drain(Vec::new(), 0));
    assert_eq!(trace.diagnostics.callback_id_collisions, 1);
    assert!(!trace.valid_for_state_analysis());
}
