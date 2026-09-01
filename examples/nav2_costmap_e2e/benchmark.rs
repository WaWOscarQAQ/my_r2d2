use super::stack::{execute_payload_round, restart_ready_stack, stack_alive};
use super::*;

#[allow(clippy::too_many_arguments)]
pub(super) fn build_benchmark_model_live(
    config: &Config,
    bindings: &[InterfaceBinding],
    nav2_ws: &Path,
    trace_dir: &Path,
    ros_setup: &Path,
    install_setup: &Path,
    payload_file: &Path,
    domain_id: &str,
    registry: &mut CallbackRegistry,
    reader: &mut TraceReader,
    schedules: &[(String, Schedule)],
    input_sequence: Option<&InputSequence>,
    stack: &mut std::process::Child,
    session: &mut TraceSession,
) -> Result<BenchmarkModel, String> {
    let lcov_root = config.lcov_dir.as_deref();
    write_benchmark_status(
        lcov_root,
        "sampling",
        config.benchmark_seconds,
        Duration::ZERO,
        0,
        0,
        0,
        0,
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
            model.edge_count(),
            model.distinct_callbacks(),
        );
        return Ok(model);
    }
    if config.benchmark_seconds == 0 {
        return Err(
            "benchmark-seconds must be > 0 when no precomputed benchmark model is supplied"
                .to_string(),
        );
    }

    let benchmark_bindings = bindings.iter().collect::<Vec<_>>();
    if benchmark_bindings.is_empty() {
        return Err("benchmark phase has no fuzzable bindings".to_string());
    }
    println!(
        "benchmark: using {} callback-trace reference bindings out of {} extracted bindings",
        benchmark_bindings.len(),
        bindings.len()
    );
    let interfaces = benchmark_bindings
        .iter()
        .map(|binding| binding.generator_interface())
        .collect::<Vec<_>>();
    let mut generator = PayloadGenerator::new(
        interfaces,
        config.generator_config(),
        config.seed ^ 0xB3A5_E1A0,
    );
    let mut builder = BenchmarkBuilder::default();
    let start = Instant::now();
    let deadline = start + Duration::from_secs(config.benchmark_seconds);
    let mut next_log = start + Duration::from_secs(30);
    let mut round = 0u64;

    println!(
        "benchmark: sampling for {}s to build callback graph and average benchmarks",
        config.benchmark_seconds
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
        let execution = match execute_payload_round(
            round,
            &round_label,
            &payload,
            binding,
            bindings,
            registry,
            reader,
            ros_setup,
            install_setup,
            payload_file,
            domain_id,
            schedules,
            config,
            input_sequence,
            stack,
            session,
        ) {
            Ok(execution) => execution,
            Err(message) => {
                builder.record_invalid_round();
                eprintln!("benchmark round {round}: {message}");
                if !stack_alive(stack) {
                    return Err(format!(
                        "benchmark round {round}: costmap stack died after execution failure"
                    ));
                }
                eprintln!("benchmark round {round}: restarting stack after failed execution");
                restart_ready_stack(
                    nav2_ws,
                    trace_dir,
                    config,
                    ros_setup,
                    install_setup,
                    domain_id,
                    bindings,
                    input_sequence,
                    stack,
                    session,
                    reader,
                    registry,
                    "benchmark",
                )?;
                write_benchmark_status(
                    lcov_root,
                    "sampling",
                    config.benchmark_seconds,
                    start.elapsed(),
                    round,
                    builder.analyzed_traces(),
                    builder.empty_traces(),
                    builder.invalid_traces(),
                    Some("stack restarted after benchmark execution failure"),
                );
                continue;
            }
        };
        if let Some(root) = lcov_root {
            write_json_report(
                &root.join(format!("benchmark/traces/trace_{round:06}.json")),
                &execution.trace,
            );
        }
        if execution.crashed {
            return Err(format!("benchmark round {round}: costmap stack crashed"));
        }
        let disposition = builder.observe(&execution.trace);
        if disposition == TraceDisposition::Invalid {
            eprintln!(
                "benchmark round {round}: invalid trace diagnostics lossy={} calls={} msgs={} {:?}",
                execution.trace.lossy,
                execution.trace.call_trace.len(),
                execution.trace.msg_trace.len(),
                execution.trace.diagnostics
            );
        }

        if Instant::now() >= next_log {
            println!(
                "benchmark: elapsed={}s rounds={} analyzed={} empty={} invalid={}",
                start.elapsed().as_secs(),
                round,
                builder.analyzed_traces(),
                builder.empty_traces(),
                builder.invalid_traces(),
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
        model.edge_count(),
        model.distinct_callbacks(),
    );
    if let Some(path) = &config.benchmark_model {
        model.save_json(path)?;
        println!("benchmark: saved model to {}", path.display());
    }
    Ok(model)
}

#[allow(clippy::too_many_arguments)]
pub(super) fn write_benchmark_status(
    lcov_root: Option<&Path>,
    phase: &str,
    configured_seconds: u64,
    elapsed: Duration,
    round: u64,
    analyzed_traces: u64,
    empty_traces: u64,
    invalid_traces: u64,
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
            "callback_graph_edges": edge_count,
            "distinct_callbacks": distinct_callbacks,
        }),
    );
}
