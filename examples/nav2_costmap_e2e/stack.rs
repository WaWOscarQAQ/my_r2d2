use super::bindings::find_binding;
use super::*;
use my_r2d2::runtime::command_template::{CommandParameters, runtime_command};
use my_r2d2::runtime::input_sequence::{InputSequence, InputSequencePhase, InputStep};
use my_r2d2::runtime::ros2_sender::{
    Ros2CommandPreview, preview_ros2_action_command, preview_ros2_parameter_command,
    preview_ros2_service_command, preview_ros2_topic_command, send_ros2_parameter_value,
    send_ros2_service_yaml, send_ros2_topic_yaml, validate_yaml_payload,
};
use serde_yaml::{Number as YamlNumber, Value as YamlValue};
use std::time::{SystemTime, UNIX_EPOCH};

pub(super) fn stack_alive(stack: &mut std::process::Child) -> bool {
    // try_wait 会回收僵尸进程；kill -0 对僵尸返回成功，不能用于存活判定。
    matches!(stack.try_wait(), Ok(None))
}

fn stack_script(nav2_ws: &Path) -> PathBuf {
    nav2_ws.join("launch_nav2_full_stack.sh")
}

const STARTUP_REGISTRATION_SETTLE_TIMEOUT: Duration = Duration::from_secs(60);
const FULL_STACK_HEALTH_RETRIES: usize = 3;
const FULL_STACK_HEALTH_POLL: Duration = Duration::from_millis(500);
const STACK_LIFECYCLE_SERVICE_PROBE_TIMEOUT_SEC: u64 = 3;
const STACK_LIFECYCLE_SHUTDOWN_TIMEOUT_SEC: u64 = 30;
const STACK_LIFECYCLE_SHUTDOWN_SETTLE: Duration = Duration::from_secs(2);
const STACK_STALE_CLEANUP_TIMEOUT: Duration = Duration::from_secs(10);
const STACK_STALE_CLEANUP_POLL: Duration = Duration::from_millis(250);
const FULL_STACK_LIFECYCLE_STALL_TIMEOUT: Duration = Duration::from_secs(210);
// PointCloudTransport is supplied as a prebuilt Lyrical dependency, so its
// callback registration tracepoint is outside this project's LLVM-instrumented
// translation units.  The endpoint remains fuzzable, but it must not block the
// instrumented-business-callback startup barrier.
const FULL_STACK_NODES: [&str; 7] = [
    "/map_server",
    "/amcl",
    "/planner_server",
    "/controller_server",
    "/smoother_server",
    "/behavior_server",
    "/bt_navigator",
];
const NAV2_ACTION_CANCEL_TYPE: &str = "action_msgs/srv/CancelGoal";
const NAV2_MANAGE_LIFECYCLE_NODES_TYPE: &str = "nav2_msgs/srv/ManageLifecycleNodes";
const NAV2_LIFECYCLE_STARTUP_REQUEST: &str = "{command: 0}";
const NAV2_LIFECYCLE_SHUTDOWN_REQUEST: &str = "{command: 4}";
const FULL_STACK_MANAGER_REQUEST_TIMEOUT_SEC: u64 = 60;
const FULL_STACK_LOCALIZATION_NODES: [&str; 2] = ["/map_server", "/amcl"];
const FULL_STACK_NAVIGATION_NODES: [&str; 5] = [
    "/planner_server",
    "/controller_server",
    "/smoother_server",
    "/behavior_server",
    "/bt_navigator",
];
const FULL_STACK_LIFECYCLE_SHUTDOWN_SERVICES: [(&str, &str); 2] = [
    ("navigation", "/lifecycle_manager_navigation/manage_nodes"),
    (
        "localization",
        "/lifecycle_manager_localization/manage_nodes",
    ),
];
const STACK_PROCESS_MARKERS: [&str; 10] = [
    "/lib/nav2_amcl/amcl",
    "/lib/nav2_behaviors/behavior_server",
    "/lib/nav2_bt_navigator/bt_navigator",
    "/lib/nav2_controller/controller_server",
    "/lib/nav2_costmap_2d/nav2_costmap_2d",
    "/lib/nav2_lifecycle_manager/lifecycle_manager",
    "/lib/nav2_map_server/map_server",
    "/lib/nav2_planner/planner_server",
    "/lib/nav2_smoother/smoother_server",
    "/scripts/r2d2_odom_publisher.py",
];
#[cfg(test)]
pub(super) const FULL_STACK_SERVICES: [&str; 26] = [
    "/local_costmap/get_cost_local_costmap",
    "/local_costmap/get_costmap",
    "/local_costmap/get_voxel_layer",
    "/local_costmap/clear_except_local_costmap",
    "/local_costmap/clear_around_local_costmap",
    "/local_costmap/clear_around_pose_local_costmap",
    "/local_costmap/clear_entirely_local_costmap",
    "/global_costmap/get_cost_global_costmap",
    "/global_costmap/get_costmap",
    "/global_costmap/get_obstacle_layer",
    "/global_costmap/get_static_layer",
    "/global_costmap/clear_except_global_costmap",
    "/global_costmap/clear_around_global_costmap",
    "/global_costmap/clear_around_pose_global_costmap",
    "/global_costmap/clear_entirely_global_costmap",
    "/reinitialize_global_localization",
    "/request_nomotion_update",
    "/set_initial_pose",
    "/map_server/map",
    "/map_server/load_map",
    "/is_path_valid",
    "/lifecycle_manager_localization/is_active",
    "/lifecycle_manager_navigation/is_active",
    "/local_costmap/keepout_filter/toggle_filter",
    "/global_costmap/keepout_filter/toggle_filter",
    "/global_costmap/speed_filter/toggle_filter",
];

fn lexical_normalize_path(path: &Path) -> PathBuf {
    use std::path::Component;

    let mut normalized = PathBuf::new();
    for component in path.components() {
        match component {
            Component::CurDir => {}
            Component::ParentDir => {
                let _ = normalized.pop();
            }
            other => normalized.push(other.as_os_str()),
        }
    }
    normalized
}

fn path_aliases(path: &Path) -> Vec<String> {
    let mut aliases = BTreeSet::new();
    aliases.insert(path.display().to_string());
    aliases.insert(lexical_normalize_path(path).display().to_string());
    if let Ok(canonical) = fs::canonicalize(path) {
        aliases.insert(canonical.display().to_string());
    }
    aliases.into_iter().collect()
}

fn stale_stack_pgids(stack_script: &Path, domain_id: &str) -> Result<BTreeSet<i32>, String> {
    let output = match runtime_command("process_list", &CommandParameters::new())
        .and_then(|mut command| command.output().map_err(|error| error.to_string()))
    {
        Ok(output) => output,
        Err(error) => return Err(format!("failed to inspect stale stack processes: {error}")),
    };
    if !output.status.success() {
        return Err("ps failed while inspecting stale stack processes".to_string());
    }

    let script_aliases = path_aliases(stack_script);
    let repo_root = stack_script
        .parent()
        .and_then(Path::parent)
        .map(lexical_normalize_path)
        .unwrap_or_else(|| PathBuf::from("/"));
    let repo_marker = repo_root.display().to_string();
    let domain_marker = format!("ROS_DOMAIN_ID={domain_id}");
    let text = String::from_utf8_lossy(&output.stdout);
    let mut stale_pgids = BTreeSet::new();
    for line in text.lines() {
        let mut parts = line.split_whitespace();
        let pid = parts.next().and_then(|value| value.parse::<i32>().ok());
        let pgid = parts.next().and_then(|value| value.parse::<i32>().ok());
        let cmd = parts.collect::<Vec<_>>().join(" ");
        let matches_stack = script_aliases.iter().any(|path| cmd.contains(path))
            || (cmd.contains("launch_nav2_full_stack.sh") && cmd.contains(&repo_marker));
        let matches_nav2_process = cmd.contains(&repo_marker)
            && STACK_PROCESS_MARKERS
                .iter()
                .any(|marker| cmd.contains(marker));
        let matches_domain = pid.is_some_and(|pid| {
            fs::read(format!("/proc/{pid}/environ")).is_ok_and(|env| {
                env.split(|byte| *byte == 0)
                    .any(|entry| entry == domain_marker.as_bytes())
            })
        });
        if matches_domain
            && (matches_stack || matches_nav2_process)
            && let Some(pgid) = pgid
            && pgid > 0
        {
            stale_pgids.insert(pgid);
        }
    }
    Ok(stale_pgids)
}

fn cleanup_stale_stack_groups(stack_script: &Path, domain_id: &str) {
    let stale_pgids = match stale_stack_pgids(stack_script, domain_id) {
        Ok(pgids) => pgids,
        Err(error) => {
            eprintln!("startup: {error}");
            return;
        }
    };

    if stale_pgids.is_empty() {
        return;
    }

    let groups = stale_pgids
        .iter()
        .map(|pgid| pgid.to_string())
        .collect::<Vec<_>>()
        .join(", ");
    eprintln!("startup: cleaning stale stack process groups: {groups}");
    for signal in ["-INT", "-TERM", "-KILL"] {
        for pgid in &stale_pgids {
            if !signal_process_group(*pgid as u32, signal) {
                eprintln!("startup: kill {signal} failed for stale process group {pgid}");
            }
        }
        if signal != "-KILL" {
            thread::sleep(Duration::from_secs(2));
        }
    }
    let deadline = Instant::now() + STACK_STALE_CLEANUP_TIMEOUT;
    loop {
        let remaining = match stale_stack_pgids(stack_script, domain_id) {
            Ok(pgids) => pgids,
            Err(error) => {
                eprintln!("startup: failed to verify stale stack cleanup: {error}");
                return;
            }
        };
        if remaining.is_empty() {
            return;
        }
        if Instant::now() >= deadline {
            let groups = remaining
                .iter()
                .map(|pgid| pgid.to_string())
                .collect::<Vec<_>>()
                .join(", ");
            eprintln!("startup: stale stack process groups still visible after cleanup: {groups}");
            return;
        }
        thread::sleep(STACK_STALE_CLEANUP_POLL);
    }
}

fn signal_process_group(pgid: u32, signal: &str) -> bool {
    let mut parameters = CommandParameters::new();
    parameters
        .insert("signal", signal)
        .insert("pgid", pgid.to_string());
    runtime_command("signal_process_group", &parameters)
        .and_then(|mut command| command.status().map_err(|error| error.to_string()))
        .is_ok_and(|status| status.success())
}

pub(super) fn terminate_stack_process_group(stack: &mut std::process::Child) {
    let stack_pid = stack.id();
    for (signal, wait) in [
        ("-INT", Duration::from_secs(5)),
        ("-TERM", Duration::from_secs(2)),
        ("-KILL", Duration::from_secs(0)),
    ] {
        if !signal_process_group(stack_pid, signal) {
            eprintln!("startup: kill {signal} failed for stack process group {stack_pid}");
        }
        let deadline = Instant::now() + wait;
        while Instant::now() < deadline {
            if !stack_alive(stack) {
                return;
            }
            thread::sleep(Duration::from_millis(100));
        }
    }
    let _ = stack.wait();
}

pub(super) fn shutdown_stack(
    ros_setup: &Path,
    install_setup: &Path,
    domain_id: &str,
    stack: &mut std::process::Child,
) {
    if stack_alive(stack) {
        request_full_stack_lifecycle_shutdown(ros_setup, install_setup, domain_id, stack);
    }
    terminate_stack_process_group(stack);
}

fn request_full_stack_lifecycle_shutdown(
    ros_setup: &Path,
    install_setup: &Path,
    domain_id: &str,
    stack: &mut std::process::Child,
) {
    for (label, service) in FULL_STACK_LIFECYCLE_SHUTDOWN_SERVICES {
        if !stack_alive(stack) {
            return;
        }
        let service_type = ros2_cli_output_with_timeout(
            ros_setup,
            install_setup,
            domain_id,
            &["service", "type", service],
            Some(STACK_LIFECYCLE_SERVICE_PROBE_TIMEOUT_SEC),
        );
        match service_type {
            Ok(output)
                if output.success && output.stdout.trim() == NAV2_MANAGE_LIFECYCLE_NODES_TYPE => {}
            Ok(output) => {
                let message = format!("{} {}", output.stdout.trim(), output.stderr.trim());
                eprintln!(
                    "shutdown: skipping {label} lifecycle manager; service type probe failed: {}",
                    message.trim()
                );
                continue;
            }
            Err(error) => {
                eprintln!(
                    "shutdown: skipping {label} lifecycle manager; service type probe failed: {error}"
                );
                continue;
            }
        }
        let output = ros2_cli_output_with_timeout(
            ros_setup,
            install_setup,
            domain_id,
            &[
                "service",
                "call",
                service,
                NAV2_MANAGE_LIFECYCLE_NODES_TYPE,
                NAV2_LIFECYCLE_SHUTDOWN_REQUEST,
            ],
            Some(STACK_LIFECYCLE_SHUTDOWN_TIMEOUT_SEC),
        );
        match output {
            Ok(output) if output.success => {
                eprintln!("shutdown: requested {label} lifecycle manager shutdown");
            }
            Ok(output) => {
                let message = format!("{} {}", output.stdout.trim(), output.stderr.trim());
                eprintln!(
                    "shutdown: {label} lifecycle manager shutdown request failed: {}",
                    message.trim()
                );
            }
            Err(error) => {
                eprintln!("shutdown: {label} lifecycle manager shutdown request failed: {error}");
            }
        }
    }
    thread::sleep(STACK_LIFECYCLE_SHUTDOWN_SETTLE);
}

fn spawn_instrumented_stack(
    nav2_ws: &Path,
    trace_dir: &Path,
    config: &Config,
    domain_id: &str,
) -> Result<std::process::Child, String> {
    let stack_script = stack_script(nav2_ws);
    cleanup_stale_stack_groups(&stack_script, domain_id);

    let mut parameters = CommandParameters::new();
    parameters.insert_path("stack_script", &stack_script);
    let mut stack_command = runtime_command("nav2_stack", &parameters)
        .map_err(|error| format!("prepare Nav2 stack command: {error}"))?;
    stack_command
        .env("ROS_DOMAIN_ID", domain_id)
        .env("R2D2_SHM_PATH", trace_dir)
        .env_remove("ASAN_OPTIONS")
        .env_remove("LD_PRELOAD")
        .env_remove("COLCON_CURRENT_PREFIX");
    let mut tsan_options = Vec::new();
    if let Some(log_dir) = &config.tsan_log_dir {
        let _ = fs::create_dir_all(log_dir);
        tsan_options.push(format!("log_path={}/tsan", log_dir.display()));
        println!("tsan: reports will be written to {}", log_dir.display());
    }
    if let Some(sancov_dir) = &config.sancov_dir {
        let _ = fs::create_dir_all(sancov_dir);
        stack_command.env("R2D2_SANCOV_DIR", sancov_dir);
        tsan_options.push("coverage=1".to_string());
        tsan_options.push(format!("coverage_dir={}", sancov_dir.display()));
        println!(
            "sancov: edge counters will be written to {}",
            sancov_dir.display()
        );
    } else {
        stack_command.env_remove("R2D2_SANCOV_DIR");
    }
    if !tsan_options.is_empty() {
        tsan_options.extend([
            "halt_on_error=0".to_string(),
            "exitcode=0".to_string(),
            "history_size=7".to_string(),
            "second_deadlock_stack=1".to_string(),
            "report_signal_unsafe=0".to_string(),
            "symbolize=0".to_string(),
        ]);
        stack_command.env("TSAN_OPTIONS", tsan_options.join(":"));
    } else {
        stack_command.env_remove("TSAN_OPTIONS");
        stack_command.env_remove("LD_PRELOAD");
    }
    let stack = stack_command
        .spawn()
        .map_err(|error| format!("spawn Nav2 stack: {error}"))?;
    println!("stack leader pid = {}", stack.id());
    Ok(stack)
}

pub(super) fn start_ready_stack(
    nav2_ws: &Path,
    trace_dir: &Path,
    config: &Config,
    ros_setup: &Path,
    install_setup: &Path,
    domain_id: &str,
) -> Result<
    (
        std::process::Child,
        TraceSession,
        TraceReader,
        CallbackRegistry,
        usize,
    ),
    String,
> {
    let stack_script = stack_script(nav2_ws);
    let session = TraceSession::create(trace_dir)
        .map_err(|error| format!("prepare shared-memory trace path: {error}"))?;
    let mut stack = spawn_instrumented_stack(nav2_ws, trace_dir, config, domain_id)?;
    let visible = wait_for_full_stack_visible(
        ros_setup,
        install_setup,
        domain_id,
        &mut stack,
        Instant::now() + startup_timeout(),
    );
    if let Err(error) = visible {
        terminate_stack_process_group(&mut stack);
        cleanup_stale_stack_groups(&stack_script, domain_id);
        return Err(error);
    }
    let mut reader = match wait_for_trace_reader(
        session.trace_dir(),
        &mut stack,
        Instant::now() + startup_timeout(),
    ) {
        Ok(reader) => reader,
        Err(error) => {
            terminate_stack_process_group(&mut stack);
            cleanup_stale_stack_groups(&stack_script, domain_id);
            return Err(error);
        }
    };
    let mut registry = CallbackRegistry::new();
    let mut registrations = match wait_full_stack_ready(
        ros_setup,
        install_setup,
        domain_id,
        &mut stack,
        &mut reader,
        &mut registry,
    ) {
        Ok(registrations) => registrations,
        Err(error) => {
            terminate_stack_process_group(&mut stack);
            cleanup_stale_stack_groups(&stack_script, domain_id);
            return Err(error);
        }
    };
    if let Err(error) = drain_startup_trace("startup ready", &mut reader, &mut registry)
        .map(|extra| registrations += extra)
    {
        terminate_stack_process_group(&mut stack);
        cleanup_stale_stack_groups(&stack_script, domain_id);
        return Err(error);
    }
    let _ = drain_runtime_snapshot("startup runtime", &mut reader, &mut registry)?;
    match wait_for_full_registration_settle(
        &mut stack,
        &mut reader,
        &mut registry,
        "startup settle",
    ) {
        Ok(extra) => registrations += extra,
        Err(error) => {
            terminate_stack_process_group(&mut stack);
            cleanup_stale_stack_groups(&stack_script, domain_id);
            return Err(error);
        }
    }
    if registry.callback_infos().is_empty() {
        terminate_stack_process_group(&mut stack);
        return Err("startup registration completed with zero complete callbacks".to_string());
    }
    Ok((stack, session, reader, registry, registrations))
}

#[allow(clippy::too_many_arguments)]
pub(super) fn restart_ready_stack(
    nav2_ws: &Path,
    trace_dir: &Path,
    config: &Config,
    ros_setup: &Path,
    install_setup: &Path,
    domain_id: &str,
    bindings: &[InterfaceBinding],
    input_sequence: Option<&InputSequence>,
    stack: &mut std::process::Child,
    session: &mut TraceSession,
    reader: &mut TraceReader,
    registry: &mut CallbackRegistry,
    phase_label: &str,
) -> Result<(), String> {
    shutdown_stack(ros_setup, install_setup, domain_id, stack);
    let _ = session.destroy();
    let (new_stack, new_session, new_reader, new_registry, registrations) = start_ready_stack(
        nav2_ws,
        trace_dir,
        config,
        ros_setup,
        install_setup,
        domain_id,
    )?;
    *stack = new_stack;
    *session = new_session;
    *reader = new_reader;
    *registry = new_registry;
    println!(
        "{phase_label}: stack restarted; ready barrier passed with {} registration records and {} complete callbacks",
        registrations,
        registry.callback_infos().len()
    );
    execute_input_sequence_phase(
        input_sequence,
        InputSequencePhase::Startup,
        &format!("{phase_label} input-sequence"),
        bindings,
        registry,
        reader,
        ros_setup,
        install_setup,
        domain_id,
        true,
    )?;
    let _ = drain_runtime_snapshot(&format!("{phase_label} bootstrap"), reader, registry)?;
    Ok(())
}

#[allow(clippy::too_many_arguments)]
pub(super) fn execute_input_sequence_phase(
    sequence: Option<&InputSequence>,
    phase: InputSequencePhase,
    phase_label: &str,
    bindings: &[InterfaceBinding],
    registry: &mut CallbackRegistry,
    reader: &mut TraceReader,
    ros_setup: &Path,
    install_setup: &Path,
    domain_id: &str,
    drain_after: bool,
) -> Result<usize, String> {
    let Some(sequence) = sequence else {
        return Ok(0);
    };
    let steps = sequence.steps_for_phase(phase).collect::<Vec<_>>();
    if steps.is_empty() {
        return Ok(0);
    }
    if phase == InputSequencePhase::Startup {
        println!(
            "{phase_label}: executing {} {} steps from {}",
            steps.len(),
            sequence.name,
            sequence.path.display()
        );
    }
    for step in steps {
        execute_input_sequence_step(
            phase_label,
            step,
            bindings,
            ros_setup,
            install_setup,
            domain_id,
        )?;
        if step.delay_after_ms > 0 {
            thread::sleep(Duration::from_millis(step.delay_after_ms));
        }
    }
    if drain_after {
        let _ = drain_runtime_snapshot(phase_label, reader, registry)?;
    }
    Ok(sequence.steps_for_phase(phase).count())
}

fn execute_input_sequence_step(
    phase_label: &str,
    step: &InputStep,
    bindings: &[InterfaceBinding],
    ros_setup: &Path,
    install_setup: &Path,
    domain_id: &str,
) -> Result<(), String> {
    let binding = find_sequence_binding(bindings, step)
        .map_err(|error| format!("{phase_label}: {}: {error}", step.label_or_name()))?;
    let payload = payload_with_fresh_header_stamps(&step.payload);
    validate_yaml_payload(&payload, &binding.interface.fields).map_err(|error| {
        format!(
            "{phase_label}: {} payload does not match {}: {error}",
            step.label_or_name(),
            binding.interface_id
        )
    })?;
    let options = match &binding.endpoint {
        EndpointBinding::Topic { options, .. } => options.clone(),
        EndpointBinding::LaserScan { .. } => Ros2TopicOptions::default(),
        _ => Ros2TopicOptions::default(),
    };
    let result = send_ros2_topic_yaml(
        ros_setup,
        install_setup,
        domain_id,
        &step.name,
        &step.ros_type,
        &payload,
        &options,
    )
    .map_err(|error| {
        format!(
            "{phase_label}: {} topic send failed: {error}",
            step.label_or_name()
        )
    });
    if let Err(error) = result {
        if step.allow_failure {
            eprintln!("{error} [allowed by input sequence]");
        } else {
            return Err(error);
        }
    }
    Ok(())
}

fn payload_with_fresh_header_stamps(payload: &YamlValue) -> YamlValue {
    let mut payload = payload.clone();
    let timestamp = current_ros_timestamp();
    refresh_header_stamps(&mut payload, timestamp);
    payload
}

fn current_ros_timestamp() -> (u64, u32) {
    let now = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default();
    (now.as_secs(), now.subsec_nanos())
}

fn refresh_header_stamps(value: &mut YamlValue, timestamp: (u64, u32)) {
    match value {
        YamlValue::Mapping(mapping) => {
            let header_key = YamlValue::String("header".to_string());
            if let Some(header) = mapping
                .get_mut(&header_key)
                .and_then(YamlValue::as_mapping_mut)
            {
                let stamp_key = YamlValue::String("stamp".to_string());
                if let Some(stamp) = header
                    .get_mut(&stamp_key)
                    .and_then(YamlValue::as_mapping_mut)
                {
                    stamp.insert(
                        YamlValue::String("sec".to_string()),
                        YamlValue::Number(YamlNumber::from(timestamp.0)),
                    );
                    stamp.insert(
                        YamlValue::String("nanosec".to_string()),
                        YamlValue::Number(YamlNumber::from(timestamp.1)),
                    );
                }
            }
            for nested in mapping.values_mut() {
                refresh_header_stamps(nested, timestamp);
            }
        }
        YamlValue::Sequence(sequence) => {
            for nested in sequence {
                refresh_header_stamps(nested, timestamp);
            }
        }
        _ => {}
    }
}

fn find_sequence_binding<'a>(
    bindings: &'a [InterfaceBinding],
    step: &InputStep,
) -> Result<&'a InterfaceBinding, String> {
    if let Some(interface_id) = &step.interface_id
        && let Some(binding) = find_binding(bindings, interface_id)
        && sequence_binding_matches(binding, step)
    {
        return Ok(binding);
    }
    bindings
        .iter()
        .find(|binding| sequence_binding_matches(binding, step))
        .ok_or_else(|| {
            format!(
                "no readiness topic binding for {} {}",
                step.name, step.ros_type
            )
        })
}

fn sequence_binding_matches(binding: &InterfaceBinding, step: &InputStep) -> bool {
    match &binding.endpoint {
        EndpointBinding::LaserScan { topic_name } => {
            step.name == *topic_name
                && (step.ros_type == "sensor_msgs/msg/LaserScan"
                    || step.interface_id.as_deref() == Some("LaserScan"))
        }
        EndpointBinding::Topic {
            topic_name,
            message_type,
            ..
        } => step.name == *topic_name && step.ros_type == *message_type,
        _ => false,
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

fn drain_startup_trace(
    round_label: &str,
    reader: &mut TraceReader,
    registry: &mut CallbackRegistry,
) -> Result<usize, String> {
    let mut registrations = ingest_registration_updates(round_label, reader, registry)?;
    let _ = reader
        .drain_runtime()
        .map_err(|error| format!("{round_label}: runtime drain failed: {error}"))?;
    registrations += ingest_registration_updates(round_label, reader, registry)?;
    Ok(registrations)
}

pub(super) fn drain_runtime_snapshot(
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

fn drain_round_runtime_until_quiet(
    round_label: &str,
    session: &mut TraceSession,
    reader: &mut TraceReader,
    registry: &mut CallbackRegistry,
    settle_timeout: Duration,
) -> Result<RuntimeDrain, String> {
    let deadline = Instant::now() + settle_timeout;
    let mut quiet_polls = 0usize;
    let mut events = Vec::new();
    let mut missed = 0u64;
    while Instant::now() < deadline {
        thread::sleep(ROUND_SETTLE_POLL);
        let drained = drain_runtime_snapshot(round_label, reader, registry)?;
        missed += drained.missed;
        if drained.events.is_empty() && drained.missed == 0 {
            quiet_polls += 1;
            if quiet_polls >= ROUND_SETTLE_POLLS {
                break;
            }
        } else {
            quiet_polls = 0;
            events.extend(drained.events);
        }
    }
    let _ = session.stop();
    Ok(RuntimeDrain { events, missed })
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
    ros2_cli_output_with_timeout(ros_setup, install_setup, domain_id, args, None)
}

fn ros2_cli_output_with_timeout(
    ros_setup: &Path,
    install_setup: &Path,
    domain_id: &str,
    args: &[&str],
    timeout_sec: Option<u64>,
) -> Result<Ros2CliOutput, String> {
    let mut cli_args = vec!["ros2".to_string()];
    cli_args.extend(
        args.iter()
            .map(|arg| (*arg).to_string())
            .collect::<Vec<_>>(),
    );
    let mut command = ros2_cli_command(ros_setup, install_setup, domain_id, &cli_args);
    if let Some(timeout_sec) = timeout_sec {
        command.env("R2D2_ROS2_CLI_TIMEOUT_SEC", timeout_sec.to_string());
    }
    let output = command
        .output()
        .map_err(|error| format!("failed to run ros2 {}: {error}", cli_args[1..].join(" ")))?;
    Ok(Ros2CliOutput {
        stdout: String::from_utf8_lossy(&output.stdout).into_owned(),
        stderr: String::from_utf8_lossy(&output.stderr).into_owned(),
        success: output.status.success(),
    })
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum LifecycleState {
    Unknown,
    Unconfigured,
    Inactive,
    Active,
    Finalized,
}

fn parse_lifecycle_state(stdout: &str, stderr: &str) -> LifecycleState {
    for line in format!("{stdout}\n{stderr}").lines() {
        let line = line.trim().to_ascii_lowercase();
        if line == "unconfigured" || line.starts_with("unconfigured [") {
            return LifecycleState::Unconfigured;
        }
        if line == "inactive" || line.starts_with("inactive [") {
            return LifecycleState::Inactive;
        }
        if line == "active" || line.starts_with("active [") {
            return LifecycleState::Active;
        }
        if line == "finalized" || line.starts_with("finalized [") {
            return LifecycleState::Finalized;
        }
    }
    LifecycleState::Unknown
}

fn wait_for_full_stack_visible(
    ros_setup: &Path,
    install_setup: &Path,
    domain_id: &str,
    stack: &mut std::process::Child,
    deadline: Instant,
) -> Result<(), String> {
    while Instant::now() < deadline {
        if !stack_alive(stack) {
            return Err("Nav2 stack died before nodes became visible".to_string());
        }
        let output = ros2_cli_output(ros_setup, install_setup, domain_id, &["node", "list"])?;
        if output.success {
            let names = output
                .stdout
                .lines()
                .map(str::trim)
                .collect::<BTreeSet<_>>();
            if FULL_STACK_NODES.iter().all(|node| names.contains(node)) {
                return Ok(());
            }
        }
        thread::sleep(STARTUP_POLL);
    }
    Err(format!(
        "timeout waiting for full Nav2 nodes: {}",
        FULL_STACK_NODES.join(", ")
    ))
}

fn missing_full_stack_nodes_visible(
    ros_setup: &Path,
    install_setup: &Path,
    domain_id: &str,
) -> Result<Vec<&'static str>, String> {
    let output = ros2_cli_output(ros_setup, install_setup, domain_id, &["node", "list"])?;
    if !output.success {
        return Err(format!(
            "ros2 node list failed while checking full-stack health: {}",
            output.stderr.trim()
        ));
    }
    let names = output
        .stdout
        .lines()
        .map(str::trim)
        .collect::<BTreeSet<_>>();
    Ok(FULL_STACK_NODES
        .iter()
        .copied()
        .filter(|node| !names.contains(node))
        .collect())
}

pub(super) fn ensure_full_stack_nodes_visible(
    round_label: &str,
    ros_setup: &Path,
    install_setup: &Path,
    domain_id: &str,
    stack: &mut std::process::Child,
) -> Result<(), String> {
    let mut last_missing = Vec::new();
    for _ in 0..FULL_STACK_HEALTH_RETRIES {
        if !stack_alive(stack) {
            return Err(format!("{round_label}: Nav2 launch process died"));
        }
        match missing_full_stack_nodes_visible(ros_setup, install_setup, domain_id) {
            Ok(missing) if missing.is_empty() => return Ok(()),
            Ok(missing) => last_missing = missing,
            Err(error) => return Err(format!("{round_label}: {error}")),
        }
        thread::sleep(FULL_STACK_HEALTH_POLL);
    }
    Err(format!(
        "{round_label}: full Nav2 stack unhealthy; missing nodes: {}",
        last_missing.join(", ")
    ))
}

fn wait_for_trace_reader(
    trace_dir: &Path,
    stack: &mut std::process::Child,
    deadline: Instant,
) -> Result<TraceReader, String> {
    let mut last_error = String::new();
    while Instant::now() < deadline {
        if !stack_alive(stack) {
            return Err("costmap stack died before startup trace became readable".to_string());
        }
        match TraceReader::open(trace_dir) {
            Ok(reader) => return Ok(reader),
            Err(error) => last_error = error.to_string(),
        }
        thread::sleep(STARTUP_POLL);
    }
    Err(format!(
        "timeout waiting for startup trace {} to become readable: {last_error}",
        trace_dir.display()
    ))
}

fn lifecycle_state_for(
    ros_setup: &Path,
    install_setup: &Path,
    domain_id: &str,
    node: &str,
) -> Result<LifecycleState, String> {
    let output = ros2_cli_output_with_timeout(
        ros_setup,
        install_setup,
        domain_id,
        &["lifecycle", "get", node],
        Some(STACK_LIFECYCLE_SERVICE_PROBE_TIMEOUT_SEC),
    )?;
    if !output.success {
        return Ok(LifecycleState::Unknown);
    }
    Ok(parse_lifecycle_state(&output.stdout, &output.stderr))
}

#[allow(clippy::too_many_arguments)]
fn wait_for_service_type(
    ros_setup: &Path,
    install_setup: &Path,
    domain_id: &str,
    stack: &mut std::process::Child,
    service_name: &str,
    expected_type: &str,
    deadline: Instant,
    reader: &mut TraceReader,
    registry: &mut CallbackRegistry,
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
        let _ = drain_startup_trace("startup service wait", reader, registry)?;
        thread::sleep(STARTUP_POLL);
    }
    Err(format!(
        "timeout waiting for service {service_name} to expose type {expected_type}"
    ))
}

fn lifecycle_states_for_nodes(
    ros_setup: &Path,
    install_setup: &Path,
    domain_id: &str,
    nodes: &[&str],
) -> Result<Vec<LifecycleState>, String> {
    nodes
        .iter()
        .map(|node| lifecycle_state_for(ros_setup, install_setup, domain_id, node))
        .collect()
}

fn lifecycle_nodes_active(
    ros_setup: &Path,
    install_setup: &Path,
    domain_id: &str,
    nodes: &[&str],
) -> Result<bool, String> {
    Ok(
        lifecycle_states_for_nodes(ros_setup, install_setup, domain_id, nodes)?
            .into_iter()
            .all(|state| state == LifecycleState::Active),
    )
}

fn format_lifecycle_states_for_nodes(nodes: &[&str], states: &[LifecycleState]) -> String {
    nodes
        .iter()
        .zip(states.iter())
        .map(|(node, state)| format!("{node}={state:?}"))
        .collect::<Vec<_>>()
        .join(", ")
}

#[allow(clippy::too_many_arguments)]
fn wait_for_named_lifecycle_nodes_active(
    label: &str,
    nodes: &[&str],
    ros_setup: &Path,
    install_setup: &Path,
    domain_id: &str,
    stack: &mut std::process::Child,
    reader: &mut TraceReader,
    registry: &mut CallbackRegistry,
) -> Result<usize, String> {
    let deadline = Instant::now() + startup_timeout();
    let mut registrations = 0usize;
    let mut last_states = Vec::new();
    while Instant::now() < deadline {
        if !stack_alive(stack) {
            return Err(format!(
                "Nav2 stack died while waiting for {label} lifecycle nodes"
            ));
        }
        let states = lifecycle_states_for_nodes(ros_setup, install_setup, domain_id, nodes)?;
        if states.iter().all(|state| *state == LifecycleState::Active) {
            return Ok(registrations);
        }
        last_states = states;
        registrations += drain_startup_trace("startup lifecycle wait", reader, registry)?;
        thread::sleep(STARTUP_POLL);
    }
    Err(format!(
        "timeout waiting for {label} lifecycle nodes active: {}",
        format_lifecycle_states_for_nodes(nodes, &last_states)
    ))
}

#[allow(clippy::too_many_arguments)]
fn request_lifecycle_manager_startup(
    label: &str,
    service_name: &str,
    nodes: &[&str],
    ros_setup: &Path,
    install_setup: &Path,
    domain_id: &str,
    stack: &mut std::process::Child,
    reader: &mut TraceReader,
    registry: &mut CallbackRegistry,
) -> Result<usize, String> {
    if lifecycle_nodes_active(ros_setup, install_setup, domain_id, nodes)? {
        return Ok(0);
    }
    wait_for_service_type(
        ros_setup,
        install_setup,
        domain_id,
        stack,
        service_name,
        NAV2_MANAGE_LIFECYCLE_NODES_TYPE,
        Instant::now() + startup_timeout(),
        reader,
        registry,
    )?;
    println!("startup: requesting {label} lifecycle manager STARTUP");
    let output = ros2_cli_output_with_timeout(
        ros_setup,
        install_setup,
        domain_id,
        &[
            "service",
            "call",
            service_name,
            NAV2_MANAGE_LIFECYCLE_NODES_TYPE,
            NAV2_LIFECYCLE_STARTUP_REQUEST,
        ],
        Some(FULL_STACK_MANAGER_REQUEST_TIMEOUT_SEC),
    )?;
    if !output.success {
        let message = format!("{} {}", output.stdout.trim(), output.stderr.trim());
        eprintln!(
            "startup: {label} lifecycle STARTUP request did not return cleanly; verifying node states anyway: {}",
            message.trim()
        );
    }
    wait_for_named_lifecycle_nodes_active(
        label,
        nodes,
        ros_setup,
        install_setup,
        domain_id,
        stack,
        reader,
        registry,
    )
}

fn request_full_stack_lifecycle_startup(
    ros_setup: &Path,
    install_setup: &Path,
    domain_id: &str,
    stack: &mut std::process::Child,
    reader: &mut TraceReader,
    registry: &mut CallbackRegistry,
) -> Result<usize, String> {
    let localization = request_lifecycle_manager_startup(
        "localization",
        "/lifecycle_manager_localization/manage_nodes",
        &FULL_STACK_LOCALIZATION_NODES,
        ros_setup,
        install_setup,
        domain_id,
        stack,
        reader,
        registry,
    )?;
    let navigation = request_lifecycle_manager_startup(
        "navigation",
        "/lifecycle_manager_navigation/manage_nodes",
        &FULL_STACK_NAVIGATION_NODES,
        ros_setup,
        install_setup,
        domain_id,
        stack,
        reader,
        registry,
    )?;
    Ok(localization + navigation)
}

fn managed_full_stack_startup_enabled() -> bool {
    std::env::var("R2D2_NAV2_AUTOSTART")
        .ok()
        .map(|value| {
            matches!(
                value.as_str(),
                "0" | "false" | "False" | "FALSE" | "no" | "No"
            )
        })
        .unwrap_or(false)
}

fn wait_full_stack_ready(
    ros_setup: &Path,
    install_setup: &Path,
    domain_id: &str,
    stack: &mut std::process::Child,
    reader: &mut TraceReader,
    registry: &mut CallbackRegistry,
) -> Result<usize, String> {
    let mut registrations = 0usize;
    if managed_full_stack_startup_enabled() {
        registrations += request_full_stack_lifecycle_startup(
            ros_setup,
            install_setup,
            domain_id,
            stack,
            reader,
            registry,
        )?;
    }
    let deadline = Instant::now() + startup_timeout();
    let mut last_progress = Instant::now();
    let mut last_states = Vec::new();
    let mut max_active_count = 0usize;
    while Instant::now() < deadline {
        if !stack_alive(stack) {
            return Err("Nav2 stack died before lifecycle nodes became active".to_string());
        }
        let mut all_active = true;
        let mut states = Vec::with_capacity(FULL_STACK_NODES.len());
        for node in FULL_STACK_NODES {
            let state = lifecycle_state_for(ros_setup, install_setup, domain_id, node)?;
            if state != LifecycleState::Active {
                all_active = false;
            }
            states.push(state);
        }
        // Fast DDS graph snapshots can temporarily omit otherwise active
        // action/service endpoints in this heavily instrumented stack.  The
        // lifecycle state plus the registration-settle barrier below gives us
        // a stable readiness signal without rejecting a healthy stack because
        // one ros2 CLI graph query was incomplete.
        if all_active {
            return Ok(registrations);
        }
        let new_registrations = drain_startup_trace("startup lifecycle wait", reader, registry)?;
        registrations += new_registrations;
        let active_count = lifecycle_active_count(&states);
        let progressed = active_count > max_active_count;
        if progressed {
            last_progress = Instant::now();
            max_active_count = max_active_count.max(active_count);
        } else if Instant::now().duration_since(last_progress) >= FULL_STACK_LIFECYCLE_STALL_TIMEOUT
        {
            return Err(format!(
                "timeout waiting for full Nav2 lifecycle/action readiness; active lifecycle count stalled at {max_active_count}/{} for {}s after {registrations} registration records: {}",
                FULL_STACK_NODES.len(),
                FULL_STACK_LIFECYCLE_STALL_TIMEOUT.as_secs(),
                format_full_stack_lifecycle_states(&states)
            ));
        }
        last_states = states;
        thread::sleep(STARTUP_POLL);
    }
    Err(format!(
        "timeout waiting for full Nav2 lifecycle/action readiness; last states: {}",
        format_full_stack_lifecycle_states(&last_states)
    ))
}

fn lifecycle_active_count(states: &[LifecycleState]) -> usize {
    states
        .iter()
        .filter(|state| **state == LifecycleState::Active)
        .count()
}

fn format_full_stack_lifecycle_states(states: &[LifecycleState]) -> String {
    if states.is_empty() {
        return "unobserved".to_string();
    }
    FULL_STACK_NODES
        .iter()
        .zip(states.iter())
        .map(|(node, state)| format!("{node}={state:?}"))
        .collect::<Vec<_>>()
        .join(", ")
}

fn wait_for_full_registration_settle(
    stack: &mut std::process::Child,
    reader: &mut TraceReader,
    registry: &mut CallbackRegistry,
    round_label: &str,
) -> Result<usize, String> {
    let deadline = Instant::now() + STARTUP_REGISTRATION_SETTLE_TIMEOUT;
    let mut total = 0usize;
    let mut last_count = 0usize;
    let mut quiet = 0usize;
    while Instant::now() < deadline {
        if !stack_alive(stack) {
            return Err(
                "Nav2 stack died while waiting for callback registration settle".to_string(),
            );
        }
        let runtime = drain_runtime_snapshot(round_label, reader, registry)?;
        total += ingest_registration_updates(round_label, reader, registry)?;
        let count = registry.callback_infos().len();
        if count > 0 && count == last_count {
            quiet += 1;
            if quiet >= 3 {
                return Ok(total);
            }
        } else {
            quiet = 0;
            last_count = count;
        }
        if runtime.events.is_empty() && runtime.missed == 0 {
            thread::sleep(STARTUP_POLL);
        }
    }
    if registry.callback_infos().is_empty() {
        Err("full Nav2 startup registration completed with zero callbacks".to_string())
    } else {
        Ok(total)
    }
}

pub(super) struct RoundExecution {
    pub(super) trace: my_r2d2::callback_profile::CallbackTrace,
    pub(super) crashed: bool,
    pub(super) stack_health_error: Option<String>,
    pub(super) sched_name: Option<String>,
    pub(super) interface_label: String,
    pub(super) endpoint_label: String,
    pub(super) send_preview: Option<Ros2CommandPreview>,
}

#[allow(clippy::too_many_arguments)]
pub(super) fn execute_payload_round(
    round_index: u64,
    round_label: &str,
    payload: &Payload,
    binding: &InterfaceBinding,
    bindings: &[InterfaceBinding],
    registry: &mut CallbackRegistry,
    reader: &mut TraceReader,
    ros_setup: &Path,
    install_setup: &Path,
    payload_file: &Path,
    domain_id: &str,
    schedules: &[(String, Schedule)],
    config: &Config,
    input_sequence: Option<&InputSequence>,
    stack: &mut std::process::Child,
    session: &mut TraceSession,
) -> Result<RoundExecution, String> {
    let interface_label = binding.interface_id.clone();
    let endpoint_label = binding.endpoint_name().to_string();
    let mut sched_name: Option<String> = None;
    let mut send_preview: Option<Ros2CommandPreview> = None;
    execute_input_sequence_phase(
        input_sequence,
        InputSequencePhase::RoundPrefix,
        &format!("{round_label} input-sequence"),
        bindings,
        registry,
        reader,
        ros_setup,
        install_setup,
        domain_id,
        false,
    )?;
    match &binding.endpoint {
        EndpointBinding::LaserScan { .. } => {
            let schedule_slot = schedules.get((round_index as usize - 1) % schedules.len().max(1));
            let (rate_hz, duration_sec, burst, burst_gap, max_publishes, seed_stamp_mode) =
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
            let stamp_mode = config
                .scan_stamp_mode_override
                .clone()
                .unwrap_or(seed_stamp_mode);
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
            if let Err(error) = sender.send(payload) {
                eprintln!("{round_label}: {error}");
            }
        }
        EndpointBinding::Topic {
            topic_name,
            message_type,
            options,
        } => {
            send_preview = preview_ros2_topic_command(
                &binding.interface,
                topic_name,
                message_type,
                options,
                payload,
            )
            .ok();
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
        }
        EndpointBinding::Service {
            service_name,
            service_type,
            timeout_sec,
        } => {
            send_preview = preview_ros2_service_command(
                &binding.interface,
                service_name,
                service_type,
                payload,
            )
            .ok();
            let sender = Ros2ServiceSender::new(
                ros_setup,
                install_setup,
                domain_id,
                service_name.clone(),
                service_type.clone(),
                binding.interface.clone(),
            )
            .with_timeout_sec(*timeout_sec);
            if let Err(error) = sender.send(payload) {
                eprintln!("{round_label}: {error}");
            }
        }
        EndpointBinding::Action {
            action_name,
            action_type,
        } => {
            send_preview =
                preview_ros2_action_command(&binding.interface, action_name, action_type, payload)
                    .ok();
            let sender = Ros2ActionSender::new(
                ros_setup,
                install_setup,
                domain_id,
                action_name.clone(),
                action_type.clone(),
                binding.interface.clone(),
            );
            if let Err(error) = sender.send(payload) {
                eprintln!("{round_label}: {error}");
            }
        }
        EndpointBinding::Parameter {
            node_name,
            parameter_name,
            profile,
            ..
        } => {
            let value = safe_parameter_payload_value(payload, *profile);
            send_preview = preview_ros2_parameter_command(node_name, parameter_name, &value).ok();
            if let Err(error) = send_ros2_parameter_value(
                ros_setup,
                install_setup,
                domain_id,
                node_name,
                parameter_name,
                &value,
            ) {
                eprintln!("{round_label}: {error}");
            }
        }
    }
    let drained = drain_round_runtime_until_quiet(
        round_label,
        session,
        reader,
        registry,
        config.round_settle_timeout,
    )?;
    let trace = profile_trace(registry, &drained);
    let parameter_restore_error = restore_parameter_after_round(
        round_label,
        binding,
        ros_setup,
        install_setup,
        domain_id,
        reader,
        registry,
    )
    .err();
    cleanup_nav2_action_goal_if_needed(
        round_label,
        binding,
        config,
        ros_setup,
        install_setup,
        domain_id,
        reader,
        registry,
    );
    let mut crashed = !stack_alive(stack);
    let mut stack_health_error = parameter_restore_error;
    if stack_health_error.is_some() {
        crashed = true;
    }
    if let Err(error) =
        ensure_full_stack_nodes_visible(round_label, ros_setup, install_setup, domain_id, stack)
    {
        crashed = true;
        stack_health_error = Some(error);
    }

    Ok(RoundExecution {
        trace,
        crashed,
        stack_health_error,
        sched_name,
        interface_label,
        endpoint_label,
        send_preview,
    })
}

fn restore_parameter_after_round(
    round_label: &str,
    binding: &InterfaceBinding,
    ros_setup: &Path,
    install_setup: &Path,
    domain_id: &str,
    reader: &mut TraceReader,
    registry: &mut CallbackRegistry,
) -> Result<(), String> {
    let EndpointBinding::Parameter {
        node_name,
        parameter_name,
        profile,
        ..
    } = &binding.endpoint
    else {
        return Ok(());
    };
    let restore_value = safe_parameter_restore_value(*profile);
    send_ros2_parameter_value(
        ros_setup,
        install_setup,
        domain_id,
        node_name,
        parameter_name,
        &restore_value,
    )
    .map_err(|error| {
        format!("{round_label}: parameter restore {node_name}.{parameter_name} failed: {error}")
    })?;
    let _ = drain_runtime_snapshot(
        &format!("{round_label} parameter restore"),
        reader,
        registry,
    );
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn cleanup_nav2_action_goal_if_needed(
    round_label: &str,
    binding: &InterfaceBinding,
    config: &Config,
    ros_setup: &Path,
    install_setup: &Path,
    domain_id: &str,
    reader: &mut TraceReader,
    registry: &mut CallbackRegistry,
) {
    if !config.action_cleanup {
        return;
    }
    let Some(cancel_service) = nav2_cancel_service_for_binding(binding) else {
        return;
    };
    let payload = cancel_all_goals_payload();
    if let Err(error) = send_ros2_service_yaml(
        ros_setup,
        install_setup,
        domain_id,
        &cancel_service,
        NAV2_ACTION_CANCEL_TYPE,
        &payload,
    ) {
        eprintln!("{round_label}: action cleanup {cancel_service} failed: {error}");
    }
    let _ = drain_runtime_snapshot(&format!("{round_label} action cleanup"), reader, registry);
}

fn nav2_cancel_service_for_binding(binding: &InterfaceBinding) -> Option<String> {
    match &binding.endpoint {
        EndpointBinding::Topic { topic_name, .. }
            if binding.interface_id == "PoseStamped" && topic_name == "/goal_pose" =>
        {
            Some("/navigate_to_pose/_action/cancel_goal".to_string())
        }
        EndpointBinding::Action { action_name, .. } => Some(format!(
            "{}/_action/cancel_goal",
            action_name.trim_end_matches('/')
        )),
        _ => None,
    }
}

fn cancel_all_goals_payload() -> serde_yaml::Value {
    serde_yaml::from_str(
        r#"
goal_info:
  goal_id:
    uuid: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
  stamp:
    sec: 0
    nanosec: 0
"#,
    )
    .expect("static CancelGoal payload is valid YAML")
}
