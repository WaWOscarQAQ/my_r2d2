use my_r2d2::callback_profile::{CallbackRegistry, profile_trace};
use my_r2d2::trace_buffer::{RegistrationSource, RuntimeEventType, TraceReader};
use my_r2d2::utils::yaml_reader::YamlEnv;
use std::collections::{BTreeMap, BTreeSet, HashMap};
use std::path::{Path, PathBuf};

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let shm_path = if args.len() > 1 {
        PathBuf::from(&args[1])
    } else {
        let env = YamlEnv::load(Path::new(env!("CARGO_MANIFEST_DIR")))
            .expect("load config/r2d2_env.yaml");
        env.require_path("R2D2_SHM_PATH")
            .expect("resolve R2D2_SHM_PATH")
    };

    let mut reader = TraceReader::open(&shm_path).expect("open trace shm");
    let registration = reader
        .drain_registration()
        .expect("drain registration ring");
    let runtime = reader.drain_runtime().expect("drain runtime ring");

    let mut registry = CallbackRegistry::new();
    registry.ingest(&registration);
    let infos = registry.callback_infos();
    let trace = profile_trace(&registry, &runtime);

    let mut counts: BTreeMap<&'static str, usize> = BTreeMap::new();
    for event in &runtime.events {
        let key = match event.event_type {
            RuntimeEventType::ExecutorExecute => "executor_execute",
            RuntimeEventType::CallbackStart => "callback_start",
            RuntimeEventType::CallbackEnd => "callback_end",
            RuntimeEventType::RclTake => "rcl_take",
            RuntimeEventType::RoundBoundary => "round_boundary",
        };
        *counts.entry(key).or_insert(0) += 1;
    }
    let known_sched = trace
        .call_trace
        .iter()
        .filter(|call| call.scheduling_latency.is_some())
        .count();
    let unknown_sched = trace.call_trace.len().saturating_sub(known_sched);

    let mut rcl_names: HashMap<u64, String> = HashMap::new();
    let mut rclcpp_types: HashMap<u64, String> = HashMap::new();
    let mut rclcpp_handlers_by_rcl: HashMap<u64, u64> = HashMap::new();
    for event in &registration.events {
        match event.source {
            RegistrationSource::Rcl => {
                rcl_names.insert(
                    event.rcl_handler,
                    format!("{}/{}", event.callback_namespace, event.callback_name),
                );
            }
            RegistrationSource::Rclcpp => {
                rclcpp_types.insert(event.rcl_handler, format!("{:?}", event.callback_type));
                rclcpp_handlers_by_rcl.insert(event.rcl_handler, event.rclcpp_handler);
            }
        }
    }

    let complete_rcl: BTreeSet<u64> = infos.iter().map(|info| info.rcl_handler).collect();
    let complete_rclcpp: BTreeSet<u64> = infos.iter().map(|info| info.rclcpp_handler).collect();
    let mut only_rcl = Vec::new();
    let mut only_rclcpp = Vec::new();
    for handler in rcl_names.keys() {
        if !complete_rcl.contains(handler) {
            only_rcl.push((*handler, rcl_names[handler].clone()));
        }
    }
    for (rcl_handler, callback_type) in &rclcpp_types {
        if !complete_rcl.contains(rcl_handler) {
            let rclcpp_handler = rclcpp_handlers_by_rcl
                .get(rcl_handler)
                .copied()
                .unwrap_or_default();
            only_rclcpp.push((*rcl_handler, rclcpp_handler, callback_type.clone()));
        }
    }
    only_rcl.sort_unstable_by_key(|(handler, _)| *handler);
    only_rclcpp.sort_unstable_by_key(|(rcl_handler, _, _)| *rcl_handler);

    let mut unknown_runtime: BTreeMap<u64, BTreeMap<&'static str, usize>> = BTreeMap::new();
    for event in &runtime.events {
        match event.event_type {
            RuntimeEventType::ExecutorExecute
            | RuntimeEventType::CallbackStart
            | RuntimeEventType::CallbackEnd => {
                if !complete_rclcpp.contains(&event.rclcpp_handler) {
                    let bucket = unknown_runtime.entry(event.rclcpp_handler).or_default();
                    let key = match event.event_type {
                        RuntimeEventType::ExecutorExecute => "executor_execute",
                        RuntimeEventType::CallbackStart => "callback_start",
                        RuntimeEventType::CallbackEnd => "callback_end",
                        _ => unreachable!(),
                    };
                    *bucket.entry(key).or_insert(0) += 1;
                }
            }
            RuntimeEventType::RclTake => {
                if !complete_rcl.contains(&event.rcl_handler) {
                    let bucket = unknown_runtime.entry(event.rcl_handler).or_default();
                    *bucket.entry("rcl_take").or_insert(0) += 1;
                }
            }
            RuntimeEventType::RoundBoundary => {}
        }
    }

    println!("shm={}", shm_path.display());
    println!(
        "registration_records={} complete_callbacks={} runtime_records={}",
        registration.events.len(),
        infos.len(),
        runtime.events.len()
    );
    println!(
        "runtime_counts executor_execute={} callback_start={} callback_end={} rcl_take={} round_boundary={}",
        counts.get("executor_execute").copied().unwrap_or(0),
        counts.get("callback_start").copied().unwrap_or(0),
        counts.get("callback_end").copied().unwrap_or(0),
        counts.get("rcl_take").copied().unwrap_or(0),
        counts.get("round_boundary").copied().unwrap_or(0),
    );
    println!(
        "profile calls={} msgs={} scheduling_known={} scheduling_unknown={} missing_invokes={} unknown_handlers={} incomplete_registrations={} registration_conflicts={}",
        trace.call_trace.len(),
        trace.msg_trace.len(),
        known_sched,
        unknown_sched,
        trace.diagnostics.missing_invokes,
        trace.diagnostics.unknown_handlers,
        trace.diagnostics.incomplete_registrations,
        trace.diagnostics.registration_conflicts,
    );
    if !infos.is_empty() {
        println!(
            "callbacks={}",
            infos
                .iter()
                .map(|info| format!("{} [{:?}]", info.name, info.callback_type))
                .collect::<Vec<_>>()
                .join(", ")
        );
    }
    if !only_rcl.is_empty() {
        println!(
            "only_rcl={}",
            only_rcl
                .iter()
                .map(|(handler, name)| format!("0x{handler:x}:{name}"))
                .collect::<Vec<_>>()
                .join(", ")
        );
    }
    if !only_rclcpp.is_empty() {
        println!(
            "only_rclcpp={}",
            only_rclcpp
                .iter()
                .map(|(rcl_handler, rclcpp_handler, callback_type)| {
                    format!("rcl=0x{rcl_handler:x}/rclcpp=0x{rclcpp_handler:x}:{callback_type}")
                })
                .collect::<Vec<_>>()
                .join(", ")
        );
    }
    if !unknown_runtime.is_empty() {
        println!("unknown_runtime_handlers={}", unknown_runtime.len());
        for (handler, event_counts) in unknown_runtime {
            let detail = event_counts
                .iter()
                .map(|(name, count)| format!("{name}={count}"))
                .collect::<Vec<_>>()
                .join(",");
            println!("unknown_handler 0x{handler:x} {detail}");
        }
    }
    println!("raw_registration_events={}", registration.events.len());
    for event in &registration.events {
        match event.source {
            RegistrationSource::Rcl => {
                println!(
                    "reg rcl     rcl=0x{:x} name={}/{}",
                    event.rcl_handler, event.callback_namespace, event.callback_name
                );
            }
            RegistrationSource::Rclcpp => {
                println!(
                    "reg rclcpp  rcl=0x{:x} rclcpp=0x{:x} type={:?}",
                    event.rcl_handler, event.rclcpp_handler, event.callback_type
                );
            }
        }
    }
}
