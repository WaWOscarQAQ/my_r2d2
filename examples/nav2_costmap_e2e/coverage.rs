use super::*;
use my_r2d2::runtime::command_template::{CommandParameters, runtime_command};
use std::io::{self, Read};

struct TargetPackage {
    name: &'static str,
    expected_branches: u64,
}

struct SancovPackage {
    name: &'static str,
    executable: &'static str,
    modules: &'static [&'static str],
}

const TARGET_PACKAGES: [TargetPackage; 9] = [
    TargetPackage {
        name: "nav2_amcl",
        expected_branches: 3936,
    },
    TargetPackage {
        name: "nav2_behaviors",
        expected_branches: 1915,
    },
    TargetPackage {
        name: "nav2_bt_navigator",
        expected_branches: 1438,
    },
    TargetPackage {
        name: "nav2_controller",
        expected_branches: 3268,
    },
    TargetPackage {
        name: "nav2_costmap_2d",
        expected_branches: 11295,
    },
    TargetPackage {
        name: "nav2_lifecycle_manager",
        expected_branches: 1303,
    },
    TargetPackage {
        name: "nav2_map_server",
        expected_branches: 3841,
    },
    TargetPackage {
        name: "nav2_planner",
        expected_branches: 1503,
    },
    TargetPackage {
        name: "nav2_smoother",
        expected_branches: 1590,
    },
];

const SANCOV_TARGET_PACKAGES: [SancovPackage; 13] = [
    SancovPackage {
        name: "nav2_amcl",
        executable: "amcl",
        modules: &[
            "amcl",
            "libamcl_core.so",
            "libmap_lib.so",
            "libmotions_lib.so",
            "libpf_lib.so",
            "libsensors_lib.so",
        ],
    },
    SancovPackage {
        name: "nav2_behaviors",
        executable: "behavior_server",
        modules: &[
            "behavior_server",
            "libbehavior_server_core.so",
            "libnav2_assisted_teleop_behavior.so",
            "libnav2_back_up_behavior.so",
            "libnav2_drive_on_heading_behavior.so",
            "libnav2_spin_behavior.so",
            "libnav2_wait_behavior.so",
        ],
    },
    SancovPackage {
        name: "nav2_bt_navigator",
        executable: "bt_navigator",
        modules: &[
            "bt_navigator",
            "libbt_navigator_core.so",
            "libnav2_navigate_through_poses.so",
            "libnav2_navigate_to_pose_navigator.so",
        ],
    },
    SancovPackage {
        name: "nav2_controller",
        executable: "controller_server",
        modules: &[
            "controller_server",
            "libadaptive_tolerance_goal_checker.so",
            "libaxis_goal_checker.so",
            "libcontroller_server_core.so",
            "libfeasible_path_handler.so",
            "libpose_progress_checker.so",
            "libposition_goal_checker.so",
            "libsimple_goal_checker.so",
            "libsimple_progress_checker.so",
            "libstopped_goal_checker.so",
        ],
    },
    SancovPackage {
        name: "nav2_costmap_2d",
        executable: "nav2_costmap_2d",
        modules: &[
            "nav2_costmap_2d",
            "libnav2_costmap_2d_client.so",
            "libnav2_costmap_2d_core.so",
            "liblayers.so",
            "libfilters.so",
        ],
    },
    SancovPackage {
        name: "nav2_lifecycle_manager",
        executable: "lifecycle_manager",
        modules: &["lifecycle_manager", "libnav2_lifecycle_manager_core.so"],
    },
    SancovPackage {
        name: "nav2_map_server",
        executable: "map_server",
        modules: &[
            "map_server",
            "libmap_io.so",
            "libmap_server_core.so",
            "libvector_object_core.so",
        ],
    },
    SancovPackage {
        name: "nav2_planner",
        executable: "planner_server",
        modules: &["planner_server", "libplanner_server_core.so"],
    },
    SancovPackage {
        name: "nav2_smoother",
        executable: "smoother_server",
        modules: &[
            "smoother_server",
            "libsavitzky_golay_smoother.so",
            "libsimple_smoother.so",
            "libsmoother_server_core.so",
        ],
    },
    SancovPackage {
        name: "nav2_behavior_tree",
        executable: "bt_navigator",
        modules: &[
            "libcheck_pose_occupancy_action_bt_node.so",
            "libcheck_stop_status_action_bt_node.so",
            "libnav2_append_goal_pose_to_goals_action_bt_node.so",
            "libnav2_are_error_codes_active_condition_bt_node.so",
            "libnav2_are_poses_near_condition_bt_node.so",
            "libnav2_assisted_teleop_action_bt_node.so",
            "libnav2_assisted_teleop_cancel_bt_node.so",
            "libnav2_back_up_action_bt_node.so",
            "libnav2_back_up_cancel_bt_node.so",
            "libnav2_behavior_tree.so",
            "libnav2_clear_costmap_service_bt_node.so",
            "libnav2_compute_and_track_route_bt_node.so",
            "libnav2_compute_and_track_route_cancel_bt_node.so",
            "libnav2_compute_path_through_poses_action_bt_node.so",
            "libnav2_compute_path_to_pose_action_bt_node.so",
            "libnav2_compute_route_bt_node.so",
            "libnav2_concatenate_paths_action_bt_node.so",
            "libnav2_controller_cancel_bt_node.so",
            "libnav2_controller_selector_bt_node.so",
            "libnav2_distance_controller_bt_node.so",
            "libnav2_distance_traveled_condition_bt_node.so",
            "libnav2_drive_on_heading_bt_node.so",
            "libnav2_drive_on_heading_cancel_bt_node.so",
            "libnav2_extract_route_nodes_as_goals_action_bt_node.so",
            "libnav2_follow_path_action_bt_node.so",
            "libnav2_get_current_pose_action_bt_node.so",
            "libnav2_get_next_few_goals_action_bt_node.so",
            "libnav2_get_pose_from_path_action_bt_node.so",
            "libnav2_globally_updated_goal_condition_bt_node.so",
            "libnav2_goal_checker_selector_bt_node.so",
            "libnav2_goal_reached_condition_bt_node.so",
            "libnav2_goal_updated_condition_bt_node.so",
            "libnav2_goal_updated_controller_bt_node.so",
            "libnav2_goal_updater_node_bt_node.so",
            "libnav2_initial_pose_received_condition_bt_node.so",
            "libnav2_is_battery_charging_condition_bt_node.so",
            "libnav2_is_battery_low_condition_bt_node.so",
            "libnav2_is_goal_nearby_condition_bt_node.so",
            "libnav2_is_stuck_condition_bt_node.so",
            "libnav2_is_within_path_tracking_bounds_condition_bt_node.so",
            "libnav2_navigate_through_poses_action_bt_node.so",
            "libnav2_navigate_to_pose_action_bt_node.so",
            "libnav2_nonblocking_sequence_bt_node.so",
            "libnav2_path_expiring_timer_condition_bt_node.so",
            "libnav2_path_handler_selector_bt_node.so",
            "libnav2_path_longer_on_approach_bt_node.so",
            "libnav2_pause_resume_controller_bt_node.so",
            "libnav2_persistent_sequence_bt_node.so",
            "libnav2_pipeline_sequence_bt_node.so",
            "libnav2_planner_selector_bt_node.so",
            "libnav2_progress_checker_selector_bt_node.so",
            "libnav2_rate_controller_bt_node.so",
            "libnav2_recovery_node_bt_node.so",
            "libnav2_reinitialize_global_localization_service_bt_node.so",
            "libnav2_remove_in_collision_goals_action_bt_node.so",
            "libnav2_remove_passed_goals_action_bt_node.so",
            "libnav2_round_robin_node_bt_node.so",
            "libnav2_single_trigger_bt_node.so",
            "libnav2_smooth_path_action_bt_node.so",
            "libnav2_smoother_selector_bt_node.so",
            "libnav2_speed_controller_bt_node.so",
            "libnav2_spin_action_bt_node.so",
            "libnav2_spin_cancel_bt_node.so",
            "libnav2_time_expired_condition_bt_node.so",
            "libnav2_toggle_collision_monitor_service_bt_node.so",
            "libnav2_transform_available_condition_bt_node.so",
            "libnav2_truncate_path_action_bt_node.so",
            "libnav2_truncate_path_local_action_bt_node.so",
            "libnav2_wait_action_bt_node.so",
            "libnav2_wait_cancel_bt_node.so",
            "libnav2_would_a_controller_recovery_help_condition_bt_node.so",
            "libnav2_would_a_planner_recovery_help_condition_bt_node.so",
            "libnav2_would_a_route_recovery_help_condition_bt_node.so",
            "libnav2_would_a_smoother_recovery_help_condition_bt_node.so",
            "libopennav_follow_action_bt_node.so",
            "libopennav_follow_cancel_bt_node.so",
            "libvalidate_path_action_bt_node.so",
        ],
    },
    SancovPackage {
        name: "nav2_mppi_controller",
        executable: "controller_server",
        modules: &[
            "libmppi_controller.so",
            "libmppi_critics.so",
            "libmppi_motion_models.so",
            "libmppi_trajectory_validators.so",
        ],
    },
    SancovPackage {
        name: "nav2_navfn_planner",
        executable: "planner_server",
        modules: &["libnav2_navfn_planner.so"],
    },
    SancovPackage {
        name: "nav2_util",
        executable: "multiple",
        modules: &["libnav2_util_core.so"],
    },
];

const COVERAGE_PROCESS_MARKERS: [&str; 9] = [
    "/lib/nav2_amcl/amcl",
    "/lib/nav2_behaviors/behavior_server",
    "/lib/nav2_bt_navigator/bt_navigator",
    "/lib/nav2_controller/controller_server",
    "/lib/nav2_costmap_2d/nav2_costmap_2d",
    "/lib/nav2_lifecycle_manager/lifecycle_manager",
    "/lib/nav2_map_server/map_server",
    "/lib/nav2_planner/planner_server",
    "/lib/nav2_smoother/smoother_server",
];

/// gcov 计数只在进程退出或 __gcov_dump() 时落盘；每轮只向目标 ROS 进程
/// 发 SIGUSR1，不能发给进程组 leader（bash/launch 会被 SIGUSR1 结束）。
pub(super) fn flush_stack_coverage(stack_pgid: u32) {
    flush_stack_signal(stack_pgid, "-USR1");
}

const SANCOV_FLUSH_MODE_ENV: &str = "R2D2_SANCOV_FLUSH";

pub(super) fn flush_stack_sancov(stack_pgid: u32) {
    match std::env::var(SANCOV_FLUSH_MODE_ENV).as_deref() {
        Ok("signal") => {
            flush_stack_signal(stack_pgid, "-USR2");
            thread::sleep(Duration::from_secs(1));
        }
        Ok("final-signal") | Ok("exit") | Ok("off") | Err(_) => {}
        Ok(other) => {
            eprintln!(
                "sancov: unsupported {SANCOV_FLUSH_MODE_ENV}={other}; using exit-only capture"
            );
        }
    }
}

pub(super) fn flush_stack_sancov_final(stack_pgid: u32) {
    match std::env::var(SANCOV_FLUSH_MODE_ENV).as_deref() {
        Ok("signal") | Ok("final-signal") => {
            flush_stack_signal(stack_pgid, "-USR2");
            thread::sleep(Duration::from_secs(1));
        }
        Ok("exit") | Ok("off") | Err(_) => {}
        Ok(other) => {
            eprintln!(
                "sancov: unsupported {SANCOV_FLUSH_MODE_ENV}={other}; using exit-only capture"
            );
        }
    }
}

fn flush_stack_signal(stack_pgid: u32, signal: &str) {
    let pids = coverage_target_pids(stack_pgid);
    if pids.is_empty() {
        eprintln!("coverage: no target ROS process found in process group {stack_pgid}");
    } else if std::env::var("R2D2_COVERAGE_DEBUG").is_ok() {
        eprintln!("coverage: {signal} targets in pgid {stack_pgid}: {pids:?}");
        for pid in &pids {
            if let Ok(env) = fs::read(format!("/proc/{pid}/environ")) {
                for entry in env.split(|byte| *byte == 0) {
                    if entry.starts_with(b"TSAN_OPTIONS=") {
                        eprintln!("coverage: pid {pid} {}", String::from_utf8_lossy(entry));
                    }
                }
            }
            if let Ok(status) = fs::read_to_string(format!("/proc/{pid}/status")) {
                for line in status.lines() {
                    if matches!(line.split(':').next(), Some("SigBlk" | "SigIgn" | "SigCgt")) {
                        eprintln!("coverage: pid {pid} {line}");
                    }
                }
            }
        }
    }
    for pid in pids {
        let mut parameters = CommandParameters::new();
        parameters
            .insert("signal", signal)
            .insert("pid", pid.to_string());
        if !runtime_command("signal_process", &parameters)
            .is_ok_and(|mut command| command.status().is_ok_and(|status| status.success()))
        {
            eprintln!("coverage: {signal} to target process {pid} failed");
        }
    }
    thread::sleep(Duration::from_secs(2));
}

fn coverage_target_pids(stack_pgid: u32) -> Vec<u32> {
    let Ok(mut command) = runtime_command("process_list", &CommandParameters::new()) else {
        return Vec::new();
    };
    let Ok(output) = command.output() else {
        return Vec::new();
    };
    String::from_utf8_lossy(&output.stdout)
        .lines()
        .filter_map(|line| {
            let mut parts = line.split_whitespace();
            let pid = parts.next()?.parse::<u32>().ok()?;
            let pgid = parts.next()?.parse::<u32>().ok()?;
            let args = parts.collect::<Vec<_>>().join(" ");
            (pgid == stack_pgid
                && pid != stack_pgid
                && COVERAGE_PROCESS_MARKERS
                    .iter()
                    .any(|marker| args.contains(marker)))
            .then_some(pid)
        })
        .collect()
}

pub(super) fn clear_stale_gcda(build_dir: &Path) {
    let mut removed = 0u64;
    if let Err(error) = remove_gcda_recursive(build_dir, &mut removed) {
        eprintln!("coverage: failed to clear stale .gcda files: {error}");
        return;
    }
    if removed > 0 {
        println!("coverage: cleared {removed} stale .gcda files");
    }
}

fn remove_gcda_recursive(path: &Path, removed: &mut u64) -> io::Result<()> {
    let Ok(entries) = fs::read_dir(path) else {
        return Ok(());
    };
    for entry in entries {
        let entry = entry?;
        let entry_path = entry.path();
        let file_type = entry.file_type()?;
        if file_type.is_dir() {
            remove_gcda_recursive(&entry_path, removed)?;
            continue;
        }
        if entry_path.extension().is_some_and(|ext| ext == "gcda") {
            fs::remove_file(&entry_path)?;
            *removed += 1;
        }
    }
    Ok(())
}

/// lcov --summary 的 branches 行（"branches......: 51.0% (1234 of 2420 branches)"）
/// 解析为 (covered, total)。
pub(super) fn branch_totals_from_summary(summary_text: &str) -> Option<(u64, u64)> {
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
fn lcov_capture_command(
    gcov_tool: Option<&str>,
    build_dir: &Path,
    output_file: &Path,
) -> Result<std::process::Command, String> {
    let mut gcov_tool_args = Vec::new();
    if let Some(tool) = gcov_tool {
        for token in tool.split_whitespace() {
            gcov_tool_args.extend(["--gcov-tool".to_string(), token.to_string()]);
        }
    }
    let mut parameters = CommandParameters::new();
    parameters
        .insert_many("gcov_tool_args", gcov_tool_args)
        .insert_path("build_dir", build_dir)
        .insert("ignore_errors", lcov_ignore_errors())
        .insert_path("output_file", output_file);
    runtime_command("lcov_capture", &parameters)
}

fn lcov_ignore_errors() -> &'static str {
    "mismatch,mismatch,empty,gcov,negative"
}

/// 累计分支计数只增不减。SIGUSR1 触发的 `__gcov_dump()` 与进行中的回调
/// 并发执行时，快照会漏掉该回调尚未走到的分支，导致读数低于上一轮；此时
/// 以上一轮读数作为累计下界，并标记本轮捕获为 dip。
pub(super) fn cumulative_branches(captured: u64, previous: u64) -> (u64, bool) {
    if captured < previous {
        (previous, true)
    } else {
        (captured, false)
    }
}

/// 抓取一轮累计覆盖：lcov --capture 整个 workspace build 目录，返回
/// branches (covered, total)；失败返回 None（不影响 fuzzing 主循环）。
pub(super) fn capture_round_coverage(
    build_dir: &Path,
    round_dir: &Path,
    gcov_tool: Option<&str>,
) -> Option<(u64, u64)> {
    fs::create_dir_all(round_dir).ok()?;
    let info_path = round_dir.join("coverage.info");
    let status = lcov_capture_command(gcov_tool, build_dir, &info_path)
        .ok()?
        .status()
        .ok()?;
    if !status.success() {
        eprintln!("coverage: lcov capture failed for {}", round_dir.display());
        return None;
    }
    let mut summary_parameters = CommandParameters::new();
    summary_parameters.insert_path("info_file", &info_path);
    let summary = runtime_command("lcov_summary", &summary_parameters)
        .ok()?
        .output()
        .ok()?;
    if !summary.status.success() {
        return None;
    }
    branch_totals_from_summary(&String::from_utf8_lossy(&summary.stdout))
}

pub(super) struct SancovCoverage {
    pub(super) all_covered_pcs: u64,
    pub(super) all_pc_increase: u64,
    pub(super) target_covered_pcs: u64,
    pub(super) target_pc_increase: u64,
    pub(super) packages: Vec<serde_json::Value>,
}

pub(super) fn capture_sancov_coverage(
    sancov_root: &Path,
    round_dir: &Path,
    seen_pcs: &mut BTreeSet<String>,
) -> Option<SancovCoverage> {
    fs::create_dir_all(round_dir).ok()?;
    let raw_dir = round_dir.join("sancov_raw");
    let _ = fs::remove_dir_all(&raw_dir);
    fs::create_dir_all(&raw_dir).ok()?;
    let mut current = BTreeSet::new();
    for path in sancov_files(sancov_root) {
        let _ = fs::copy(&path, raw_dir.join(path.file_name()?));
        read_sancov_pcs(&path, &mut current);
    }
    let all_before = seen_pcs.len();
    let target_before = sancov_target_pc_count(seen_pcs);
    seen_pcs.extend(current);
    let all_after = seen_pcs.len() as u64;
    let target_after = sancov_target_pc_count(seen_pcs);
    let packages = package_sancov_reports(seen_pcs, &round_dir.join("packages"));
    Some(SancovCoverage {
        all_covered_pcs: all_after,
        all_pc_increase: all_after - all_before as u64,
        target_covered_pcs: target_after,
        target_pc_increase: target_after - target_before,
        packages,
    })
}

fn sancov_files(root: &Path) -> Vec<PathBuf> {
    let Ok(entries) = fs::read_dir(root) else {
        return Vec::new();
    };
    entries
        .filter_map(Result::ok)
        .map(|entry| entry.path())
        .filter(|path| path.extension().is_some_and(|ext| ext == "sancov"))
        .collect()
}

fn read_sancov_pcs(path: &Path, pcs: &mut BTreeSet<String>) {
    let Some(module) = sancov_module(path) else {
        return;
    };
    let Ok(mut file) = fs::File::open(path) else {
        return;
    };
    let mut bytes = Vec::new();
    if file.read_to_end(&mut bytes).is_err() || bytes.len() < 16 {
        return;
    }
    let (chunks, _) = bytes[8..].as_chunks::<8>();
    for chunk in chunks {
        let pc = u64::from_le_bytes(*chunk);
        if pc != 0 {
            pcs.insert(format!("{module}:0x{pc:x}"));
        }
    }
}

fn sancov_module(path: &Path) -> Option<String> {
    let file = path.file_name()?.to_str()?;
    let stem = file.strip_suffix(".sancov")?;
    if let Some((module, pid)) = stem.rsplit_once('.')
        && pid.parse::<u32>().is_ok()
    {
        return Some(module.to_string());
    }
    Some(stem.to_string())
}

pub(super) fn package_sancov_reports(
    seen_pcs: &BTreeSet<String>,
    output_dir: &Path,
) -> Vec<serde_json::Value> {
    let _ = fs::create_dir_all(output_dir);
    let mut reports = Vec::new();
    for package in SANCOV_TARGET_PACKAGES {
        let covered = seen_pcs
            .iter()
            .filter(|pc| {
                package
                    .modules
                    .iter()
                    .any(|module| pc.starts_with(&format!("{module}:")))
            })
            .count() as u64;
        let report = json!({
            "package": package.name,
            "executable": package.executable,
            "modules": package.modules,
            "coverage_ok": covered > 0,
            "sancov_covered_pcs": covered,
        });
        write_json_report(&output_dir.join(format!("{}.json", package.name)), &report);
        reports.push(report);
    }
    write_json_report(&output_dir.join("packages.json"), &reports);
    reports
}

pub(super) fn sancov_target_pc_count(seen_pcs: &BTreeSet<String>) -> u64 {
    seen_pcs
        .iter()
        .filter(|pc| {
            SANCOV_TARGET_PACKAGES.iter().any(|package| {
                package
                    .modules
                    .iter()
                    .any(|module| pc.starts_with(&format!("{module}:")))
            })
        })
        .count() as u64
}

pub(super) fn package_coverage_reports(
    info_path: &Path,
    output_dir: &Path,
) -> Vec<serde_json::Value> {
    let _ = fs::create_dir_all(output_dir);
    let mut reports = Vec::new();
    for package in TARGET_PACKAGES {
        let package_info = output_dir.join(format!("{}.info", package.name));
        let pattern = format!("*/navigation2/{}/*", package.name);
        let mut extract_parameters = CommandParameters::new();
        extract_parameters
            .insert_path("info_file", info_path)
            .insert("source_pattern", &pattern)
            .insert_path("output_file", &package_info);
        let ok = runtime_command("lcov_extract", &extract_parameters)
            .is_ok_and(|mut command| command.status().is_ok_and(|status| status.success()));
        let totals = ok.then(|| {
            let mut summary_parameters = CommandParameters::new();
            summary_parameters.insert_path("info_file", &package_info);
            runtime_command("lcov_summary", &summary_parameters)
                .ok()
                .and_then(|mut command| command.output().ok())
                .and_then(|summary| {
                    summary.status.success().then(|| {
                        branch_totals_from_summary(&String::from_utf8_lossy(&summary.stdout))
                    })?
                })
        });
        let (covered, total) = totals.flatten().unwrap_or((0, 0));
        let report = json!({
            "package": package.name,
            "coverage_ok": ok && total > 0,
            "branch_covered_total": covered,
            "branch_total": total,
            "expected_branch_total": package.expected_branches,
            "matches_expected_branch_total": total == package.expected_branches,
            "source_filter": pattern,
        });
        write_json_report(&output_dir.join(format!("{}.json", package.name)), &report);
        reports.push(report);
    }
    write_json_report(&output_dir.join("packages.json"), &reports);
    reports
}

/// 收尾：costmap 进程组终止（exit 时 gcov 做最终 dump）后，抓取总覆盖、
/// 生成 genhtml HTML 报告，返回最终 branches (covered, total)。
pub(super) fn finalize_coverage(
    build_dir: &Path,
    lcov_root: &Path,
    gcov_tool: Option<&str>,
) -> Option<(u64, u64)> {
    let total_info = lcov_root.join("coverage_total.info");
    let status = lcov_capture_command(gcov_tool, build_dir, &total_info)
        .ok()?
        .status()
        .ok()?;
    if !status.success() {
        return None;
    }
    let html_dir = lcov_root.join("lcov_html");
    let _ = fs::remove_dir_all(&html_dir);
    fs::create_dir_all(&html_dir).ok()?;
    let mut html_parameters = CommandParameters::new();
    html_parameters
        .insert_path("info_file", &total_info)
        .insert_path("output_dir", &html_dir);
    let _ = runtime_command("genhtml", &html_parameters).map(|mut command| command.status());
    let mut summary_parameters = CommandParameters::new();
    summary_parameters.insert_path("info_file", &total_info);
    let summary = runtime_command("lcov_summary", &summary_parameters)
        .ok()?
        .output()
        .ok()?;
    if !summary.status.success() {
        return None;
    }
    branch_totals_from_summary(&String::from_utf8_lossy(&summary.stdout))
}
