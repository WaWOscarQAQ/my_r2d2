use super::*;

pub(super) fn ros_share_root(ros_setup: &Path) -> Result<PathBuf, String> {
    let ros_root = ros_setup
        .parent()
        .ok_or_else(|| format!("cannot infer ROS root from {}", ros_setup.display()))?;
    Ok(ros_root.join("share"))
}

fn take_interface(
    by_name: &mut BTreeMap<String, Interface>,
    name: &str,
) -> Result<Interface, String> {
    by_name
        .remove(name)
        .ok_or_else(|| format!("{name} interface not extracted"))
}

pub(super) fn extract_stack_bindings(
    interface_roots: &[PathBuf],
) -> Result<Vec<InterfaceBinding>, String> {
    let mut bindings = extract_costmap_bindings(interface_roots)?;
    bindings.extend(extract_nav2_full_stack_bindings(interface_roots)?);
    bindings.extend(safe_parameter_bindings());
    Ok(bindings)
}

fn manifest(binding: InterfaceBinding) -> InterfaceBinding {
    binding.with_source(InputSource::Manifest)
}

fn interface_path(interface_roots: &[PathBuf], relative: &str) -> PathBuf {
    interface_roots
        .iter()
        .map(|root| root.join(relative))
        .find(|path| path.exists())
        .unwrap_or_else(|| {
            interface_roots
                .first()
                .map(|root| root.join(relative))
                .unwrap_or_else(|| PathBuf::from(relative))
        })
}

/// Dry run：从 ROS 安装树及本地 Nav2 源码树提取 costmap topic/service 接口。
fn extract_costmap_bindings(interface_roots: &[PathBuf]) -> Result<Vec<InterfaceBinding>, String> {
    let extractor = FileExtractor::new(
        vec![
            interface_path(interface_roots, "sensor_msgs/msg/LaserScan.msg"),
            interface_path(interface_roots, "nav_msgs/msg/OccupancyGrid.msg"),
            interface_path(interface_roots, "map_msgs/msg/OccupancyGridUpdate.msg"),
            interface_path(interface_roots, "nav2_msgs/srv/GetCosts.srv"),
            interface_path(interface_roots, "nav2_msgs/srv/GetCostmap.srv"),
            interface_path(
                interface_roots,
                "nav2_msgs/srv/ClearCostmapExceptRegion.srv",
            ),
            interface_path(interface_roots, "nav2_msgs/srv/ClearCostmapAroundRobot.srv"),
            interface_path(interface_roots, "nav2_msgs/srv/ClearCostmapAroundPose.srv"),
            interface_path(interface_roots, "nav2_msgs/srv/ClearEntireCostmap.srv"),
        ],
        interface_roots.to_vec(),
    );
    let mut by_name = extractor
        .extract()
        .map_err(|e| e.to_string())?
        .into_iter()
        .map(|interface| (interface.name.clone(), interface))
        .collect::<BTreeMap<_, _>>();
    let mut bindings = vec![
        manifest(InterfaceBinding::new(
            take_interface(&mut by_name, "LaserScan")?,
            EndpointBinding::LaserScan {
                topic_name: "/scan".to_string(),
            },
        )),
        manifest(InterfaceBinding::new(
            take_interface(&mut by_name, "OccupancyGrid")?,
            EndpointBinding::Topic {
                topic_name: "/map".to_string(),
                message_type: "nav_msgs/msg/OccupancyGrid".to_string(),
                options: Ros2TopicOptions {
                    qos_depth: Some(1),
                    qos_reliability: Some("reliable".to_string()),
                    qos_durability: Some("transient_local".to_string()),
                    keep_alive_sec: 0.5,
                    ..Ros2TopicOptions::default()
                },
            },
        )),
    ];
    let get_cost = take_interface(&mut by_name, "GetCosts")?;
    let get_costmap = take_interface(&mut by_name, "GetCostmap")?;
    let clear_except = take_interface(&mut by_name, "ClearCostmapExceptRegion")?;
    let clear_around = take_interface(&mut by_name, "ClearCostmapAroundRobot")?;
    let clear_pose = take_interface(&mut by_name, "ClearCostmapAroundPose")?;
    let clear_entire = take_interface(&mut by_name, "ClearEntireCostmap")?;
    bindings.extend(full_costmap_services(
        get_cost,
        get_costmap,
        clear_except,
        clear_around,
        clear_pose,
        clear_entire,
    ));
    Ok(bindings)
}

fn full_costmap_services(
    get_cost: Interface,
    get_costmap: Interface,
    clear_except: Interface,
    clear_around: Interface,
    clear_pose: Interface,
    clear_entire: Interface,
) -> Vec<InterfaceBinding> {
    vec![
        manifest(service(
            get_cost.clone(),
            "/local_costmap/get_cost_local_costmap",
            "nav2_msgs/srv/GetCosts",
        )),
        manifest(service_alias(
            "GetCosts@global_costmap",
            get_cost,
            "/global_costmap/get_cost_global_costmap",
            "nav2_msgs/srv/GetCosts",
        )),
        service(
            get_costmap.clone(),
            "/local_costmap/get_costmap",
            "nav2_msgs/srv/GetCostmap",
        ),
        service_alias(
            "GetCostmap@local_voxel_layer",
            get_costmap.clone(),
            "/local_costmap/get_voxel_layer",
            "nav2_msgs/srv/GetCostmap",
        ),
        service_alias(
            "GetCostmap@global_costmap",
            get_costmap.clone(),
            "/global_costmap/get_costmap",
            "nav2_msgs/srv/GetCostmap",
        ),
        service_alias(
            "GetCostmap@global_obstacle_layer",
            get_costmap.clone(),
            "/global_costmap/get_obstacle_layer",
            "nav2_msgs/srv/GetCostmap",
        ),
        service_alias(
            "GetCostmap@global_static_layer",
            get_costmap,
            "/global_costmap/get_static_layer",
            "nav2_msgs/srv/GetCostmap",
        ),
        manifest(service(
            clear_except.clone(),
            "/local_costmap/clear_except_local_costmap",
            "nav2_msgs/srv/ClearCostmapExceptRegion",
        )),
        manifest(service_alias(
            "ClearCostmapExceptRegion@global_costmap",
            clear_except,
            "/global_costmap/clear_except_global_costmap",
            "nav2_msgs/srv/ClearCostmapExceptRegion",
        )),
        manifest(service(
            clear_around.clone(),
            "/local_costmap/clear_around_local_costmap",
            "nav2_msgs/srv/ClearCostmapAroundRobot",
        )),
        manifest(service_alias(
            "ClearCostmapAroundRobot@global_costmap",
            clear_around,
            "/global_costmap/clear_around_global_costmap",
            "nav2_msgs/srv/ClearCostmapAroundRobot",
        )),
        manifest(service(
            clear_pose.clone(),
            "/local_costmap/clear_around_pose_local_costmap",
            "nav2_msgs/srv/ClearCostmapAroundPose",
        )),
        manifest(service_alias(
            "ClearCostmapAroundPose@global_costmap",
            clear_pose,
            "/global_costmap/clear_around_pose_global_costmap",
            "nav2_msgs/srv/ClearCostmapAroundPose",
        )),
        manifest(service(
            clear_entire.clone(),
            "/local_costmap/clear_entirely_local_costmap",
            "nav2_msgs/srv/ClearEntireCostmap",
        )),
        manifest(service_alias(
            "ClearEntireCostmap@global_costmap",
            clear_entire,
            "/global_costmap/clear_entirely_global_costmap",
            "nav2_msgs/srv/ClearEntireCostmap",
        )),
    ]
}

fn extract_nav2_full_stack_bindings(
    interface_roots: &[PathBuf],
) -> Result<Vec<InterfaceBinding>, String> {
    let extractor = FileExtractor::new(
        vec![
            interface_path(
                interface_roots,
                "geometry_msgs/msg/PoseWithCovarianceStamped.msg",
            ),
            interface_path(interface_roots, "geometry_msgs/msg/PoseStamped.msg"),
            interface_path(interface_roots, "geometry_msgs/msg/TwistStamped.msg"),
            interface_path(interface_roots, "geometry_msgs/msg/Polygon.msg"),
            interface_path(interface_roots, "geometry_msgs/msg/PolygonStamped.msg"),
            interface_path(interface_roots, "tf2_msgs/msg/TFMessage.msg"),
            interface_path(interface_roots, "nav_msgs/msg/Odometry.msg"),
            interface_path(interface_roots, "nav2_msgs/msg/SpeedLimit.msg"),
            interface_path(interface_roots, "nav2_msgs/msg/CostmapFilterInfo.msg"),
            interface_path(interface_roots, "nav_msgs/msg/OccupancyGrid.msg"),
            interface_path(interface_roots, "nav2_msgs/msg/Costmap.msg"),
            interface_path(interface_roots, "nav2_msgs/msg/CostmapUpdate.msg"),
            interface_path(interface_roots, "std_msgs/msg/String.msg"),
            interface_path(interface_roots, "std_srvs/srv/Empty.srv"),
            interface_path(interface_roots, "std_srvs/srv/Trigger.srv"),
            interface_path(interface_roots, "std_srvs/srv/SetBool.srv"),
            interface_path(interface_roots, "nav2_msgs/srv/SetInitialPose.srv"),
            interface_path(interface_roots, "nav_msgs/srv/GetMap.srv"),
            interface_path(interface_roots, "nav2_msgs/srv/LoadMap.srv"),
            interface_path(interface_roots, "nav2_msgs/srv/IsPathValid.srv"),
            interface_path(interface_roots, "nav2_msgs/action/NavigateToPose.action"),
            interface_path(
                interface_roots,
                "nav2_msgs/action/NavigateThroughPoses.action",
            ),
            interface_path(interface_roots, "nav2_msgs/action/ComputePathToPose.action"),
            interface_path(
                interface_roots,
                "nav2_msgs/action/ComputePathThroughPoses.action",
            ),
            interface_path(interface_roots, "nav2_msgs/action/FollowPath.action"),
            interface_path(interface_roots, "nav2_msgs/action/SmoothPath.action"),
            interface_path(interface_roots, "nav2_msgs/action/Spin.action"),
            interface_path(interface_roots, "nav2_msgs/action/BackUp.action"),
            interface_path(interface_roots, "nav2_msgs/action/DriveOnHeading.action"),
            interface_path(interface_roots, "nav2_msgs/action/AssistedTeleop.action"),
            interface_path(interface_roots, "nav2_msgs/action/Wait.action"),
        ],
        interface_roots.to_vec(),
    );
    let preempt_teleop = FileExtractor::new(
        vec![interface_path(interface_roots, "std_msgs/msg/Empty.msg")],
        interface_roots.to_vec(),
    )
    .extract()
    .map_err(|e| e.to_string())?
    .into_iter()
    .next()
    .ok_or_else(|| "std_msgs/Empty interface not extracted".to_string())?;
    let mut by_name = extractor
        .extract()
        .map_err(|e| e.to_string())?
        .into_iter()
        .map(|interface| (interface.name.clone(), interface))
        .collect::<BTreeMap<_, _>>();
    let empty = take_interface(&mut by_name, "Empty")?;
    let trigger = take_interface(&mut by_name, "Trigger")?;
    let set_bool = take_interface(&mut by_name, "SetBool")?;
    let polygon = take_interface(&mut by_name, "Polygon")?;
    let polygon_stamped = take_interface(&mut by_name, "PolygonStamped")?;
    let tf_message = take_interface(&mut by_name, "TFMessage")?;
    let costmap = take_interface(&mut by_name, "Costmap")?;
    let costmap_update = take_interface(&mut by_name, "CostmapUpdate")?;
    let costmap_filter_info = take_interface(&mut by_name, "CostmapFilterInfo")?;
    let occupancy_grid = take_interface(&mut by_name, "OccupancyGrid")?;
    let selector = take_interface(&mut by_name, "String")?;
    Ok(vec![
        manifest(InterfaceBinding::new(
            take_interface(&mut by_name, "PoseWithCovarianceStamped")?,
            EndpointBinding::Topic {
                topic_name: "/initialpose".to_string(),
                message_type: "geometry_msgs/msg/PoseWithCovarianceStamped".to_string(),
                options: Ros2TopicOptions::default(),
            },
        )),
        InterfaceBinding::new(
            take_interface(&mut by_name, "Odometry")?,
            EndpointBinding::Topic {
                topic_name: "/odom".to_string(),
                message_type: "nav_msgs/msg/Odometry".to_string(),
                options: Ros2TopicOptions::default(),
            },
        ),
        manifest(InterfaceBinding::new(
            take_interface(&mut by_name, "PoseStamped")?,
            EndpointBinding::Topic {
                topic_name: "/goal_pose".to_string(),
                message_type: "geometry_msgs/msg/PoseStamped".to_string(),
                options: Ros2TopicOptions::default(),
            },
        )),
        manifest(InterfaceBinding::new(
            take_interface(&mut by_name, "SpeedLimit")?,
            EndpointBinding::Topic {
                topic_name: "/speed_limit".to_string(),
                message_type: "nav2_msgs/msg/SpeedLimit".to_string(),
                options: Ros2TopicOptions::default(),
            },
        )),
        costmap_filter_info_topic(
            "CostmapFilterInfo@keepout",
            costmap_filter_info.clone(),
            "/keepout_costmap_filter_info",
        ),
        costmap_filter_info_topic(
            "CostmapFilterInfo@speed",
            costmap_filter_info,
            "/speed_costmap_filter_info",
        ),
        occupancy_grid_topic(
            "OccupancyGrid@keepout_filter_mask",
            occupancy_grid.clone(),
            "/keepout_filter_mask",
        ),
        occupancy_grid_topic(
            "OccupancyGrid@speed_filter_mask",
            occupancy_grid,
            "/speed_filter_mask",
        ),
        selector_topic(
            "String@planner_selector",
            selector.clone(),
            "/planner_selector",
        ),
        selector_topic(
            "String@controller_selector",
            selector.clone(),
            "/controller_selector",
        ),
        selector_topic(
            "String@goal_checker_selector",
            selector.clone(),
            "/goal_checker_selector",
        ),
        selector_topic(
            "String@progress_checker_selector",
            selector.clone(),
            "/progress_checker_selector",
        ),
        selector_topic(
            "String@path_handler_selector",
            selector,
            "/path_handler_selector",
        ),
        InterfaceBinding::new(
            take_interface(&mut by_name, "TwistStamped")?,
            EndpointBinding::Topic {
                topic_name: "/cmd_vel_teleop".to_string(),
                message_type: "geometry_msgs/msg/TwistStamped".to_string(),
                options: non_blocking_topic_options(),
            },
        ),
        manifest(InterfaceBinding::alias(
            "Empty@preempt_teleop",
            preempt_teleop,
            EndpointBinding::Topic {
                topic_name: "/preempt_teleop".to_string(),
                message_type: "std_msgs/msg/Empty".to_string(),
                options: non_blocking_topic_options(),
            },
        )),
        manifest(InterfaceBinding::new(
            polygon.clone(),
            EndpointBinding::Topic {
                topic_name: "/local_costmap/footprint".to_string(),
                message_type: "geometry_msgs/msg/Polygon".to_string(),
                options: Ros2TopicOptions::default(),
            },
        )),
        manifest(InterfaceBinding::alias(
            "Polygon@global_costmap",
            polygon,
            EndpointBinding::Topic {
                topic_name: "/global_costmap/footprint".to_string(),
                message_type: "geometry_msgs/msg/Polygon".to_string(),
                options: Ros2TopicOptions::default(),
            },
        )),
        costmap_topic(
            "Costmap@local_costmap_raw",
            costmap.clone(),
            "/local_costmap/costmap_raw",
            "nav2_msgs/msg/Costmap",
        ),
        costmap_update_topic(
            "CostmapUpdate@local_costmap_raw_updates",
            costmap_update.clone(),
            "/local_costmap/costmap_raw_updates",
        ),
        costmap_topic(
            "Costmap@global_costmap_raw",
            costmap,
            "/global_costmap/costmap_raw",
            "nav2_msgs/msg/Costmap",
        ),
        costmap_update_topic(
            "CostmapUpdate@global_costmap_raw_updates",
            costmap_update,
            "/global_costmap/costmap_raw_updates",
        ),
        published_footprint_topic(
            "PolygonStamped@local_published_footprint",
            polygon_stamped.clone(),
            "/local_costmap/published_footprint",
        ),
        published_footprint_topic(
            "PolygonStamped@global_published_footprint",
            polygon_stamped,
            "/global_costmap/published_footprint",
        ),
        tf_topic("TFMessage", tf_message.clone(), "/tf", tf_topic_options()),
        tf_topic(
            "TFMessage@tf_static",
            tf_message,
            "/tf_static",
            tf_static_topic_options(),
        ),
        manifest(service(
            empty.clone(),
            "/reinitialize_global_localization",
            "std_srvs/srv/Empty",
        )),
        manifest(service_alias(
            "Empty@request_nomotion_update",
            empty,
            "/request_nomotion_update",
            "std_srvs/srv/Empty",
        )),
        manifest(service(
            take_interface(&mut by_name, "SetInitialPose")?,
            "/set_initial_pose",
            "nav2_msgs/srv/SetInitialPose",
        )),
        service(
            take_interface(&mut by_name, "GetMap")?,
            "/map_server/map",
            "nav_msgs/srv/GetMap",
        ),
        manifest(service(
            take_interface(&mut by_name, "LoadMap")?,
            "/map_server/load_map",
            "nav2_msgs/srv/LoadMap",
        )),
        manifest(service_with_timeout(
            take_interface(&mut by_name, "IsPathValid")?,
            "/is_path_valid",
            "nav2_msgs/srv/IsPathValid",
            Some(3),
        )),
        service(
            trigger.clone(),
            "/lifecycle_manager_localization/is_active",
            "std_srvs/srv/Trigger",
        ),
        service_alias(
            "Trigger@lifecycle_manager_navigation",
            trigger,
            "/lifecycle_manager_navigation/is_active",
            "std_srvs/srv/Trigger",
        ),
        service_alias_with_timeout(
            "SetBool@local_keepout_filter",
            set_bool.clone(),
            "/local_costmap/keepout_filter/toggle_filter",
            "std_srvs/srv/SetBool",
            Some(3),
        ),
        service_alias_with_timeout(
            "SetBool@global_keepout_filter",
            set_bool.clone(),
            "/global_costmap/keepout_filter/toggle_filter",
            "std_srvs/srv/SetBool",
            Some(3),
        ),
        service_alias_with_timeout(
            "SetBool@global_speed_filter",
            set_bool,
            "/global_costmap/speed_filter/toggle_filter",
            "std_srvs/srv/SetBool",
            Some(3),
        ),
        manifest(action(&mut by_name, "NavigateToPose", "/navigate_to_pose")?),
        manifest(action(
            &mut by_name,
            "NavigateThroughPoses",
            "/navigate_through_poses",
        )?),
        manifest(action(
            &mut by_name,
            "ComputePathToPose",
            "/compute_path_to_pose",
        )?),
        manifest(action(
            &mut by_name,
            "ComputePathThroughPoses",
            "/compute_path_through_poses",
        )?),
        manifest(action(&mut by_name, "FollowPath", "/follow_path")?),
        manifest(action(&mut by_name, "SmoothPath", "/smooth_path")?),
        manifest(action(&mut by_name, "Spin", "/spin")?),
        manifest(action(&mut by_name, "BackUp", "/backup")?),
        manifest(action(&mut by_name, "DriveOnHeading", "/drive_on_heading")?),
        manifest(action(&mut by_name, "AssistedTeleop", "/assisted_teleop")?),
        manifest(action(&mut by_name, "Wait", "/wait")?),
    ])
}

fn safe_parameter_bindings() -> Vec<InterfaceBinding> {
    vec![
        parameter_binding(
            "Parameter@amcl.save_pose_rate",
            "/amcl",
            "save_pose_rate",
            SafeParameterProfile::F64 {
                restore: 0.5,
                min: 0.05,
                max: 5.0,
            },
        ),
        parameter_binding(
            "Parameter@planner_server.expected_planner_frequency",
            "/planner_server",
            "expected_planner_frequency",
            SafeParameterProfile::F64 {
                restore: 20.0,
                min: 0.1,
                max: 50.0,
            },
        ),
        parameter_binding(
            "Parameter@controller_server.controller_frequency",
            "/controller_server",
            "controller_frequency",
            SafeParameterProfile::F64 {
                restore: 20.0,
                min: 1.0,
                max: 50.0,
            },
        ),
        parameter_binding(
            "Parameter@controller_server.min_x_velocity_threshold",
            "/controller_server",
            "min_x_velocity_threshold",
            SafeParameterProfile::F64 {
                restore: 0.001,
                min: 0.0,
                max: 0.25,
            },
        ),
        parameter_binding(
            "Parameter@controller_server.failure_tolerance",
            "/controller_server",
            "failure_tolerance",
            SafeParameterProfile::F64 {
                restore: 0.3,
                min: -1.0,
                max: 5.0,
            },
        ),
    ]
}

fn parameter_binding(
    interface_id: &str,
    node_name: &str,
    parameter_name: &str,
    profile: SafeParameterProfile,
) -> InterfaceBinding {
    InterfaceBinding::alias(
        interface_id,
        Interface::new(
            interface_id,
            Kind::Parameter,
            vec![Field::new("value", profile.primitive())],
        ),
        EndpointBinding::Parameter {
            endpoint_name: format!("{node_name}:{parameter_name}"),
            node_name: node_name.to_string(),
            parameter_name: parameter_name.to_string(),
            profile,
        },
    )
    .with_source(InputSource::SafeParameterProfile)
}

fn action(
    by_name: &mut BTreeMap<String, Interface>,
    name: &str,
    action_name: &str,
) -> Result<InterfaceBinding, String> {
    Ok(InterfaceBinding::new(
        take_interface(by_name, name)?,
        EndpointBinding::Action {
            action_name: action_name.to_string(),
            action_type: format!("nav2_msgs/action/{name}"),
        },
    ))
}

fn service(interface: Interface, service_name: &str, service_type: &str) -> InterfaceBinding {
    service_with_timeout(interface, service_name, service_type, None)
}

fn service_with_timeout(
    interface: Interface,
    service_name: &str,
    service_type: &str,
    timeout_sec: Option<u64>,
) -> InterfaceBinding {
    InterfaceBinding::new(
        interface,
        EndpointBinding::Service {
            service_name: service_name.to_string(),
            service_type: service_type.to_string(),
            timeout_sec,
        },
    )
}

fn non_blocking_topic_options() -> Ros2TopicOptions {
    Ros2TopicOptions {
        wait_matching_subscriptions: Some(0),
        keep_alive_sec: 0.1,
        ..Ros2TopicOptions::default()
    }
}

fn selector_topic_options() -> Ros2TopicOptions {
    Ros2TopicOptions {
        wait_matching_subscriptions: Some(0),
        keep_alive_sec: 0.5,
        qos_depth: Some(1),
        qos_reliability: Some("reliable".to_string()),
        qos_durability: Some("transient_local".to_string()),
        ..Ros2TopicOptions::default()
    }
}

fn selector_topic(interface_id: &str, interface: Interface, topic_name: &str) -> InterfaceBinding {
    InterfaceBinding::alias(
        interface_id,
        interface,
        EndpointBinding::Topic {
            topic_name: topic_name.to_string(),
            message_type: "std_msgs/msg/String".to_string(),
            options: selector_topic_options(),
        },
    )
}

fn costmap_stream_topic_options(depth: u32) -> Ros2TopicOptions {
    Ros2TopicOptions {
        wait_matching_subscriptions: Some(0),
        keep_alive_sec: 0.5,
        qos_depth: Some(depth),
        qos_reliability: Some("reliable".to_string()),
        qos_durability: Some("transient_local".to_string()),
        ..Ros2TopicOptions::default()
    }
}

fn costmap_filter_topic_options() -> Ros2TopicOptions {
    Ros2TopicOptions {
        wait_matching_subscriptions: Some(0),
        keep_alive_sec: 1.0,
        qos_depth: Some(1),
        qos_reliability: Some("reliable".to_string()),
        qos_durability: Some("transient_local".to_string()),
        ..Ros2TopicOptions::default()
    }
}

fn costmap_filter_info_topic(
    interface_id: &str,
    interface: Interface,
    topic_name: &str,
) -> InterfaceBinding {
    InterfaceBinding::alias(
        interface_id,
        interface,
        EndpointBinding::Topic {
            topic_name: topic_name.to_string(),
            message_type: "nav2_msgs/msg/CostmapFilterInfo".to_string(),
            options: costmap_filter_topic_options(),
        },
    )
}

fn occupancy_grid_topic(
    interface_id: &str,
    interface: Interface,
    topic_name: &str,
) -> InterfaceBinding {
    InterfaceBinding::alias(
        interface_id,
        interface,
        EndpointBinding::Topic {
            topic_name: topic_name.to_string(),
            message_type: "nav_msgs/msg/OccupancyGrid".to_string(),
            options: costmap_filter_topic_options(),
        },
    )
}

fn costmap_topic(
    interface_id: &str,
    interface: Interface,
    topic_name: &str,
    message_type: &str,
) -> InterfaceBinding {
    InterfaceBinding::alias(
        interface_id,
        interface,
        EndpointBinding::Topic {
            topic_name: topic_name.to_string(),
            message_type: message_type.to_string(),
            options: costmap_stream_topic_options(1),
        },
    )
}

fn costmap_update_topic(
    interface_id: &str,
    interface: Interface,
    topic_name: &str,
) -> InterfaceBinding {
    InterfaceBinding::alias(
        interface_id,
        interface,
        EndpointBinding::Topic {
            topic_name: topic_name.to_string(),
            message_type: "nav2_msgs/msg/CostmapUpdate".to_string(),
            options: costmap_stream_topic_options(10),
        },
    )
}

fn published_footprint_topic(
    interface_id: &str,
    interface: Interface,
    topic_name: &str,
) -> InterfaceBinding {
    InterfaceBinding::alias(
        interface_id,
        interface,
        EndpointBinding::Topic {
            topic_name: topic_name.to_string(),
            message_type: "geometry_msgs/msg/PolygonStamped".to_string(),
            options: Ros2TopicOptions {
                wait_matching_subscriptions: Some(0),
                keep_alive_sec: 0.5,
                qos_depth: Some(10),
                qos_reliability: Some("reliable".to_string()),
                qos_durability: Some("volatile".to_string()),
                ..Ros2TopicOptions::default()
            },
        },
    )
}

fn tf_topic_options() -> Ros2TopicOptions {
    Ros2TopicOptions {
        wait_matching_subscriptions: Some(0),
        keep_alive_sec: 0.5,
        qos_depth: Some(100),
        qos_reliability: Some("reliable".to_string()),
        qos_durability: Some("volatile".to_string()),
        ..Ros2TopicOptions::default()
    }
}

fn tf_static_topic_options() -> Ros2TopicOptions {
    Ros2TopicOptions {
        wait_matching_subscriptions: Some(0),
        keep_alive_sec: 1.0,
        qos_depth: Some(1),
        qos_reliability: Some("reliable".to_string()),
        qos_durability: Some("transient_local".to_string()),
        ..Ros2TopicOptions::default()
    }
}

fn tf_topic(
    interface_id: &str,
    interface: Interface,
    topic_name: &str,
    options: Ros2TopicOptions,
) -> InterfaceBinding {
    InterfaceBinding::alias(
        interface_id,
        interface,
        EndpointBinding::Topic {
            topic_name: topic_name.to_string(),
            message_type: "tf2_msgs/msg/TFMessage".to_string(),
            options,
        },
    )
}

fn service_alias(
    interface_id: &str,
    interface: Interface,
    service_name: &str,
    service_type: &str,
) -> InterfaceBinding {
    service_alias_with_timeout(interface_id, interface, service_name, service_type, None)
}

fn service_alias_with_timeout(
    interface_id: &str,
    interface: Interface,
    service_name: &str,
    service_type: &str,
    timeout_sec: Option<u64>,
) -> InterfaceBinding {
    InterfaceBinding::alias(
        interface_id,
        interface,
        EndpointBinding::Service {
            service_name: service_name.to_string(),
            service_type: service_type.to_string(),
            timeout_sec,
        },
    )
}

pub(super) fn find_binding<'a>(
    bindings: &'a [InterfaceBinding],
    interface_id: &str,
) -> Option<&'a InterfaceBinding> {
    bindings
        .iter()
        .find(|binding| binding.interface_id == interface_id)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashSet;

    fn configured_interface_roots() -> Vec<PathBuf> {
        let repo_root = Path::new(env!("CARGO_MANIFEST_DIR"));
        let env = YamlEnv::load(repo_root).unwrap();
        let ros_setup = env.require_path("R2D2_ROS_SETUP").unwrap();
        vec![
            ros_setup.parent().unwrap().join("share"),
            env.require_path("R2D2_NAV2_SOURCE_ROOT").unwrap(),
        ]
    }

    fn endpoint_names(bindings: &[InterfaceBinding]) -> HashSet<&str> {
        bindings
            .iter()
            .map(InterfaceBinding::endpoint_name)
            .collect()
    }

    #[test]
    fn full_stack_route_covers_discovered_business_inputs() {
        let bindings = extract_stack_bindings(&configured_interface_roots()).unwrap();
        let names = endpoint_names(&bindings);
        let service_names = bindings
            .iter()
            .filter_map(|binding| match &binding.endpoint {
                EndpointBinding::Service { service_name, .. } => Some(service_name.as_str()),
                _ => None,
            })
            .collect::<HashSet<_>>();
        let interface_ids = bindings
            .iter()
            .map(|binding| binding.interface_id.as_str())
            .collect::<HashSet<_>>();

        assert_eq!(bindings.len(), 69);
        assert_eq!(interface_ids.len(), bindings.len());
        assert_eq!(
            service_names,
            crate::stack::FULL_STACK_SERVICES
                .iter()
                .copied()
                .collect::<HashSet<_>>()
        );
        for endpoint in [
            "/goal_pose",
            "/speed_limit",
            "/keepout_costmap_filter_info",
            "/speed_costmap_filter_info",
            "/keepout_filter_mask",
            "/speed_filter_mask",
            "/planner_selector",
            "/controller_selector",
            "/goal_checker_selector",
            "/progress_checker_selector",
            "/path_handler_selector",
            "/cmd_vel_teleop",
            "/preempt_teleop",
            "/local_costmap/footprint",
            "/global_costmap/footprint",
            "/local_costmap/costmap_raw",
            "/local_costmap/costmap_raw_updates",
            "/global_costmap/costmap_raw",
            "/global_costmap/costmap_raw_updates",
            "/local_costmap/published_footprint",
            "/global_costmap/published_footprint",
            "/tf",
            "/tf_static",
            "/local_costmap/get_voxel_layer",
            "/global_costmap/get_obstacle_layer",
            "/global_costmap/get_static_layer",
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
            "/assisted_teleop",
            "/amcl:save_pose_rate",
            "/planner_server:expected_planner_frequency",
            "/controller_server:controller_frequency",
            "/controller_server:min_x_velocity_threshold",
            "/controller_server:failure_tolerance",
        ] {
            assert!(names.contains(endpoint), "missing binding for {endpoint}");
        }

        let planner_selector = find_binding(&bindings, "String@planner_selector").unwrap();
        let EndpointBinding::Topic { options, .. } = &planner_selector.endpoint else {
            panic!("planner selector must stay a topic binding");
        };
        assert_eq!(options.wait_matching_subscriptions, Some(0));
        assert_eq!(options.qos_depth, Some(1));
        assert_eq!(options.qos_reliability.as_deref(), Some("reliable"));
        assert_eq!(options.qos_durability.as_deref(), Some("transient_local"));

        let local_costmap = find_binding(&bindings, "Costmap@local_costmap_raw").unwrap();
        let EndpointBinding::Topic { options, .. } = &local_costmap.endpoint else {
            panic!("local costmap raw must stay a topic binding");
        };
        assert_eq!(options.wait_matching_subscriptions, Some(0));
        assert_eq!(options.qos_depth, Some(1));
        assert_eq!(options.qos_reliability.as_deref(), Some("reliable"));
        assert_eq!(options.qos_durability.as_deref(), Some("transient_local"));
    }

    #[test]
    fn input_attribution_is_report_only_metadata() {
        let bindings = extract_stack_bindings(&configured_interface_roots()).unwrap();
        let navigate = find_binding(&bindings, "NavigateToPose").unwrap();
        let scan = find_binding(&bindings, "LaserScan").unwrap();
        let costmap = find_binding(&bindings, "GetCostmap").unwrap();
        let selector = find_binding(&bindings, "String@planner_selector").unwrap();
        let parameter = find_binding(
            &bindings,
            "Parameter@controller_server.controller_frequency",
        )
        .unwrap();

        assert_eq!(navigate.input_kind(), Kind::Action);
        assert_eq!(scan.input_kind(), Kind::Topic);
        assert_eq!(costmap.input_kind(), Kind::Service);
        assert_eq!(selector.input_kind(), Kind::Topic);
        assert_eq!(scan.source, InputSource::Manifest);
        assert_eq!(navigate.source, InputSource::Manifest);
        assert_eq!(selector.source, InputSource::RuntimeExtension);
        assert_eq!(parameter.input_kind(), Kind::Parameter);
        assert_eq!(parameter.source, InputSource::SafeParameterProfile);
    }

    #[test]
    fn aliased_endpoints_keep_the_original_ros_schema_name() {
        let bindings = extract_stack_bindings(&configured_interface_roots()).unwrap();
        let binding = find_binding(&bindings, "GetCostmap@global_static_layer").unwrap();

        assert_eq!(binding.interface.name, "GetCostmap");
        assert_eq!(binding.generator_interface().name, binding.interface_id);
        assert_eq!(binding.endpoint_name(), "/global_costmap/get_static_layer");
    }
}
