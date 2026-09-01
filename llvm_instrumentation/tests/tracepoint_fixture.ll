declare void @ros_trace_rcl_subscription_init(ptr, ptr, ptr, ptr, i64)
declare void @ros_trace_rclcpp_subscription_init(ptr, ptr)
declare void @ros_trace_rclcpp_subscription_callback_added(ptr, ptr)
declare void @ros_trace_callback_start(ptr, i1)
declare void @ros_trace_callback_end(ptr)
declare void @ros_trace_rcl_take(ptr)
declare void @ros_trace_rclcpp_service_callback_added(ptr, ptr)
declare i32 @__gxx_personality_v0(...)

define void @fixture(ptr %rcl, ptr %node, ptr %object, ptr %callback, ptr %name)
    personality ptr @__gxx_personality_v0 {
entry:
  call void @ros_trace_rcl_subscription_init(ptr %rcl, ptr %node, ptr null, ptr %name, i64 10)
  call void @ros_trace_rclcpp_subscription_init(ptr %rcl, ptr %object)
  call void @ros_trace_rclcpp_subscription_callback_added(ptr %object, ptr %callback)
  call void @ros_trace_callback_start(ptr %callback, i1 false)
  call void @ros_trace_rcl_take(ptr %object)
  call void @ros_trace_callback_end(ptr %callback)
  invoke void @ros_trace_rclcpp_service_callback_added(ptr %rcl, ptr %callback)
      to label %done unwind label %failed

done:
  ret void

failed:
  %landing = landingpad { ptr, i32 } cleanup
  ret void
}
