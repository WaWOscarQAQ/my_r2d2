#ifndef TRACER_TRACERS_H
#define TRACER_TRACERS_H

#include "tracer/trace_records.h"

#include <cstddef>
#include <cstdint>

namespace tracer {

// Current time in nanoseconds on the monotonic clock. Real instrumented
// call sites stamp events with this; the mock passes fixed values.
std::uint64_t now_ns() noexcept;

// Allocates the shared memory object with the two ring buffers. Real
// instrumentation calls this once from the RCL layer initialization; the
// paper puts the shared memory initialization tracer at that layer.
void init(const char* shm_name);
void init(const char* shm_name, std::uint64_t registration_capacity,
          std::uint64_t runtime_capacity);

// Registration tracers, writing to the callback registration buffer.
void rclcpp_callback_init(const void* rclcpp_handler, const void* rcl_handler,
                          CallbackType callback_type) noexcept;
void rcl_callback_init(const char* callback_name, const void* rcl_handler) noexcept;

// Runtime tracers, writing to the runtime execution buffer.
void executor_execute(const void* rclcpp_handler, std::uint64_t invoke_timestamp) noexcept;
void callback_start(const void* rclcpp_handler, std::uint64_t start_timestamp) noexcept;
void callback_end(const void* rclcpp_handler, std::uint64_t end_timestamp) noexcept;
void rcl_take(const void* rcl_handler, std::uint64_t buffer_size,
              std::uint64_t pub_timestamp, std::uint64_t sub_timestamp) noexcept;

// Test accessors over the mapped image; null before init().
const void* image();
std::size_t image_size();

}  // namespace tracer

#endif
