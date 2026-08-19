#include "tracer/trace_records.h"
#include "tracer/shared_memory.h"
#include "tracer/tracers.h"

#include <cassert>
#include <cstdint>
#include <string>
#include <sys/mman.h>
#include <unistd.h>

int main() {
  const auto* handler = reinterpret_cast<const void*>(0x1000);
  const auto* rcl_handler = reinterpret_cast<const void*>(0x2000);

  // Instrumentation may be reached before RCL shared-memory setup. Every
  // hot-path tracer must be a harmless no-op in that state.
  tracer::rclcpp_callback_init(handler, rcl_handler,
                               tracer::CallbackType::Subscription);
  tracer::rcl_callback_init("before_init", rcl_handler);
  tracer::executor_execute(handler, 10);
  tracer::callback_start(handler, 20);
  tracer::callback_end(handler, 30);
  tracer::rcl_take(rcl_handler, 64, 1, 2);
  assert(tracer::image() == nullptr);

  const std::string shm_name = "my_r2d2_tracer_safety_" + std::to_string(getpid());
  tracer::init(shm_name.c_str(), 4, 4);
  tracer::rclcpp_callback_init(handler, rcl_handler,
                               tracer::CallbackType::Subscription);
  const std::string long_name(tracer::kCallbackNameCapacity + 72, 'x');
  tracer::rcl_callback_init(long_name.c_str(), rcl_handler);

  const auto* image = static_cast<const std::uint8_t*>(tracer::image());
  const auto* header = reinterpret_cast<const tracer::SharedHeader*>(image);
  const auto* records = reinterpret_cast<const tracer::RegistrationRecord*>(
      image + header->registration_records_offset);
  assert(records[1].source == tracer::RegistrationSource::Rcl);
  assert(records[1].callback_name_len == tracer::kCallbackNameCapacity);
  assert((records[1].flags & tracer::kRegistrationFlagNameTruncated) != 0);

  shm_unlink(shm_name.c_str());
  return 0;
}
