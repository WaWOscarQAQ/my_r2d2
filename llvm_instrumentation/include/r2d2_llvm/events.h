#ifndef R2D2_LLVM_EVENTS_H
#define R2D2_LLVM_EVENTS_H

#include <cstdint>

namespace r2d2_llvm {

// Keep these values synchronized with the LLVM pass. The generic five-field
// hook keeps the instrumented IR independent from ROS headers and ABI details.
enum class Event : std::uint32_t {
  RclNodeInit = 0,
  RclSubscriptionInit = 1,
  RclServiceInit = 2,
  RclcppSubscriptionInit = 3,
  RclcppSubscriptionCallbackAdded = 4,
  RclcppServiceCallbackAdded = 5,
  RclcppTimerCallbackAdded = 6,
  RclcppTimerLinkNode = 7,
  ExecutorExecute = 8,
  CallbackStart = 9,
  CallbackEnd = 10,
  RclTake = 11,
};

}  // namespace r2d2_llvm

extern "C" void __r2d2_llvm_trace(
    std::uint32_t event, const void* arg0, const void* arg1,
    const char* text0, const char* text1) noexcept;

extern "C" void __r2d2_llvm_rcl_take(
    const void* rcl_handler, const void* message, std::uint64_t buffer_size,
    std::uint64_t source_timestamp) noexcept;

#endif
