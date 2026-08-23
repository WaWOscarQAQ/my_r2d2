#ifndef TRACER_TRACE_RECORDS_H
#define TRACER_TRACE_RECORDS_H

#include <cstddef>
#include <cstdint>

namespace tracer {

// Shared memory identity. The Rust reader rejects anything else.
inline constexpr std::uint32_t kTraceMagic = 0x52523244;  // "R2D"
// Version 2: registration records carry the callback namespace (paper
// §4.1.1 lists it as a registration attribute) and the runtime ring accepts
// RoundBoundary marker events.
inline constexpr std::uint32_t kTraceVersion = 2;

// Which tracer produced a registration record.
enum class RegistrationSource : std::uint32_t {
    Rclcpp = 0,
    Rcl = 1,
};

// The callback types recorded by rclcpp_callback_init().
enum class CallbackType : std::uint32_t {
    Subscription = 0,
    Timer = 1,
    Service = 2,
};

// The event kinds written by the runtime tracers.
enum class RuntimeEventType : std::uint32_t {
    ExecutorExecute = 0,
    CallbackStart = 1,
    CallbackEnd = 2,
    RclTake = 3,
    // Harness-written marker delimiting one payload execution round. The
    // paper analyzes "the current callback trace" after each payload but
    // does not disclose a boundary event format; this marker is a
    // reproduction choice.
    RoundBoundary = 4,
};

// Fixed capacity of the callback name field in a registration record.
inline constexpr std::size_t kCallbackNameCapacity = 128;
// Fixed capacity of the callback namespace field. The paper (§4.1.1) lists
// the namespace as a registration attribute; its capacity is a reproduction
// choice.
inline constexpr std::size_t kCallbackNamespaceCapacity = 64;
inline constexpr std::uint32_t kRegistrationFlagNameTruncated = 1U;
inline constexpr std::uint32_t kRegistrationFlagNamespaceTruncated = 2U;

// One entry in the callback registration buffer. The two registration
// tracers write the fields they know: rclcpp_callback_init() fills the
// handlers and the type, rcl_callback_init() fills the name and namespace.
// Records from the two tracers are associated through rcl_handler.
struct RegistrationRecord {
    RegistrationSource source;
    CallbackType callback_type;
    std::uint64_t rclcpp_handler;
    std::uint64_t rcl_handler;
    std::uint32_t callback_name_len;
    std::uint32_t flags;
    char callback_name[kCallbackNameCapacity];
    std::uint32_t callback_namespace_len;
    char callback_namespace[kCallbackNamespaceCapacity];
};

// 228 bytes of fields plus 4 bytes of tail padding (alignof == 8).
static_assert(sizeof(RegistrationRecord) == 232, "layout must match the Rust reader");

// One entry in the runtime execution buffer. Only the fields named by the
// paper are meaningful per event kind; rcl_take() fills the message
// passing fields and leaves rclcpp_handler zero. `aux` carries the round
// id for RoundBoundary events and stays zero for the paper-defined kinds.
struct RuntimeRecord {
    RuntimeEventType event_type;
    std::uint32_t aux;
    std::uint64_t rclcpp_handler;
    std::uint64_t timestamp;
    std::uint64_t rcl_handler;
    std::uint64_t buffer_size;
    std::uint64_t pub_timestamp;
    std::uint64_t sub_timestamp;
};

static_assert(sizeof(RuntimeRecord) == 56, "layout must match the Rust reader");

}  // namespace tracer

#endif
