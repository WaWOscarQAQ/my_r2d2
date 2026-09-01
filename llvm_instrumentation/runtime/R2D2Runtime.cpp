#include "r2d2_llvm/events.h"

#include "tracer/tracers.h"

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace {

using r2d2_llvm::Event;

std::atomic<bool> ready{false};
std::mutex init_mutex;
std::mutex registry_mutex;
std::unordered_map<const void*, const void*> subscription_handles;
std::unordered_map<const void*, const void*> pending_subscription_callbacks;
std::unordered_map<const void*, const void*> callbacks_by_rcl;
std::unordered_set<const void*> named_rcl_handles;
std::unordered_set<const void*> complete_callbacks;
std::unordered_map<const void*, const void*> timer_nodes;
std::unordered_map<const void*, const void*> timer_callbacks;
std::unordered_map<const void*, std::pair<std::string, std::string>> nodes;

const void* canonical_handle(const void* handle) noexcept {
  // ROS tracepoints consistently publish the address of the rcl handle itself
  // (rcl_subscription_t, rcl_service_t, rcl_timer_t, ...).  Dereferencing it
  // depended on an old private struct layout and turns the first data field
  // into a bogus identity on newer ROS releases such as Lyrical.
  return handle;
}

std::string shared_memory_name() {
  const char* path = std::getenv("R2D2_SHM_PATH");
  if (path == nullptr || *path == '\0') {
    return {};
  }
  std::string value(path);
  const std::size_t slash = value.find_last_of('/');
  return slash == std::string::npos ? value : value.substr(slash + 1);
}

bool runtime_enabled() noexcept {
  const char* mode = std::getenv("R2D2_TRACER_MODE");
  return mode != nullptr && std::strcmp(mode, "runtime") == 0;
}

bool ensure_ready() noexcept {
  if (!runtime_enabled()) {
    return false;
  }
  if (ready.load(std::memory_order_acquire)) {
    return true;
  }
  std::lock_guard<std::mutex> lock(init_mutex);
  if (ready.load(std::memory_order_acquire)) {
    return true;
  }
  const std::string name = shared_memory_name();
  if (name.empty()) {
    return false;
  }
  try {
    tracer::init(name.c_str());
    ready.store(true, std::memory_order_release);
  } catch (...) {
    try {
      tracer::attach(name.c_str());
      ready.store(true, std::memory_order_release);
    } catch (...) {
      ready.store(false, std::memory_order_release);
    }
  }
  return ready.load(std::memory_order_acquire);
}

std::string entity_namespace(const char* full_name) {
  if (full_name == nullptr || *full_name == '\0') {
    return "/";
  }
  const std::string name(full_name);
  const std::size_t slash = name.find_last_of('/');
  return slash == std::string::npos || slash == 0 ? "/" : name.substr(0, slash);
}

std::uint64_t wall_now_ns() noexcept {
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
}

void register_rclcpp(const void* rcl_handle, const void* callback,
                     tracer::CallbackType type) {
  const void* canonical = canonical_handle(rcl_handle);
  if (canonical == nullptr || callback == nullptr) {
    return;
  }
  callbacks_by_rcl[canonical] = callback;
  tracer::rclcpp_callback_init(callback, canonical, type);
  if (named_rcl_handles.count(canonical) != 0) {
    complete_callbacks.insert(callback);
  }
}

void register_rcl_name(const char* name, const char* callback_namespace,
                       const void* rcl_handle) {
  const void* canonical = canonical_handle(rcl_handle);
  if (canonical == nullptr) {
    return;
  }
  tracer::rcl_callback_init(name != nullptr ? name : "",
                            callback_namespace != nullptr ? callback_namespace : "/",
                            canonical);
  named_rcl_handles.insert(canonical);
  const auto callback = callbacks_by_rcl.find(canonical);
  if (callback != callbacks_by_rcl.end()) {
    complete_callbacks.insert(callback->second);
  }
}

void register_subscription_callback(const void* object, const void* callback) {
  const auto handle = subscription_handles.find(object);
  if (handle == subscription_handles.end()) {
    pending_subscription_callbacks[object] = callback;
    return;
  }
  register_rclcpp(handle->second, callback, tracer::CallbackType::Subscription);
}

void link_subscription(const void* rcl_handle, const void* object) {
  subscription_handles[object] = rcl_handle;
  const auto pending = pending_subscription_callbacks.find(object);
  if (pending != pending_subscription_callbacks.end()) {
    register_rclcpp(rcl_handle, pending->second, tracer::CallbackType::Subscription);
    pending_subscription_callbacks.erase(pending);
  }
}

void register_timer_name(const void* timer_handle) {
  const void* canonical_timer = timer_handle;
  const auto node = timer_nodes.find(canonical_timer);
  if (node == timer_nodes.end()) {
    return;
  }
  const auto info = nodes.find(node->second);
  if (info == nodes.end()) {
    return;
  }
  const std::string& node_name = info->second.first;
  const std::string& node_namespace = info->second.second;
  std::string full_name;
  if (node_namespace.empty() || node_namespace == "/") {
    full_name = "/" + node_name;
  } else {
    full_name = node_namespace + "/" + node_name;
  }
  full_name += "::timer";
  register_rcl_name(full_name.c_str(), node_namespace.c_str(), canonical_timer);
}

bool has_complete_rcl_callback(const void* canonical) {
  const auto callback = callbacks_by_rcl.find(canonical);
  return callback != callbacks_by_rcl.end() &&
         complete_callbacks.count(callback->second) != 0;
}

void record_rcl_take_unlocked(const void* canonical, std::uint64_t buffer_size,
                              std::uint64_t source_timestamp) {
  const std::uint64_t sub_timestamp = wall_now_ns();
  std::uint64_t pub_timestamp = source_timestamp;
  if (pub_timestamp == 0 || pub_timestamp >= sub_timestamp) {
    pub_timestamp = sub_timestamp > 0 ? sub_timestamp - 1 : 0;
  }
  const std::uint64_t observed_size = buffer_size == 0 ? 1 : buffer_size;
  tracer::rcl_take(canonical, observed_size, pub_timestamp, sub_timestamp);
}

}  // namespace

extern "C" void __r2d2_llvm_trace(
    std::uint32_t raw_event, const void* arg0, const void* arg1,
    const char* text0, const char* text1) noexcept {
  if (!ensure_ready()) {
    return;
  }

  try {
    const Event event = static_cast<Event>(raw_event);
    std::lock_guard<std::mutex> lock(registry_mutex);
    switch (event) {
      case Event::RclNodeInit: {
        const void* node = canonical_handle(arg0);
        nodes[node] = {text0 != nullptr ? text0 : "", text1 != nullptr ? text1 : "/"};
        for (const auto& timer : timer_nodes) {
          if (timer.second == node) {
            register_timer_name(timer.first);
          }
        }
        break;
      }
      case Event::RclSubscriptionInit:
        register_rcl_name(text0, entity_namespace(text0).c_str(), arg0);
        break;
      case Event::RclServiceInit:
        register_rcl_name(text0, entity_namespace(text0).c_str(), arg0);
        break;
      case Event::RclcppSubscriptionInit:
        link_subscription(arg0, arg1);
        break;
      case Event::RclcppSubscriptionCallbackAdded:
        register_subscription_callback(arg0, arg1);
        break;
      case Event::RclcppServiceCallbackAdded:
        register_rclcpp(arg0, arg1, tracer::CallbackType::Service);
        break;
      case Event::RclcppTimerCallbackAdded: {
        const void* timer = canonical_handle(arg0);
        timer_callbacks[timer] = arg1;
        register_rclcpp(arg0, arg1, tracer::CallbackType::Timer);
        register_timer_name(timer);
        break;
      }
      case Event::RclcppTimerLinkNode: {
        const void* timer = canonical_handle(arg0);
        timer_nodes[timer] = canonical_handle(arg1);
        register_timer_name(timer);
        break;
      }
      case Event::ExecutorExecute: {
        const auto callback = callbacks_by_rcl.find(canonical_handle(arg0));
        if (callback != callbacks_by_rcl.end() &&
            complete_callbacks.count(callback->second) != 0) {
          tracer::executor_execute(callback->second, tracer::now_ns());
        }
        break;
      }
      case Event::CallbackStart:
        if (complete_callbacks.count(arg0) != 0) {
          tracer::callback_start(arg0, tracer::now_ns());
        }
        break;
      case Event::CallbackEnd:
        if (complete_callbacks.count(arg0) != 0) {
          tracer::callback_end(arg0, tracer::now_ns());
        }
        break;
      case Event::RclTake: {
        const void* canonical = canonical_handle(arg0);
        if (canonical != nullptr && has_complete_rcl_callback(canonical)) {
          record_rcl_take_unlocked(canonical, 1, 0);
        }
        break;
      }
    }
  } catch (...) {
    // Instrumentation must never alter ROS control flow.
  }
}

extern "C" void __r2d2_llvm_rcl_take(
    const void* rcl_handler, const void* /*message*/, std::uint64_t buffer_size,
    std::uint64_t source_timestamp) noexcept {
  if (!ensure_ready()) {
    return;
  }

  try {
    const void* canonical = canonical_handle(rcl_handler);
    if (canonical == nullptr) {
      return;
    }
    {
      std::lock_guard<std::mutex> lock(registry_mutex);
      if (!has_complete_rcl_callback(canonical)) {
        return;
      }
      record_rcl_take_unlocked(canonical, buffer_size, source_timestamp);
    }
  } catch (...) {
    // Instrumentation must never alter ROS control flow.
  }
}
