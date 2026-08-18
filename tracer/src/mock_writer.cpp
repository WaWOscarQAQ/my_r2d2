// Deterministic mock event source for the tracer module.
//
// Writes a fixed sequence of registration and runtime events into the
// tracer shared memory so the Rust reader can be validated end to end
// without a real ROS runtime. With --fixture, dumps the resulting shared
// memory image (mutex slots zeroed) to a file for the pure-Rust tests.

#include "tracer/circular_buffer.h"
#include "tracer/shared_memory.h"
#include "tracer/trace_records.h"
#include "tracer/tracers.h"

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <sys/mman.h>
#include <vector>

namespace {

constexpr std::uint64_t kSubscriptionRclcppHandler = 0x1000;
constexpr std::uint64_t kSubscriptionRclHandler = 0x2000;
constexpr std::uint64_t kTimerRclcppHandler = 0x3000;
constexpr std::uint64_t kTimerRclHandler = 0x4000;

std::uint64_t align_up(std::uint64_t value, std::uint64_t alignment) {
  return (value + alignment - 1) / alignment * alignment;
}

// The canonical deterministic sequence; the Rust tests assert the same
// content for both the golden fixture and the live round trip.
void write_sequence() {
  tracer::rclcpp_callback_init(reinterpret_cast<const void*>(kSubscriptionRclcppHandler),
                               reinterpret_cast<const void*>(kSubscriptionRclHandler),
                               tracer::CallbackType::Subscription);
  tracer::rcl_callback_init("/cmd_vel_callback",
                            reinterpret_cast<const void*>(kSubscriptionRclHandler));
  tracer::rclcpp_callback_init(reinterpret_cast<const void*>(kTimerRclcppHandler),
                               reinterpret_cast<const void*>(kTimerRclHandler),
                               tracer::CallbackType::Timer);
  tracer::rcl_callback_init("timer_callback", reinterpret_cast<const void*>(kTimerRclHandler));

  tracer::executor_execute(reinterpret_cast<const void*>(kSubscriptionRclcppHandler), 100);
  tracer::callback_start(reinterpret_cast<const void*>(kSubscriptionRclcppHandler), 200);
  tracer::callback_end(reinterpret_cast<const void*>(kSubscriptionRclcppHandler), 300);
  tracer::rcl_take(reinterpret_cast<const void*>(kSubscriptionRclHandler), 512, 50, 90);
  tracer::executor_execute(reinterpret_cast<const void*>(kTimerRclcppHandler), 400);
  tracer::callback_start(reinterpret_cast<const void*>(kTimerRclcppHandler), 500);
  tracer::callback_end(reinterpret_cast<const void*>(kTimerRclcppHandler), 600);
  tracer::rcl_take(reinterpret_cast<const void*>(kTimerRclHandler), 1024, 350, 390);
}

// Overwrites each ring beyond its capacity: registration_capacity + 2
// callbacks (two records each) and runtime_capacity + 2 events, with
// handlers and timestamps derived from the index.
void write_overflow_sequence(std::uint64_t registration_capacity,
                             std::uint64_t runtime_capacity) {
  for (std::uint64_t i = 0; i < registration_capacity + 2; ++i) {
    const std::uint64_t handler = 0xa000 + i;
    tracer::rclcpp_callback_init(reinterpret_cast<const void*>(handler),
                                 reinterpret_cast<const void*>(handler + 0x100),
                                 tracer::CallbackType::Subscription);
    tracer::rcl_callback_init("overflow_callback",
                              reinterpret_cast<const void*>(handler + 0x100));
  }
  for (std::uint64_t i = 0; i < runtime_capacity + 2; ++i) {
    tracer::executor_execute(reinterpret_cast<const void*>(0xa000 + i), 1000 + i * 10);
  }
}

void dump_fixture(const std::string& path, std::uint64_t registration_capacity,
                  std::uint64_t runtime_capacity) {
  const auto* image = static_cast<const std::uint8_t*>(tracer::image());
  const std::size_t size = tracer::image_size();
  std::vector<std::uint8_t> copy(image, image + size);

  const std::uint64_t registration_ring = align_up(sizeof(tracer::SharedHeader), 8);
  const std::uint64_t runtime_ring = registration_ring + sizeof(tracer::RingHeader) +
                                     registration_capacity *
                                         sizeof(tracer::RegistrationRecord);
  std::memset(copy.data() + registration_ring, 0, sizeof(pthread_mutex_t));
  std::memset(copy.data() + runtime_ring, 0, sizeof(pthread_mutex_t));

  std::ofstream out(path, std::ios::binary);
  out.write(reinterpret_cast<const char*>(copy.data()),
            static_cast<std::streamsize>(copy.size()));
  if (!out) {
    std::cerr << "failed to write fixture " << path << std::endl;
    std::exit(1);
  }
}

}  // namespace

int main(int argc, char** argv) {
  std::string shm_name;
  std::string fixture_path;
  std::uint64_t registration_capacity = 8;
  std::uint64_t runtime_capacity = 8;
  bool overflow = false;
  bool cleanup = false;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    const auto next_value = [&](const char* flag) -> std::string {
      if (i + 1 >= argc) {
        std::cerr << flag << " needs a value" << std::endl;
        std::exit(2);
      }
      return argv[++i];
    };
    if (arg == "--fixture") {
      fixture_path = next_value("--fixture");
    } else if (arg == "--reg-capacity") {
      registration_capacity = std::stoull(next_value("--reg-capacity"));
    } else if (arg == "--runtime-capacity") {
      runtime_capacity = std::stoull(next_value("--runtime-capacity"));
    } else if (arg == "--overflow") {
      overflow = true;
    } else if (arg == "--cleanup") {
      cleanup = true;
    } else if (arg.rfind("--", 0) == 0) {
      std::cerr << "unknown flag: " << arg << std::endl;
      return 2;
    } else if (shm_name.empty()) {
      shm_name = arg;
    } else {
      std::cerr << "unexpected argument: " << arg << std::endl;
      return 2;
    }
  }

  if (shm_name.empty()) {
    std::cerr << "usage: mock_writer <shm_name> [--fixture <path>]"
              << " [--reg-capacity N] [--runtime-capacity N]"
              << " [--overflow] [--cleanup]" << std::endl;
    return 2;
  }

  try {
    tracer::init(shm_name.c_str(), registration_capacity, runtime_capacity);
    if (overflow) {
      write_overflow_sequence(registration_capacity, runtime_capacity);
    } else {
      write_sequence();
    }
    if (!fixture_path.empty()) {
      dump_fixture(fixture_path, registration_capacity, runtime_capacity);
    }
  } catch (const std::exception& error) {
    std::cerr << "mock_writer: " << error.what() << std::endl;
    return 1;
  }

  if (cleanup) {
    shm_unlink(shm_name.c_str());
  }
  return 0;
}
