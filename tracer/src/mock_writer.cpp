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
#include <thread>
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
  tracer::rcl_callback_init("/cmd_vel_callback", "/robot",
                            reinterpret_cast<const void*>(kSubscriptionRclHandler));
  tracer::rclcpp_callback_init(reinterpret_cast<const void*>(kTimerRclcppHandler),
                               reinterpret_cast<const void*>(kTimerRclHandler),
                               tracer::CallbackType::Timer);
  tracer::rcl_callback_init("timer_callback", "/robot",
                            reinterpret_cast<const void*>(kTimerRclHandler));

  tracer::executor_execute(reinterpret_cast<const void*>(kSubscriptionRclcppHandler), 100);
  tracer::callback_start(reinterpret_cast<const void*>(kSubscriptionRclcppHandler), 200);
  tracer::callback_end(reinterpret_cast<const void*>(kSubscriptionRclcppHandler), 300);
  tracer::rcl_take(reinterpret_cast<const void*>(kSubscriptionRclHandler), 512, 50, 90);
  tracer::executor_execute(reinterpret_cast<const void*>(kTimerRclcppHandler), 400);
  tracer::callback_start(reinterpret_cast<const void*>(kTimerRclcppHandler), 500);
  tracer::callback_end(reinterpret_cast<const void*>(kTimerRclcppHandler), 600);
  tracer::rcl_take(reinterpret_cast<const void*>(kTimerRclHandler), 1024, 350, 390);
  // Delimits the single payload round this sequence represents.
  tracer::round_boundary(1, 700);
}

// Overwrites each ring beyond its capacity: reg_capacity + 2
// callbacks (two records each) and rt_capacity + 2 events, with
// handlers and timestamps derived from the index.
void write_overflow_sequence(std::uint64_t reg_capacity,
                             std::uint64_t rt_capacity) {
  for (std::uint64_t i = 0; i < reg_capacity + 2; ++i) {
    const std::uint64_t handler = 0xa000 + i;
    tracer::rclcpp_callback_init(reinterpret_cast<const void*>(handler),
                                 reinterpret_cast<const void*>(handler + 0x100),
                                 tracer::CallbackType::Subscription);
    tracer::rcl_callback_init("overflow_callback", "/overflow",
                              reinterpret_cast<const void*>(handler + 0x100));
  }
  for (std::uint64_t i = 0; i < rt_capacity + 2; ++i) {
    tracer::executor_execute(reinterpret_cast<const void*>(0xa000 + i), 1000 + i * 10);
  }
}

// One fuzzing round for the Rust end-to-end example: the fixed two-callback
// registration plus a single runtime cycle whose callback latencies, message
// size and timer participation are controlled by the caller. The Rust side
// derives them deterministically from the payload under test.
void write_live_sequence(std::uint64_t sched_sub, std::uint64_t exec_sub,
                         std::uint64_t sched_timer, std::uint64_t exec_timer,
                         std::uint64_t buffer_size, std::uint64_t pub_timestamp,
                         std::uint64_t sub_timestamp, bool skip_timer) {
  tracer::rclcpp_callback_init(reinterpret_cast<const void*>(kSubscriptionRclcppHandler),
                               reinterpret_cast<const void*>(kSubscriptionRclHandler),
                               tracer::CallbackType::Subscription);
  tracer::rcl_callback_init("/cmd_vel_callback", "/robot",
                            reinterpret_cast<const void*>(kSubscriptionRclHandler));
  tracer::rclcpp_callback_init(reinterpret_cast<const void*>(kTimerRclcppHandler),
                               reinterpret_cast<const void*>(kTimerRclHandler),
                               tracer::CallbackType::Timer);
  tracer::rcl_callback_init("timer_callback", "/robot",
                            reinterpret_cast<const void*>(kTimerRclHandler));

  constexpr std::uint64_t kInvokeSub = 100;
  constexpr std::uint64_t kInvokeTimer = 400;
  tracer::executor_execute(reinterpret_cast<const void*>(kSubscriptionRclcppHandler),
                           kInvokeSub);
  tracer::callback_start(reinterpret_cast<const void*>(kSubscriptionRclcppHandler),
                         kInvokeSub + sched_sub);
  tracer::callback_end(reinterpret_cast<const void*>(kSubscriptionRclcppHandler),
                       kInvokeSub + sched_sub + exec_sub);
  tracer::rcl_take(reinterpret_cast<const void*>(kSubscriptionRclHandler), buffer_size,
                   pub_timestamp, sub_timestamp);
  if (!skip_timer) {
    tracer::executor_execute(reinterpret_cast<const void*>(kTimerRclcppHandler),
                             kInvokeTimer);
    tracer::callback_start(reinterpret_cast<const void*>(kTimerRclcppHandler),
                           kInvokeTimer + sched_timer);
    tracer::callback_end(reinterpret_cast<const void*>(kTimerRclcppHandler),
                         kInvokeTimer + sched_timer + exec_timer);
  }
}

// High-rate writer for the concurrent reader stress test: each thread
// registers one subscription callback with a thread-derived handler set,
// then writes `rounds` execute/start/end/take cycles as fast as possible.
// Small ring capacities force continuous overflow so the Rust reader can
// prove it observes losses but never torn records.
void write_stress(std::uint64_t rounds, std::uint64_t threads) {
  auto worker = [rounds](std::uint64_t thread_index) {
    const std::uint64_t base = 0x10000 + thread_index * 0x1000;
    const auto* rclcpp_handler = reinterpret_cast<const void*>(base);
    const auto* rcl_handler = reinterpret_cast<const void*>(base + 0x100);
    tracer::rclcpp_callback_init(rclcpp_handler, rcl_handler,
                                 tracer::CallbackType::Subscription);
    tracer::rcl_callback_init("stress_callback", "/stress", rcl_handler);
    for (std::uint64_t i = 0; i < rounds; ++i) {
      tracer::executor_execute(rclcpp_handler, tracer::now_ns());
      tracer::callback_start(rclcpp_handler, tracer::now_ns());
      tracer::callback_end(rclcpp_handler, tracer::now_ns());
      tracer::rcl_take(rcl_handler, 512, tracer::now_ns(), tracer::now_ns());
    }
  };

  std::vector<std::thread> pool;
  for (std::uint64_t t = 1; t < threads; ++t) {
    pool.emplace_back(worker, t);
  }
  worker(0);
  for (auto& thread : pool) {
    thread.join();
  }
}

void dump_fixture(const std::string& path, std::uint64_t reg_capacity,
                  std::uint64_t rt_capacity) {
  const auto* image = static_cast<const std::uint8_t*>(tracer::image());
  const std::size_t size = tracer::image_size();
  std::vector<std::uint8_t> copy(image, image + size);

  const std::uint64_t registration_ring = align_up(sizeof(tracer::SharedHeader), 8);
  const std::uint64_t runtime_ring = registration_ring + sizeof(tracer::RingHeader) +
                                     reg_capacity *
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
  std::uint64_t reg_capacity = 8;
  std::uint64_t rt_capacity = 16;
  bool overflow = false;
  bool cleanup = false;
  bool live = false;
  bool skip_timer = false;
  bool crash = false;
  bool stress = false;
  std::uint64_t stress_rounds = 1000;
  std::uint64_t threads = 1;
  std::int64_t mark_round = -1;
  std::uint64_t sched_sub = 10;
  std::uint64_t exec_sub = 100;
  std::uint64_t sched_timer = 10;
  std::uint64_t exec_timer = 100;
  std::uint64_t buffer_size = 512;
  std::uint64_t pub_timestamp = 50;
  std::uint64_t sub_timestamp = 90;

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
      reg_capacity = std::stoull(next_value("--reg-capacity"));
    } else if (arg == "--runtime-capacity") {
      rt_capacity = std::stoull(next_value("--runtime-capacity"));
    } else if (arg == "--overflow") {
      overflow = true;
    } else if (arg == "--cleanup") {
      cleanup = true;
    } else if (arg == "--live") {
      live = true;
    } else if (arg == "--sched-sub") {
      sched_sub = std::stoull(next_value("--sched-sub"));
    } else if (arg == "--exec-sub") {
      exec_sub = std::stoull(next_value("--exec-sub"));
    } else if (arg == "--sched-timer") {
      sched_timer = std::stoull(next_value("--sched-timer"));
    } else if (arg == "--exec-timer") {
      exec_timer = std::stoull(next_value("--exec-timer"));
    } else if (arg == "--size") {
      buffer_size = std::stoull(next_value("--size"));
    } else if (arg == "--pub") {
      pub_timestamp = std::stoull(next_value("--pub"));
    } else if (arg == "--sub") {
      sub_timestamp = std::stoull(next_value("--sub"));
    } else if (arg == "--no-timer") {
      skip_timer = true;
    } else if (arg == "--stress") {
      stress = true;
    } else if (arg == "--stress-rounds") {
      stress_rounds = std::stoull(next_value("--stress-rounds"));
    } else if (arg == "--threads") {
      threads = std::stoull(next_value("--threads"));
    } else if (arg == "--mark-round") {
      mark_round = std::stoll(next_value("--mark-round"));
    } else if (arg == "--crash") {
      crash = true;
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
              << " [--overflow] [--cleanup] [--mark-round N]" << std::endl
              << "       mock_writer <shm_name> --live [--sched-sub N]"
              << " [--exec-sub N] [--sched-timer N] [--exec-timer N]"
              << " [--size N] [--pub N] [--sub N] [--no-timer] [--crash]"
              << " [--mark-round N]" << std::endl
              << "       mock_writer <shm_name> --stress [--stress-rounds N]"
              << " [--threads N] [--reg-capacity N] [--runtime-capacity N]"
              << std::endl;
    return 2;
  }

  try {
    tracer::init(shm_name.c_str(), reg_capacity, rt_capacity);
    if (stress) {
      write_stress(stress_rounds, threads);
    } else if (live) {
      write_live_sequence(sched_sub, exec_sub, sched_timer, exec_timer, buffer_size,
                          pub_timestamp, sub_timestamp, skip_timer);
    } else if (overflow) {
      write_overflow_sequence(reg_capacity, rt_capacity);
    } else {
      write_sequence();
    }
    if (mark_round >= 0) {
      tracer::round_boundary(static_cast<std::uint32_t>(mark_round), tracer::now_ns());
    }
    if (!fixture_path.empty()) {
      dump_fixture(fixture_path, reg_capacity, rt_capacity);
    }
  } catch (const std::exception& error) {
    std::cerr << "mock_writer: " << error.what() << std::endl;
    return 1;
  }

  if (cleanup) {
    shm_unlink(shm_name.c_str());
  }
  // 139 mimics a SIGSEGV exit, used by the end-to-end example to simulate
  // a system crash after the trace for this round was fully written.
  return crash ? 139 : 0;
}
