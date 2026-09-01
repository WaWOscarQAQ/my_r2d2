#include "tracer/circular_buffer.h"
#include "tracer/shared_memory.h"
#include "tracer/trace_records.h"
#include "tracer/tracers.h"

#include <cerrno>
#include <atomic>
#include <cstdint>
#include <csignal>
#include <cstring>
#include <ctime>
#include <dlfcn.h>
#include <fcntl.h>
#include <mutex>
#include <new>
#include <stdexcept>
#include <string>
#include <thread>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

namespace tracer {

namespace {

constexpr std::uint64_t kDefaultRegistrationCapacity = 16384;
constexpr std::uint64_t kDefaultRuntimeCapacity = 262144;

std::uint64_t align_up(std::uint64_t value, std::uint64_t alignment) {
  return (value + alignment - 1) / alignment * alignment;
}

std::uint64_t ring_bytes(std::uint64_t capacity, std::uint64_t record_size) {
  return sizeof(RingHeader) + capacity * record_size;
}

SharedMemory g_shm;
RingBuffer<RegistrationRecord> g_registration;
RingBuffer<RuntimeRecord> g_runtime;

std::uint64_t address_of(const void* pointer) {
  std::uint64_t value = 1469598103934665603ULL;
  value ^= static_cast<std::uint64_t>(::getpid());
  value *= 1099511628211ULL;
  value ^= reinterpret_cast<std::uint64_t>(pointer);
  value *= 1099511628211ULL;
  return value;
}

// Copies a NUL-terminated string into a fixed-capacity record field,
// truncating and setting `truncated_flag` when it does not fit.
void copy_string_field(char* dest, std::size_t capacity, const char* source,
                       std::uint32_t truncated_flag, std::uint32_t* len_out,
                       std::uint32_t* flags) {
  const std::size_t observed_len = ::strnlen(source, capacity + 1);
  const std::size_t len = observed_len > capacity ? capacity : observed_len;
  if (observed_len > capacity) {
    *flags |= truncated_flag;
  }
  *len_out = static_cast<std::uint32_t>(len);
  std::memcpy(dest, source, len);
}

[[noreturn]] void throw_system(const char* operation) {
  throw std::runtime_error(std::string(operation) + ": " + std::strerror(errno));
}

using SanitizerCovDump = void (*)();
SanitizerCovDump g_sanitizer_cov_dump = nullptr;
int g_sancov_signal_pipe[2] = {-1, -1};
std::once_flag g_sancov_signal_install_once;

void resolve_sancov_dump() noexcept {
  if (g_sanitizer_cov_dump == nullptr) {
    g_sanitizer_cov_dump =
        reinterpret_cast<SanitizerCovDump>(dlsym(RTLD_DEFAULT, "__sanitizer_cov_dump"));
  }
}

void request_sancov_dump_on_signal(int) {
  const int fd = g_sancov_signal_pipe[1];
  if (fd >= 0) {
    const char byte = 1;
    const ssize_t ignored = ::write(fd, &byte, sizeof(byte));
    (void)ignored;
  }
}

void sancov_dump_worker() noexcept {
  char buffer[64];
  for (;;) {
    const ssize_t n = ::read(g_sancov_signal_pipe[0], buffer, sizeof(buffer));
    if (n < 0) {
      if (errno == EINTR) {
        continue;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      continue;
    }
    if (n == 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      continue;
    }
    resolve_sancov_dump();
    if (g_sanitizer_cov_dump != nullptr) {
      g_sanitizer_cov_dump();
    }
  }
}

void install_sancov_signal_handler() noexcept {
  const char* dir = std::getenv("R2D2_SANCOV_DIR");
  if (dir != nullptr && *dir != '\0') {
    try {
      std::call_once(g_sancov_signal_install_once, []() {
        resolve_sancov_dump();
        if (g_sanitizer_cov_dump == nullptr) {
          return;
        }
        if (::pipe(g_sancov_signal_pipe) != 0) {
          g_sancov_signal_pipe[0] = -1;
          g_sancov_signal_pipe[1] = -1;
          return;
        }
        const int flags = ::fcntl(g_sancov_signal_pipe[1], F_GETFL, 0);
        if (flags >= 0) {
          (void)::fcntl(g_sancov_signal_pipe[1], F_SETFL, flags | O_NONBLOCK);
        }
        std::thread(sancov_dump_worker).detach();
        std::signal(SIGUSR2, request_sancov_dump_on_signal);
      });
    } catch (...) {
      // Coverage dumping must never alter ROS control flow.
    }
  }
}

__attribute__((constructor)) void install_sancov_signal_handler_at_load() {
  install_sancov_signal_handler();
}

}  // namespace

std::uint64_t now_ns() noexcept {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<std::uint64_t>(ts.tv_sec) * 1000000000ULL +
         static_cast<std::uint64_t>(ts.tv_nsec);
}

void init(const char* shm_name, std::uint64_t reg_capacity,
          std::uint64_t rt_capacity) {
  if (g_shm.data() != nullptr) {
    throw std::logic_error("tracer shared memory already initialized");
  }
  if (reg_capacity == 0 || rt_capacity == 0) {
    throw std::invalid_argument("capacities must be positive");
  }

  const std::uint64_t registration_ring =
      ring_bytes(reg_capacity, sizeof(RegistrationRecord));
  const std::uint64_t runtime_ring =
      ring_bytes(rt_capacity, sizeof(RuntimeRecord));
  const std::uint64_t shm_size =
      align_up(sizeof(SharedHeader), 8) + registration_ring + runtime_ring;

  g_shm = SharedMemory::create(shm_name, shm_size);

  auto* header = new (g_shm.data()) SharedHeader();
  header->magic = 0;
  header->version = 0;
  header->shm_size = shm_size;
  header->reg_capacity = reg_capacity;
  header->reg_records_offset =
      align_up(sizeof(SharedHeader), 8) + sizeof(RingHeader);
  header->rt_capacity = rt_capacity;
  header->rt_records_offset =
      align_up(sizeof(SharedHeader), 8) + registration_ring + sizeof(RingHeader);

  auto* base = static_cast<std::uint8_t*>(g_shm.data());
  g_registration.init(base + align_up(sizeof(SharedHeader), 8), reg_capacity);
  g_runtime.init(base + align_up(sizeof(SharedHeader), 8) + registration_ring,
                 rt_capacity);

  // Publish the magic/version only after the ring headers are initialized.
  // Other instrumented ROS processes may attach concurrently as soon as
  // shm_open succeeds; if they see a valid SharedHeader before RingHeader
  // capacity is written, writers can later divide by zero in RingBuffer::push.
  std::atomic_thread_fence(std::memory_order_release);
  header->magic = kTraceMagic;
  header->version = kTraceVersion;

  install_sancov_signal_handler();
}

void init(const char* shm_name) {
  init(shm_name, kDefaultRegistrationCapacity, kDefaultRuntimeCapacity);
}

void attach(const char* shm_name) {
  if (g_shm.data() != nullptr) {
    throw std::logic_error("tracer shared memory already initialized");
  }
  g_shm = SharedMemory::open(shm_name);

  auto* base = static_cast<std::uint8_t*>(g_shm.data());
  const auto* header = static_cast<const SharedHeader*>(g_shm.data());
  if (header->magic != kTraceMagic || header->version != kTraceVersion) {
    throw std::runtime_error("shared memory magic/version mismatch");
  }
  if (header->shm_size != g_shm.size() ||
      header->reg_records_offset > header->shm_size ||
      header->rt_records_offset > header->shm_size) {
    throw std::runtime_error("shared memory header is inconsistent");
  }

  const std::uint64_t registration_ring =
      ring_bytes(header->reg_capacity, sizeof(RegistrationRecord));
  g_registration.attach(base + align_up(sizeof(SharedHeader), 8));
  g_runtime.attach(base + align_up(sizeof(SharedHeader), 8) + registration_ring);
  if (g_registration.capacity() != header->reg_capacity ||
      g_runtime.capacity() != header->rt_capacity ||
      g_registration.capacity() == 0 || g_runtime.capacity() == 0) {
    throw std::runtime_error("shared memory ring header is inconsistent");
  }

  install_sancov_signal_handler();
}

void rclcpp_callback_init(const void* rclcpp_handler, const void* rcl_handler,
                          CallbackType callback_type) noexcept {
  if (g_shm.data() == nullptr) {
    return;
  }
  RegistrationRecord record{};
  record.source = RegistrationSource::Rclcpp;
  record.callback_type = callback_type;
  record.rclcpp_handler = address_of(rclcpp_handler);
  record.rcl_handler = address_of(rcl_handler);
  g_registration.push(record);
}

void rcl_callback_init(const char* callback_name, const char* callback_namespace,
                       const void* rcl_handler) noexcept {
  if (g_shm.data() == nullptr || callback_name == nullptr ||
      callback_namespace == nullptr) {
    return;
  }
  RegistrationRecord record{};
  record.source = RegistrationSource::Rcl;
  record.rcl_handler = address_of(rcl_handler);
  copy_string_field(record.callback_name, kCallbackNameCapacity, callback_name,
                    kRegistrationFlagNameTruncated, &record.callback_name_len,
                    &record.flags);
  copy_string_field(record.callback_namespace, kCallbackNamespaceCapacity,
                    callback_namespace, kRegistrationFlagNamespaceTruncated,
                    &record.callback_namespace_len, &record.flags);
  g_registration.push(record);
}

void executor_execute(const void* rclcpp_handler,
                      std::uint64_t invoke_timestamp) noexcept {
  if (g_shm.data() == nullptr) {
    return;
  }
  RuntimeRecord record{};
  record.event_type = RuntimeEventType::ExecutorExecute;
  record.rclcpp_handler = address_of(rclcpp_handler);
  record.timestamp = invoke_timestamp;
  g_runtime.push(record);
}

void callback_start(const void* rclcpp_handler,
                    std::uint64_t start_timestamp) noexcept {
  if (g_shm.data() == nullptr) {
    return;
  }
  RuntimeRecord record{};
  record.event_type = RuntimeEventType::CallbackStart;
  record.rclcpp_handler = address_of(rclcpp_handler);
  record.timestamp = start_timestamp;
  g_runtime.push(record);
}

void callback_end(const void* rclcpp_handler,
                  std::uint64_t end_timestamp) noexcept {
  if (g_shm.data() == nullptr) {
    return;
  }
  RuntimeRecord record{};
  record.event_type = RuntimeEventType::CallbackEnd;
  record.rclcpp_handler = address_of(rclcpp_handler);
  record.timestamp = end_timestamp;
  g_runtime.push(record);
}

void rcl_take(const void* rcl_handler, std::uint64_t buffer_size,
              std::uint64_t pub_timestamp, std::uint64_t sub_timestamp) noexcept {
  if (g_shm.data() == nullptr) {
    return;
  }
  RuntimeRecord record{};
  record.event_type = RuntimeEventType::RclTake;
  record.rcl_handler = address_of(rcl_handler);
  record.buffer_size = buffer_size;
  record.pub_timestamp = pub_timestamp;
  record.sub_timestamp = sub_timestamp;
  g_runtime.push(record);
}

void round_boundary(std::uint32_t round_id, std::uint64_t timestamp) noexcept {
  if (g_shm.data() == nullptr) {
    return;
  }
  RuntimeRecord record{};
  record.event_type = RuntimeEventType::RoundBoundary;
  record.aux = round_id;
  record.timestamp = timestamp;
  g_runtime.push(record);
}

const void* image() { return g_shm.data(); }
std::size_t image_size() { return g_shm.size(); }

SharedMemory::SharedMemory(SharedMemory&& other) noexcept { *this = std::move(other); }

SharedMemory& SharedMemory::operator=(SharedMemory&& other) noexcept {
  if (this != &other) {
    if (data_ != nullptr) {
      munmap(data_, size_);
    }
    if (fd_ >= 0) {
      close(fd_);
    }
    fd_ = other.fd_;
    name_ = std::move(other.name_);
    data_ = other.data_;
    size_ = other.size_;
    other.fd_ = -1;
    other.data_ = nullptr;
    other.size_ = 0;
  }
  return *this;
}

SharedMemory::~SharedMemory() {
  if (data_ != nullptr) {
    munmap(data_, size_);
  }
  if (fd_ >= 0) {
    close(fd_);
  }
}

SharedMemory SharedMemory::create(const char* name, std::size_t size) {
  const int fd = shm_open(name, O_CREAT | O_EXCL | O_RDWR, 0600);
  if (fd < 0) {
    throw_system("shm_open");
  }
  if (ftruncate(fd, static_cast<off_t>(size)) != 0) {
    const int error = errno;
    close(fd);
    shm_unlink(name);
    errno = error;
    throw_system("ftruncate");
  }
  void* data = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  if (data == MAP_FAILED) {
    const int error = errno;
    close(fd);
    shm_unlink(name);
    errno = error;
    throw_system("mmap");
  }
  SharedMemory result;
  result.fd_ = fd;
  result.name_ = name;
  result.data_ = data;
  result.size_ = size;
  return result;
}

SharedMemory SharedMemory::open(const char* name) {
  const int fd = shm_open(name, O_RDWR, 0);
  if (fd < 0) {
    throw_system("shm_open");
  }
  struct stat st {};
  if (fstat(fd, &st) != 0) {
    const int error = errno;
    close(fd);
    errno = error;
    throw_system("fstat");
  }
  void* data = mmap(nullptr, static_cast<std::size_t>(st.st_size), PROT_READ | PROT_WRITE,
                    MAP_SHARED, fd, 0);
  if (data == MAP_FAILED) {
    const int error = errno;
    close(fd);
    errno = error;
    throw_system("mmap");
  }
  SharedMemory result;
  result.fd_ = fd;
  result.name_ = name;
  result.data_ = data;
  result.size_ = static_cast<std::size_t>(st.st_size);
  return result;
}

void SharedMemory::unlink() {
  if (!name_.empty()) {
    shm_unlink(name_.c_str());
  }
}

}  // namespace tracer
