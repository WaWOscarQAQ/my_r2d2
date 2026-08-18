#include "tracer/circular_buffer.h"
#include "tracer/shared_memory.h"
#include "tracer/trace_records.h"
#include "tracer/tracers.h"

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <new>
#include <stdexcept>
#include <string>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

namespace tracer {

namespace {

constexpr std::uint64_t kDefaultRegistrationCapacity = 1024;
constexpr std::uint64_t kDefaultRuntimeCapacity = 4096;

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
  return reinterpret_cast<std::uint64_t>(pointer);
}

void require_init() {
  if (g_shm.data() == nullptr) {
    throw std::logic_error("tracer shared memory not initialized; call tracer::init() first");
  }
}

[[noreturn]] void throw_system(const char* operation) {
  throw std::runtime_error(std::string(operation) + ": " + std::strerror(errno));
}

}  // namespace

std::uint64_t now_ns() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<std::uint64_t>(ts.tv_sec) * 1000000000ULL +
         static_cast<std::uint64_t>(ts.tv_nsec);
}

void init(const char* shm_name, std::uint64_t registration_capacity,
          std::uint64_t runtime_capacity) {
  if (g_shm.data() != nullptr) {
    throw std::logic_error("tracer shared memory already initialized");
  }
  if (registration_capacity == 0 || runtime_capacity == 0) {
    throw std::invalid_argument("capacities must be positive");
  }

  const std::uint64_t registration_ring =
      ring_bytes(registration_capacity, sizeof(RegistrationRecord));
  const std::uint64_t runtime_ring =
      ring_bytes(runtime_capacity, sizeof(RuntimeRecord));
  const std::uint64_t shm_size =
      align_up(sizeof(SharedHeader), 8) + registration_ring + runtime_ring;

  g_shm = SharedMemory::create(shm_name, shm_size);

  auto* header = new (g_shm.data()) SharedHeader();
  header->magic = kTraceMagic;
  header->version = kTraceVersion;
  header->shm_size = shm_size;
  header->registration_capacity = registration_capacity;
  header->registration_records_offset =
      align_up(sizeof(SharedHeader), 8) + sizeof(RingHeader);
  header->runtime_capacity = runtime_capacity;
  header->runtime_records_offset =
      align_up(sizeof(SharedHeader), 8) + registration_ring + sizeof(RingHeader);

  auto* base = static_cast<std::uint8_t*>(g_shm.data());
  g_registration.init(base + align_up(sizeof(SharedHeader), 8), registration_capacity);
  g_runtime.init(base + align_up(sizeof(SharedHeader), 8) + registration_ring,
                 runtime_capacity);
}

void init(const char* shm_name) {
  init(shm_name, kDefaultRegistrationCapacity, kDefaultRuntimeCapacity);
}

void rclcpp_callback_init(const void* rclcpp_handler, const void* rcl_handler,
                          CallbackType callback_type) {
  require_init();
  RegistrationRecord record{};
  record.source = RegistrationSource::Rclcpp;
  record.callback_type = callback_type;
  record.rclcpp_handler = address_of(rclcpp_handler);
  record.rcl_handler = address_of(rcl_handler);
  g_registration.push(record);
}

void rcl_callback_init(const char* callback_name, const void* rcl_handler) {
  require_init();
  RegistrationRecord record{};
  record.source = RegistrationSource::Rcl;
  record.rcl_handler = address_of(rcl_handler);
  const std::size_t len = std::strlen(callback_name);
  if (len > kCallbackNameCapacity) {
    throw std::invalid_argument("callback name exceeds record capacity");
  }
  record.callback_name_len = static_cast<std::uint32_t>(len);
  std::memcpy(record.callback_name, callback_name, len);
  g_registration.push(record);
}

void executor_execute(const void* rclcpp_handler, std::uint64_t invoke_timestamp) {
  require_init();
  RuntimeRecord record{};
  record.event_type = RuntimeEventType::ExecutorExecute;
  record.rclcpp_handler = address_of(rclcpp_handler);
  record.timestamp = invoke_timestamp;
  g_runtime.push(record);
}

void callback_start(const void* rclcpp_handler, std::uint64_t start_timestamp) {
  require_init();
  RuntimeRecord record{};
  record.event_type = RuntimeEventType::CallbackStart;
  record.rclcpp_handler = address_of(rclcpp_handler);
  record.timestamp = start_timestamp;
  g_runtime.push(record);
}

void callback_end(const void* rclcpp_handler, std::uint64_t end_timestamp) {
  require_init();
  RuntimeRecord record{};
  record.event_type = RuntimeEventType::CallbackEnd;
  record.rclcpp_handler = address_of(rclcpp_handler);
  record.timestamp = end_timestamp;
  g_runtime.push(record);
}

void rcl_take(const void* rcl_handler, std::uint64_t buffer_size,
              std::uint64_t pub_timestamp, std::uint64_t sub_timestamp) {
  require_init();
  RuntimeRecord record{};
  record.event_type = RuntimeEventType::RclTake;
  record.rcl_handler = address_of(rcl_handler);
  record.buffer_size = buffer_size;
  record.pub_timestamp = pub_timestamp;
  record.sub_timestamp = sub_timestamp;
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
  const int fd = shm_open(name, O_CREAT | O_RDWR, 0600);
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
