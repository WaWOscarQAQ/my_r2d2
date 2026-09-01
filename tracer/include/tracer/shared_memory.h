#ifndef TRACER_SHARED_MEMORY_H
#define TRACER_SHARED_MEMORY_H

#include <cstddef>
#include <cstdint>
#include <string>

namespace tracer {

// Fixed header at the start of the shared memory object. The Rust reader
// mirrors these offsets.
struct SharedHeader {
    std::uint32_t magic;
    std::uint32_t version;
    std::uint64_t shm_size;
    std::uint64_t reg_capacity;
    std::uint64_t reg_records_offset;
    std::uint64_t rt_capacity;
    std::uint64_t rt_records_offset;
};

static_assert(sizeof(SharedHeader) == 48, "layout must match the Rust reader");

// RAII handle over a POSIX shared memory object. `create` truncates and
// maps a new object; `open` maps an existing one. Destroying the handle
// unmaps but does not unlink, so a reader can keep opening the object;
// call unlink() explicitly when the object is no longer needed.
class SharedMemory {
 public:
  SharedMemory() = default;
  SharedMemory(const SharedMemory&) = delete;
  SharedMemory& operator=(const SharedMemory&) = delete;
  SharedMemory(SharedMemory&& other) noexcept;
  SharedMemory& operator=(SharedMemory&& other) noexcept;
  ~SharedMemory();

  static SharedMemory create(const char* name, std::size_t size);
  static SharedMemory open(const char* name);

  void* data() const { return data_; }
  std::size_t size() const { return size_; }
  void unlink();

 private:
  int fd_ = -1;
  std::string name_;
  void* data_ = nullptr;
  std::size_t size_ = 0;
};

}  // namespace tracer

#endif
