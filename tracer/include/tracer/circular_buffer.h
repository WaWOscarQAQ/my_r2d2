#ifndef TRACER_CIRCULAR_BUFFER_H
#define TRACER_CIRCULAR_BUFFER_H

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <new>
#include <pthread.h>
#include <type_traits>

namespace tracer {

// Lock-guarded header of one ring buffer, stored in shared memory. The
// mutex comes first so the Rust reader skips a fixed-size slot and reads
// the three counters at known offsets.
struct RingHeader {
    pthread_mutex_t mutex;
    std::atomic<std::uint64_t> write_index;
    std::atomic<std::uint64_t> overflow_count;
    std::uint64_t capacity;
};

// The Rust reader mirrors these constants; keep them in sync.
static_assert(sizeof(pthread_mutex_t) == 40,
              "Rust reader assumes the glibc x86-64 mutex size");
static_assert(sizeof(RingHeader) == 64, "layout must match the Rust reader");
static_assert(std::atomic<std::uint64_t>::is_always_lock_free,
              "the reader relies on lock-free 64-bit counters");

// Ring buffer view over shared memory. Writers serialize on the mutex;
// readers never lock and only follow write_index, which is published
// after the record payload is fully written.
template <typename Record>
class RingBuffer {
  static_assert(std::is_trivially_copyable<Record>::value,
                "records are memcpy'd into shared memory");

 public:
  static constexpr std::size_t header_size() { return sizeof(RingHeader); }

  // Constructs the header in place at `ring_start` and initializes the
  // process-shared mutex. The record array lives right after the header.
  void init(void* ring_start, std::size_t capacity) {
    header_ = new (ring_start) RingHeader();
    records_ = reinterpret_cast<Record*>(
        reinterpret_cast<std::uint8_t*>(ring_start) + sizeof(RingHeader));

    pthread_mutexattr_t attr;
    pthread_mutexattr_init(&attr);
    pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);
    pthread_mutex_init(&header_->mutex, &attr);
    pthread_mutexattr_destroy(&attr);

    header_->write_index.store(0, std::memory_order_relaxed);
    header_->overflow_count.store(0, std::memory_order_relaxed);
    header_->capacity = capacity;
  }

  // Writes one record and publishes its index. Overwriting the oldest
  // record increments overflow_count so readers can detect data loss.
  void push(const Record& record) {
    pthread_mutex_lock(&header_->mutex);
    const std::uint64_t index =
        header_->write_index.load(std::memory_order_relaxed);
    if (index >= header_->capacity) {
      header_->overflow_count.fetch_add(1, std::memory_order_relaxed);
    }
    records_[index % header_->capacity] = record;
    header_->write_index.store(index + 1, std::memory_order_release);
    pthread_mutex_unlock(&header_->mutex);
  }

  const Record* records() const { return records_; }
  std::uint64_t write_index() const {
    return header_->write_index.load(std::memory_order_acquire);
  }
  std::uint64_t overflow_count() const {
    return header_->overflow_count.load(std::memory_order_relaxed);
  }
  std::uint64_t capacity() const { return header_->capacity; }

 private:
  RingHeader* header_ = nullptr;
  Record* records_ = nullptr;
};

}  // namespace tracer

#endif
