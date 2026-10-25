#ifndef EBUS_ESP_TIMING_POLICY_HPP
#define EBUS_ESP_TIMING_POLICY_HPP

#include <cstdint>

// Integer-only policy shared by the ESP ISR and host tests. All ISR helpers
// are forced inline: no flash-resident out-of-line calls during cache stalls.
namespace ebus_timing {
#if defined(__GNUC__)
#define EBUS_TIMING_INLINE inline __attribute__((always_inline))
#else
#define EBUS_TIMING_INLINE inline
#endif

// ESP32-C3's toolchain can emit flash-resident std::atomic<bool>::load
// helpers at -Os. Aligned word load/store builtins compile to native loads,
// stores and fences. fetch_add is task-only (RISC-V may emulate its RMW).
class AtomicWord {
 public:
  constexpr AtomicWord(uint32_t value = 0) : value_(value) {}
  AtomicWord(const AtomicWord&) = delete;
  AtomicWord& operator=(const AtomicWord&) = delete;
  EBUS_TIMING_INLINE uint32_t load() const {
    return __atomic_load_n(&value_, __ATOMIC_ACQUIRE);
  }
  EBUS_TIMING_INLINE void store(uint32_t value) {
    __atomic_store_n(&value_, value, __ATOMIC_RELEASE);
  }
  uint32_t fetch_add(uint32_t value) {
    return __atomic_fetch_add(&value_, value, __ATOMIC_ACQ_REL);
  }
 private:
  alignas(4) uint32_t value_;
};
static_assert(alignof(AtomicWord) >= 4);

inline constexpr uint32_t earliest_us = 4300;
inline constexpr uint32_t latest_us = 4456;
// Engineering allowance for GPIO ISR timestamp latency and UART/adapter
// launch latency. Validate these margins on the actual board; they are not
// measured calibration values. Late software execution is rejected separately.
inline constexpr uint32_t guard_us = 32;
inline constexpr uint32_t arm_margin_us = 15;
inline constexpr uint32_t edge_tolerance_us = 40;

struct SynStamp {
  uint64_t start_us = 0;
  uint32_t generation = 0;
  bool valid = false;
};

struct Edges {
  uint64_t time[4]{};
  uint32_t generation = 0;
  uint8_t count = 0;

  EBUS_TIMING_INLINE void clear() {
    count = 0;
    ++generation;
  }

  EBUS_TIMING_INLINE void push(uint64_t now) {
    time[0] = time[1];
    time[1] = time[2];
    time[2] = time[3];
    time[3] = now;
    if (count < 4) ++count;
    ++generation;
  }

  EBUS_TIMING_INLINE static bool close(uint64_t actual, uint32_t expected) {
    return actual >= expected - edge_tolerance_us &&
           actual <= expected + edge_tolerance_us;
  }

  // 0xAA is sent LSB first: start+bit0 stay low, so falling edges are
  // at 0, 3, 5, 7 bit times (1250, 833, 834 us between edges), not 0,2,4,6.
  // UART byte delivery only validates the captured pattern; it never supplies
  // an inferred start timestamp. No nearest-edge fallback is permitted.
  EBUS_TIMING_INLINE SynStamp syn(uint64_t now) const {
    if (count != 4 || now < time[3] || now < time[0] ||
        now - time[0] < 3900 || now - time[0] > latest_us - guard_us ||
        !close(time[1] - time[0], 1250) ||
        !close(time[2] - time[1], 833) ||
        !close(time[3] - time[2], 834) ||
        !close(time[3] - time[0], 2917)) return {};
    return {time[0], generation, true};
  }
};

EBUS_TIMING_INLINE bool targetValid(uint32_t window, uint32_t offset) {
  return window >= offset && window - offset >= earliest_us + guard_us &&
         window - offset <= latest_us - guard_us;
}

EBUS_TIMING_INLINE bool deadline(const SynStamp& syn, uint64_t now,
                                  uint32_t window, uint32_t offset,
                                  uint64_t& result) {
  if (!syn.valid || now < syn.start_us || !targetValid(window, offset))
    return false;
  result = syn.start_us + window - offset;
  return now < result && result - now >= arm_margin_us;
}

EBUS_TIMING_INLINE bool canFire(const SynStamp& syn, uint64_t deadline_us,
                                uint64_t now, uint32_t generation,
                                bool tx_idle) {
  return syn.valid && tx_idle && generation == syn.generation &&
         now >= deadline_us && now >= syn.start_us &&
         now - syn.start_us >= earliest_us + guard_us &&
         now - syn.start_us <= latest_us - guard_us;
}

#undef EBUS_TIMING_INLINE
}  // namespace ebus_timing

#endif  // EBUS_ESP_TIMING_POLICY_HPP
