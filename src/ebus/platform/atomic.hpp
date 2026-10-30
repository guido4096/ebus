/*
 * Copyright (C) 2026 Roland Jax, Guido Jansen
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#pragma once

#include <cstdint>

namespace ebus::detail::platform {

/**
 * @brief Atomic word for ISR-shared state without flash-resident helpers.
 *
 * ESP32-C3's toolchain can emit flash-resident std::atomic<bool>::load
 * helpers at -Os. Aligned word load/store builtins compile to native loads,
 * stores and fences. fetch_add is task-only (RISC-V may emulate its RMW).
 * Use std::atomic for task-to-task state instead.
 */
class Atomic {
 public:
  constexpr Atomic(uint32_t value = 0) : value_(value) {}
  Atomic(const Atomic&) = delete;
  Atomic& operator=(const Atomic&) = delete;
  [[gnu::always_inline]] uint32_t load() const {
    return __atomic_load_n(&value_, __ATOMIC_ACQUIRE);
  }
  [[gnu::always_inline]] void store(uint32_t value) {
    __atomic_store_n(&value_, value, __ATOMIC_RELEASE);
  }
  uint32_t fetch_add(uint32_t value) {
    return __atomic_fetch_add(&value_, value, __ATOMIC_ACQ_REL);
  }

 private:
  alignas(4) uint32_t value_;
};
static_assert(alignof(Atomic) >= 4);

}  // namespace ebus::detail::platform
