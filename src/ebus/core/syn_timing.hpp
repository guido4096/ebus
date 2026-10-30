/*
 * Copyright (C) 2026 Roland Jax, Guido Jansen
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#pragma once

#include <cstdint>
#include <ebus/detail/protocol_limits.hpp>

/**
 * @file syn_timing.hpp
 * Integer-only SYN arbitration timing policy shared by the ESP bus ISR and
 * host tests. Window bounds live in `BusLimits` (protocol_limits.hpp).
 * All ISR helpers are forced inline: no flash-resident out-of-line calls
 * during cache stalls.
 */

namespace ebus::detail::syn {

struct Stamp {
  uint64_t start_us = 0;
  uint32_t generation = 0;
  bool valid = false;
};

/**
 * @brief Rolling history of the newest four falling-edge timestamps (us).
 */
struct Edges {
  uint64_t time[4]{};
  uint32_t generation = 0;
  uint8_t count = 0;

  /** @brief Drops all edges; generation advances so stale stamps expire. */
  [[gnu::always_inline]] void clear() {
    count = 0;
    ++generation;
  }

  /** @brief Appends one edge; generation advances on every change. */
  [[gnu::always_inline]] void push(uint64_t now) {
    time[0] = time[1];
    time[1] = time[2];
    time[2] = time[3];
    time[3] = now;
    if (count < 4) ++count;
    ++generation;
  }

  /**
   * @brief True if actual is within BusLimits::edge_tolerance_us of
   * expected (us).
   */
  [[gnu::always_inline]] static bool within(uint64_t actual,
                                            uint32_t expected) {
    return actual >= expected - BusLimits::edge_tolerance_us &&
           actual <= expected + BusLimits::edge_tolerance_us;
  }

  /**
   * @brief Validates the captured 0xAA pattern and returns the SYN stamp.
   *
   * 0xAA is sent LSB first: start+bit0 stay low, so falling edges are
   * at 0, 3, 5, 7 bit times (1250, 833, 834 us between edges), not 0,2,4,6.
   * UART byte delivery only validates the captured pattern; it never supplies
   * an inferred start timestamp. No nearest-edge fallback is permitted.
   * Returns a stamp anchored at the oldest of the four edges, or an invalid
   * stamp if fewer edges, wrong spacing, or delivery age outside
   * [3900, t_z_max - guard] us.
   */
  [[gnu::always_inline]] Stamp stamp(uint64_t now) const {
    if (count != 4 || now < time[3] || now < time[0] || now - time[0] < 3900 ||
        now - time[0] > BusLimits::t_z_max_us - BusLimits::guard_us ||
        !within(time[1] - time[0], 1250) || !within(time[2] - time[1], 833) ||
        !within(time[3] - time[2], 834))
      return {};
    return {time[0], generation, true};
  }
};

/**
 * @brief True if window (us) lies in the validator band
 * [t_z_min + guard, t_z_max - guard] = [4332, 4424].
 */
[[gnu::always_inline]] inline bool windowValid(uint32_t window) {
  return window >= BusLimits::t_z_min_us + BusLimits::guard_us &&
         window <= BusLimits::t_z_max_us - BusLimits::guard_us;
}

/**
 * @brief Absolute fire time for a stamped SYN; false if it cannot be armed.
 *
 * result is stamp.start_us + window (us). Fails if the stamp is invalid,
 * now already passed the start, the window is out of band, or the remaining
 * lead time is below arm_margin_us.
 */
[[gnu::always_inline]] inline bool deadline(const Stamp& stamp, uint64_t now,
                                            uint32_t window, uint64_t& result) {
  if (!stamp.valid || now < stamp.start_us || !windowValid(window))
    return false;
  result = stamp.start_us + window;
  return now < result && result - now >= BusLimits::arm_margin_us;
}

/**
 * @brief Absolute time gate for firing at deadline_us; state vetoes
 * (withdrawn request, changed edge generation, busy bus) are the caller's
 * business — a quiet defer to the next SYN, not an anomaly.
 *
 * Requires a valid stamp, matching edge generation, idle TX, now at or
 * after the deadline, and now within [t_z_min + guard, t_z_max] us of the
 * SYN start.
 */
[[gnu::always_inline]] inline bool canFire(const Stamp& stamp,
                                           uint64_t deadline_us, uint64_t now,
                                           uint32_t generation, bool tx_idle) {
  return stamp.valid && tx_idle && generation == stamp.generation &&
         now >= deadline_us && now >= stamp.start_us &&
         now - stamp.start_us >= BusLimits::t_z_min_us + BusLimits::guard_us &&
         now - stamp.start_us <= BusLimits::t_z_max_us;
}

}  // namespace ebus::detail::syn
