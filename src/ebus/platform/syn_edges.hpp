/*
 * Copyright (C) 2026 Roland Jax
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#pragma once

#include <cstdint>
#include <cstdlib>
#include <ebus/detail/protocol_limits.hpp>

namespace ebus::detail::platform {

// Per-gap tolerance in us. The 3- and 2-bit gaps differ by 417 us, so this must
// stay below that to tell them apart; it leaves room for the 2400 baud +/-1.2 %
// tolerance (spec 2) and ISR timestamp jitter.
inline constexpr int64_t syn_edge_tolerance_us = 200;

// Detects a SYN (0xAA) from the four newest falling-edge timestamps (oldest
// first, in us).
//
// Why: arbitration timing is counted from the falling edge of the SYN's start
// bit (tZmin = 4300 us, spec 10.8 and 6.3). The UART delivers the byte only
// after its stop bit, ~4167 us later plus task latency, so the start edge has to
// be recovered from the edge timestamps. This pattern identifies it.
//
// Frame (spec 3): start bit (low), D0..D7 with D0 sent FIRST (LSB first), stop
// bit (high). Spec 5.1 gives SYN as AAH = 10101010 (D7..D0), so on the wire D0
// is 0 and the line stays low through the start bit and D0:
//
//   bit time:  0  1  2  3  4  5  6  7  8  9
//   level:     0  0  1  0  1  0  1  0  1  1    start, D0..D7, stop
//   falling:   ^        ^     ^     ^
//
// Edges at 0, 3, 5, 7 bit times: gaps of 3, 2, 2 bit times = 1250, 833, 834 us
// (417 us per bit). Equal 2-bit gaps are the pattern of 0x55, not SYN.
//
// Only valid once the received byte is known to be 0xAA: 0x2A, 0x2B and 0xAB
// produce the same four edges.
inline bool isSynEdgePattern(int64_t e0, int64_t e1, int64_t e2, int64_t e3) {
  constexpr int64_t two_bits = static_cast<int64_t>(2.0f * Physical::bit_time_us);
  constexpr int64_t three_bits =
      static_cast<int64_t>(3.0f * Physical::bit_time_us);
  return std::llabs((e1 - e0) - three_bits) < syn_edge_tolerance_us &&
         std::llabs((e2 - e1) - two_bits) < syn_edge_tolerance_us &&
         std::llabs((e3 - e2) - two_bits) < syn_edge_tolerance_us;
}

}  // namespace ebus::detail::platform
