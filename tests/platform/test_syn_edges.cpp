/*
 * Copyright (C) 2026 Roland Jax
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <array>
#include <catch2/catch_all.hpp>
#include <cmath>
#include <cstdint>
#include <vector>

#include "platform/syn_edges.hpp"

using namespace ebus::detail;

namespace {

// Falling edges (us) of one UART frame: start bit, 8 data bits LSB first,
// stop bit. The line idles high.
std::vector<int64_t> fallingEdges(uint8_t byte, int64_t frame_start_us) {
  std::vector<int64_t> edges;
  bool previous_high = true;
  for (int bit = 0; bit < 10; ++bit) {
    const bool high = bit == 0 ? false : bit == 9 ? true : (byte >> (bit - 1)) & 1;
    if (previous_high && !high)
      edges.push_back(frame_start_us +
                      std::llround(bit * Physical::bit_time_us));
    previous_high = high;
  }
  return edges;
}

bool matches(const std::vector<int64_t>& e, size_t first = 0) {
  return platform::isSynEdgePattern(e[first], e[first + 1], e[first + 2],
                                    e[first + 3]);
}

}  // namespace

TEST_CASE("SYN edges: 0xAA has falling edges at 0, 3, 5 and 7 bit times",
          "[platform][syn_edges]") {
  const auto edges = fallingEdges(0xaa, 0);
  REQUIRE(edges.size() == 4);
  CHECK(edges[1] - edges[0] == 1250);
  CHECK(edges[2] - edges[1] == 833);
  CHECK(edges[3] - edges[2] == 834);
}

TEST_CASE("SYN edges: a real 0xAA frame is recognised", "[platform][syn_edges]") {
  SECTION("at the timer origin") {
    REQUIRE(matches(fallingEdges(0xaa, 0)));
  }
  SECTION("at an arbitrary timestamp") {
    REQUIRE(matches(fallingEdges(0xaa, 123456789)));
  }
  SECTION("with edge jitter inside the tolerance") {
    auto edges = fallingEdges(0xaa, 1000);
    edges[1] += 60;
    edges[2] -= 60;
    edges[3] += 40;
    REQUIRE(matches(edges));
  }
}

TEST_CASE("SYN edges: wrong spacing is rejected", "[platform][syn_edges]") {
  SECTION("equal 833 us gaps, the previously assumed pattern") {
    CHECK_FALSE(platform::isSynEdgePattern(0, 833, 1667, 2500));
  }
  SECTION("edges too far apart") {
    CHECK_FALSE(platform::isSynEdgePattern(0, 2500, 3333, 4167));
  }
  SECTION("jitter beyond the tolerance") {
    auto edges = fallingEdges(0xaa, 0);
    edges[1] += platform::syn_edge_tolerance_us;
    CHECK_FALSE(matches(edges));
  }
}

TEST_CASE("SYN edges: 0xAA is recognised after any preceding byte",
          "[platform][syn_edges]") {
  for (int previous = 0; previous < 256; ++previous) {
    auto edges = fallingEdges(static_cast<uint8_t>(previous), 0);
    const auto syn = fallingEdges(0xaa, std::llround(10 * Physical::bit_time_us));
    edges.insert(edges.end(), syn.begin(), syn.end());
    INFO("previous=" << previous);
    // The detector reads the newest four edges once the SYN byte arrived.
    REQUIRE(matches(edges, edges.size() - 4));
  }
}

TEST_CASE("SYN edges: the pattern is shared only with bytes of equal edge shape",
          "[platform][syn_edges]") {
  // Bit 0 and bit 7 add no falling edge, so 0x2A, 0x2B, 0xAA and 0xAB look the
  // same. The detector is therefore only valid once the byte is known to be
  // 0xAA, which the caller checks first.
  for (int byte = 0; byte < 256; ++byte) {
    const auto edges = fallingEdges(static_cast<uint8_t>(byte), 0);
    const bool same_shape = (byte & 0x7e) == 0x2a;
    INFO("byte=" << byte);
    CHECK((edges.size() == 4 && matches(edges)) == same_shape);
  }
}