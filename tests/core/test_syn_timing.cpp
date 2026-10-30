/*
 * Copyright (C) 2026 Roland Jax
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include <catch2/catch_all.hpp>
#include <cmath>
#include <cstdint>
#include <ebus/detail/protocol_limits.hpp>
#include <vector>

#include "core/syn_timing.hpp"
#include "platform/atomic.hpp"

using namespace ebus::detail;

namespace {

// Falling edges (us) of one UART frame: start bit, 8 data bits LSB first,
// stop bit. The line idles high.
std::vector<int64_t> fallingEdges(uint8_t byte, int64_t frame_start_us) {
  std::vector<int64_t> edges;
  bool previous_high = true;
  for (int bit = 0; bit < 10; ++bit) {
    const bool high = bit == 0   ? false
                      : bit == 9 ? true
                                 : (byte >> (bit - 1)) & 1;
    if (previous_high && !high)
      edges.push_back(frame_start_us +
                      std::llround(bit * Physical::bit_time_us));
    previous_high = high;
  }
  return edges;
}

// Four newest edges pushed, delivery age fixed inside the accepted window.
syn::Edges edgesFixture(int64_t base_us = 0) {
  syn::Edges edges;
  for (const auto t : fallingEdges(0xaa, base_us))
    edges.push(static_cast<uint64_t>(t));
  return edges;
}

syn::Edges edgesOf(std::vector<int64_t> raw) {
  syn::Edges edges;
  for (const auto t : raw) edges.push(static_cast<uint64_t>(t));
  return edges;
}

constexpr int64_t delivery_age_us = 4000;  // inside [3900, t_z_max - guard]

}  // namespace

TEST_CASE("SYN edges: 0xAA has falling edges at 0, 3, 5 and 7 bit times",
          "[core][syn_timing]") {
  const auto edges = fallingEdges(0xaa, 0);
  REQUIRE(edges.size() == 4);
  CHECK(edges[1] - edges[0] == 1250);
  CHECK(edges[2] - edges[1] == 833);
  CHECK(edges[3] - edges[2] == 834);
}

TEST_CASE("Edges::stamp: a real 0xAA frame is recognised",
          "[core][syn_timing]") {
  SECTION("at the timer origin") {
    auto edges = edgesFixture();
    CHECK(edges.stamp(delivery_age_us).valid);
  }
  SECTION("at an arbitrary timestamp") {
    auto edges = edgesFixture(123456789);
    CHECK(edges.stamp(123456789 + delivery_age_us).valid);
  }
  SECTION("with edge jitter inside the tolerance") {
    auto raw = fallingEdges(0xaa, 1000);
    raw[1] += 60;
    raw[3] += 40;
    auto edges = edgesOf(raw);
    CHECK(edges.stamp(1000 + delivery_age_us).valid);
  }
  SECTION("with jitter exactly at the tolerance") {
    auto raw = fallingEdges(0xaa, 0);
    raw[1] += BusLimits::edge_tolerance_us;
    auto edges = edgesOf(raw);
    CHECK(edges.stamp(delivery_age_us).valid);
  }
}

TEST_CASE("Edges::stamp: wrong spacing is rejected", "[core][syn_timing]") {
  SECTION("equal 833 us gaps, the previously assumed pattern") {
    auto edges = edgesOf({0, 833, 1667, 2500});
    CHECK_FALSE(edges.stamp(delivery_age_us).valid);
  }
  SECTION("edges too far apart") {
    auto edges = edgesOf({0, 2500, 3333, 4167});
    CHECK_FALSE(edges.stamp(4167 + delivery_age_us).valid);
  }
  SECTION("jitter beyond the tolerance") {
    auto raw = fallingEdges(0xaa, 0);
    raw[1] += BusLimits::edge_tolerance_us + 1;
    auto edges = edgesOf(raw);
    CHECK_FALSE(edges.stamp(delivery_age_us).valid);
  }
}

TEST_CASE("Edges::stamp: delivery age window is enforced",
          "[core][syn_timing]") {
  auto edges = edgesFixture();
  CHECK(edges.stamp(3900).valid);
  CHECK_FALSE(edges.stamp(3899).valid);
  CHECK(edges.stamp(BusLimits::t_z_max_us - BusLimits::guard_us).valid);
  CHECK_FALSE(
      edges.stamp(BusLimits::t_z_max_us - BusLimits::guard_us + 1).valid);
}

TEST_CASE("Edges::stamp: 0xAA is recognised after any preceding byte",
          "[core][syn_timing]") {
  // The detector reads the newest four edges once the SYN byte arrived;
  // delivery age stays valid for the SYN frame start at 10 bit times.
  const int64_t syn_start_us = std::llround(10 * Physical::bit_time_us);
  for (int previous = 0; previous < 256; ++previous) {
    auto raw = fallingEdges(static_cast<uint8_t>(previous), 0);
    const auto syn = fallingEdges(0xaa, syn_start_us);
    raw.insert(raw.end(), syn.begin(), syn.end());
    auto edges = edgesOf(raw);
    INFO("previous=" << previous);
    CHECK(edges.stamp(syn_start_us + delivery_age_us).valid);
  }
}

TEST_CASE("Edges::stamp: shared only with bytes of equal edge shape",
          "[core][syn_timing]") {
  // Bit 0 and bit 7 add no falling edge, so 0x2A, 0x2B, 0xAA and 0xAB look
  // the same. The detector is therefore only valid once the byte is known
  // to be 0xAA, which the caller checks first.
  for (int byte = 0; byte < 256; ++byte) {
    auto edges = edgesOf(fallingEdges(static_cast<uint8_t>(byte), 0));
    const bool same_shape = (byte & 0x7e) == 0x2a;
    INFO("byte=" << byte);
    CHECK(edges.stamp(delivery_age_us).valid == same_shape);
  }
}

TEST_CASE("Edges: generation tracks every change", "[core][syn_timing]") {
  auto edges = edgesFixture();
  const uint32_t generation = edges.generation;
  CHECK(edges.stamp(delivery_age_us).generation == generation);

  edges.clear();
  CHECK(edges.generation == generation + 1);
  CHECK_FALSE(edges.stamp(delivery_age_us).valid);

  edges.push(100);
  CHECK(edges.generation == generation + 2);
}

TEST_CASE("windowValid: accepts exactly the validator band",
          "[core][syn_timing]") {
  CHECK_FALSE(syn::windowValid(4331));
  CHECK(syn::windowValid(4332));
  CHECK(syn::windowValid(4350));
  CHECK(syn::windowValid(4424));
  CHECK_FALSE(syn::windowValid(4425));
}

TEST_CASE("deadline: arming arithmetic", "[core][syn_timing]") {
  const syn::Stamp stamp{1000, 7, true};

  SECTION("valid arm inside the window") {
    uint64_t deadline = 0;
    REQUIRE(syn::deadline(stamp, 1500, 4350, deadline));
    CHECK(deadline == 5350);
  }
  SECTION("arm margin: rejected below 15 us lead time") {
    uint64_t deadline = 0;
    CHECK_FALSE(syn::deadline(stamp, 5340, 4350, deadline));
  }
  SECTION("already past the deadline") {
    uint64_t deadline = 0;
    CHECK_FALSE(syn::deadline(stamp, 5350, 4350, deadline));
  }
  SECTION("invalid stamp or window") {
    uint64_t deadline = 0;
    CHECK_FALSE(syn::deadline({}, 1500, 4350, deadline));
    CHECK_FALSE(syn::deadline(stamp, 1500, 4425, deadline));
    CHECK_FALSE(syn::deadline(stamp, 999, 4350, deadline));
  }
}

TEST_CASE("canFire: absolute time constraints", "[core][syn_timing]") {
  const syn::Stamp stamp{1000, 7, true};
  const uint64_t deadline = 1000 + 4350;

  SECTION("fires at the deadline") {
    CHECK(syn::canFire(stamp, deadline, 1000 + 4350, 7, true));
  }
  SECTION("never before the deadline") {
    CHECK_FALSE(syn::canFire(stamp, deadline, 1000 + 4349, 7, true));
  }
  SECTION("window lower bound") {
    const uint64_t early_deadline = 1000 + 4300;
    CHECK(syn::canFire(stamp, early_deadline,
                       1000 + BusLimits::t_z_min_us + BusLimits::guard_us, 7,
                       true));
    CHECK_FALSE(syn::canFire(
        stamp, early_deadline,
        1000 + BusLimits::t_z_min_us + BusLimits::guard_us - 1, 7, true));
  }
  SECTION("regression: full tZmax is accepted, one us later is not") {
    CHECK(syn::canFire(stamp, deadline, 1000 + BusLimits::t_z_max_us, 7, true));
    CHECK_FALSE(syn::canFire(stamp, deadline, 1000 + BusLimits::t_z_max_us + 1,
                             7, true));
  }
  SECTION("state vetoes are the caller's business") {
    CHECK_FALSE(syn::canFire(stamp, deadline, 1000 + 4350, 8, true));
    CHECK_FALSE(syn::canFire(stamp, deadline, 1000 + 4350, 7, false));
    CHECK_FALSE(syn::canFire({}, deadline, 1000 + 4350, 7, true));
  }
}

TEST_CASE("Atomic: ISR-safe load/store", "[core][syn_timing]") {
  platform::Atomic word;
  CHECK(word.load() == 0);
  word.store(42);
  CHECK(word.load() == 42);
  CHECK(word.fetch_add(8) == 42);
  CHECK(word.load() == 50);
}
