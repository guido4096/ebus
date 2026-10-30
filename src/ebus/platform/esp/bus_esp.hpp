/*
 * Copyright (C) 2025-2026 Roland Jax, Guido Jansen
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#pragma once

#if defined(ESP_PLATFORM) && !EBUS_SIMULATION
#include <freertos/FreeRTOS.h>

#include <cstdint>
#include <ebus/config.hpp>
#include <ebus/detail/protocol_limits.hpp>
#include <ebus/status.hpp>
#include <ebus/types.hpp>
#include <functional>
#include <memory>

#include "core/bus_events.hpp"
#include "core/syn_timing.hpp"
#include "driver/gptimer.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "platform/atomic.hpp"
#include "platform/bus_base.hpp"
#include "platform/service_thread.hpp"
#include "soc/timer_group_struct.h"

namespace ebus::detail {
class Request;
class BusMonitor;
}  // namespace ebus::detail

namespace ebus::detail::platform {

/**
 * ESP-specific implementation of the eBUS physical layer.
 * Handles serial port configuration and asynchronous byte reading via a
 * background thread.
 */
class BusEsp : public BusBase {
 public:
  // Public Types & Constants
  struct CriticalSection {
    portMUX_TYPE* mux;
    void lock() { portENTER_CRITICAL(mux); }
    void unlock() { portEXIT_CRITICAL(mux); }
  };

  struct CriticalSectionISR {
    portMUX_TYPE* mux;
    void lock() { portENTER_CRITICAL_ISR(mux); }
    void unlock() { portEXIT_CRITICAL_ISR(mux); }
  };

  // Lifecycle & Static Factories
  explicit BusEsp(const BusConfig& config, const RuntimeConfig& runtime,
                  detail::Request* request, detail::BusMonitor* bus_monitor);
  ~BusEsp();
  void start();
  void stop();

  // Special Members & Operators
  BusEsp(const BusEsp&) = delete;
  BusEsp& operator=(const BusEsp&) = delete;

  // Configuration
  void setWindow(const uint16_t window_us);
  void setRuntimeConfig(const RuntimeConfig& runtime);

  // Working Methods
  void writeByte(const uint8_t byte);
  // Pushes a whole block (e.g. a preloaded master part) into the TX FIFO
  // in one step: a single wakeup instead of one per byte. Echoes are still
  // verified per byte on the RX side; see Handler::stageMasterBulk.
  void writeBytes(ByteView bytes);
  // ISR QQ-write timestamp vs now (0 never wrote -> infinitely stale, so a
  // win without a QQ write can never preload).
  uint32_t qqWriteAgeUs() const override;
  // RX-edge/TX activity age (falling edges + our writes both reset it).
  uint64_t lastActivityAgeUs() const override;
  // Stamps an immediate (non-timer) QQ write for the stale-win guard.
  void noteQqWrite() override;

  // Status/Telemetry
  platform::ServiceThread::Status getThreadStatus() const;
  platform::ServiceThread::Status getSynThreadStatus() const;
  ebus::BusStatus fetchStatus() const;

 private:
  BusConfig config_;
  RuntimeConfig runtime_;

  detail::Request* request_ = nullptr;
  detail::BusMonitor* bus_monitor_ = nullptr;

  uart_port_t uart_port_num_;
  uint8_t rx_pin_;
  uint8_t tx_pin_;

  gptimer_handle_t gp_timer_ = nullptr;
  gptimer_handle_t syn_gp_timer_ = nullptr;
  // Driver-allocated TIMG unit behind gp_timer_ (see identifyGpTimerUnit):
  // the handle is opaque; the IRAM ISRs need the MMIO pair to read the raw
  // count while the flash cache is off (gptimer_get_raw_count is flash code).
  timg_dev_t* gp_timer_hw_ = nullptr;
  uint32_t gp_timer_num_ = 0;
  esp_timer_handle_t syn_postpone_timer_ = nullptr;

  std::unique_ptr<ServiceThread> syn_worker_;

  std::unique_ptr<ServiceThread> worker_;
  Atomic running_;

  // All edge timestamps and arbitration deadlines use the same continuously
  // running GPTimer clock. esp_timer timestamps are only for diagnostics/idle.
  syn::Edges edges_;
  syn::Stamp armed_syn_;
  uint64_t qq_deadline_us_ = 0;
  uint32_t consumed_syn_generation_ = 0;
  uint8_t qq_address_ = 0;
  uint32_t qq_request_generation_ = 0;
  bool syn_notification_pending_ = false;
  bool recovering_ = true;
  bool gpio_registered_ = false;
  bool gpio_service_owned_ = false;
  bool timer_started_ = false;

  bool bus_request_flag_ = false;
  bool start_bit_flag_ = false;
  bool micros_delay_flag_ = false;
  bool micros_window_flag_ = false;
  // QQ-timer single-shot guard (timer_mux_): exactly one QQ write per
  // arbitration. Set on SYN-arm, cleared on fire or when contender traffic
  // makes our entry obsolete (defer to the next SYN instead of jamming).
  bool qq_timer_armed_ = false;
  // Timestamp (esp_timer us) of the last QQ byte written. SYNs processed
  // within one byte time after our own QQ must not re-arm: they are our
  // abort-SYN echo or backlog-adjacent duplicates, and firing into them
  // doubles QQ into our own echo window (self-sustaining abort chain).
  int64_t last_qq_write_us_ = 0;

  int64_t micros_last_delay_ = 0;
  int64_t micros_last_window_ = 0;

  int64_t last_activity_micros_ = 0;
  int64_t syn_intent_time_ = 0;
  int64_t syn_postpone_delta_us_ = 0;
  uint32_t syn_postponed_count_ = 0;
  bool syn_active_{false};

  Atomic syn_running_;
  uint64_t syn_base_us_ = 0;    // Base SYN interval in microseconds
  uint64_t syn_unique_us_ = 0;  // Unique SYN interval in microseconds

  // platform handles
  QueueHandle_t uart_event_queue_ = nullptr;
  portMUX_TYPE timer_mux_ = portMUX_INITIALIZER_UNLOCKED;
  portMUX_TYPE listener_mux_ = portMUX_INITIALIZER_UNLOCKED;

  void recordUtilization(uint8_t byte);
  void recoverUart();

  // setup helpers
  void configureUart();
  void configureGpio();
  void configureTimer();
  // Resolves gp_timer_hw_/gp_timer_num_ after gptimer_start (task context).
  void identifyGpTimerUnit();

  // UART event task: reads UART events and notifies listeners
  static void s_ebusUartEventRunner(void* arg);
  void ebusUartEventRunner();  // instance worker used by static trampoline

  static void s_onSynPostpone(void* arg);

  // ISR: Save the falling edges in order to estimate the sync byte
  static void IRAM_ATTR s_onFallingEdge(void* arg);
  void onFallingEdge();

  // ISR: Write request byte at the exact time
  static bool IRAM_ATTR s_onBusIsrTimer(gptimer_handle_t timer,
                                        const gptimer_alarm_event_data_t* edata,
                                        void* user_ctx);
  static bool IRAM_ATTR s_onSynGenTimer(gptimer_handle_t timer,
                                        const gptimer_alarm_event_data_t* edata,
                                        void* user_ctx);

  bool onBusIsrTimer();
  bool onSynGenTimer();
};

}  // namespace ebus::detail::platform

#endif  // ESP_PLATFORM
