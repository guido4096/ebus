/*
 * Copyright (C) 2025-2026 Roland Jax, Guido Jansen
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#if defined(ESP_PLATFORM) && !EBUS_SIMULATION
#include "platform/esp/bus_esp.hpp"

#include <esp_timer.h>
#include <hal/timer_ll.h>
#include <hal/uart_ll.h>

#include <cmath>
#include <ebus/protocol_math.hpp>

#include "core/bus_monitor.hpp"
#include "core/request.hpp"
#include "driver/gpio.h"
#include "esp_clk_tree.h"
#include "platform/system.hpp"
#include "utils/logger.hpp"

namespace ebus::detail::platform {

namespace {
// The IRAM ISRs must read the arbitration timer while the flash cache may be
// off (GPIO edges during NVS/flash operations), where gptimer_get_raw_count
// is unreachable. This is the same capture-then-read the driver performs,
// inlined to pure MMIO; identifyGpTimerUnit() resolves the opaque handle to
// (hw, timer_num) once at init.
[[gnu::always_inline]] inline uint64_t rawCountIr(timg_dev_t* hw,
                                                  uint32_t timer_num) {
  timer_ll_trigger_soft_capture(hw, timer_num);
  return timer_ll_get_counter_value(hw, timer_num);
}
}  // namespace

BusEsp::BusEsp(const BusConfig& config, const RuntimeConfig& runtime,
               Request* request, BusMonitor* bus_monitor)
    : config_(config),
      runtime_(runtime),
      request_(request),
      bus_monitor_(bus_monitor),
      uart_port_num_(static_cast<uart_port_t>(config.uart_port)),
      rx_pin_(config.rx_pin),
      tx_pin_(config.tx_pin) {
  // Initialize postponement timer
  esp_timer_create_args_t postpone_args = {};
  postpone_args.callback = &BusEsp::s_onSynPostpone;
  postpone_args.arg = this;
  postpone_args.dispatch_method = ESP_TIMER_TASK;
  postpone_args.name = "ebusSynPostpone";
  postpone_args.skip_unhandled_events = true;
  ESP_ERROR_CHECK(esp_timer_create(&postpone_args, &syn_postpone_timer_));

  configureUart();
  configureTimer();
  configureGpio();
}

BusEsp::~BusEsp() { stop(); }

void BusEsp::start() {
  if (running_.load()) return;

  if (!uart_event_queue_) {
    configureUart();
    configureTimer();
    configureGpio();
    setRuntimeConfig(runtime_);
  }
  if (!uart_event_queue_) {
    EBUS_LOG_ERROR(
        "Cannot start ebus_bus: UART driver not installed (queue null)");
    return;
  }

  running_.store(true);
  worker_ = std::make_unique<ServiceThread>(
      "ebus_bus", [this] { ebusUartEventRunner(); },
      OrchestrationLimits::bus_stack_size, OrchestrationLimits::bus_priority);
  worker_->start();
}

void BusEsp::stop() {
  running_.store(false);
  syn_running_.store(false);
  portENTER_CRITICAL(&timer_mux_);
  qq_timer_armed_ = false;
  recovering_ = true;
  edges_.clear();
  armed_syn_ = {};
  bus_request_flag_ = false;
  start_bit_flag_ = false;
  portEXIT_CRITICAL(&timer_mux_);

  // Quiesce the task and GPIO ISR before freeing their timer/UART resources.
  // start() can recreate resources for the same controller instance.
  if (gpio_registered_) {
    gpio_intr_disable(static_cast<gpio_num_t>(rx_pin_));
    gpio_isr_handler_remove(static_cast<gpio_num_t>(rx_pin_));
    gpio_registered_ = false;
  }
  if (gpio_service_owned_) {
    gpio_uninstall_isr_service();
    gpio_service_owned_ = false;
  }
  if (worker_) worker_->join();
  if (syn_worker_) syn_worker_->join();
  if (syn_postpone_timer_) {
    esp_timer_stop(syn_postpone_timer_);
    esp_timer_delete(syn_postpone_timer_);
    syn_postpone_timer_ = nullptr;
  }
  if (syn_gp_timer_) {
    gptimer_stop(syn_gp_timer_);
    gptimer_disable(syn_gp_timer_);
    gptimer_del_timer(syn_gp_timer_);
    syn_gp_timer_ = nullptr;
  }
  if (gp_timer_) {
    if (timer_started_) gptimer_stop(gp_timer_);
    gptimer_disable(gp_timer_);
    gptimer_del_timer(gp_timer_);
    gp_timer_ = nullptr;
  }
  if (uart_event_queue_) {
    uart_driver_delete(uart_port_num_);
    uart_event_queue_ = nullptr;
  }
}

void BusEsp::setWindow(const uint16_t window_us) {
  portENTER_CRITICAL(&timer_mux_);
  // Validate window against limits
  runtime_.bus.window_us = (window_us < BusLimits::window_min_us ||
                            window_us > BusLimits::window_max_us)
                               ? ebus::RuntimeConfig{}.bus.window_us
                               : window_us;
  portEXIT_CRITICAL(&timer_mux_);
}

void BusEsp::setRuntimeConfig(const RuntimeConfig& runtime) {
  bool was_enabled;
  uint64_t base_us = static_cast<uint64_t>(BusLimits::Syn::base_ms) * 1000;
  uint64_t unique_us =
      base_us +  //
      (static_cast<uint64_t>(runtime.address) *
       static_cast<uint64_t>(BusLimits::Syn::address_factor_ms) * 1000) +
      (static_cast<uint64_t>(BusLimits::Syn::tolerance_ms) * 1000);

  portENTER_CRITICAL(&timer_mux_);
  was_enabled = runtime_.bus.syn_gen;
  runtime_ = runtime;

  // Validate window inside the critical section
  if (runtime_.bus.window_us < BusLimits::window_min_us ||
      runtime_.bus.window_us > BusLimits::window_max_us)
    runtime_.bus.window_us = ebus::RuntimeConfig{}.bus.window_us;

  syn_base_us_ = base_us;
  syn_unique_us_ = unique_us;
  portEXIT_CRITICAL(&timer_mux_);

  syn_running_.store(runtime_.bus.syn_gen);
  if (runtime_.bus.syn_gen) {
    gptimer_alarm_config_t alarm_config = {};
    alarm_config.alarm_count = syn_unique_us_;
    alarm_config.reload_count = 0;
    alarm_config.flags.auto_reload_on_alarm = true;

    if (!was_enabled) {
      syn_running_.store(true);
      syn_active_ = false;
      gptimer_set_alarm_action(syn_gp_timer_, &alarm_config);
      gptimer_start(syn_gp_timer_);
    } else {
      // Restart timer to apply the new t_unique count immediately
      gptimer_stop(syn_gp_timer_);
      gptimer_set_raw_count(syn_gp_timer_, 0);
      gptimer_set_alarm_action(syn_gp_timer_, &alarm_config);
      gptimer_start(syn_gp_timer_);
    }
  } else if (was_enabled) {
    syn_running_.store(false);
    gptimer_stop(syn_gp_timer_);
  }
}

void BusEsp::writeByte(const uint8_t byte) {
  portENTER_CRITICAL(&timer_mux_);
  const bool blocked = qq_timer_armed_ || recovering_;
  portEXIT_CRITICAL(&timer_mux_);
  if (blocked || !running_.load()) return;
  lockAndInvoke(listeners_mutex_, getWriteListeners(), byte);

  if (bus_monitor_) bus_monitor_->transmit.markBegin();

  // One UART owner, no TX ring or blocking driver mutex in the
  // launch path. A protocol callback arriving behind RX backlog must not
  // transmit into a newer telegram. Recovery runs on the bus task.
  size_t buffered = 0;
  uart_get_buffered_data_len(uart_port_num_, &buffered);
  portENTER_CRITICAL(&timer_mux_);
  auto* hw = UART_LL_GET_HW(uart_port_num_);
  if (!qq_timer_armed_ && !recovering_ && buffered == 0 &&
      uxQueueMessagesWaiting(uart_event_queue_) == 0 &&
      uart_ll_get_rxfifo_len(hw) == 0 &&
      uart_ll_get_txfifo_len(hw) == UART_LL_FIFO_DEF_LEN &&
      esp_timer_get_time() - last_activity_micros_ <= 6000) {
    // RX may report an echo halfway through the previous stop bit. Allow
    // one verified successor in the empty FIFO; hardware finishes the stop
    // bit before launching it. Arbitration, in contrast, requires TX idle.
    uart_ll_write_txfifo(hw, &byte, 1);
    last_activity_micros_ = esp_timer_get_time();
  } else {
    start_bit_flag_ = true;
  }
  portEXIT_CRITICAL(&timer_mux_);

  if (bus_monitor_) bus_monitor_->transmit.markEnd();
}

void BusEsp::writeBytes(ByteView bytes) {
  // Hardware master transmission is echo-paced by Handler::stageMasterBulk.
  // Refuse an unexpected block rather than preload unverified bytes.
  if (bytes.size() == 1)
    writeByte(bytes[0]);
  else if (!bytes.empty()) {
    portENTER_CRITICAL(&timer_mux_);
    start_bit_flag_ = true;
    portEXIT_CRITICAL(&timer_mux_);
  }
}

uint32_t BusEsp::qqWriteAgeUs() const {
  int64_t written = 0;
  portENTER_CRITICAL(const_cast<portMUX_TYPE*>(&timer_mux_));
  written = last_qq_write_us_;
  portEXIT_CRITICAL(const_cast<portMUX_TYPE*>(&timer_mux_));
  if (written == 0) return UINT32_MAX;
  const int64_t age = esp_timer_get_time() - written;
  if (age < 0) return 0;
  return age > static_cast<int64_t>(UINT32_MAX) ? UINT32_MAX
                                                : static_cast<uint32_t>(age);
}

uint64_t BusEsp::lastActivityAgeUs() const {
  int64_t last = 0;
  portENTER_CRITICAL(const_cast<portMUX_TYPE*>(&timer_mux_));
  last = last_activity_micros_;
  portEXIT_CRITICAL(const_cast<portMUX_TYPE*>(&timer_mux_));
  if (last == 0) return 0;  // never observed: not idle, keep old behavior
  const int64_t age = esp_timer_get_time() - last;
  if (age < 0) return 0;
  return static_cast<uint64_t>(age);
}

void BusEsp::noteQqWrite() {
  portENTER_CRITICAL(&timer_mux_);
  last_qq_write_us_ = esp_timer_get_time();
  last_activity_micros_ = last_qq_write_us_;
  portEXIT_CRITICAL(&timer_mux_);
}

ServiceThread::Status BusEsp::getThreadStatus() const {
  if (worker_) {
    return worker_->status();
  }
  return ServiceThread::Status{"ebus_bus", -1, -1};
}

ServiceThread::Status BusEsp::getSynThreadStatus() const {
  if (syn_worker_) {
    return syn_worker_->status();
  }
  return ServiceThread::Status{};
}

ebus::BusStatus BusEsp::fetchStatus() const {
  auto map =
      [](const platform::ServiceThread::Status& s) -> ebus::ThreadStatus {
    return {s.name, s.task_stack_bytes, s.task_stack_free_bytes};
  };
  return {map(getThreadStatus()), map(getSynThreadStatus())};
}

void BusEsp::recordUtilization(uint8_t byte) {
  // 1 (start bit) + zero bits in data.
  if (bus_monitor_) bus_monitor_->recordLowBits(countZeroBits(byte) + 1);
}

void BusEsp::configureUart() {
  uart_config_t uart_config = {};
  uart_config.baud_rate = Physical::baud_rate;
  uart_config.data_bits = UART_DATA_8_BITS;
  uart_config.parity = UART_PARITY_DISABLE;
  uart_config.stop_bits = UART_STOP_BITS_1;
  uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  uart_config.rx_flow_ctrl_thresh = 0;
  uart_config.source_clk = UART_SCLK_APB;
  uart_config.flags.allow_pd = false;
  uart_config.flags.backup_before_sleep = false;

  // Install UART driver with synchronized event queue sizes
  uart_driver_delete(uart_port_num_);  // Ensure port is clean
  esp_err_t err = ESP_FAIL;
  int retry_count = BusLimits::platform::Esp::uart_install_retries;
  while (retry_count-- > 0) {
    err = uart_driver_install(
        uart_port_num_, BusLimits::platform::Esp::buffer_size,
        0,  // No asynchronous TX ring; never queue a telegram behind QQ.
        BusLimits::platform::Esp::uart_event_queue_size, &uart_event_queue_,
        ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL3);
    if (err == ESP_OK) break;

    EBUS_LOG_ERROR("uart_driver_install attempt failed: " +
                   std::string(esp_err_to_name(err)) + ". Retrying...");
    if (retry_count > 0)
      vTaskDelay(
          pdMS_TO_TICKS(BusLimits::platform::Esp::uart_install_retry_delay_ms));
  }

  // Do not continue with half-initialized timing hardware.
  ESP_ERROR_CHECK(err);
  ESP_ERROR_CHECK(uart_param_config(uart_port_num_, &uart_config));

  ESP_ERROR_CHECK(uart_set_pin(uart_port_num_, tx_pin_, rx_pin_,
                               UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
  ESP_ERROR_CHECK(uart_set_rx_full_threshold(uart_port_num_, 1));
  ESP_ERROR_CHECK(uart_set_rx_timeout(uart_port_num_, 1));
}

void BusEsp::configureGpio() {
  gpio_config_t gpio_conf = {};
  gpio_conf.pin_bit_mask = (1ULL << rx_pin_);
  gpio_conf.mode = GPIO_MODE_INPUT;
  gpio_conf.pull_up_en = GPIO_PULLUP_ENABLE;
  gpio_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  gpio_conf.intr_type = GPIO_INTR_NEGEDGE;

  esp_err_t err = gpio_config(&gpio_conf);
  ESP_ERROR_CHECK(err);

  // Install GPIO ISR service with flags for IRAM and high priority (level 3)
  err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL3);
  // An already installed GPIO service has unknown interrupt/cache flags;
  // do not silently inherit it for a timing-critical input.
  ESP_ERROR_CHECK(err);
  gpio_service_owned_ = true;

  // Register the ISR handler
  err = gpio_isr_handler_add(static_cast<gpio_num_t>(rx_pin_), &s_onFallingEdge,
                             this);
  ESP_ERROR_CHECK(err);
  gpio_registered_ = true;
}

void BusEsp::configureTimer() {
  gptimer_config_t gpt_config_arb = {};
  gpt_config_arb.clk_src = GPTIMER_CLK_SRC_DEFAULT;
  gpt_config_arb.direction = GPTIMER_COUNT_UP;
  gpt_config_arb.resolution_hz = BusLimits::platform::Esp::timer_resolution_hz;
  gpt_config_arb.intr_priority = BusLimits::platform::Esp::timer_intr_priority;
  gpt_config_arb.flags.intr_shared = false;
  gpt_config_arb.flags.allow_pd = false;

  ESP_ERROR_CHECK(gptimer_new_timer(&gpt_config_arb, &gp_timer_));

  gptimer_config_t gpt_config_syn = {};
  gpt_config_syn.clk_src = GPTIMER_CLK_SRC_DEFAULT;
  gpt_config_syn.direction = GPTIMER_COUNT_UP;
  gpt_config_syn.resolution_hz = BusLimits::platform::Esp::timer_resolution_hz;
  gpt_config_syn.intr_priority = BusLimits::platform::Esp::timer_intr_priority;
  gpt_config_syn.flags.intr_shared = false;
  gpt_config_syn.flags.allow_pd = false;

  ESP_ERROR_CHECK(gptimer_new_timer(&gpt_config_syn, &syn_gp_timer_));

  gptimer_event_callbacks_t arb_cbs = {};
  arb_cbs.on_alarm = s_onBusIsrTimer;
  ESP_ERROR_CHECK(gptimer_register_event_callbacks(gp_timer_, &arb_cbs, this));
  ESP_ERROR_CHECK(gptimer_enable(gp_timer_));
  ESP_ERROR_CHECK(gptimer_start(gp_timer_));
  timer_started_ = true;

  // Resolve the driver-allocated TIMG unit for the IRAM ISR reads (see
  // rawCountIr); runs here, before configureGpio registers the edge ISR.
  identifyGpTimerUnit();

  gptimer_event_callbacks_t syn_cbs = {};
  syn_cbs.on_alarm = s_onSynGenTimer;
  ESP_ERROR_CHECK(
      gptimer_register_event_callbacks(syn_gp_timer_, &syn_cbs, this));
  ESP_ERROR_CHECK(gptimer_enable(syn_gp_timer_));
}

void BusEsp::identifyGpTimerUnit() {
  // gptimer's handle is opaque, but the IRAM ISRs need the raw MMIO pair for
  // rawCountIr(). Resolve it here (task context, flash on, driver API legal):
  // match the driver's own count against every TIMG unit -- magnitude first
  // (only our just-started timer sits near the reference age), then over a
  // short delay by rate (the never-started SYN timer stays frozen).
  timg_dev_t* found_hw = nullptr;
  uint32_t found_num = 0;
  for (uint32_t group = 0; group < TIMG_LL_INST_NUM; ++group) {
    timg_dev_t* hw = TIMER_LL_GET_HW(group);
    for (uint32_t num = 0; num < TIMG_LL_GPTIMERS_PER_INST; ++num) {
      uint64_t ref0 = 0;
      ESP_ERROR_CHECK(gptimer_get_raw_count(gp_timer_, &ref0));
      const uint64_t before = rawCountIr(hw, num);
      if (before + 50000 < ref0 || before > ref0 + 50000) continue;
      vTaskDelay(pdMS_TO_TICKS(2));
      uint64_t ref1 = 0;
      ESP_ERROR_CHECK(gptimer_get_raw_count(gp_timer_, &ref1));
      const uint64_t after = rawCountIr(hw, num);
      const uint64_t cand = after - before;
      const uint64_t want = ref1 - ref0;
      // Ratio-based so it holds for any timer resolution and tick rate:
      // the running unit tracks the reference, a frozen one does not move.
      if (cand + want / 4 < want || cand > want + want / 4) continue;
      ESP_ERROR_CHECK(found_hw == nullptr ? ESP_OK : ESP_ERR_INVALID_STATE);
      found_hw = hw;
      found_num = num;
    }
  }
  ESP_ERROR_CHECK(found_hw != nullptr ? ESP_OK : ESP_ERR_NOT_FOUND);

  // Final cross-check: direct MMIO agrees with the driver on the found unit.
  uint64_t check = 0;
  ESP_ERROR_CHECK(gptimer_get_raw_count(gp_timer_, &check));
  const uint64_t direct = rawCountIr(found_hw, found_num);
  const int64_t drift =
      static_cast<int64_t>(direct) - static_cast<int64_t>(check);
  const int64_t limit =
      static_cast<int64_t>(BusLimits::platform::Esp::timer_resolution_hz / 10);
  ESP_ERROR_CHECK(drift < 0 || drift > limit ? ESP_ERR_INVALID_STATE : ESP_OK);

  gp_timer_hw_ = found_hw;
  gp_timer_num_ = found_num;
}

// static trampoline for FreeRTOS task
void BusEsp::s_ebusUartEventRunner(void* arg) {
  BusEsp* self = static_cast<BusEsp*>(arg);
  if (self) self->ebusUartEventRunner();
}

void BusEsp::recoverUart() {
  portENTER_CRITICAL(&timer_mux_);
  qq_timer_armed_ = false;
  armed_syn_ = {};
  edges_.clear();
  recovering_ = true;
  bus_request_flag_ = false;
  micros_delay_flag_ = false;
  micros_window_flag_ = false;
  start_bit_flag_ = false;
  portEXIT_CRITICAL(&timer_mux_);
  gptimer_set_alarm_action(gp_timer_, nullptr);
  uart_flush_input(uart_port_num_);
  xQueueReset(uart_event_queue_);
  request_->withdrawBusRequest();
  request_->startBit();
}

void BusEsp::ebusUartEventRunner() {
  uart_event_t uart_event;
  uint8_t data[SequenceLimits::default_capacity * 2];

  while (running_.load()) {
    if (!uart_event_queue_) {
      break;
    }

    if (xQueueReceive(
            uart_event_queue_, &uart_event,
            pdMS_TO_TICKS(BusLimits::platform::Esp::event_timeout_ms))) {
      if (!running_.load()) break;
      portENTER_CRITICAL(&timer_mux_);
      const bool failed_tx = start_bit_flag_;
      portEXIT_CRITICAL(&timer_mux_);
      if (failed_tx) {
        if (bus_monitor_) bus_monitor_->recordIsrStartBitError();
        recoverUart();
        continue;
      }
      if (uart_event.type == UART_FIFO_OVF ||
          uart_event.type == UART_BUFFER_FULL ||
          uart_event.type == UART_FRAME_ERR ||
          uart_event.type == UART_PARITY_ERR || uart_event.type == UART_BREAK) {
        recoverUart();
        continue;
      }
      if (uart_event.type == UART_DATA) {
        if (uart_event.size > sizeof(data)) {
          recoverUart();
          continue;
        }
        const int len =
            uart_read_bytes(uart_port_num_, data, uart_event.size, 0);
        if (len != static_cast<int>(uart_event.size)) {
          recoverUart();
          continue;
        }

        if (len > 1 && bus_monitor_) {
          bool has_syn = false;
          for (int j = 0; j < len; ++j) {
            if (data[j] == Symbols::syn) {
              has_syn = true;
              break;
            }
          }
          bus_monitor_->recordMultiByteEvent(has_syn);
        }

        // Use first byte's arrival time as cycle start (excludes idle wait)
        const auto loop_start = Clock::now();

        for (int i = 0; i < len; ++i) {
          const auto arrival_time = Clock::now();
          const uint8_t byte = data[i];

          bool suppress_syn_bus_event = false;
          bool notify_syn = false;
          bool recovering = false;
          portENTER_CRITICAL(&timer_mux_);
          notify_syn = syn_notification_pending_;
          syn_notification_pending_ = false;
          recovering = recovering_;
          portEXIT_CRITICAL(&timer_mux_);
          if (notify_syn) lockAndInvoke(listeners_mutex_, getSynListeners());

          // After a framing/overflow error, observe a clean SYN before
          // permitting protocol writes. Never use delivery time as a start.
          const uint64_t timer_now = rawCountIr(gp_timer_hw_, gp_timer_num_);
          portENTER_CRITICAL(&timer_mux_);
          const auto stamp =
              byte == Symbols::syn ? edges_.stamp(timer_now) : syn::Stamp{};
          if (recovering_ && stamp.valid) recovering_ = false;
          recovering = recovering_;
          portEXIT_CRITICAL(&timer_mux_);
          if (recovering) continue;

          if (byte == Symbols::syn && request_->busRequestPending()) {
            uint64_t deadline = 0;
            uint32_t backlog = uxQueueMessagesWaiting(uart_event_queue_);
            size_t rx_buffered = 0;
            uart_get_buffered_data_len(uart_port_num_, &rx_buffered);
            bool armed = false;
            portENTER_CRITICAL(&timer_mux_);
            if (!qq_timer_armed_ && stamp.valid &&
                stamp.generation != consumed_syn_generation_ && backlog == 0 &&
                len == 1 && rx_buffered == 0 &&
                syn::deadline(stamp, timer_now, runtime_.bus.window_us,
                              deadline)) {
              // Snapshot address and generation. ISR never follows a mutable
              // Request object or invokes application callbacks.
              qq_address_ = request_->busRequestAddress();
              qq_request_generation_ = request_->busRequestGeneration();
              armed_syn_ = stamp;
              qq_deadline_us_ = deadline;
              consumed_syn_generation_ = stamp.generation;
              qq_timer_armed_ = true;
              armed = true;
            }
            portEXIT_CRITICAL(&timer_mux_);
            if (bus_monitor_) {
              bus_monitor_->recordUartBacklog(backlog);
              if (!armed) bus_monitor_->recordTimerArmDenied();
            }
            if (armed) {
              gptimer_alarm_config_t alarm{};
              alarm.alarm_count = deadline;
              // Absolute count on a timer that is NEVER reset on SYN. Even
              // if setup is preempted, the ISR's final check rejects lateness.
              if (gptimer_set_alarm_action(gp_timer_, &alarm) != ESP_OK) {
                portENTER_CRITICAL(&timer_mux_);
                qq_timer_armed_ = false;
                portEXIT_CRITICAL(&timer_mux_);
              }
              portENTER_CRITICAL(&timer_mux_);
              micros_last_delay_ = deadline - timer_now;
              micros_delay_flag_ = true;
              portEXIT_CRITICAL(&timer_mux_);
            }
            if (request_->busRequestIsExternal()) suppress_syn_bus_event = true;
          } else {
            portENTER_CRITICAL(&timer_mux_);
            qq_timer_armed_ = false;
            portEXIT_CRITICAL(&timer_mux_);
            request_->withdrawBusRequest();
          }

          CriticalSection r_lock{&listener_mux_};
          lockAndInvoke(r_lock, getReadListeners(), byte);
          recordUtilization(byte);

          // capture ISR flags and timing atomically and clear globals
          BusEvent bus_event;
          bus_event.byte = byte;

          portENTER_CRITICAL(&timer_mux_);
          // An alarm can fire while this SYN event is still being handled.
          // Its completion belongs to the QQ echo, never to the SYN itself.
          bus_event.bus_request = byte != Symbols::syn && bus_request_flag_;
          bus_event.start_bit = start_bit_flag_;
          const bool has_delay = micros_delay_flag_;
          const bool has_window = micros_window_flag_;
          const int64_t last_delay = micros_last_delay_;
          const int64_t last_window = micros_last_window_;

          int64_t postpone_sample = 0;
          if (syn_postpone_delta_us_ > 0) {
            postpone_sample = syn_postpone_delta_us_;
            syn_postpone_delta_us_ = 0;
          }
          uint32_t postponed_count = 0;
          if (syn_postponed_count_ > 0) {
            postponed_count = syn_postponed_count_;
            syn_postponed_count_ = 0;
          }

          // Reset the global ISR flags after consumption
          if (byte != Symbols::syn) bus_request_flag_ = false;
          start_bit_flag_ = false;
          micros_delay_flag_ = false;
          micros_window_flag_ = false;
          portEXIT_CRITICAL(&timer_mux_);

          if (bus_monitor_) {
            if (has_delay)
              bus_monitor_->delay.addSample(static_cast<float>(last_delay));
            if (has_window)
              bus_monitor_->window.addSample(static_cast<float>(last_window));
            if (postpone_sample > 0) {
              bus_monitor_->syn_postpone.addSample(
                  static_cast<float>(postpone_sample));
            }
            if (postponed_count > 0) {
              bus_monitor_->recordIsrSynPostponed(postponed_count);
            }
          }

          bus_event.timestamp = arrival_time;

          CriticalSection ev_lock{&listener_mux_};
          if (!suppress_syn_bus_event) {
            lockAndInvoke(ev_lock, getBusEventListeners(), bus_event);
          }

          // Reset SYN Timer (Arbitration Logic)
          portENTER_CRITICAL(&timer_mux_);
          const bool syn_enabled = runtime_.bus.syn_gen;
          const uint64_t base_us = syn_base_us_;
          const uint64_t unique_us = syn_unique_us_;
          portEXIT_CRITICAL(&timer_mux_);

          if (syn_enabled) {
            uint64_t next_interval;
            portENTER_CRITICAL(&timer_mux_);
            if (syn_active_ && byte == Symbols::syn) {  // Check echo for 0xAA
              next_interval = base_us;
            } else {
              syn_active_ = false;
              next_interval = unique_us;
            }
            portEXIT_CRITICAL(&timer_mux_);

            // Update the hardware timer alarm
            gptimer_alarm_config_t alarm_config = {};
            alarm_config.alarm_count = next_interval;
            alarm_config.reload_count = 0;
            alarm_config.flags.auto_reload_on_alarm = true;
            gptimer_stop(syn_gp_timer_);  // Ensure thread-safe reconfiguration
            gptimer_set_raw_count(syn_gp_timer_, 0);  // Restart count
            gptimer_set_alarm_action(syn_gp_timer_, &alarm_config);
            gptimer_start(syn_gp_timer_);
          }
        }

        // Record active cycle duration (excludes queue wait time)
        auto loop_duration =
            std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() -
                                                                  loop_start)
                .count();
        if (bus_monitor_) {
          bus_monitor_->loop_cycle.addSample(
              static_cast<uint32_t>(loop_duration));
        }
      }
    }
  }
}

void BusEsp::s_onSynPostpone(void* arg) {
  BusEsp* inst = static_cast<BusEsp*>(arg);
  if (inst) inst->onSynGenTimer();
}

// static ISR trampoline -> instance method
void IRAM_ATTR BusEsp::s_onFallingEdge(void* arg) {
  BusEsp* inst = reinterpret_cast<BusEsp*>(arg);
  if (inst) inst->onFallingEdge();
}

void IRAM_ATTR BusEsp::onFallingEdge() {
  if (!gp_timer_hw_) return;
  const uint64_t now = rawCountIr(gp_timer_hw_, gp_timer_num_);
  portENTER_CRITICAL_ISR(&timer_mux_);
  last_activity_micros_ = esp_timer_get_time();
  edges_.push(now);
  portEXIT_CRITICAL_ISR(&timer_mux_);
}

// static ISR trampoline -> instance method
bool IRAM_ATTR BusEsp::s_onBusIsrTimer(gptimer_handle_t timer,
                                       const gptimer_alarm_event_data_t* edata,
                                       void* user_ctx) {
  BusEsp* inst = reinterpret_cast<BusEsp*>(user_ctx);
  return inst ? inst->onBusIsrTimer() : false;
}

bool IRAM_ATTR BusEsp::onBusIsrTimer() {
  portENTER_CRITICAL_ISR(&timer_mux_);
  if (!qq_timer_armed_) {
    portEXIT_CRITICAL_ISR(&timer_mux_);
    return false;
  }
  qq_timer_armed_ = false;
  const bool timer_ok = gp_timer_hw_ != nullptr;
  const uint64_t now = timer_ok ? rawCountIr(gp_timer_hw_, gp_timer_num_) : 0;
  auto* hw = UART_LL_GET_HW(uart_port_num_);
  // Check immediately before FIFO access, with interrupts masked. A
  // withdrawn request, intervening edge, occupied TX shifter, or unread RX
  // invalidates the armed intent: quiet defer, the next SYN re-arms it.
  // Only a broken timestamp or a hard window breach means the arbitration
  // stack itself misfired; that is consumed by the next UART event on the
  // bus task and triggers UART recovery.
  if (timer_ok && running_.load() && request_->busRequestPending() &&
      request_->busRequestGeneration() == qq_request_generation_ &&
      syn::canFire(armed_syn_, qq_deadline_us_, now, edges_.generation,
                   uart_ll_is_tx_idle(hw)) &&
      uart_ll_get_rxfifo_len(hw) == 0 &&
      uxQueueMessagesWaitingFromISR(uart_event_queue_) == 0) {
    // UART ISR may already have drained new input out of the hardware FIFO.
    // Its queued event must also veto TX while the bus task is still busy.
    uart_ll_write_txfifo(hw, &qq_address_, 1);
    last_qq_write_us_ = esp_timer_get_time();
    last_activity_micros_ = last_qq_write_us_;
    micros_last_window_ = now - armed_syn_.start_us;
    bus_request_flag_ = true;
    micros_window_flag_ = true;
  } else if (!timer_ok || !armed_syn_.valid ||
             now - armed_syn_.start_us <
                 BusLimits::t_z_min_us + BusLimits::guard_us ||
             now - armed_syn_.start_us > BusLimits::t_z_max_us) {
    start_bit_flag_ = true;
  }
  portEXIT_CRITICAL_ISR(&timer_mux_);
  return false;
}

bool IRAM_ATTR BusEsp::s_onSynGenTimer(gptimer_handle_t timer,
                                       const gptimer_alarm_event_data_t* edata,
                                       void* user_ctx) {
  BusEsp* inst = reinterpret_cast<BusEsp*>(user_ctx);
  return inst ? inst->onSynGenTimer() : false;
}

bool IRAM_ATTR BusEsp::onSynGenTimer() {
  if (!running_.load() || !syn_running_.load()) return false;
  int64_t now = esp_timer_get_time();

  portENTER_CRITICAL_ISR(&timer_mux_);
  int64_t last_activity = last_activity_micros_;

  // Carrier Sense: yield and postpone if bus was active within the last 5ms
  // (Duration of a byte at 2400 baud + safety margin)
  if (now - last_activity < (BusLimits::Syn::carrier_sense_ms * 1000)) {
    if (syn_intent_time_ == 0) syn_intent_time_ = now;
    syn_postponed_count_++;
    portEXIT_CRITICAL_ISR(&timer_mux_);

    // Schedule a re-check in 2ms using the ISR-safe esp_timer
    esp_timer_start_once(syn_postpone_timer_,
                         BusLimits::Syn::postpone_ms * 1000);
    return false;
  }

  if (syn_intent_time_ > 0) {
    syn_postpone_delta_us_ = now - syn_intent_time_;
    syn_intent_time_ = 0;
  }

  auto* hw = UART_LL_GET_HW(uart_port_num_);
  if (qq_timer_armed_ || !uart_ll_is_tx_idle(hw) ||
      uart_ll_get_rxfifo_len(hw) != 0 ||
      uxQueueMessagesWaitingFromISR(uart_event_queue_) != 0) {
    portEXIT_CRITICAL_ISR(&timer_mux_);
    return false;
  }
  syn_active_ = true;
  syn_notification_pending_ = true;
  last_activity_micros_ = now;
  uint8_t syn = Symbols::syn;
  uart_ll_write_txfifo(hw, &syn, 1);
  portEXIT_CRITICAL_ISR(&timer_mux_);
  return false;
}

}  // namespace ebus::detail::platform

#endif  // ESP_PLATFORM
