#include "comms_endpoint.hpp"

#include <cstdio>
#include <unistd.h>

#include "api.h"
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"
#include "pros_transport.hpp"

namespace vexlink::pros_transport
{
namespace
{

constexpr std::uint32_t UI_INTERVAL_MS = 100;

void updateDisplay(
  const char * role_name, bool linked, const PayloadQueue & queue,
  std::uint32_t tx_count, std::uint32_t rx_count,
  std::uint32_t dropped_count, std::uint32_t error_count)
{
  char line[32];
  pros::lcd::set_text(1, linked ? "Radio linked" : "No radio link");
  std::snprintf(
    line, sizeof(line), "%s TX%u RX%u Q%u", role_name,
    static_cast<unsigned int>(tx_count),
    static_cast<unsigned int>(rx_count),
    static_cast<unsigned int>(queue.size()));
  pros::lcd::set_text(2, line);
  std::snprintf(
    line, sizeof(line), "Drop%u Err%u",
    static_cast<unsigned int>(dropped_count),
    static_cast<unsigned int>(error_count));
  pros::lcd::set_text(3, line);
}

} // namespace

void runCommsEndpoint(
  pros::Link & link, const char * role_name,
  std::uint32_t radio_tx_interval_ms,
  PeriodicCallback periodic_callback,
  std::uint32_t callback_interval_ms)
{
  if (radio_tx_interval_ms == 0 ||
    (periodic_callback != nullptr && callback_interval_ms == 0))
  {
    pros::lcd::set_text(1, "Invalid comms config");
    return;
  }

  const int input_fd = fileno(stdin);
  HostSerialParser serial_parser;
  PayloadQueue outbound_queue;
  std::uint32_t tx_count = 0;
  std::uint32_t rx_count = 0;
  std::uint32_t dropped_count = 0;
  std::uint32_t error_count = 0;

  std::uint32_t now = pros::millis();
  std::uint32_t last_radio_tx = now;
  std::uint32_t last_ui_update = now;
  std::uint32_t last_callback = now;

  while (true) {
    const HostReadResult host_result =
      readHostSerial(input_fd, serial_parser, outbound_queue);
    dropped_count += host_result.frames_dropped;
    error_count += host_result.frames_rejected + (host_result.io_error ? 1U : 0U);

    const bool linked = link.connected();
    if (linked) {
      while (true) {
        Payload payload{};
        const RadioReceiveResult receive_result = receiveRadioPayload(link, payload);
        if (receive_result == RadioReceiveResult::EMPTY) {
          break;
        }
        if (receive_result == RadioReceiveResult::IO_ERROR) {
          ++error_count;
          break;
        }
        if (receive_result == RadioReceiveResult::INVALID) {
          ++error_count;
          continue;
        }

        if (writePayloadToHost(payload)) {
          ++rx_count;
        } else {
          ++error_count;
        }
      }
    }

    now = pros::millis();
    if (intervalElapsed(now, last_radio_tx, radio_tx_interval_ms) && linked) {
      Payload payload{};
      if (outbound_queue.front(payload)) {
        if (sendRadioPayload(link, payload)) {
          outbound_queue.discardFront();
          ++tx_count;
        } else {
          ++error_count;
        }
      }
    }

    if (periodic_callback != nullptr &&
      intervalElapsed(now, last_callback, callback_interval_ms) &&
      !periodic_callback())
    {
      ++error_count;
    }

    if (intervalElapsed(now, last_ui_update, UI_INTERVAL_MS)) {
      updateDisplay(
        role_name, linked, outbound_queue, tx_count, rx_count,
        dropped_count, error_count);
    }

    pros::delay(LOOP_INTERVAL_MS);
  }
}

} // namespace vexlink::pros_transport
