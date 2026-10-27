#pragma once

#include <cstdint>

#include "pros/link.hpp"

namespace vexlink::pros_transport
{

using PeriodicCallback = bool (*)();

void runCommsEndpoint(
  pros::Link & link, const char * role_name,
  std::uint32_t radio_tx_interval_ms,
  PeriodicCallback periodic_callback = nullptr,
  std::uint32_t callback_interval_ms = 0);

} // namespace vexlink::pros_transport
