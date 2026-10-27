#include "main.h"
#include "../../common/comms_endpoint.hpp"
#include "../../common/pros_transport.hpp"

constexpr std::uint8_t RADIO_PORT = 10;
constexpr char LINK_ID[] = "my_vexlink";

void initialize()
{
  pros::lcd::initialize();
  pros::lcd::set_text(0, "VEXLink | Worker");
  pros::lcd::set_text(1, "Waiting for link");
}

void disabled() {}
void competition_initialize() {}
void autonomous() {}

void opcontrol()
{
  static pros::Link link(RADIO_PORT, LINK_ID, pros::E_LINK_RX);
  vexlink::pros_transport::runCommsEndpoint(
    link, "Worker", vexlink::pros_transport::WORKER_TX_INTERVAL_MS);
}
