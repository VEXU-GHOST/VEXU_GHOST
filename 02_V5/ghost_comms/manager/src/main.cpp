#include "main.h"
#include "../../common/comms_endpoint.hpp"
#include "../../common/pros_transport.hpp"

constexpr std::uint8_t RADIO_PORT = 11;
constexpr std::uint8_t IMU_PORT = 1;
constexpr char LINK_ID[] = "my_vexlink";
constexpr std::uint32_t IMU_INTERVAL_MS = 20;

pros::IMU imu(IMU_PORT);

bool sendImuSerial()
{
  vexlink::ImuMsg message{};
  message.heading = static_cast<float>(imu.get_heading());

  const pros::euler_s_t euler = imu.get_euler();
  message.pitch = static_cast<float>(euler.pitch);
  message.roll = static_cast<float>(euler.roll);

  const pros::imu_gyro_s_t gyro = imu.get_gyro_rate();
  message.gyro_x = static_cast<float>(gyro.x);
  message.gyro_y = static_cast<float>(gyro.y);
  message.gyro_z = static_cast<float>(gyro.z);

  const pros::imu_accel_s_t accel = imu.get_accel();
  message.accel_x = static_cast<float>(accel.x);
  message.accel_y = static_cast<float>(accel.y);
  message.accel_z = static_cast<float>(accel.z);

  std::uint8_t buffer[vexlink::SERIAL_MAX_PACKET_SIZE];
  const std::size_t packet_size = vexlink::encode_serial(
    buffer, sizeof(buffer), vexlink::MsgType::IMU_DATA, message);
  return vexlink::pros_transport::writeSerialFrame(
    buffer, sizeof(buffer), packet_size);
}

void initialize()
{
  pros::lcd::initialize();
  pros::lcd::set_text(0, "VEXLink | Manager");
  pros::lcd::set_text(1, "IMU calibrating");

  imu.reset();
  while (imu.is_calibrating()) {
    pros::delay(50);
  }
  pros::lcd::set_text(1, "IMU ready");
}

void disabled() {}
void competition_initialize() {}
void autonomous() {}

void opcontrol()
{
  static pros::Link link(RADIO_PORT, LINK_ID, pros::E_LINK_TX);
  vexlink::pros_transport::runCommsEndpoint(
    link, "Manager", vexlink::pros_transport::MANAGER_TX_INTERVAL_MS,
    sendImuSerial, IMU_INTERVAL_MS);
}
