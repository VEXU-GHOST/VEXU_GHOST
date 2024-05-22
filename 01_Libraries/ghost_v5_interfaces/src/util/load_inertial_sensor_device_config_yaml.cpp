#include <ghost_util/yaml_utils.hpp>
#include <ghost_v5_interfaces/util/load_inertial_sensor_device_config_yaml.hpp>

using ghost_util::loadYAMLParam;
using namespace ghost_v5_interfaces::devices;
namespace ghost_v5_interfaces
{

namespace util
{

InertialSensorDeviceData::SerialConfig loadInertialSensorSerialConfigFromYAML(
  YAML::Node node,
  bool verbose)
{
  InertialSensorDeviceData::SerialConfig config;
  loadYAMLParam(node, "send_accel_data", config.send_accel_data, verbose);
  loadYAMLParam(node, "send_gyro_data", config.send_gyro_data, verbose);
  loadYAMLParam(node, "send_heading_data", config.send_heading_data, verbose);
  return config;
}

void loadInertialSensorDeviceConfigFromYAML(
  YAML::Node node,
  std::string sensor_name,
  std::shared_ptr<InertialSensorDeviceConfig> sensor_device_config_ptr,
  bool verbose)
{
  // Get device base config
  auto device_node = node["devices"][sensor_name];
  if (!device_node) {
    throw std::runtime_error(
            "[loadInertialSensorDeviceConfigFromYAML] Error: Inertial Sensor " + sensor_name +
            " not found!");
  }

  // Get name of sensor-specific config
  if (!device_node["config"]) {
    throw std::runtime_error(
            "[loadInertialSensorDeviceConfigFromYAML] Error: Inertial Sensor Config not found for sensor " + sensor_name +
            "!");
  }
  auto config_name = device_node["config"].as<std::string>();

  // Retrieve sensor-specific config node
  auto config_node = node["device_configurations"][config_name];
  if (!config_node) {
    throw std::runtime_error(
            "[loadInertialSensorDeviceConfigFromYAML] Error: Config not found for name " + config_name +
            "!");
  }

  if (config_node["serial"]) {
    sensor_device_config_ptr->serial_config =
      loadInertialSensorSerialConfigFromYAML(config_node["serial"], verbose);
  }

  // Set base attributes
  sensor_device_config_ptr->name = sensor_name;
  sensor_device_config_ptr->type = device_type_e::INERTIAL_SENSOR;

  if (device_node["type"].as<std::string>() != "INERTIAL_SENSOR") {
    throw std::runtime_error(
            "[loadInertialSensorDeviceConfigFromYAML] Error: Device type is incorrect for Inertial Sensor " + sensor_name +
            "!");
  }

  if (!loadYAMLParam(device_node, "port", sensor_device_config_ptr->port, verbose)) {
    throw std::runtime_error(
            "[loadInertialSensorDeviceConfigFromYAML] Error: Port not specified for Inertial Sensor " + sensor_name +
            "!");
  }
}

} // namespace util

} // namespace ghost_v5_interfaces
