#include <ghost_util/yaml_utils.hpp>
#include <ghost_v5_interfaces/rotation_sensor/load_rotation_sensor_device_config_yaml.hpp>

using ghost_util::loadYAMLParam;

namespace ghost_v5_interfaces {

namespace util {

void loadRotationSensorDeviceConfigFromYAML(YAML::Node node,
                                            std::string sensor_name,
                                            std::shared_ptr<RotationSensorDeviceConfig> sensor_device_config_ptr,
                                            bool verbose){
	// Get device base config
	auto device_node = node["devices"][sensor_name];
	if(!device_node){
		throw std::runtime_error("[loadRotationSensorDeviceConfigFromYAML] Error: Rotation Sensor " + sensor_name + " not found!");
	}

	// Set base attributes
	sensor_device_config_ptr->name = sensor_name;
	sensor_device_config_ptr->type = device_type_e::ROTATION_SENSOR;

	if(device_node["type"].as<std::string>() != "ROTATION_SENSOR"){
		throw std::runtime_error("[loadRotationSensorDeviceConfigFromYAML] Error: Device type is incorrect for Rotation Sensor " + sensor_name + "!");
	}

	if(!loadYAMLParam(device_node, "port", sensor_device_config_ptr->port, verbose)){
		throw std::runtime_error("[loadRotationSensorDeviceConfigFromYAML] Error: Port not specified for Rotation Sensor " + sensor_name + "!");
	}

	// Set Sensor specific attributes
	loadYAMLParam(device_node, "reversed", sensor_device_config_ptr->reversed, verbose);
	loadYAMLParam(device_node, "data_rate", sensor_device_config_ptr->data_rate, verbose);
}

} // namespace util

} // namespace ghost_v5_interfaces