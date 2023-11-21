#include <ghost_util/yaml_utils.hpp>
#include <ghost_v5_core/devices/motor/load_motor_config_yaml.hpp>

namespace ghost_v5_core {

namespace util {

DCMotorModel::Config loadMotorModelConfigFromYAML(YAML::Node node, bool verbose){
	DCMotorModel::Config config;
	loadYAMLParam(node, "free_speed", config.free_speed, verbose);
	loadYAMLParam(node, "stall_torque", config.stall_torque, verbose);
	loadYAMLParam(node, "free_current", config.free_current, verbose);
	loadYAMLParam(node, "stall_current", config.stall_current, verbose);
	loadYAMLParam(node, "nominal_voltage", config.nominal_voltage, verbose);
	loadYAMLParam(node, "gear_ratio", config.gear_ratio, verbose);
	return config;
}

MotorController::Config loadMotorControllerConfigFromYAML(YAML::Node node, bool verbose){
	MotorController::Config config;
	loadYAMLParam(node, "pos_gain", config.pos_gain, verbose);
	loadYAMLParam(node, "vel_gain", config.vel_gain, verbose);
	loadYAMLParam(node, "ff_vel_gain", config.ff_vel_gain, verbose);
	loadYAMLParam(node, "ff_torque_gain", config.ff_torque_gain, verbose);
	return config;
}

SecondOrderLowPassFilter::Config loadLowPassFilterConfigFromYAML(YAML::Node node, bool verbose){
	SecondOrderLowPassFilter::Config config;
	loadYAMLParam(node, "cutoff_frequency", config.cutoff_frequency, verbose);
	loadYAMLParam(node, "damping_ratio", config.damping_ratio, verbose);
	loadYAMLParam(node, "timestep", config.timestep, verbose);
	return config;
}

bool loadEncoderUnitFromYAML(YAML::Node node, ghost_encoder_unit& encoder_unit_value){
	if(!node["encoder_units"]){
		return false;
	}

	auto encoder_unit = node["encoder_units"].as<std::string>();

	if(encoder_unit == "DEGREES"){
		encoder_unit_value = ghost_encoder_unit::ENCODER_DEGREES;
	}
	else if(encoder_unit == "ROTATIONS"){
		encoder_unit_value = ghost_encoder_unit::ENCODER_ROTATIONS;
	}
	else if(encoder_unit == "COUNTS"){
		encoder_unit_value = ghost_encoder_unit::ENCODER_COUNTS;
	}
	else{
		return false;
	}
	return true;
}

bool loadGearsetFromYAML(YAML::Node node, ghost_gearset& gearset_value){
	if(!node["gearset"]){
		return false;
	}

	auto gearset = node["gearset"].as<int>();

	if(gearset == 100){
		gearset_value = ghost_gearset::GEARSET_100;
	}
	else if(gearset == 200){
		gearset_value = ghost_gearset::GEARSET_200;
	}
	else if(gearset == 600){
		gearset_value = ghost_gearset::GEARSET_600;
	}
	else{
		return false;
	}

	return true;
}

bool loadBrakeModeFromYAML(YAML::Node node, ghost_brake_mode& brake_mode_value){
	if(!node["encoder_units"]){
		return false;
	}

	auto brake_mode = node["brake_mode"].as<std::string>();

	if(brake_mode == "COAST"){
		brake_mode_value = ghost_brake_mode::BRAKE_MODE_COAST;
	}
	else if(brake_mode == "BRAKE"){
		brake_mode_value = ghost_brake_mode::BRAKE_MODE_BRAKE;
	}
	else if(brake_mode == "HOLD"){
		brake_mode_value = ghost_brake_mode::BRAKE_MODE_HOLD;
	}
	else{
		return false;
	}

	return true;
}

void loadMotorDeviceConfigFromYAML(YAML::Node node,
                                   std::string motor_name,
                                   std::shared_ptr<MotorDeviceConfig> motor_device_config_ptr,
                                   bool verbose){
	// Unpack into device YAML and config YAML
	auto device_node = node["devices"][motor_name];
	auto config_node = node["device_configurations"][device_node["config"].as<std::string>()];

	if(!device_node){
		throw std::runtime_error("[loadMotorDeviceConfigFromYAML] Error: Motor " + motor_name + " not found!");
	}

	// Set base attributes
	motor_device_config_ptr->name = motor_name;
	motor_device_config_ptr->type = device_type_e::MOTOR;

	if(device_node["type"].as<std::string>() != "MOTOR"){
		throw std::runtime_error("[loadMotorDeviceConfigFromYAML] Error: Device type is incorrect for motor " + motor_name + "!");
	}

	if(!loadYAMLParam(device_node, "port", motor_device_config_ptr->port, false)){
		throw std::runtime_error("[loadMotorDeviceConfigFromYAML] Error: Port not specified for motor " + motor_name + "!");
	}

	// Set Motor specific attributes
	loadYAMLParam(device_node, "reversed", motor_device_config_ptr->reversed, false);

	if(config_node["model"]){
		motor_device_config_ptr->model_config = loadMotorModelConfigFromYAML(config_node["model"]);
	}

	if(config_node["controller"]){
		motor_device_config_ptr->controller_config = loadMotorControllerConfigFromYAML(config_node["controller"]);
	}

	if(config_node["filter"]){
		motor_device_config_ptr->filter_config = loadLowPassFilterConfigFromYAML(config_node["filter"]);
	}

	if(!loadEncoderUnitFromYAML(config_node, motor_device_config_ptr->encoder_units) && verbose){
		std::cout << "[loadV5MotorInterfaceConfig] Error: Failed to load Encoder Units." << std::endl;
	}

	if(!loadGearsetFromYAML(config_node, motor_device_config_ptr->gearset) && verbose){
		std::cout << "[loadV5MotorInterfaceConfig] Error: Failed to load Gearset." << std::endl;
	}

	if(!loadBrakeModeFromYAML(config_node, motor_device_config_ptr->brake_mode) && verbose){
		std::cout << "[loadV5MotorInterfaceConfig] Error: Failed to load Brake Mode." << std::endl;
	}
}

// DeviceInterfaceMap loadDeviceInterfaceMapFromYAML(YAML::Node node, bool verbose){
// 	DeviceInterfaceMap device_interface_map;
// 	// Iterate through each device defined in the YAML file
// 	for(auto it = node["devices"].begin(); it != node["devices"].end(); it++){
// 		// Unpack Device Name and associated YAML Node
// 		std::string device_name = it->first.as<std::string>();
// 		YAML::Node device_yaml_node = it->second;

// 		// Load device type and device_config (if it exists)
// 		std::string device_type, device_config_name;
// 		loadYAMLParam(device_yaml_node, "type", device_type, verbose);
// 		bool config_found = loadYAMLParam(device_yaml_node, "config", device_config_name, verbose);

// 		// Custom initialization based on device type
// 		switch(device_type_name_enum_map.at(device_type)){
// 			case device_type_e::MOTOR:
// 			{
// 				// Load motor config (or default if not specified)
// 				std::shared_ptr<V5MotorConfig> config_ptr;
// 				if(config_found){
// 					config_ptr = loadV5MotorConfigFromYAML(node["device_configurations"][device_config_name]);
// 				}
// 				else{
// 					config_ptr = std::make_shared<V5MotorConfig>();
// 				}

// 				device_interface_map[device_name] = std::make_shared<V5MotorInterface>(config_ptr);
// 				device_interface_map[device_name]->data = std::make_shared<DeviceData>();
// 			}
// 			break;

// 			default:
// 			{
// 				throw std::runtime_error("[loadDeviceInterfaceMapFromYAML] Error: Device type " + device_type + " is not currently supported.");
// 			}
// 			break;
// 		}

// 		// Set device base attributes
// 		device_interface_map[device_name]->name = device_name;
// 		device_interface_map[device_name]->port = device_yaml_node["port"].as<int>();
// 		device_interface_map[device_name]->type = device_type_name_enum_map.at(device_type);
// 	}
// 	return device_interface_map;
// }

} // namespace util

} // namespace ghost_v5_core