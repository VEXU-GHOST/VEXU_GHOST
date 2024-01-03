#include <ghost_util/yaml_utils.hpp>
#include <ghost_v5_interfaces/devices/joystick_device_interface.hpp>
#include <ghost_v5_interfaces/devices/motor_device_interface.hpp>
#include <ghost_v5_interfaces/devices/rotation_sensor_device_interface.hpp>
#include <ghost_v5_interfaces/util/device_config_factory_utils.hpp>
#include <ghost_v5_interfaces/util/load_motor_device_config_yaml.hpp>
#include <ghost_v5_interfaces/util/load_rotation_sensor_device_config_yaml.hpp>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <unordered_map>

using ghost_util::loadYAMLParam;
using namespace ghost_v5_interfaces::devices;

namespace ghost_v5_interfaces {

namespace util {

std::shared_ptr<DeviceConfigMap> loadRobotConfigFromYAML(YAML::Node node, bool verbose){
	auto device_config_map_ptr = std::make_shared<DeviceConfigMap>();
	bool use_partner_joystick = false;
	loadYAMLParam(node["port_configuration"], "use_partner_joystick", use_partner_joystick, false);

	// Load primary joystick
	auto joy_master = std::make_shared<JoystickDeviceConfig>();
	joy_master->name = MAIN_JOYSTICK_NAME;
	joy_master->port = -1;
	joy_master->is_partner = false;
	joy_master->type = device_type_e::JOYSTICK;
	device_config_map_ptr->addDeviceConfig(joy_master);

	if(use_partner_joystick){
		auto joy_partner = std::make_shared<JoystickDeviceConfig>();
		joy_partner->name = PARTNER_JOYSTICK_NAME;
		joy_partner->port = -2;
		joy_partner->type = device_type_e::JOYSTICK;
		joy_partner->is_partner = true;
		device_config_map_ptr->addDeviceConfig(joy_partner);
	}

	// Iterate through each device defined in the YAML file
	for(auto it = node["port_configuration"]["devices"].begin(); it != node["port_configuration"]["devices"].end(); it++){
		// Unpack Device Name and associated YAML Node
		std::string device_name = it->first.as<std::string>();
		YAML::Node device_yaml_node = it->second;

		// Load device type and device_config (if it exists)
		std::string device_type;
		loadYAMLParam(device_yaml_node, "type", device_type, verbose);

		std::shared_ptr<DeviceConfig> device_config_base_ptr;
		// Custom initialization based on device type
		switch(STRING_TO_DEVICE_TYPE_MAP.at(device_type)){
			case device_type_e::MOTOR:
			{
				auto motor_config_ptr = std::make_shared<MotorDeviceConfig>();
				// Load motor config (or default if not specified)
				std::string device_config_name;
				if(loadYAMLParam(device_yaml_node, "config", device_config_name, verbose)){
					loadMotorDeviceConfigFromYAML(node["port_configuration"], device_name, motor_config_ptr);
				}
				device_config_base_ptr = motor_config_ptr;
			}
			break;

			case device_type_e::ROTATION_SENSOR:
			{
				// Load motor config (or default if not specified)
				auto rotation_sensor_config_ptr = std::make_shared<RotationSensorDeviceConfig>();
				loadRotationSensorDeviceConfigFromYAML(node["port_configuration"], device_name, rotation_sensor_config_ptr);
				device_config_base_ptr = rotation_sensor_config_ptr;
			}
			break;

			case device_type_e::INVALID:
			{
				throw std::runtime_error("[loadDeviceInterfaceMapFromYAML] Error: Device name " + device_name + " has invalid type.");
			}
			break;

			default:
			{
				throw std::runtime_error("[loadDeviceInterfaceMapFromYAML] Error: Device type " + device_type + " is not currently supported.");
			}
			break;
		}

		// Set device base attributes
		device_config_base_ptr->name = device_name;
		loadYAMLParam(device_yaml_node, "port", device_config_base_ptr->port, verbose);
		device_config_base_ptr->type = STRING_TO_DEVICE_TYPE_MAP.at(device_type);

		device_config_map_ptr->addDeviceConfig(device_config_base_ptr);
	}
	return device_config_map_ptr;
}

std::shared_ptr<devices::DeviceConfigMap> loadRobotConfigFromYAMLFile(std::string filepath, bool verbose){
	YAML::Node config_yaml;
	try{
		config_yaml = YAML::LoadFile(filepath);
	}
	catch(...){
		throw std::runtime_error("[loadRobotConfigFromYAMLFile] Error: YAML file " + filepath + " not found!");
	}
	return loadRobotConfigFromYAML(config_yaml, verbose);
}


const std::unordered_map<ghost_encoder_unit, std::string> MOTOR_ENCODER_UNIT_STRING_MAP{
	{ghost_encoder_unit::ENCODER_DEGREES,   "ghost_v5_interfaces::devices::ghost_encoder_unit::ENCODER_DEGREES"},
	{ghost_encoder_unit::ENCODER_ROTATIONS, "ghost_v5_interfaces::devices::ghost_encoder_unit::ENCODER_ROTATIONS"},
	{ghost_encoder_unit::ENCODER_COUNTS,    "ghost_v5_interfaces::devices::ghost_encoder_unit::ENCODER_COUNTS"},
	{ghost_encoder_unit::ENCODER_INVALID,   "ghost_v5_interfaces::devices::ghost_encoder_unit::ENCODER_INVALID"}
};

const std::unordered_map<ghost_gearset, std::string> MOTOR_GEARSET_STRING_MAP{
	{ghost_gearset::GEARSET_100, "ghost_v5_interfaces::devices::ghost_gearset::GEARSET_100"},
	{ghost_gearset::GEARSET_200, "ghost_v5_interfaces::devices::ghost_gearset::GEARSET_200"},
	{ghost_gearset::GEARSET_600, "ghost_v5_interfaces::devices::ghost_gearset::GEARSET_600"}
};

const std::unordered_map<ghost_brake_mode, std::string> MOTOR_BRAKE_MODE_STRING_MAP{
	{ghost_brake_mode::BRAKE_MODE_COAST,      "ghost_v5_interfaces::devices::ghost_brake_mode::BRAKE_MODE_COAST"},
	{ghost_brake_mode::BRAKE_MODE_BRAKE,      "ghost_v5_interfaces::devices::ghost_brake_mode::BRAKE_MODE_BRAKE"},
	{ghost_brake_mode::BRAKE_MODE_HOLD,       "ghost_v5_interfaces::devices::ghost_brake_mode::BRAKE_MODE_HOLD"},
	{ghost_brake_mode::BRAKE_MODE_INVALID,    "ghost_v5_interfaces::devices::ghost_brake_mode::BRAKE_MODE_INVALID"}
};

const std::unordered_map<bool, std::string> BOOL_STRING_MAP{
	{true, "true"},
	{false, "false"}
};

void generateCodeFromRobotConfig(std::shared_ptr<DeviceConfigMap> config_ptr, std::string output_filepath){
	// Check if file already exists and overwrites
	auto fs_path = std::filesystem::path(output_filepath);
	if(std::filesystem::exists(fs_path)){
		std::filesystem::remove(fs_path);
	}

	std::ofstream output_file;
	output_file.open(output_filepath);
	output_file << "/////////////////////////////////////////////////////////////////////\n";
	output_file << "/// This file was automatically generated by ghost_v5_interfaces. ///\n";
	output_file << "/// DO NOT MODIFY, IT WILL BE DELETED ON THE NEXT COMPILE.        ///\n";
	output_file << "/////////////////////////////////////////////////////////////////////\n\n";

	output_file << "#include <memory>\n";
	output_file << "#include \"ghost_v5_interfaces/devices/device_config_map.hpp\"\n";
	output_file << "#include \"ghost_v5_interfaces/devices/motor_device_interface.hpp\"\n";
	output_file << "#include \"ghost_v5_interfaces/devices/rotation_sensor_device_interface.hpp\"\n";
	output_file << "#include \"ghost_v5_interfaces/devices/joystick_device_interface.hpp\"\n";
	output_file << "\n";
	output_file << "// This is externed as raw C code so we can resolve the symbols in the shared object easily for unit testing.\n";
	output_file << "// It returns a raw pointer to a dynamically allocated object, so if you are poking around, please wrap in a smart pointer!\n";
	output_file << "extern \"C\" ghost_v5_interfaces::devices::DeviceConfigMap* getRobotConfig(void) {\n";
	output_file << "\tghost_v5_interfaces::devices::DeviceConfigMap* robot_config = new ghost_v5_interfaces::devices::DeviceConfigMap;\n";
	output_file << "\n";

	// Generate code from DeviceConfigMap
	for(const auto& [key, val] : *config_ptr){
		if(val->type == device_type_e::MOTOR){
			auto config_ptr = val->as<const MotorDeviceConfig>();
			std::string motor_name = config_ptr->name;

			output_file << "\tstd::shared_ptr<ghost_v5_interfaces::devices::MotorDeviceConfig> " + motor_name + " = std::make_shared<ghost_v5_interfaces::devices::MotorDeviceConfig>();\n";
			output_file << "\t" + motor_name + "->" + "port = " + std::to_string(config_ptr->port) + ";\n";
			output_file << "\t" + motor_name + "->" + "name = \"" + motor_name + "\";\n";
			output_file << "\t" + motor_name + "->" + "type = ghost_v5_interfaces::devices::device_type_e::MOTOR;\n";
			output_file << "\t" + motor_name + "->" + "reversed = " + BOOL_STRING_MAP.at(config_ptr->reversed) + ";\n";
			output_file << "\t" + motor_name + "->" + "encoder_units = " + MOTOR_ENCODER_UNIT_STRING_MAP.at(config_ptr->encoder_units) + ";\n";
			output_file << "\t" + motor_name + "->" + "gearset = " + MOTOR_GEARSET_STRING_MAP.at(config_ptr->gearset) + ";\n";
			output_file << "\t" + motor_name + "->" + "brake_mode = " + MOTOR_BRAKE_MODE_STRING_MAP.at(config_ptr->brake_mode) + ";\n";
			output_file << "\t" + motor_name + "->" + "filter_config.cutoff_frequency = " + std::to_string(config_ptr->filter_config.cutoff_frequency) + ";\n";
			output_file << "\t" + motor_name + "->" + "filter_config.damping_ratio = " +    std::to_string(config_ptr->filter_config.damping_ratio) + ";\n";
			output_file << "\t" + motor_name + "->" + "filter_config.timestep = " +         std::to_string(config_ptr->filter_config.timestep) + ";\n";
			output_file << "\t" + motor_name + "->" + "model_config.free_speed = " +        std::to_string(config_ptr->model_config.free_speed) + ";\n";
			output_file << "\t" + motor_name + "->" + "model_config.stall_torque = " +      std::to_string(config_ptr->model_config.stall_torque) + ";\n";
			output_file << "\t" + motor_name + "->" + "model_config.free_current = " +      std::to_string(config_ptr->model_config.free_current) + ";\n";
			output_file << "\t" + motor_name + "->" + "model_config.stall_current = " +     std::to_string(config_ptr->model_config.stall_current) + ";\n";
			output_file << "\t" + motor_name + "->" + "model_config.nominal_voltage = " +   std::to_string(config_ptr->model_config.nominal_voltage) + ";\n";
			output_file << "\t" + motor_name + "->" + "model_config.gear_ratio = " +        std::to_string(config_ptr->model_config.gear_ratio) + ";\n";
			output_file << "\t" + motor_name + "->" + "controller_config.pos_gain = " +         std::to_string(config_ptr->controller_config.pos_gain) + ";\n";
			output_file << "\t" + motor_name + "->" + "controller_config.vel_gain = " +         std::to_string(config_ptr->controller_config.vel_gain) + ";\n";
			output_file << "\t" + motor_name + "->" + "controller_config.ff_vel_gain = " +      std::to_string(config_ptr->controller_config.ff_vel_gain) + ";\n";
			output_file << "\t" + motor_name + "->" + "controller_config.ff_torque_gain = " +   std::to_string(config_ptr->controller_config.ff_torque_gain) + ";\n";
			output_file << "\t" + motor_name + "->" + "serial_config.send_position_command = " +   std::to_string(config_ptr->serial_config.send_position_command) + ";\n";
			output_file << "\t" + motor_name + "->" + "serial_config.send_velocity_command = " +   std::to_string(config_ptr->serial_config.send_velocity_command) + ";\n";
			output_file << "\t" + motor_name + "->" + "serial_config.send_voltage_command = " +   std::to_string(config_ptr->serial_config.send_voltage_command) + ";\n";
			output_file << "\t" + motor_name + "->" + "serial_config.send_torque_command = " +   std::to_string(config_ptr->serial_config.send_torque_command) + ";\n";
			output_file << "\t" + motor_name + "->" + "serial_config.send_torque_data = " +   std::to_string(config_ptr->serial_config.send_torque_data) + ";\n";
			output_file << "\t" + motor_name + "->" + "serial_config.send_voltage_data = " +   std::to_string(config_ptr->serial_config.send_voltage_data) + ";\n";
			output_file << "\t" + motor_name + "->" + "serial_config.send_current_data = " +   std::to_string(config_ptr->serial_config.send_current_data) + ";\n";
			output_file << "\t" + motor_name + "->" + "serial_config.send_power_data = " +   std::to_string(config_ptr->serial_config.send_power_data) + ";\n";
			output_file << "\t" + motor_name + "->" + "serial_config.send_temp_data = " +   std::to_string(config_ptr->serial_config.send_temp_data) + ";\n";
			output_file << "\trobot_config->addDeviceConfig(" + motor_name + ");\n";
			output_file << "\n";
		}
		else if(val->type == device_type_e::ROTATION_SENSOR){
			auto config_ptr = val->as<const RotationSensorDeviceConfig>();
			std::string sensor_name = config_ptr->name;

			output_file << "\tstd::shared_ptr<ghost_v5_interfaces::devices::RotationSensorDeviceConfig> " + sensor_name + " = std::make_shared<ghost_v5_interfaces::devices::RotationSensorDeviceConfig>();\n";
			output_file << "\t" + sensor_name + "->" + "port = " + std::to_string(config_ptr->port) + ";\n";
			output_file << "\t" + sensor_name + "->" + "name = \"" + sensor_name + "\";\n";
			output_file << "\t" + sensor_name + "->" + "type = ghost_v5_interfaces::devices::device_type_e::ROTATION_SENSOR;\n";
			output_file << "\t" + sensor_name + "->" + "reversed = " + BOOL_STRING_MAP.at(config_ptr->reversed) + ";\n";
			output_file << "\t" + sensor_name + "->" + "data_rate = " + std::to_string(config_ptr->data_rate) + ";\n";
			output_file << "\t" + sensor_name + "->" + "serial_config.send_angle_data = " + std::to_string(config_ptr->serial_config.send_angle_data) + ";\n";
			output_file << "\t" + sensor_name + "->" + "serial_config.send_position_data = " + std::to_string(config_ptr->serial_config.send_position_data) + ";\n";
			output_file << "\t" + sensor_name + "->" + "serial_config.send_velocity_data = " + std::to_string(config_ptr->serial_config.send_velocity_data) + ";\n";
			output_file << "\trobot_config->addDeviceConfig(" + sensor_name + ");\n";
			output_file << "\n";
		}
		else if(val->type == device_type_e::JOYSTICK){
			auto config_ptr = val->as<const JoystickDeviceConfig>();
			std::string joy_name = config_ptr->name;

			output_file << "\tstd::shared_ptr<ghost_v5_interfaces::devices::JoystickDeviceConfig> " + joy_name + " = std::make_shared<ghost_v5_interfaces::devices::JoystickDeviceConfig>();\n";
			output_file << "\t" + joy_name + "->" + "port = " + std::to_string(config_ptr->port) + ";\n";
			output_file << "\t" + joy_name + "->" + "name = \"" + joy_name + "\";\n";
			output_file << "\t" + joy_name + "->" + "type = ghost_v5_interfaces::devices::device_type_e::JOYSTICK;\n";
			output_file << "\t" + joy_name + "->" + "is_partner = " + BOOL_STRING_MAP.at(config_ptr->is_partner) + ";\n";
			output_file << "\trobot_config->addDeviceConfig(" + joy_name + ");\n";
			output_file << "\n";
		}
		else if(val->type == device_type_e::INVALID){
			std::cout << "[WARNING] Device " + val->name + " has invalid device type. Skipping this entry.";
		}
		else{
			std::cout << "[WARNING] Device " + val->name + " has unsupported device type. Skipping.";
		}
	}

	output_file << "\treturn robot_config;\n";
	output_file << "}\n";

	output_file.close();
}

} // namespace util

} // namespace ghost_v5_interfaces