#include <ghost_v5_interfaces/devices/device_config_map.hpp>
#include <ghost_v5_interfaces/devices/device_interfaces.hpp>
#include <gtest/gtest.h>

using namespace ghost_v5_interfaces::devices;

/**
 * @brief DeviceConfig is a pure virtual class, so we need a derived class to test DeviceConfigMap functionality.
 */
class TestDeviceConfig : public DeviceConfig {
public:
	int a = 2;
	bool b = false;

	std::shared_ptr<DeviceBase> clone() const override {
		return std::make_shared<TestDeviceConfig>(*this);
	}

	bool operator==(const DeviceBase &rhs) const override {
		const TestDeviceConfig *m_rhs = dynamic_cast<const TestDeviceConfig *>(&rhs);
		if(m_rhs){
			return (port == m_rhs->port) && (name == m_rhs->name) && (type == m_rhs->type) &&
			       (a == m_rhs->a) && (b == m_rhs->b);
		}
		else{
			// Failed to cast base class, thus can't be equal.
			return false;
		}
	}
};

/**
 * @brief DeviceConfig is a pure virtual class, so we need a derived class to test DeviceConfigMap functionality.
 */
class TestDeviceConfig2 : public DeviceConfig {
public:
	bool c = false;

	std::shared_ptr<DeviceBase> clone() const override {
		return std::make_shared<TestDeviceConfig2>(*this);
	}

	bool operator==(const DeviceBase &rhs) const override {
		const TestDeviceConfig2 *m_rhs = dynamic_cast<const TestDeviceConfig2 *>(&rhs);
		if(m_rhs){
			return (port == m_rhs->port) && (name == m_rhs->name) && (type == m_rhs->type) && (c == m_rhs->c);
		}
		else{
			// Failed to cast base class, thus can't be equal.
			return false;
		}
	}
};

/**
 * @brief Test equality operator
 */
TEST(TestDeviceConfigInterfaces, testDeviceEqualityOperator){
	std::shared_ptr<DeviceConfig> device_1 = std::make_shared<TestDeviceConfig>();
	std::shared_ptr<DeviceConfig> device_2 = std::make_shared<TestDeviceConfig>();
	EXPECT_EQ(*device_1, *device_2);
}

/**
 * @brief Test that we can cast DeviceBase to its derived class.
 */
TEST(TestDeviceConfigInterfaces, testDeviceBaseCastToDerived){
	std::shared_ptr<DeviceBase> device = std::make_shared<TestDeviceConfig>();
	EXPECT_NO_THROW(auto derived_ptr = device->as<TestDeviceConfig>());
}

/**
 * @brief Test that the downcast helper will fail as if downcasting to the wrong type
 */
TEST(TestDeviceConfigInterfaces, testDeviceBaseCastToDerivedIncorrectDerivedType){
	std::shared_ptr<DeviceBase> device = std::make_shared<TestDeviceConfig>();
	EXPECT_THROW(auto derived_ptr = device->as<TestDeviceConfig2>(), std::runtime_error);
}

/**
 * @brief Test that we can cast DeviceConfig to its derived class.
 */
TEST(TestDeviceConfigInterfaces, testDeviceConfigCastToDerived){
	std::shared_ptr<DeviceConfig> device = std::make_shared<TestDeviceConfig>();
	EXPECT_NO_THROW(auto derived_ptr = device->as<TestDeviceConfig>());
}

/**
 * @brief Test that the downcast helper will fail as expected on classes that aren't derived from DeviceConfig.
 */
TEST(TestDeviceConfigInterfaces, testDeviceBaseCastFailsOnNonDerivedClass){
	std::shared_ptr<DeviceBase> device = std::make_shared<TestDeviceConfig>();
	EXPECT_THROW(auto derived_ptr = device->as<std::string>(), std::runtime_error);
}

/**
 * @brief Test that the downcast helper will fail as expected on classes that aren't derived from DeviceConfig.
 */
TEST(TestDeviceConfigInterfaces, testDeviceConfigCastFailsOnNonDerivedClass){
	std::shared_ptr<DeviceConfig> device = std::make_shared<TestDeviceConfig>();
	EXPECT_THROW(auto derived_ptr = device->as<std::string>(), std::runtime_error);
}

/**
 * @brief Test that we can cast DeviceBase to its derived virtual class, DeviceConfig.
 */
TEST(TestDeviceConfigInterfaces, testDeviceBaseCastToDeviceConfig){
	std::shared_ptr<DeviceBase> device = std::make_shared<TestDeviceConfig>();
	EXPECT_NO_THROW(auto derived_ptr = device->as<DeviceConfig>());
}

/**
 * @brief Test we can add a device config without error.
 */
TEST(TestDeviceConfigInterfaces, testAddDevice){
	auto motor_config = std::make_shared<TestDeviceConfig>();
	motor_config->port = 2;
	motor_config->name = "motor1";
	motor_config->type = device_type_e::MOTOR;

	DeviceConfigMap device_config_map;
	EXPECT_NO_THROW(device_config_map.addDeviceConfig(motor_config));
}

/**
 * @brief Test we can retrieve previously added device configs and cast to their derived type.
 */
TEST(TestDeviceConfigInterfaces, testDerivedDeviceClass){
	// Make derived object
	auto motor_config = std::make_shared<TestDeviceConfig>();
	motor_config->port = 2;
	motor_config->name = "motor1";
	motor_config->type  = device_type_e::MOTOR;
	motor_config->a = 4;
	motor_config->b = true;

	DeviceConfigMap device_config_map;
	EXPECT_NO_THROW(device_config_map.addDeviceConfig(motor_config));

	// Get the configuration as a pointer to base class (DeviceConfig) and cast to original type.
	auto device_base = device_config_map.getDeviceConfig("motor1");
	auto device_derived = std::dynamic_pointer_cast<const TestDeviceConfig>(device_base);

	// Verify cast is successful
	EXPECT_TRUE(device_derived != nullptr);

	// Verify objects are equivalent (no data lost)
	EXPECT_EQ(*device_derived, *motor_config);
}

/**
 * @brief Test that if we add a device and then change the original data, it does not change inside the DeviceConfigMap.
 */
TEST(TestDeviceConfigInterfaces, testAddDeviceByValue){
	// Make default device
	auto motor_config = std::make_shared<TestDeviceConfig>();
	motor_config->port = 2;
	motor_config->name = "motor1";
	motor_config->type = device_type_e::MOTOR;

	// Add to map
	DeviceConfigMap device_config_map;
	device_config_map.addDeviceConfig(motor_config);

	// Change original device
	motor_config->name = "motor2";

	// Get device by name and cast to base c
	auto motor_config_original = device_config_map.getDeviceConfig("motor1");

	EXPECT_TRUE(motor_config != motor_config_original);
}

/**
 * @brief Test that adding devices with the same port throws an error.
 */
TEST(TestDeviceConfigInterfaces, testDuplicatePortNumber){
	// Make two devices with the same port
	auto motor_1_config = std::make_shared<TestDeviceConfig>();
	motor_1_config->port = 2;
	motor_1_config->name = "motor1";
	motor_1_config->type = device_type_e::MOTOR;

	auto motor_2_config = std::make_shared<TestDeviceConfig>();
	motor_2_config->port = 2;
	motor_2_config->name = "motor2";
	motor_2_config->type = device_type_e::MOTOR;

	DeviceConfigMap device_config_map;
	EXPECT_NO_THROW(device_config_map.addDeviceConfig(motor_1_config));
	EXPECT_THROW(device_config_map.addDeviceConfig(motor_2_config), std::runtime_error);
}

/**
 * @brief Test that adding devices with the same name throws an error.
 */
TEST(TestDeviceConfigInterfaces, testDuplicateDeviceName){
	// Make two devices with the same name
	auto motor_1_config = std::make_shared<TestDeviceConfig>();
	motor_1_config->port = 1;
	motor_1_config->name = "motor2";
	motor_1_config->type = device_type_e::MOTOR;

	auto motor_2_config = std::make_shared<TestDeviceConfig>();
	motor_2_config->port = 2;
	motor_2_config->name = "motor2";
	motor_2_config->type = device_type_e::MOTOR;

	DeviceConfigMap device_config_map;
	EXPECT_NO_THROW(device_config_map.addDeviceConfig(motor_1_config));
	EXPECT_THROW(device_config_map.addDeviceConfig(motor_2_config), std::runtime_error);
}

/**
 * @brief Test that requesting a non-existent device throws a runtime error.
 */
TEST(TestDeviceConfigInterfaces, testGetNonexistentValueThrows){
	DeviceConfigMap device_config_map;
	EXPECT_THROW(device_config_map.getDeviceConfig("test"), std::runtime_error);
}

TEST(TestDeviceConfigInterfaces, testCloneDeviceConfigMap){
	auto config_1 = std::make_shared<TestDeviceConfig>();
	config_1->port = 2;
	config_1->name = "motor_1";
	config_1->type = device_type_e::MOTOR;
	config_1->a = 17;

	auto config_2 = std::make_shared<TestDeviceConfig>();
	config_1->port = 3;
	config_1->name = "sensor_1";
	config_1->type = device_type_e::ROTATION_SENSOR;
	config_1->a = 99;
	config_1->b = true;

	auto device_config_map_ptr = std::make_shared<DeviceConfigMap>();
	EXPECT_NO_THROW(device_config_map_ptr->addDeviceConfig(config_1));
	EXPECT_NO_THROW(device_config_map_ptr->addDeviceConfig(config_2));

	std::shared_ptr<DeviceConfigMap> cloned_config_map_ptr;
	EXPECT_NO_THROW(cloned_config_map_ptr = device_config_map_ptr->clone());

	EXPECT_EQ(*cloned_config_map_ptr, *device_config_map_ptr);

	for(auto it = device_config_map_ptr->begin(); it != device_config_map_ptr->end(); it++){
		auto name = it->first;
		auto device_config_ptr = it->second;

		EXPECT_TRUE(cloned_config_map_ptr->contains(name));
		EXPECT_FALSE(device_config_ptr == cloned_config_map_ptr->getDeviceConfig(name));
	}
}