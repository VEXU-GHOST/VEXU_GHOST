#include <ghost_sim_examples/v5_robot_simulator_plugin.hpp>

#include <ghost_msgs/msg/v5_actuator_command.hpp>

#include <ghost_control/models/dc_motor_model.hpp>
#include <ghost_control/motor_controller.hpp>

#include <ghost_v5_interfaces/robot_hardware_interface.hpp>

#include <ghost_util/parsing_util.hpp>

using Eigen::Dynamic;
using Eigen::Matrix;
using Eigen::RowMajor;

using ghost_control::DCMotorModel;
using ghost_control::MotorController;

namespace gazebo
{

class V5RobotSimulatorPluginPrivate
{
public:
  V5RobotSimulatorPluginPrivate() = default;
  gazebo::event::ConnectionPtr updateConnection;
  gazebo::physics::ModelPtr model_;
  gazebo_ros::Node::SharedPtr ros_node_;

  Eigen::MatrixXd actuator_jacobian_;
  std::vector<std::string> motor_names_;
  std::vector<std::string> joint_names_;
//   double free_speed_;
//   double stall_torque_;
//   double free_current_;
//   double stall_current_;
//   double nominal_voltage_;
//   double gear_ratio_;

  std::unordered_map<std::string, std::shared_ptr<DCMotorModel>> motor_model_map_;
  std::unordered_map<std::string, std::shared_ptr<MotorController>> motor_controller_map_;

  Eigen::VectorXd joint_angles_;
  Eigen::VectorXd joint_velocities_;
  Eigen::VectorXd joint_efforts_;

  rclcpp::Subscription<ghost_msgs::msg::V5ActuatorCommand>::SharedPtr actuator_command_sub_;

  std::mutex actuator_update_callback_mutex;
};

V5RobotSimulatorPlugin::V5RobotSimulatorPlugin()
: impl_(std::make_unique<V5RobotSimulatorPluginPrivate>())
{
}

void V5RobotSimulatorPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  impl_->ros_node_ = gazebo_ros::Node::Get(sdf);
  impl_->model_ = model;
  impl_->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&V5RobotSimulatorPlugin::OnUpdate, this));

  loadConfig(sdf);
  setupRobotHardwareInterface();
  setupMotorModels();
  setupROSComms();
}

void V5RobotSimulatorPlugin::setupRobotHardwareInterface()
{

  // Somehow, we get the YAML filepath.
  
  // instantiate RobotHardwareInterface from YAML (see V5RobotPlugin?).
  // Get YAML path from ROS Param
  impl_->declare_parameter("robot_hardware_config_worlds_24.yaml", "");
  std::string robot_config_yaml_path =
    impl_->get_parameter("robot_hardware_config_worlds_24.yaml").as_string();

  // Load RobotHardwareInterface from YAML
  auto device_config_map = loadRobotConfigFromYAMLFile(robot_config_yaml_path);
  rhi_ptr_ = std::make_shared<RobotHardwareInterface>(
    device_config_map,
    hardware_type_e::V5_BRAIN);
  

  // TODO: Your code here.


  /*
    // Get YAML path from ROS Param
  node_ptr_->declare_parameter("robot_config_yaml_path", "");
  std::string robot_config_yaml_path =
    node_ptr_->get_parameter("robot_config_yaml_path").as_string();

  // Load RobotHardwareInterface from YAML
  auto device_config_map = loadRobotConfigFromYAMLFile(robot_config_yaml_path);
  rhi_ptr_ = std::make_shared<RobotHardwareInterface>(
    device_config_map,
    hardware_type_e::V5_BRAIN);
    */
}

void V5RobotSimulatorPlugin::setupMotorModels()
{
  motor_config= rhi_ptr_->getDeviceConfig<MotorDeviceConfig>("DRIVE_LEFT_FRONT_LEFT_MOTOR");
  
// For every motor we want to simulate:
//      Get the motor config from RobotHardwareInterface
//              EX. motor_config = rhi_ptr_->getDeviceConfig<MotorDeviceConfig>("left_drive_2");
//      make a new controller and add it to the motor_controller_map_.
//              Then we can access it via motor_controller_map_.at("my_motor")
//      make a new motor model and add it to the motor_model_map_
//              Then we can access it via motor_model_map_.at("my_motor")

// TODO: Your code here!
}

void V5RobotSimulatorPlugin::setupROSComms()
{
  impl_->actuator_command_sub_ =
    impl_->ros_node_->create_subscription<ghost_msgs::msg::V5ActuatorCommand>(
    "v5/actuator_command",
    10,
    [this](const ghost_msgs::msg::V5ActuatorCommand::SharedPtr msg) {
      // Actuator Command Callback

      std::unique_lock update_lock(impl_->actuator_update_callback_mutex);


      update_lock.unlock();


      // 1) Acquire actuator update mutex (two threads are sharing the same data).
      // 2) Update the RobotHardwareInterface with the new actuator commands from the msg

      // TODO: Your code here!

    });


  impl_->sensor_update_pub_ = 
  impl_->ros_node_->create_subscription<ghost_msgs::msg::V5ActuatorCommand>(
    "v5/sensor_command",
    10,
    [this](const ghost_msgs::msg::V5ActuatorCommand::SharedPtr msg) {
      std::unique_lock update_lock(impl_->actuator_update_callback_mutex);// Add a publisher to publishes V5SensorUpdate!
  // Do we need to do like, rclcpp::spin in another thread?

  // TODO: Your code here!

}

void V5RobotSimulatorPlugin::loadConfig(sdf::ElementPtr sdf)
{
  auto logger = impl_->ros_node_->get_logger();

  std::vector<std::string> params{
    "joint_names",
    "motor_names",
    "actuator_jacobian",
    // "free_speed",
    // "stall_torque",
    // "free_current",
    // "stall_current",
    // "nominal_voltage",
    // "gear_ratio"
  };

  for (std::string & param : params) {
    if (!sdf->HasElement(param)) {
      std::string err_string = "[V5RobotSimulatorPlugin] Missing <" + param + ">, cannot proceed";
      RCLCPP_ERROR(logger, err_string.c_str());
      return;
    }
  }

  // Parse input plugin parameters
  impl_->joint_names_ = ghost_util::getVectorFromString<std::string>(
    sdf->GetElement("joint_names")->Get<std::string>(), ' ');
  impl_->motor_names_ = ghost_util::getVectorFromString<std::string>(
    sdf->GetElement("motor_names")->Get<std::string>(), ' ');
//   impl_->gear_ratio_ = ghost_util::convertFromString<float>(
//     sdf->GetElement("gear_ratio")->Get<std::string>());
//   impl_->nominal_voltage_ = ghost_util::convertFromString<float>(
//     sdf->GetElement("nominal_voltage")->Get<std::string>());
//   impl_->stall_current_ = ghost_util::convertFromString<float>(
//     sdf->GetElement("stall_current")->Get<std::string>());
//   impl_->free_current_ = ghost_util::convertFromString<float>(
//     sdf->GetElement("free_current")->Get<std::string>());
//   impl_->stall_torque_ = ghost_util::convertFromString<float>(
//     sdf->GetElement("stall_torque")->Get<std::string>());
//   impl_->free_speed_ = ghost_util::convertFromString<float>(
//     sdf->GetElement("free_speed")->Get<std::string>());

  // Define eigen vector sizes using number of joints
  impl_->joint_angles_.resize(impl_->joint_names_.size());
  impl_->joint_velocities_.resize(impl_->joint_names_.size());
  impl_->joint_efforts_.resize(impl_->joint_names_.size());

  std::vector<double> actuator_jacobian_temp = ghost_util::getVectorFromString<double>(
    sdf->GetElement(
      "actuator_jacobian")->Get<std::string>(), ' ');

  // Input Validation
  if (actuator_jacobian_temp.size() != impl_->motor_names_.size() * impl_->joint_names_.size()) {
    std::string err_string =
      "[V5RobotSimulatorPlugin], Actuator Jacobian is incorrect size, cannot proceed!";
    RCLCPP_ERROR(logger, err_string.c_str());
    return;
  }

  // Populate Eigen Matrices for each jacobian
  impl_->actuator_jacobian_ = Eigen::Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(
    actuator_jacobian_temp.data(), impl_->joint_names_.size(), impl_->motor_names_.size());

  std::cout << "------------------------------" << std::endl;
  std::cout << "--- V5RobotSimulatorPlugin ---" << std::endl;
  std::cout << "------------------------------" << std::endl;

  std::cout << "Actuator Jacobian:\n" << impl_->actuator_jacobian_ << std::endl;
  std::cout << std::endl;

  std::cout << "Motor Names:" << std::endl;
  for (const auto & motor : impl_->motor_names_) {
    std::cout << motor << std::endl;
  }
  std::cout << std::endl;

  std::cout << "Joint Names:" << std::endl;
  for (const auto & joint : impl_->joint_names_) {
    std::cout << joint << std::endl;
  }
  std::cout << std::endl;

//   std::cout << "--- Motor Model Config ---" << std::endl;
//   std::cout << "Gear Ratio: " << impl_->gear_ratio_ << std::endl;
//   std::cout << "Nominal Voltage: " << impl_->nominal_voltage_ << std::endl;
//   std::cout << "Stall Current: " << impl_->stall_current_ << std::endl;
//   std::cout << "Free Current: " << impl_->free_current_ << std::endl;
//   std::cout << "Stall Torque: " << impl_->stall_torque_ << std::endl;
//   std::cout << "Free Speed: " << impl_->free_speed_ << std::endl;
}

void V5RobotSimulatorPlugin::OnUpdate()
{
  // 1) Update each motor controller in the motor_controller_map_ with the latest actuator commands from RobotHardwareInterface.
  // 2) Update the joint_angles_ and joint_velocities_ vectors with new gazebo sim data.
  // 3) At some point, update the RobotHardwareInterface with sensor data from gazebo, and then publish a V5SensorUpdate.
  // 4) Using the actuator_jacobian_, convert the gazebo joint data to motor joint data (i.e. current motor position/velocity)
  // 5) Update each motor controller in the motor_controller_map_ with the latest motor sensor data.
  // 6) For every motor MODEL, update the state as well.
  // 7) Now, get voltage output from each motor controller and apply it to every motor model. Then, get the resulting torque.
  // 8) Load the motor torques into a vector, and use the actuator_jacobian_ to get the joint torques.
  // 9) Apply joint torques to each torque in gazebo

  std::unique_lock update_lock(impl_->actuator_update_callback_mutex);


  update_lock.unlock();
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(V5RobotSimulatorPlugin)
} // namespace gazebo
