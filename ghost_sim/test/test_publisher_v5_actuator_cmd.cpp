/*
 * Filename: gazebo_joint_pid_plugin
 * Created Date: Sunday August 7th 2022
 * Author: Maxx Wilson
 * Author Email: JesseMaxxWilson@utexas.edu
 * 
 * Last Modified: Saturday September 10th 2022 10:50:34 am
 * Modified By: Maxx Wilson
 */

#include "test_publisher_v5_actuator_cmd.hpp"

namespace ghost_sim
{

    testPublisherV5ActuatorCmd::testPublisherV5ActuatorCmd() : Node("test_publisher_v5_actuator_cmd")
    {
        rclcpp::TimerBase::SharedPtr timer_;
        
        // Initialize Publisher
        output_pub_ = this->create_publisher<ghost_msgs::msg::V5ActuatorCommand>("v5actuator/setpoint", 10);

    }


    // Pseudo motor command data
    void testPublisherV5ActuatorCmd::publishData(){
        int loop_index = 0;

        ghost_msgs::msg::V5ActuatorCommand msg = ghost_msgs::msg::V5ActuatorCommand();
        msg.header.frame_id = "base_link";
        msg.header.stamp = this->get_clock()->now();
        msg.msg_id = 1; // TODO: is this right?

        // Lamda for-each expression to make all elements of motor_arr 1
        for_each(begin(msg.motor_commands), end(msg.motor_commands), [&](ghost_msgs::msg::V5MotorCommand& motor_cmd){
            motor_cmd = this->populateMotorCmd(loop_index);
            loop_index +=1;
        });

        // RCLCPP_INFO(
        //     this->get_logger(),
        //     "Publishing to setpoint");
        output_pub_ -> publish(msg);

    }

    ghost_msgs::msg::V5MotorCommand testPublisherV5ActuatorCmd::populateMotorCmd(const int loop_index){
        ghost_msgs::msg::V5MotorCommand v5_motor_cmd = ghost_msgs::msg::V5MotorCommand();
        v5_motor_cmd.motor_name = "motor_" + std::to_string(loop_index);
        v5_motor_cmd.device_id = loop_index;
        v5_motor_cmd.desired_angle = loop_index * 2; //degrees
        v5_motor_cmd.desired_velocity = 0.0;
        v5_motor_cmd.desired_torque = 0.0;
        v5_motor_cmd.desired_voltage = 0.0;
        v5_motor_cmd.current_limit = 5; //milliAmps

        v5_motor_cmd.angle_control = true;
        v5_motor_cmd.velocity_control = false;
        v5_motor_cmd.torque_control = false;
        v5_motor_cmd.voltage_control = false;

        return v5_motor_cmd; 
    }

}  // namespace ghost_sim

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    ghost_sim::testPublisherV5ActuatorCmd test_pub_ = ghost_sim::testPublisherV5ActuatorCmd{};
    while(rclcpp::ok()){
        test_pub_.publishData();
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    // rclcpp::shutdown();
    return 0;
}