#include "ghost_tank/pdcontrol.hpp"
#include "ghost_tank/bt_nodes/turnToPoint.hpp"
#include <cmath>

namespace ghost_tank {

TurnToPoint::TurnToPoint(const std::string & name, const BT::NodeConfig & config) :
    BT::StatefulActionNode(name, config)
{
    blackboard_ = config.blackboard;
    BT_Util::get_from_blackboard(blackboard_, "tank_model_ptr", tank_model_ptr_);

    posX_m = BT_Util::get_input<double>(this, "posX_tiles") * tile_to_meters;
    posY_m = BT_Util::get_input<double>(this, "posY_tiles") * tile_to_meters;
    timeout_ms = BT_Util::get_input<int>(this, "timeout_ms");
    angle_exit_threshold_rad = BT_Util::get_input<double>(this, "angle_exit_threshold_deg", 5.0) * ghost_util::DEG_TO_RAD;
}

BT::PortsList TurnToPoint::providedPorts() {
    return {
        BT::InputPort<double>("posX_tiles"),
        BT::InputPort<double>("posY_tiles"),
        BT::InputPort<int>("timeout_ms"),
        BT::InputPort<double>("angle_exit_threshold_deg")
    };
}

BT::NodeStatus TurnToPoint::onStart() {
    double cur_x = tank_model_ptr_->getWorldTwist().x();
    double cur_y = tank_model_ptr_->getWorldTwist().y();

    start_time_ = std::chrono::system_clock::now();
    des_ang_rad = std::atan2(posY_m - cur_y, posX_m - cur_x) + M_PI;

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TurnToPoint::onRunning() {
    double theta_err_rad = std::fabs((tank_model_ptr_->getWorldTwist().z() - des_ang_rad));
    bool angle_satisfied = theta_err_rad < angle_exit_threshold_rad;

    int time_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start_time_).count();
    if (angle_satisfied) {
        tank_model_ptr_->driveCommand(0.0, 0.0);
        return BT::NodeStatus::SUCCESS;
    }

    Eigen::Vector3d final_pose_ = Eigen::Vector3d(0.0, 0.0, des_ang_rad);
    auto command = pd_control_ptr_->theta_pid(tank_model_ptr_->getWorldPose(), tank_model_ptr_->getWorldTwist(), final_pose_);

    tank_model_ptr_->driveCommand(command[0], command[1]);

    return BT::NodeStatus::RUNNING;
}

void TurnToPoint::onHalted() {
    resetStatus();
}

} // namespace ghost_tank
