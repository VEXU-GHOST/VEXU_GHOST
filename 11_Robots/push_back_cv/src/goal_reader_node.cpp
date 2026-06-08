#include <vector>
#include <string>

#include "push_back_cv/msg/field_block.hpp"
#include "push_back_cv/msg/goal_state_array.hpp"
#include "yaml-cpp/yaml.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "push_back_cv/msg/field_block_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include <functional>
#include <stdexcept>


class GoalReaderNode : public rclcpp::Node {
private:
    struct GoalRegion {
        std::string id;
        double x_min, x_max, y_min, y_max, z_min, z_max;
    };
    void loadGoals(const std::string & path)
    {
        YAML::Node root = YAML::LoadFile(path);
        if (!root["goals"]) {
            throw std::runtime_error("goal_regions.yaml missing 'goals' key");
        }
        for (const auto & node : root["goals"]) {
            GoalRegion g;
            g.id = node["id"].as<std::string>();
            g.x_min = node["x_min"].as<double>();
            g.x_max = node["x_max"].as<double>();
            g.y_min = node["y_min"].as<double>();
            g.y_max = node["y_max"].as<double>();
            g.z_min = node["z_min"].as<double>();
            g.z_max = node["z_max"].as<double>();
            goals_.push_back(g);
        }
    }
    void onBlocks(const push_back_cv::msg::FieldBlockArray::SharedPtr msg)
    {
        push_back_cv::msg::GoalStateArray out;
        out.header.stamp = now();
        out.header.frame_id = "map";

        for (const auto & region : goals_) {
            push_back_cv::msg::GoalState gs;
            gs.id = region.id;
            gs.red_count = 0;
            gs.blue_count = 0;

            for (const auto & block : msg->blocks) {
            if (!inPrism(block, region)) {
                continue;
            }
            block.is_red ? gs.red_count++ : gs.blue_count++;
            }
            out.goals.push_back(gs);
        }

        pub_->publish(out);

        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 500,
            "%zu blocks -> %zu goal states", msg->blocks.size(), out.goals.size());
    }
public:
    std::vector<GoalRegion> goals_;
    rclcpp::Publisher<push_back_cv::msg::GoalStateArray>::SharedPtr pub_;

    bool inPrism(const push_back_cv::msg::FieldBlock & b, const GoalRegion & g)
    {
        return b.x >= g.x_min && b.x <= g.x_max &&
                b.y >= g.y_min && b.y <= g.y_max &&
                b.z >= g.z_min && b.z <= g.z_max;
    }

    GoalReaderNode() : Node("goal_reader") {
        declare_parameter<std::string>("goal_regions_file", "");
        std::string path = get_parameter("goal_regions_file").as_string();
        if (path.empty()) {
        path = ament_index_cpp::get_package_share_directory("push_back_cv")
            + "/config/goal_regions.yaml";
        }
        loadGoals(path);

        pub_ = create_publisher<push_back_cv::msg::GoalStateArray>("/field/goals", 10);
        RCLCPP_INFO(get_logger(), "Loaded %zu goal region(s) from %s", goals_.size(), path.c_str());
        sub_ = create_subscription<push_back_cv::msg::FieldBlockArray>(
            "/field/blocks", 10,
            std::bind(&GoalReaderNode::onBlocks, this, std::placeholders::_1));
        RCLCPP_INFO(get_logger(), "Starting goal_reader");
    }
private:
    rclcpp::Subscription<push_back_cv::msg::FieldBlockArray>::SharedPtr sub_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoalReaderNode>());
    rclcpp::shutdown();
    return 0;
}