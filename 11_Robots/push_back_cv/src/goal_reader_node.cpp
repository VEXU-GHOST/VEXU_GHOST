#include <functional>
#include <stdexcept>
#include <string>
#include <vector>
#include <limits>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "push_back_cv/msg/field_block.hpp"
#include "push_back_cv/msg/field_block_array.hpp"
#include "push_back_cv/msg/goal_state_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

class GoalReaderNode : public rclcpp::Node {
public:
  GoalReaderNode()
  : Node("goal_reader")
  {
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
  static constexpr uint8_t CONTROL_NONE = 0;
  static constexpr uint8_t CONTROL_RED = 1;
  static constexpr uint8_t CONTROL_BLUE = 2;

  enum class GoalLogic { None, MinY, MaxY, Both };

  struct GoalRegion {
    std::string id;
    double x_min, x_max, y_min, y_max, z_min, z_max;
    GoalLogic goal_logic = GoalLogic::None;
    std::vector<std::string> publish_ids;
  };

  static bool inPrism(const push_back_cv::msg::FieldBlock & b, const GoalRegion & g)
  {
    return b.x >= g.x_min && b.x <= g.x_max &&
           b.y >= g.y_min && b.y <= g.y_max &&
           b.z >= g.z_min && b.z <= g.z_max;
  }

  static double distSqToCenter(const push_back_cv::msg::FieldBlock & b, const GoalRegion & g)
  {
    const double cx = 0.5 * (g.x_min + g.x_max);
    const double cy = 0.5 * (g.y_min + g.y_max);
    const double cz = 0.5 * (g.z_min + g.z_max);
    const double dx = b.x - cx;
    const double dy = b.y - cy;
    const double dz = b.z - cz;
    return dx * dx + dy * dy + dz * dz;
  }

  static uint8_t colorOf(const push_back_cv::msg::FieldBlock & b)
  {
    return b.is_red ? CONTROL_RED : CONTROL_BLUE;
  }

  static uint8_t controlFromExtremeY(
    const std::vector<const push_back_cv::msg::FieldBlock *> & blocks,
    bool pick_min)
  {
    if (blocks.empty()) {
      return CONTROL_NONE;
    }
    const push_back_cv::msg::FieldBlock * best = blocks[0];
    for (const auto * block : blocks) {
      if (pick_min ? (block->y < best->y) : (block->y > best->y)) {
        best = block;
      }
    }
    return colorOf(*best);
  }

  static push_back_cv::msg::GoalState makeGoalState(
    const std::string & id,
    uint32_t red_count, uint32_t blue_count, uint8_t control)
  {
    push_back_cv::msg::GoalState gs;
    gs.id = id;
    gs.red_count = red_count;
    gs.blue_count = blue_count;
    gs.control = control;
    return gs;
  }

  static void parseGoalLogic(const YAML::Node & node, GoalRegion & g)
  {
    const std::string logic =
      node["goal_logic"] ? node["goal_logic"].as<std::string>() : "none";
    if (logic == "min_y") {
      g.goal_logic = GoalLogic::MinY;
    } else if (logic == "max_y") {
      g.goal_logic = GoalLogic::MaxY;
    } else if (logic == "both") {
      g.goal_logic = GoalLogic::Both;
      if (node["publish_ids"]) {
        for (const auto & pid : node["publish_ids"]) {
          g.publish_ids.push_back(pid.as<std::string>());
        }
      }
      if (g.publish_ids.size() != 2) {
        throw std::runtime_error(
          "goal_logic both requires exactly 2 publish_ids for " + g.id);
      }
    } else {
      g.goal_logic = GoalLogic::None;
    }
  }

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
      parseGoalLogic(node, g);
      goals_.push_back(g);
    }
  }

  void onBlocks(const push_back_cv::msg::FieldBlockArray::SharedPtr msg)
  {
    push_back_cv::msg::GoalStateArray out;
    out.header.stamp = now();
    out.header.frame_id = "map";

    std::vector<std::vector<const push_back_cv::msg::FieldBlock *>> blocks_by_region(
      goals_.size());

    for (const auto & block : msg->blocks) {
      int best_idx = -1;
      double best_dist = std::numeric_limits<double>::max();
      for (size_t i = 0; i < goals_.size(); ++i) {
        if (!inPrism(block, goals_[i])) {
          continue;
        }
        const double dist = distSqToCenter(block, goals_[i]);
        if (dist < best_dist) {
          best_dist = dist;
          best_idx = static_cast<int>(i);
        }
      }
      if (best_idx >= 0) {
        blocks_by_region[static_cast<size_t>(best_idx)].push_back(&block);
      }
    }

    for (size_t i = 0; i < goals_.size(); ++i) {
      const auto & region = goals_[i];
      const auto & in_zone = blocks_by_region[i];

      uint32_t red_count = 0;
      uint32_t blue_count = 0;
      for (const auto * block : in_zone) {
        block->is_red ? red_count++ : blue_count++;
      }

      switch (region.goal_logic) {
        case GoalLogic::MinY:
          out.goals.push_back(makeGoalState(
            region.id, red_count, blue_count, controlFromExtremeY(in_zone, true)));
          break;
        case GoalLogic::MaxY:
          out.goals.push_back(makeGoalState(
            region.id, red_count, blue_count, controlFromExtremeY(in_zone, false)));
          break;
        case GoalLogic::Both:
          out.goals.push_back(makeGoalState(
            region.publish_ids[0], red_count, blue_count,
            controlFromExtremeY(in_zone, true)));
          out.goals.push_back(makeGoalState(
            region.publish_ids[1], red_count, blue_count,
            controlFromExtremeY(in_zone, false)));
          break;
        case GoalLogic::None:
        default:
          out.goals.push_back(makeGoalState(
            region.id, red_count, blue_count, CONTROL_NONE));
          break;
      }
    }

    pub_->publish(out);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 500,
      "%zu blocks -> %zu goal states", msg->blocks.size(), out.goals.size());
  }

  std::vector<GoalRegion> goals_;
  rclcpp::Publisher<push_back_cv::msg::GoalStateArray>::SharedPtr pub_;
  rclcpp::Subscription<push_back_cv::msg::FieldBlockArray>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalReaderNode>());
  rclcpp::shutdown();
  return 0;
}
