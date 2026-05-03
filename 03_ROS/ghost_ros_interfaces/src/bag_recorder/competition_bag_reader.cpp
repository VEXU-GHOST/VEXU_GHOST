#include <chrono>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_storage/storage_options.hpp"

#include "ghost_msgs/msg/states/v5_competition.hpp"

class AutonomousTimeFinder : public rclcpp::Node
{
public:
  AutonomousTimeFinder(const std::string & bag_filename)
  : Node("autonomous_time_finder"), bag_file_(bag_filename)
  {
    find_autonomous_times();
  }

private:
  void find_autonomous_times()
  {
    rosbag2_cpp::readers::SequentialReader reader;
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bag_file_;
    storage_options.storage_id = "sqlite3";
    reader.open(storage_options);

    bool was_autonomous = false;
    rclcpp::Time start_time, end_time;

    while (reader.has_next()) {
      auto bag_msg = reader.read_next();

      // Filter for your competition topic
      if (bag_msg->topic_name != "/ghost_mvoid") {
        continue;
      }

      // Deserialize the message
      ghost_msgs::msg::states::V5Competition msg;
      rclcpp::SerializedMessage serialized_msg(*bag_msg->serialized_data);
      rclcpp::Serialization<ghost_msgs::msg::states::V5Competition> serializer;
      serializer.deserialize_message(&serialized_msg, &msg);

      double t = bag_msg->time_stamp * 1e-9; // convert from nanoseconds to seconds
      bool autonomous = msg.is_autonomous;
      bool disabled   = msg.is_disabled;

      // Detect start (false->true & not disabled)
      if (autonomous && !was_autonomous && !disabled) {
        start_time = rclcpp::Time(bag_msg->time_stamp);
        std::cout << "AUTONOMOUS START at " << t << " sec\n";
        was_autonomous = true;
      }

      // Detect end (true->false)
      if (!autonomous && was_autonomous) {
        end_time = rclcpp::Time(bag_msg->time_stamp);
        std::cout << "AUTONOMOUS END at " << t << " sec\n";
        break; // stop after finding the interval
      }
    }

  std::cout << "\nTo generate a smaller bag, run:\n\n";
  std::cout << "printf \"output_bags:\\n"
          << "- uri: small_bag\\n"
          << "  all_topics: true\\n"
          << "  start_time_ns: " << start_time.nanoseconds() << "\\n"
          << "  end_time_ns: " << end_time.nanoseconds() << "\\n"
          << "\" > out.yaml\n";

  std::cout << "ros2 bag convert -i " << bag_file_ << " --output-options out.yaml\n";
 
  }

  std::string bag_file_;
};

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <bag_file>" << std::endl;
    return 1;
  }

  rclcpp::init(argc, argv);
  auto node = std::make_shared<AutonomousTimeFinder>(argv[1]);
  rclcpp::shutdown();
  return 0;
}
