#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "test.cpp"

// file that contains the custom nodes definitions
// #include "dummy_nodes.h"
// using namespace DummyNodes;

class RunTreeNode : public rclcpp::Node {
public:
		RunTreeNode() :
				rclcpp::Node("run_tree_node"){
				declare_parameter<std::string>("bt_path");
				bt_path_ = get_parameter("bt_path").as_string();

				start_sub_ = create_subscription<std_msgs::msg::Bool>(
						"run_tree",
						10,
						[this](const std_msgs::msg::Bool::SharedPtr msg){
						if(msg->data){
								run_tree();
						}
				});
				run_tree();// find other solution
		}

private:
		std::string bt_path_;

		rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_sub_;

		void run_tree(){
				// todo: add ros node to blackboard
					// maybe not, seems like not necessary
				// todo: move this to another class an import
				BT::BehaviorTreeFactory factory;

				// add all nodes here
				factory.registerNodeType<SaySomething>("SaySomething");

				auto tree = factory.createTreeFromFile(bt_path_);
				tree.tickWhileRunning();
		}
};

// class RunTreePublisher : public rclcpp::Node {
// public:
// 		RunTreePublisher() :
// 				Node("run_tree_publisher"){
// 				// rclcpp::TimerBase::SharedPtr timer_;

// 				// Initialize Publisher
// 				output_pub_ = this->create_publisher<std_msgs::msg::Bool>("run_tree", 10);
// 		}
// 		void startTree(){
// 				std_msgs::msg::Bool msg;
// 				msg.data = true;
// 				output_pub_->publish(msg);
// 		}
// private:
// 		rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr output_pub_;
// };

int main(int argc, char *argv[]){
		rclcpp::init(argc, argv);
		auto node = std::make_shared<RunTreeNode>();
		rclcpp::spin(node);

		rclcpp::shutdown();
		return 0;
}