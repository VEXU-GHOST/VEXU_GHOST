#include "behaviortree_cpp/behavior_tree.h"

// SyncActionNode (synchronous action) with an input port.
class SaySomething : public BT::SyncActionNode
{
public:
  // If your Node has ports, you must use this constructor signature 
  SaySomething(const std::string& name, const BT::NodeConfig& config)
    : SyncActionNode(name, config)
  { }

  // It is mandatory to define this STATIC method.
  static BT::PortsList providedPorts()
  {
    // This action has a single input port called "message"
    return { /*BT::InputPort<std::string>("message")*/ };
  }

  // Override the virtual function tick()
  BT::NodeStatus tick() override
  {
    // Expected<std::string> msg = getInput<std::string>("message");
    // // Check if expected is valid. If not, throw its error
    // if (!msg)
    // {
    //   throw BT::RuntimeError("missing required input [message]: ", 
    //                           msg.error() );
    // }
    // use the method value() to extract the valid message.
    
    return BT::NodeStatus::SUCCESS;
  }
};