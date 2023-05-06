#include <string>
#include <iostream>

#include "BTS/SpeechDrinks.hpp"
#include "gb_dialog/DialogInterfaces.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace recepcionist_cibernots {

SpeechDrinks::SpeechDrinks(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
{
  config().blackboard->get("node", node_);
  config().blackboard->get("dialog_iface", dialog_iface_);
}

BT::NodeStatus SpeechDrinks::tick()
{
  rclcpp::spin_some(dialog_iface_.get_node_base_interface());
  dialog_iface_->listen();
  if (result_.intent == "drink") {    
    config().blackboard->set("drink", result_.intent.param_);
    dialog_iface_->speak("Sure, I'll bring you a drink.");
    
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::RUNNING;
  }

}

}  // namespace recepcionist_cibernots

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<recepcionist_cibernots::SpeechDrinks>("SpeechDrinks");
}
