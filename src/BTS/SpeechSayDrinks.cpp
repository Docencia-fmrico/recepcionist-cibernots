#include <string>
#include <iostream>

#include "BTS/SpeechSayDrinks.hpp"
#include "gb_dialog/DialogInterfaces.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace recepcionist_cibernots {

SpeechSayDrinks::SpeechSayDrinks(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
{
  config().blackboard->get("node", node_);
  config().blackboard->get("dialog_iface", dialog_iface_);
}

BT::NodeStatus SpeechSayDrinks::tick()
{
  std::ostringstream oss;
  std::string phrase;

  std::string drinkstr;

  rclcpp::spin_some(dialog_iface_.get_node_base_interface());
  dialog_iface_->listen();
  if (result_.intent == "Ask Drink") {    
    config().blackboard->get("drink", drinkstr);
    oss << "I want a " << name << " for our guest";
    phrase = oss.str();
  
    dialog_iface_->speak(phrase);
    
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::RUNNING;
  }

}

}  // namespace recepcionist_cibernots

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<recepcionist_cibernots::SpeechSayDrinks>("SpeechSayDrinks");
}
