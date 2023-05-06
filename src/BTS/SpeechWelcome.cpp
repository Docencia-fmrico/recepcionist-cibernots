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
}

BT::NodeStatus SpeechDrinks::tick()
{
  rclcpp::spin_some(dialog_iface_.get_node_base_interface());
  dialog_iface_->listen();
  if (result_.intent == "Default Welcom Intent") {    
    dialog_iface_->speak(result_.fulfillment_text);

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
