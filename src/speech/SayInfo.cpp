#include "speech/SayInfo.h"
#include <string>

namespace speech
{
  SayInfo::SayInfo(const std::string& name, const BT::NodeConfiguration& config)
  : BT::ActionNodeBase(name, config),
    nh_("~")
  {}

  void
  SayInfo::halt()
  {
    ROS_INFO("SayInfo halt");
  }

  BT::NodeStatus
  SayInfo::tick()
  {
    std::ostringstream oss;
    std::string phrase;

    BT::TreeNode::getInput("Info", info_);

    oss << "Their name is " << info_.name << ", and their favorite drink is " << info_.drink;
    phrase = oss.str();

    forwarder.speak(phrase);

    return BT::NodeStatus::SUCCESS;
  }

};  // namespace speech

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<speech::SayInfo>("SayInfo");
}