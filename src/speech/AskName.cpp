#include "speech/AskName.h"
#include <string>

namespace speech
{
  AskName::AskName(const std::string& name, const BT::NodeConfiguration& config)
  : BT::ActionNodeBase(name, config),
    nh_("~"),
    firsttick_(true)
  {}

  void
  AskName::halt()
  {
    ROS_INFO("AskName halt");
  }

  BT::NodeStatus
  AskName::tick()
  {
    ROS_INFO("First tick Name: %d", firsttick_);
    if (firsttick_)
    {
      firsttick_ = false;
      forwarder.done_ = false;
      forwarder.speak("What is your name?");
    }

    ROS_INFO("Done Name: %d", forwarder.done_);
    if (forwarder.done_)
    {
      forwarder.speak(forwarder.response_);
      forwarder.done_ = false;
      firsttick_ = true;
      ROS_INFO("PARAMETRO: %s", forwarder.param_.c_str());
      info_.name = forwarder.param_;
      BT::TreeNode::setOutput("Info", info_);
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      forwarder.listen();
      return BT::NodeStatus::RUNNING;
    }
  }

};  // namespace speech

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<speech::AskName>("AskName");
}