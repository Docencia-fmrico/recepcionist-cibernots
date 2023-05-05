#include "behaviortree_cpp_v3/behavior_tree.h"
#include "speech/AskDrink.h"
#include <string>

namespace speech
{
  AskDrink::AskDrink(const std::string& name, const BT::NodeConfiguration& config)
  : BT::ActionNodeBase(name, config),
    nh_("~"),
    firsttick_(true)
  {}

  void
  AskDrink::halt()
  {
    ROS_INFO("AskDrink halt");
  }

  BT::NodeStatus
  AskDrink::tick()
  {

    ROS_INFO("First tick Drink: %d", firsttick_);
    if (firsttick_)
    {
      firsttick_ = false;
      forwarder.done_ = false;
      forwarder.speak("What is your favourite drink?");
    }

    ROS_INFO("Done Drink: %d", forwarder.done_);
    if (forwarder.done_)
    {
      forwarder.speak(forwarder.response_);
      forwarder.done_ = false;
      firsttick_ = true;
      ROS_INFO("PARAMETRO: %s", forwarder.param_.c_str());
      BT::TreeNode::getInput("Info", info_);

      info_.drink = forwarder.param_;
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
  factory.registerNodeType<speech::AskDrink>("AskDrink");
}