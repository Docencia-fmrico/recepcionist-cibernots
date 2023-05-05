#ifndef SPEECH_SAYINFO_H
#define SPEECH_SAYINFO_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "speech/PersonInfo.h"
#include "speech/Chat.h"
#include "string"
#include <sstream>

namespace speech
{
class SayInfo : public BT::ActionNodeBase
{
  public:
    explicit SayInfo(const std::string& name, const BT::NodeConfiguration& config);

    void halt();

    BT::NodeStatus tick();

    static BT::PortsList providedPorts()
    {
      return {BT::InputPort<PInfo>("Info")};
    }

  private:
    Chat forwarder;
    PInfo info_;
    ros::NodeHandle nh_;
};
};  // namespace speech

#endif  // SPEECH_SAYINFO_H