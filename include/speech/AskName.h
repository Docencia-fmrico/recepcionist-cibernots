#ifndef SPEECH_ASKNAME_H
#define SPEECH_ASKNAME_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "speech/PersonInfo.h"
#include "speech/Chat.h"
#include <string>

namespace speech
{
class AskName : public BT::ActionNodeBase
{
  public:
    explicit AskName(const std::string& name, const BT::NodeConfiguration& config);

    void halt();

    BT::NodeStatus tick();

    static BT::PortsList providedPorts()
    {
      return { BT::BidirectionalPort<PInfo>("Info")};
    }

  private:
    Chat forwarder;
    PInfo info_;
    ros::NodeHandle nh_;
    bool firsttick_;

};
};  // namespace speech

#endif  // SPEECH_ASKNAME_H