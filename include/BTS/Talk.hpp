#ifndef TALK_NODE_HPP
#define TALK_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "gb_dialog/DialogInterface.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "dialogflow_ros2_interfaces/msg/dialogflow_result.hpp"

namespace gb_dialog
{

class TAlk : public BT::ActionNodeBase
{
public:
  SpeechBasicsNode(const std::string &name, const BT::NodeConfiguration &config);

  BT::NodeStatus tick() override;
  
  static BT::PortsList providedPorts()
  {
    return BT::PortsList({});
  }

private:
  gb_dialog::Speech forwarder;
  std::shared_ptr<DialogInterface> dialog_interface_;
  rclcpp::Node::SharedPtr node_;
  gb_dialog::DialogInterface dialog_;
  bool failed_;
  bool firsttick_;
};

}  // namespace gb_dialog

#endif  // SPEECH_BASICS_NODE_HPP
