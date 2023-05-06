#ifndef BT_NODES__SpeechNode_HPP_
#define BT_NODES__SpeechNode_HPP_

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <string>
#include "gb_dialog/DialogInterfaces.hpp"
#include "sound_play.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include <memory>

namespace recepcionist_cibernots
{

class SpeechNode : public BT::ActionNodeBase
{
public:
  explicit SpeechNode(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void halt() {}
  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList({});
  }
  dialogflow_ros2_interfaces::msg::DialogflowResult result_;

private:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace recepcionist_cibernots

#endif  // BT_NODES__SpeechNode_HPP_

