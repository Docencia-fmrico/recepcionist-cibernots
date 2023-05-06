#ifndef BTS__SPEECHNODE_HPP_
#define BTS__SPEECHNODE_HPP_

#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "rclcpp/rclcpp.hpp"

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

private:
    rclcpp::Node::SharedPtr node_;
    std::string name_;
    std::string drink_;
  
};

}  // namespace recepcionist_cibernots

#endif  // BTS__SPEECHNODE_HPP_