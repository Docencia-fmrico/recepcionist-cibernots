// Copyright 2023 cibernots
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef BTS__SERVEDRINK_HPP_
#define BTS__SERVEDRINK_HPP_

#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/string.hpp>

#include "BTS/DialogFlow/DialogInterface.hpp"

namespace recepcionist_cibernots
{
//using namespace std::std_msgs;
class ServeDrink : public BT::ActionNodeBase
{
public:
  explicit ServeDrink(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void halt() {}
  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return {};
  }


private:
  rclcpp::Node::SharedPtr node_;
  // dialogflow_ros2_interfaces::msg::DialogflowResult dialog_;
  gb_dialog::DialogInterface dialog_;

  // Variable para guardar el nombre de tipo dialogflow_ros2_interfaces::msg::DialogflowResult
  std::string drink_;
};

}  // namespace recepcionist_cibernots

#endif  // BTS__ASK_HPP_
