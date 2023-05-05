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

#ifndef BTS__WAITPERSON_HPP_
#define BTS__WAITPERSON_HPP_

#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "rclcpp/rclcpp.hpp"

namespace recepcionist_cibernots
{

class WaitPerson : public BT::ActionNodeBase
{
public:
  explicit WaitPerson(
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
  
};

}  // namespace recepcionist_cibernots

#endif  // BTS__WAITPERSON_HPP_
