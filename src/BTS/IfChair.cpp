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

#include <string>
#include <iostream>
#include <memory>

#include "BTS/IfChair.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "rclcpp/rclcpp.hpp"

namespace recepcionist_cibernots
{
using namespace std::chrono_literals;

IfChair::IfChair(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf),
{
  config().blackboard->get("node", node_);
}

BT::NodeStatus
IfChair::tick()
{ 
    /*
        Código donde se escribirá la parte de detectar sillas
    */

    // para ir probando la navegación, este nodo, devuelve success
    return BT::NodeStatus::SUCCESS;
}

}  // namespace recepcionist_cibernots

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<recepcionist_cibernots::IfChair>("IfChair");
}