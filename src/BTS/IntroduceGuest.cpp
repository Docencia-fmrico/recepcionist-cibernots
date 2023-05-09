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

#include "BTS/IntroduceGuest.hpp"

#include "BTS/DialogFlow/DialogInterface.hpp"
#include "sound_play.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"

#include "rclcpp/rclcpp.hpp"

using namespace std::placeholders;  
namespace ph = std::placeholders;

namespace recepcionist_cibernots
{
// using namespace gb_dialog;

IntroduceGuest::IntroduceGuest(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  name_ = "";
  config().blackboard->get("node", node_);
}

BT::NodeStatus
IntroduceGuest::tick()
{
  config().blackboard->get("NAME", name_);

  if (name_ == "")
  {
    RCLCPP_INFO(node_->get_logger(), "[IntroduceGuest] ESPERANDO NOMBRE");
    return BT::NodeStatus::RUNNING;
  }

  RCLCPP_INFO(node_->get_logger(), "[IntroduceGuest] PRESENTATION");
  dialog_.speak("Hi everyone, this is " + name_);

  // rclcpp::spin_some(dialog_.get_node_base_interface());

  RCLCPP_INFO(node_->get_logger(), "SUCCESS");
  return BT::NodeStatus::SUCCESS;
}

}  // namespace recepcionist_cibernots

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<recepcionist_cibernots::IntroduceGuest>("IntroduceGuest");
}
