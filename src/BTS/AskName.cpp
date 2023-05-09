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

#include "BTS/AskName.hpp"

#include "BTS/DialogFlow/DialogInterface.hpp"
#include "sound_play.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"

#include "rclcpp/rclcpp.hpp"

using namespace std::placeholders;  
namespace ph = std::placeholders;

namespace recepcionist_cibernots
{
// using namespace gb_dialog;
using namespace std::chrono_literals;
AskName::AskName(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
  // listening_(true);
: BT::ActionNodeBase(xml_tag_name, conf)
{
  listening_ = false;
  name_ = "";

  config().blackboard->get("node", node_);

    dialog_.registerCallback(std::bind(&AskName::noIntentCB, this, ph::_1));
    dialog_.registerCallback(
    std::bind(&AskName::nameCB, this, ph::_1), "nameCB");
}


void AskName::noIntentCB(dialogflow_ros2_interfaces::msg::DialogflowResult result)
{
  name_ = "";
  RCLCPP_INFO(node_->get_logger(), "[AskName] NOINTENBT");
}

void AskName::nameCB(dialogflow_ros2_interfaces::msg::DialogflowResult result)
{
  name_ = result.parameters[1].value[0].c_str();
  RCLCPP_INFO(node_->get_logger(), "[AskName] nameCB");
  RCLCPP_INFO(node_->get_logger(), "[AskName] nameCB: intent [%s]", name_.c_str());
  config().blackboard->set("NAME", name_);
}


BT::NodeStatus
AskName::tick()
{ 
  if (status() == BT::NodeStatus::IDLE)
  {
    RCLCPP_INFO(node_->get_logger(), "[AskName] nameCB: speak->What is your name?");
    start_time_ = node_->now();
    dialog_.speak("What is your name?");
  }

  rclcpp::spin_some(dialog_.get_node_base_interface());

  if (node_->now()-start_time_ < 1.5s) {
    return BT::NodeStatus::RUNNING;
  }

  if (!listening_)
  {
    listening_ = true;
    RCLCPP_INFO(node_->get_logger(), "VOY A ESCUCHAR");
    dialog_.listen();
  }

  if (name_ == "" && node_->now()-start_time_ > 10s) {
    listening_ = false;
  }

  if (name_ == "")
  {
    return BT::NodeStatus::RUNNING;
  }

  listening_ = false;
  RCLCPP_INFO(node_->get_logger(), "SUCCESS");
  dialog_.speak("Nice to meet you " + name_ +", follow me");
  return BT::NodeStatus::SUCCESS;
}

}  // namespace recepcionist_cibernots

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<recepcionist_cibernots::AskName>("AskName");
}
