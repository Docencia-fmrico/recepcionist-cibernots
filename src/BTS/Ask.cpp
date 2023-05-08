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

#include "BTS/Ask.hpp"

#include "BTS/DialogFlow/DialogInterface.hpp"
#include "sound_play.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"

#include "rclcpp/rclcpp.hpp"

using namespace std::placeholders;  
namespace ph = std::placeholders;

namespace recepcionist_cibernots
{
// using namespace gb_dialog;

Ask::Ask(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
  // listening_(true);
: BT::ActionNodeBase(xml_tag_name, conf)
{
  listening_ = true;
  name_ = "";
  config().blackboard->get("node", node_);

    dialog_.registerCallback(std::bind(&Ask::noIntentCB, this, ph::_1));
    dialog_.registerCallback(
      std::bind(&Ask::welcomeIntentCB, this, ph::_1),
      "Default Welcome Intent");
    dialog_.registerCallback(
      std::bind(&Ask::requestNameIntentCB, this, ph::_1),
      "RequestName");
    dialog_.registerCallback(
      std::bind(&Ask::igRequestIntentCB, this, ph::_1),
      "IgRequest");
}


void Ask::noIntentCB(dialogflow_ros2_interfaces::msg::DialogflowResult result)
{
  name_ = result.parameters[1].value[0];
}

void Ask::welcomeIntentCB(dialogflow_ros2_interfaces::msg::DialogflowResult result)
{
  name_ = result.parameters[1].value[0];
}

void Ask::requestNameIntentCB(dialogflow_ros2_interfaces::msg::DialogflowResult result)
{
  name_ = result.parameters[1].value[0];
}

void Ask::igRequestIntentCB(dialogflow_ros2_interfaces::msg::DialogflowResult result)
{
  name_ = result.parameters[1].value[0];
}


BT::NodeStatus
Ask::tick()
{
  if (status() == BT::NodeStatus::IDLE)
  {
    dialog_.speak("Hi, what's your name?");
  }

  rclcpp::spin_some(dialog_.get_node_base_interface());

  if (!listening_)
  {
    listening_ = true;
    dialog_.listen();
  }

  if (name_ == "")
  {
    return BT::NodeStatus::RUNNING;
  }

  dialog_.speak("Follow me " + name_);
  setOutput("Request", name_);
  name_ = "";
  listening_ = false;
  
  return BT::NodeStatus::SUCCESS;
}

}  // namespace recepcionist_cibernots

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<recepcionist_cibernots::Ask>("Ask");
}
