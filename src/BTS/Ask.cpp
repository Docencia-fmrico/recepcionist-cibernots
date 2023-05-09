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
  listening_ = false;
  name_ = "";
  drink_ = "";

  config().blackboard->get("node", node_);

    dialog_.registerCallback(std::bind(&Ask::noIntentCB, this, ph::_1));
    dialog_.registerCallback(
      std::bind(&Ask::nameCB, this, ph::_1),
      "nameCB");
    dialog_.registerCallback(
      std::bind(&Ask::drinkCB, this, ph::_1),
      "drinkCB");
    dialog_.registerCallback(
      std::bind(&Ask::bartenderCB, this, ph::_1),
      "bartenderCB");
}


void Ask::noIntentCB(dialogflow_ros2_interfaces::msg::DialogflowResult result)
{
  name_ = result.parameters[1].value[0];
}

void Ask::nameCB(dialogflow_ros2_interfaces::msg::DialogflowResult result)
{
  name_ = result.parameters[1].value[0];
  setOutput<std::string>("name_received", name_);
}

void Ask::drinkCB(dialogflow_ros2_interfaces::msg::DialogflowResult result)
{
  drink_ = result.parameters[1].value[0];
  setOutput<std::string>("drink_received", drink_);
}

void Ask::bartenderCB(dialogflow_ros2_interfaces::msg::DialogflowResult result)
{
  dialog_.speak("Thanks, I will see you later bartender");
}



BT::NodeStatus
Ask::tick()
{

  // dependiendo del puerto de entrada, se ejecuta una accion u otra
  // si el puerto de entrada es "Request", se ejecuta la accion de pedir el nombre
  
  getInput<std::string>("cases", cases_);
  // name_ = getInput<std::string>("name_received", nameAsk);
  // drink_ = getInput<std::string>("drink_received", drinkAsk);

  
  if (status() == BT::NodeStatus::IDLE)
  {
      if (cases_ == "ask name"){
        RCLCPP_INFO(node_->get_logger(), "[ASK] nameCB: speak->What is your name?");
        dialog_.speak("What is your name?");
      } else if (cases_ == "say name") {
        RCLCPP_INFO(node_->get_logger(), "[ASK] nameCB: intent [%s]", name_);
        dialog_.speak("Hi everyone, this is " + name_); 
      } else if (cases_ == "ask drink") {
        RCLCPP_INFO(node_->get_logger(), "[ASK] drinkCB: speak->What do you want to drink?", drink_);
        dialog_.speak("What do you want to drink?");
      } else if (cases_ == "say drink") {
        RCLCPP_INFO(node_->get_logger(), "[ASK] drinkCB: intent [%s]", drink_);
        dialog_.speak("Hi bartender, I want a " + drink_);
      }
  }

  rclcpp::spin_some(dialog_.get_node_base_interface());

  if (!listening_ && cases_ != "" && (cases_ != "say name" || cases_ != "say drink"))
  {
    listening_ = true;
    dialog_.listen();
  }

  if (cases_ == "")
  {
    return BT::NodeStatus::RUNNING;
  }


  if (cases_ == "ask name"){
    dialog_.speak("Hi " + name_ + ", what do you want to drink?");
  } else if (cases_ == "ask drink"){
    dialog_.speak("Ok " + name_ + ", I will ask the bartender for a " + drink_);
  } else if (cases_ == "say drink"){
    dialog_.speak("Thak you for the " + drink_ + ", bartender");
  }


  listening_ = false;
  
  return BT::NodeStatus::SUCCESS;
}

}  // namespace recepcionist_cibernots

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<recepcionist_cibernots::Ask>("Ask");
}
