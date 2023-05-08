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
  
}



BT::NodeStatus
Ask::tick()
{

  // dependiendo del puerto de entrada, se ejecuta una accion u otra
  // si el puerto de entrada es "Request", se ejecuta la accion de pedir el nombre
  case_ = getInput<std::string>("case").value();
  name_ = getInput<std::string>("name_received").value();
  drink_ = getInput<std::string>("drink_received").value();

  
  if (status() == BT::NodeStatus::IDLE)
  {
      if (case_ == "ask name"){
        dialog_.speak("What is your name?");
      } else if (case_ == "say name") {
        dialog_.speak("Hi everyone, this is " + name_); 
      } else if (case_ == "ask drink") {
        dialog_.speak("What do you want to drink?");
      } else if (case_ == "say drink") {
        dialog_.speak("Hi bartender, I want a " + drink_);
      }
  }

  rclcpp::spin_some(dialog_.get_node_base_interface());

  if (!listening_)
  {
    listening_ = true;
    dialog_.listen();
  }

  if (case_ == "")
  {
    return BT::NodeStatus::RUNNING;
  }


  if (case_ == "ask name"){
    dialog_.speak("Hi " + name_ + ", what do you want to drink?");
  } else if (case_ == "ask drink"){
    dialog_.speak("Ok " + name_ + ", I will ask the bartender for a " + drink_);
  } else if (case_ == "say drink"){
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
