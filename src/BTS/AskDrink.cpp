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

#include "BTS/AskDrink.hpp"

#include "BTS/DialogFlow/DialogInterface.hpp"
#include "sound_play.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"

#include "rclcpp/rclcpp.hpp"

using namespace std::placeholders;  
namespace ph = std::placeholders;
using namespace std::chrono_literals;
namespace recepcionist_cibernots
{
// using Drinkspace gb_dialog;

AskDrink::AskDrink(
  const std::string & xml_tag_Drink,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_Drink, conf)
{
  listening_ = false;
  Drink_ = "";

  config().blackboard->get("node", node_);

  dialog_.registerCallback(std::bind(&AskDrink::noIntentCB, this, ph::_1));
  dialog_.registerCallback(std::bind(&AskDrink::DrinkCB, this, ph::_1), "drinkCB");
}


void AskDrink::noIntentCB(dialogflow_ros2_interfaces::msg::DialogflowResult result)
{
  Drink_ = "";
  RCLCPP_INFO(node_->get_logger(), "[AskDrink] NOINTENT");
}

void AskDrink::DrinkCB(dialogflow_ros2_interfaces::msg::DialogflowResult result)
{
  Drink_ = result.parameters[1].value[0].c_str();
  RCLCPP_INFO(node_->get_logger(), "[AskDrink] DrinkCB: intent [%s]", Drink_.c_str());
  config().blackboard->set("Drink", Drink_);
}


BT::NodeStatus
AskDrink::tick()
{ 
  if (status() == BT::NodeStatus::IDLE)
  {
    start_time_ = node_->now();
    RCLCPP_INFO(node_->get_logger(), "[AskDrink] DrinkCB: speak->What is your Drink?");
    dialog_.speak("What would you like to drink?");
  }

  rclcpp::spin_some(dialog_.get_node_base_interface());

  if (node_->now()-start_time_ < 3.5s) {
    return BT::NodeStatus::RUNNING;
  }

  if (!listening_)
  {
    listening_ = true;
    RCLCPP_INFO(node_->get_logger(), "[AskDrink] VOY A ESCUCHAR");
    dialog_.listen();
  }

  if (Drink_ == "" && node_->now()-start_time_ > 10s) {
    listening_ = false;
  }

  if (Drink_ == "")
  {
    // listening_ = false;
    return BT::NodeStatus::RUNNING;
  }

  listening_ = false;
  dialog_.speak("I'll bring you your drink");
  RCLCPP_INFO(node_->get_logger(), "SUCCESS");
  return BT::NodeStatus::SUCCESS;
}

}  // Drinkspace recepcionist_cibernots

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<recepcionist_cibernots::AskDrink>("AskDrink");
}
