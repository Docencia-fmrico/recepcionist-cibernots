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

#include "BTS/OrderDrink.hpp"

#include "BTS/DialogFlow/DialogInterface.hpp"
#include "sound_play.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"

#include "rclcpp/rclcpp.hpp"

using namespace std::placeholders;  
namespace ph = std::placeholders;

namespace recepcionist_cibernots
{
using namespace std::chrono_literals;

OrderDrink::OrderDrink(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  drink_brmn_ = "";
  listening_ = false;
  config().blackboard->get("node", node_);
  dialog_.registerCallback(std::bind(&OrderDrink::noIntentCB, this, ph::_1));
  dialog_.registerCallback(std::bind(&OrderDrink::BarmanCB, this, ph::_1), "barmanCB");
}


void OrderDrink::noIntentCB(dialogflow_ros2_interfaces::msg::DialogflowResult result)
{
  drink_brmn_ = "";
  RCLCPP_INFO(node_->get_logger(), "[OrderDrink] NOINTENBT");
}

void OrderDrink::BarmanCB(dialogflow_ros2_interfaces::msg::DialogflowResult result)
{
  drink_brmn_ = result.parameters[1].value[0].c_str();
  RCLCPP_INFO(node_->get_logger(), "[OrderDrink] DrinkCB: intent [%s]", drink_.c_str());
  config().blackboard->set("Drink_brmn", drink_brmn_);
}

BT::NodeStatus
OrderDrink::tick()
{
  config().blackboard->get("Drink_brmn", drink_brmn_);
  config().blackboard->get("Drink", drink_);

  if (status() == BT::NodeStatus::IDLE)
  {
    start_time_ = node_->now();
    RCLCPP_INFO(node_->get_logger(), "[OrderDrink] PRESENTATION");
    dialog_.speak("Hey Barman, can you pour me some " + drink_ + "?");
  }

  rclcpp::spin_some(dialog_.get_node_base_interface());
  
  if (node_->now()-start_time_ < 3.5s) {
    return BT::NodeStatus::RUNNING;
  }

  if (!listening_)
  {
    listening_ = true;
    RCLCPP_INFO(node_->get_logger(), "VOY A ESCUCHAR");
    dialog_.listen();
  }

  if (drink_brmn_ == "" && node_->now()-start_time_ > 10s) {
    listening_ = false;
  }

  if (drink_brmn_ == "")
  {
    return BT::NodeStatus::RUNNING;
  }

  listening_ = false;
  RCLCPP_INFO(node_->get_logger(), "SUCCESS");
  return BT::NodeStatus::SUCCESS;
}

}  // namespace recepcionist_cibernots

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<recepcionist_cibernots::OrderDrink>("OrderDrink");
}
