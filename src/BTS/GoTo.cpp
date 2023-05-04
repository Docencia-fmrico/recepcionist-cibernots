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

#include "BTS/GoTo.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "rclcpp/rclcpp.hpp"

namespace recepcionist_cibernots
{
GoTo::GoTo(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: recepcionist_cibernots::BtActionNode<nav2_msgs::action::NavigateToPose>(xml_tag_name, action_name,
    conf)
{
}

void
GoTo::on_tick()
{
  geometry_msgs::msg::PoseStamped goal;
  getInput("Point", goal);

  switch goal
  {
    /* 
        cambiar el tener aquí las coordenadas explicitamente
        por extraerlas del yaml
    */
    case "Door":
        goal.pose.position.x = -0.481;
        goal.pose.position.y = 3.09;

    case "Party":
        wp.pose.position.x = -22.6;
        goal.pose.position.y = 9.63;

    case "Barman":
        goal.pose.position.x = -31;
        goal.pose.position.y = -8.87;

    case "Person":
        /*
            Publicar en IndicateChair las coordenadas de la silla indicada
            y suscribirnos aqui al topic donde se publique.
            En desarrollo
        */
  }

  goal_.pose = goal;
}

BT::NodeStatus
GoTo::on_success()
{
  RCLCPP_INFO(node_->get_logger(), "navigation Suceeded");

  return BT::NodeStatus::SUCCESS;
}

}  // namespace recepcionist_cibernots

#include "behaviortree_cpp_v3/bt_factory.h"
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<recepcionist_cibernots::GoTo>(
        name, "navigate_to_pose", config);
    };

  factory.registerBuilder<recepcionist_cibernots::GoTo>(
    "GoTo", builder);
}
