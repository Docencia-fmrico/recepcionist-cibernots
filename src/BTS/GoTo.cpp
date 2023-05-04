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
    conf),
    current_(0)
  {
  geometry_msgs::msg::PoseStamped wp;
  wp.header.frame_id = "map";
  wp.pose.orientation.w = 1.0;

  // wp1
  wp.pose.position.x = 7.68;
  wp.pose.position.y = -1.25;
  wp_.push_back(wp);

  // wp2
  wp.pose.position.x = 2.53;
  wp.pose.position.y = 5.62;
  wp_.push_back(wp);

  // wp3
  wp.pose.position.x = 1.17;
  wp.pose.position.y = 6.5;
  wp_.push_back(wp);
  }

geometry_msgs::msg::PoseStamped 
GoTo::getCheckpoint() {
  if (static_cast<size_t>(current_) >= wp_.size()) {
    throw std::runtime_error("No more waypoints available.");
  }
  return wp_[current_++];
}

void
GoTo::on_tick()
{
  geometry_msgs::msg::PoseStamped goal;
  goal = getCheckpoint();

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
BT_REGISTER_NODES(factory)
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
