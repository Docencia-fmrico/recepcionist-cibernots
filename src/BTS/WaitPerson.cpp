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

#include "BTS/WaitPerson.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace recepcionist_cibernots
{
using namespace std::chrono_literals;

WaitPerson::WaitPerson(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf),
  tf_buffer_(),
  tf_listener_(tf_buffer_)
{
  config().blackboard->get("node", node_);
}

BT::NodeStatus
WaitPerson::tick()
{
  // If exists the tf, pass to the next and if not, search the person
  geometry_msgs::msg::TransformStamped odom2person_msg;
  tf2::Stamped<tf2::Transform> odom2person;

  // Search for the tf of the person
  try {
    odom2person_msg = tf_buffer_.lookupTransform(
      "base_link", "detected_person",
      tf2::TimePointZero);
    tf2::fromMsg(odom2person_msg, odom2person);
    RCLCPP_INFO(node_->get_logger(), "TRANSFORMADA ENCONTRADA:%s", odom2person_msg.child_frame_id.c_str());
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(node_->get_logger(), "person transform not found: %s", ex.what());
    return BT::NodeStatus::RUNNING;
  }
  if ((node_->now() - rclcpp::Time(odom2person_msg.header.stamp)) > TF_PERSON_TIMEOUT){
    RCLCPP_WARN(node_->get_logger(), "person transform not found");
    return BT::NodeStatus::RUNNING;
  }

  // Calculate distance using pythagoras theorem
  double distance = sqrt(odom2person.getOrigin().x()*odom2person.getOrigin().x() + odom2person.getOrigin().y()*odom2person.getOrigin().y());

  // If the distance is less than 1.5, the person is there
  if (std::abs(distance) <= 1.5) {
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

}  // namespace recepcionist_cibernots

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<recepcionist_cibernots::WaitPerson>("WaitPerson");
}