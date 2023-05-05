// Copyright 2021 Intelligent Robotics Lab
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

#include "BTS/IfChair.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>

#include "rclcpp/rclcpp.hpp"

namespace recepcionist_cibernots
{
using namespace std::chrono_literals;

IfChair::IfChair(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf),
  tf_buffer_(),
  tf_listener_(tf_buffer_)
{
  config().blackboard->get("node", node_);
  vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("output_vel", 100);
}

BT::NodeStatus
IfChair::tick()
{ 
  // If exists the tf, pass to the next and if not, search the object

  geometry_msgs::msg::TransformStamped base2obj_msg;
  tf2::Stamped<tf2::Transform> base2obj;
  RCLCPP_INFO(node_->get_logger(), "IfCHAIR: BUSCANDO TRANSFORMADA detected_obj");
  // Search for the tf of the object
  try {
    base2obj_msg = tf_buffer_.lookupTransform(
      "base_link", "detected_obj",
      tf2::TimePointZero);
    tf2::fromMsg(base2obj_msg, base2obj);
    RCLCPP_INFO(node_->get_logger(), "IfCHAIR: TRANSFORMADA ENCONTRADA:%s", base2obj_msg.child_frame_id.c_str());
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(node_->get_logger(), "IfCHAIR: object transform not found: %s", ex.what());
    return BT::NodeStatus::FAILURE;
  }
  if ((node_->now() - rclcpp::Time(base2obj_msg.header.stamp)) > TF_CHAIR_TIMEOUT){
    RCLCPP_WARN(node_->get_logger(), "IfCHAIR: SEGUNDO IF object transform not found");
    return BT::NodeStatus::FAILURE;
  }
  RCLCPP_INFO(node_->get_logger(),"IfCHAIR: DEVUELVO SUCCESS, TS: %f", (node_->now() - rclcpp::Time(base2obj_msg.header.stamp)).seconds());
  geometry_msgs::msg::Twist out_vel;
  out_vel.angular.z = 0.0f;

  vel_pub_->publish(out_vel);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace recepcionist_cibernots

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<recepcionist_cibernots::IfChair>("IfChair");
}