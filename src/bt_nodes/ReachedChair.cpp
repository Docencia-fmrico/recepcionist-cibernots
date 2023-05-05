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
#include <math.h>
#include <cmath>

#include "bt_nodes/ReachedChair.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>

#include "kobuki_ros_interfaces/msg/sound.hpp"

namespace recepcionist_cibernots
{
using std::placeholders::_1;
using namespace std::chrono_literals;


ReachedChair::ReachedChair(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf),
  tf_buffer_(),
  tf_listener_(tf_buffer_)
{
  config().blackboard->get("node", node_);

  // Sound publisher
  sound_pub_ = node_->create_publisher<kobuki_ros_interfaces::msg::Sound>("output_sound", 10);
}

BT::NodeStatus
ReachedChair::tick()
{

  geometry_msgs::msg::TransformStamped odom2obj_msg;
  tf2::Stamped<tf2::Transform> odom2obj;

  // Search for the tf
  try {
    odom2obj_msg = tf_buffer_.lookupTransform(
      "base_link", "detected_obj",
      tf2::TimePointZero);
    tf2::fromMsg(odom2obj_msg, odom2obj);
    kobuki_ros_interfaces::msg::Sound msg;
    msg.value = kobuki_ros_interfaces::msg::Sound::CLEANINGEND;
    
    sound_pub_->publish(msg);
    return BT::NodeStatus::SUCCESS;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(node_->get_logger(), "REACHEDCHAIR: obj transform not found: %s", ex.what());
    return BT::NodeStatus::FAILURE;
  }

  // Calculate distance using pythagoras theorem
  double distance = sqrt(odom2obj.getOrigin().x()*odom2obj.getOrigin().x() + odom2obj.getOrigin().y()*odom2obj.getOrigin().y());
  RCLCPP_INFO(node_->get_logger(),"REACHEDCHAIR: DISTANCIA A LA PERSONA: %f ## Condicion: %d", distance, (std::abs(distance) <= 1.25));
  // If the distance is less than 1.35, the obj is reached
  if (std::abs(distance) <= 1.35) {
    // Send sound
    kobuki_ros_interfaces::msg::Sound msg;
    msg.value = kobuki_ros_interfaces::msg::Sound::CLEANINGEND;
    
    sound_pub_->publish(msg);
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

}  // namespace recepcionist_cibernots

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<recepcionist_cibernots::ReachedChair>("ReachedChair");
}