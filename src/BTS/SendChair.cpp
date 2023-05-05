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

#include "BTS/SendChair.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "std_msgs/msg/string.hpp"
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


SendChair::SendChair(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf),
  tf_buffer_(),
  tf_listener_(tf_buffer_),
  counter_(0)
{
  config().blackboard->get("node", node_);
  ch_pub_ = node_->create_publisher<std_msgs::msg::String>("/chairs", 10);
}

BT::NodeStatus
SendChair::tick()
{
  geometry_msgs::msg::TransformStamped odom2chair_msg;
  tf2::Stamped<tf2::Transform> odom2chair;

  for (int i = 0; i < 4;)
  {
    // Search for the tf
    try {
        odom2chair_msg = tf_buffer_.lookupTransform(
        "base_link", chairs_[i],
        tf2::TimePointZero);
        tf2::fromMsg(odom2chair_msg, odom2chair);
        angs_chairs_[i] = std::atan2(odom2chair.getOrigin().y(), fabs(odom2chair.getOrigin().x()));
        i++;
    } catch (tf2::TransformException & ex) {
        RCLCPP_WARN(node_->get_logger(), "SendCHAIR: chair transform not found: %s", ex.what());
    }
  }
  
  int menor = angs_chairs_[0]; // Variable para almacenar el número menor
  int indice = 0; // Variable para almacenar el índice del número menor

  // Iteramos por el arreglo buscando el número menor y su índice
  for (int i = 1; i < 6; i++) {
    if (angs_chairs_[i] < menor) {
      menor = angs_chairs_[i];
      indice = i;
    }
  }

  std_msgs::msg::String chair;
  
  chair.data = chairs_[indice];

  ch_pub_->publish(chair);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace recepcionist_cibernots

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<recepcionist_cibernots::SendChair>("SendChair");
}