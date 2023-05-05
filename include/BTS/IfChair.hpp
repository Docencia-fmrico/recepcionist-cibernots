// Copyright 2023 Intelligent Robotics Lab
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

#ifndef BTS__IFCHAIR_HPP_
#define BTS__IFCHAIR_HPP_

#include <string>
#include <chrono>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "tf2_ros/transform_broadcaster.h"
// #include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
//#include "tf2_msgs/msg/tf_message.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace recepcionist_cibernots
{
using namespace std::chrono_literals;
class IfChair : public BT::ActionNodeBase
{
public:
  explicit IfChair(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void halt() {}
  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList({});
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Node::SharedPtr node_;
  
  // buffer and listener for tf
  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  const rclcpp::Duration TF_CHAIR_TIMEOUT {1s};
};

}  // namespace recepcionist_cibernots

#endif  // BT_NODES__IfCHAIR_HPP_
