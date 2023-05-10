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

#ifndef BTS__SENDCHAIR_HPP_
#define BTS__SENDCHAIR_HPP_

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "rclcpp/rclcpp.hpp"
#include "kobuki_ros_interfaces/msg/sound.hpp"
#include "std_msgs/msg/string.hpp"

namespace recepcionist_cibernots
{

class SendChair : public BT::ActionNodeBase
{
public:
  explicit SendChair(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void halt() {}
  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList({});
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ch_pub_;
  rclcpp::Node::SharedPtr node_;

  // buffer and listener for tf
  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  int counter_;
  std::string chairs_[4] = {"chair1","chair2","chair3","chair4"};
  int angs_chairs_[4];
};

}  // namespace recepcionist_cibernots

#endif  // BT_NODES__SENDCHAIR_HPP_
