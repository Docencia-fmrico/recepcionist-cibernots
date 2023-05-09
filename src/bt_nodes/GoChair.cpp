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

#include <string>
#include <iostream>
#include <math.h>
#include <cmath>

#include "bt_nodes/GoChair.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>



namespace recepcionist_cibernots
{
using namespace std::chrono_literals;

GoChair::GoChair(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf),
  // Create PID controllers with min/max values of reference and output
  linear_pid_(0.1, 0.7, 0.0, 0.5),
  angular_pid_(0.0, M_PI_2, 0.5, 2.0),
  tf_buffer_(),
  tf_listener_(tf_buffer_)
{
  config().blackboard->get("node", node_);

  vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("output_vel", 10);
  
  // Set PID kp, ki and kd erors.
  angular_pid_.set_pid(0.6, 0.08, 0.32);
  
}

BT::NodeStatus
GoChair::tick()
{
  geometry_msgs::msg::TransformStamped base2obj_msg;
  tf2::Stamped<tf2::Transform> base2obj;

  // Search for the obj tf
  try {
    base2obj_msg = tf_buffer_.lookupTransform(
      "base_link", "detected_obj",
      tf2::TimePointZero);
    tf2::fromMsg(base2obj_msg, base2obj);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(node_->get_logger(), "GOCHAIR: obj transform not found: %s", ex.what());
    return BT::NodeStatus::RUNNING;
  }
  RCLCPP_INFO(node_->get_logger(), "GOCHAIR: VOY A LA SILLA");
  // Calculate the distance to the obj using pitagoras theorem
  double distance = sqrt(base2obj.getOrigin().x()*base2obj.getOrigin().x() + base2obj.getOrigin().y()*base2obj.getOrigin().y());
  
  // Calculate the linear velocity using the PID controller
  double linear_vel = linear_pid_.get_output(distance);
  
  // Calculate the angular error using the arc tangent
  auto err_ang = std::atan2(base2obj.getOrigin().y(), fabs(base2obj.getOrigin().x()));

  // Calculate the angular velocity using the PID controller
  double angular_vel = angular_pid_.get_output(err_ang);

  
  geometry_msgs::msg::Twist vel_msgs;

  // Publish the linear and angular velocities
  vel_msgs.linear.x = linear_vel;
  vel_msgs.angular.z = angular_vel;
  
  // vel_pub_->publish(vel_msgs);

  return BT::NodeStatus::RUNNING;
}


}  // namespace recepcionist_cibernots

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<recepcionist_cibernots::GoChair>("GoChair");
}
