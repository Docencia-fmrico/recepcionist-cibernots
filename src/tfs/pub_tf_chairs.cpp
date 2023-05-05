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

#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>

#include <memory>

#include "tfs/pub_tf_chairs.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "rclcpp/rclcpp.hpp"

namespace tfs
{

using std::placeholders::_1;
using namespace std::chrono_literals;

PubTfChairs::PubTfChairs()
: Node("PubTfChairs")
{
  tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);

  // Transform from odom to obj
  for (int i = 0; i < 4; i++) {
    tf2::Transform odom2chair;
    odom2chair.setOrigin(tf2::Vector3(2, 4, 0));
    odom2chair.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

    // Publish the transform
    geometry_msgs::msg::TransformStamped odom2chair_msg;
    odom2chair_msg.transform = tf2::toMsg(odom2chair);  
    // odom2chair_msg.header.stamp = detection3D_msg->header.stamp;
    odom2chair_msg.header.frame_id = "map";
    odom2chair_msg.child_frame_id = name_chairs_[i];
    tf_broadcaster_->sendTransform(odom2chair_msg);  
  }
  
}
}  // namespace tfs
