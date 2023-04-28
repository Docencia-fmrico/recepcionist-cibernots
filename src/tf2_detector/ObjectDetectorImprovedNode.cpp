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

#include "tf2_detector/ObjectDetectorImprovedNode.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "rclcpp/rclcpp.hpp"

namespace tf2_detector
{

using std::placeholders::_1;
using namespace std::chrono_literals;

ObjectDetectorImprovedNode::ObjectDetectorImprovedNode()
: Node("object_detector_improved"),
  tf_buffer_(),
  tf_listener_(tf_buffer_)
{
    detection_sub_ = create_subscription<vision_msgs::msg::Detection3DArray>(
    "input_detection_3d", rclcpp::SensorDataQoS(),
    std::bind(&ObjectDetectorImprovedNode::image3D_callback, this, _1));

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

}

void
ObjectDetectorImprovedNode::image3D_callback(vision_msgs::msg::Detection3DArray::UniquePtr detection3D_msg)
{

  // Publish a transform in the position of each obj detected through bounding boxes
  for (const auto & obj : detection3D_msg->detections) {
    tf2::Transform camera2obj;
    /* In z, the distance at which the robot is from the obj is detected, that is, if you approach it decreases, if you approach it increases.*/
    camera2obj.setOrigin(tf2::Vector3(obj.bbox.center.position.x, obj.bbox.center.position.y, obj.bbox.center.position.z));
    camera2obj.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
    geometry_msgs::msg::TransformStamped odom2camera_msg;
    tf2::Stamped<tf2::Transform> odom2camera;
    try {
      odom2camera_msg = tf_buffer_.lookupTransform(
        "odom", detection3D_msg->header.frame_id.c_str(),
        tf2::timeFromSec(rclcpp::Time(detection3D_msg->header.stamp).seconds()));
      tf2::fromMsg(odom2camera_msg, odom2camera);
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(), "Camera transform not found: %s", ex.what());
      return;
    }

    // Transform from odom to obj
    tf2::Transform odom2obj = odom2camera * camera2obj;

    // Publish the transform
    geometry_msgs::msg::TransformStamped odom2obj_msg;
    odom2obj_msg.transform = tf2::toMsg(odom2obj);

    odom2obj_msg.header.stamp = detection3D_msg->header.stamp;
    odom2obj_msg.header.frame_id = "odom";
    odom2obj_msg.child_frame_id = "detected_obj";
    tf_broadcaster_->sendTransform(odom2obj_msg);

  }
}

}  // namespace tf2_detector
