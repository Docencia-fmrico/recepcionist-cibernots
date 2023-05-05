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
#include <memory>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include <tf2_ros/static_transform_broadcaster.h>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "rclcpp/rclcpp.hpp"

void pub_tf_chairs(std::string names[], int n, const rclcpp::Node::SharedPtr node, std::array<tf2::Vector3, 4> positions) {

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);

  for (int i = 0; i < n; i++) {
    tf2::Transform map2chair;
    map2chair.setOrigin(positions[i]);
    map2chair.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

    // Publish the transform
    geometry_msgs::msg::TransformStamped map2chair_msg;
    map2chair_msg.transform = tf2::toMsg(map2chair);  
    map2chair_msg.header.stamp = node->now();
    map2chair_msg.header.frame_id = "map";
    map2chair_msg.child_frame_id = names[i];
    tf_broadcaster_->sendTransform(map2chair_msg);  
  }
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("recepcionist");

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  factory.registerFromPlugin(loader.getOSName("bt_SearchChair_node"));
  factory.registerFromPlugin(loader.getOSName("bt_FindChair_node"));
  factory.registerFromPlugin(loader.getOSName("bt_SendChair_node"));

  std::string pkgpath = ament_index_cpp::get_package_share_directory("recepcionist_cibernots");
  std::string xml_file = pkgpath + "/behavior_tree_xml/bt_visionchair.xml";

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);
  BT::Tree tree = factory.createTreeFromFile(xml_file, blackboard);

  auto publisher_zmq = std::make_shared<BT::PublisherZMQ>(tree, 10, 1666, 1667);

  rclcpp::Rate rate(10);

  // Crea un array de 4 vectores que contienen tres floats
  std::array<tf2::Vector3, 4> position_chairs;

  // Inicializa cada objeto con tres floats
  position_chairs[0] = tf2::Vector3(1.0, 2.0, 0.0);
  position_chairs[1] = tf2::Vector3(4.0, 5.0, 0.0);
  position_chairs[2] = tf2::Vector3(7.0, 8.0, 0.0);
  position_chairs[3] = tf2::Vector3(12.0, 8.0, 0.0);

  std::string name_chairs[4] = {"Chair1","Chair2","Chair3","Chair4"};
  pub_tf_chairs(name_chairs, 4, node, position_chairs);

  bool finish = false;
  while (!finish && rclcpp::ok()) {

    finish = tree.rootNode()->executeTick() == BT::NodeStatus::SUCCESS;

    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
