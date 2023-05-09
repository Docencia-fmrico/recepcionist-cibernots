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
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include <tf2/LinearMath/Quaternion.h>

#include "rclcpp/rclcpp.hpp"


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("recepcionist");

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  factory.registerFromPlugin(loader.getOSName("bt_goTo_node"));
  // factory.registerFromPlugin(loader.getOSName("bt_waitPerson_node"));
  // factory.registerFromPlugin(loader.getOSName("bt_ask_node"));
  // factory.registerFromPlugin(loader.getOSName("bt_introduce_node"));
  // factory.registerFromPlugin(loader.getOSName("bt_ifChair_node"));
  // factory.registerFromPlugin(loader.getOSName("bt_searchChair_node"));
  // factory.registerFromPlugin(loader.getOSName("bt_indicateChair_node"));
  // factory.registerFromPlugin(loader.getOSName("bt_sendChair_node"));

  std::string pkgpath = ament_index_cpp::get_package_share_directory("recepcionist_cibernots");
  std::string xml_file = pkgpath + "/behavior_tree_xml/Goto.xml";
  // std::string xml_file = pkgpath + "/behavior_tree_xml/bt_recepcionist.xml";

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);

  geometry_msgs::msg::PoseStamped wp;
  tf2::Quaternion myQuaternion;
  wp.header.frame_id = "map";
  wp.pose.orientation.w = 1.0;

  // inicio
  wp.pose.position.x = 0.0;
  wp.pose.position.y = 0.0;
  

  // puerta
  wp.pose.position.x = 6.4;
  wp.pose.position.y = -0.95;
  myQuaternion.setRPY(0.0,0.0,(-3.50/2));
  myQuaternion=myQuaternion.normalize();
  wp.pose.orientation.x = myQuaternion[0];
  wp.pose.orientation.y = myQuaternion[1];
  wp.pose.orientation.z = myQuaternion[2];
  wp.pose.orientation.w = myQuaternion[3];
  blackboard->set("Door",wp );
  
  // Party
  wp.pose.position.x = 0.32;
  wp.pose.position.y = 3.43;
  myQuaternion.setRPY(0.0,0.0,(-3.1/4));
  myQuaternion=myQuaternion.normalize();
  wp.pose.orientation.x = myQuaternion[0];
  wp.pose.orientation.y = myQuaternion[1];
  wp.pose.orientation.z = myQuaternion[2];
  wp.pose.orientation.w = myQuaternion[3];
  blackboard->set("Party",wp );

  // chair1
  wp.pose.position.x = 0.32;
  wp.pose.position.y = 3.43;
  myQuaternion.setRPY(0.0,0.0,(-3.1/4));
  myQuaternion=myQuaternion.normalize();
  wp.pose.orientation.x = myQuaternion[0];
  wp.pose.orientation.y = myQuaternion[1];
  wp.pose.orientation.z = myQuaternion[2];
  wp.pose.orientation.w = myQuaternion[3];
  blackboard->set("chair1",wp );

  // chair2
  wp.pose.position.x = 0.32;
  wp.pose.position.y = 3.43;
  myQuaternion.setRPY(0.0,0.0,(-3.64/2));
  myQuaternion=myQuaternion.normalize();
  wp.pose.orientation.x = myQuaternion[0];
  wp.pose.orientation.y = myQuaternion[1];
  wp.pose.orientation.z = myQuaternion[2];
  wp.pose.orientation.w = myQuaternion[3];
  blackboard->set("chair2",wp );
  
  // chair3
  wp.pose.position.x = -0.217;
  wp.pose.position.y = 2.83;
  myQuaternion.setRPY(0.0,0.0,(-3.64/2));
  myQuaternion=myQuaternion.normalize();
  wp.pose.orientation.x = myQuaternion[0];
  wp.pose.orientation.y = myQuaternion[1];
  wp.pose.orientation.z = myQuaternion[2];
  wp.pose.orientation.w = myQuaternion[3];
  blackboard->set("chair3",wp );
  
  // chair4
  wp.pose.position.x = -0.217;
  wp.pose.position.y = 2.83;
  myQuaternion.setRPY(0.0,0.0,(-3.64/2));
  myQuaternion=myQuaternion.normalize();
  wp.pose.orientation.x = myQuaternion[0];
  wp.pose.orientation.y = myQuaternion[1];
  wp.pose.orientation.z = myQuaternion[2];
  wp.pose.orientation.w = myQuaternion[3];
  blackboard->set("chair4",wp );
  
  // Barman
  wp.pose.position.x = 0.248;
  wp.pose.position.y = 6.64;
  myQuaternion.setRPY(0.0,0.0,(3.1/2));
  myQuaternion=myQuaternion.normalize();
  wp.pose.orientation.x = myQuaternion[0];
  wp.pose.orientation.y = myQuaternion[1];
  wp.pose.orientation.z = myQuaternion[2];
  wp.pose.orientation.w = myQuaternion[3];
  blackboard->set("Barman", wp);

  BT::Tree tree = factory.createTreeFromFile(xml_file, blackboard);

  auto publisher_zmq = std::make_shared<BT::PublisherZMQ>(tree, 10, 1666, 1667);

  rclcpp::Rate rate(10);

  bool finish = false;
  while (!finish && rclcpp::ok()) {
    finish = tree.rootNode()->executeTick() != BT::NodeStatus::RUNNING;

    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
