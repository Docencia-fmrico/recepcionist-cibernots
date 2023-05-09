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

#ifndef TFS__PUB_TF_CHAIRS_HPP_
#define TFS__PUB_TF_CHAIRS_HPP_

#include <tf2_ros/static_transform_broadcaster.h>

#include <memory>

#include "rclcpp/rclcpp.hpp"

namespace tfs
{

class PubTfChairs : public rclcpp::Node
{
public:
  PubTfChairs();

private:
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
  std::string name_chairs_[4] = {"Chair1","Chair2","Chair3","Chair4"};
};

}  // namespace tfs
#endif  // TF2_DETECTOR__OBJECTDETECTORIMPROVEDNODE_HPP_
