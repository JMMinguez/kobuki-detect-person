// Copyright 2024 Intelligent Robotics Lab
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

#ifndef FOLLOW_PERSON_CPP__TF_PUBLISHER_HPP_
#define FOLLOW_PERSON_CPP__TF_PUBLISHER_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "vision_msgs/msg/detection3_d_array.hpp"

namespace follow_person_cpp
{
class TFPublisherNode : public rclcpp::Node
{
public:
  TFPublisherNode();
  
  bool isPersonDetected();

private:
  void detection_callback(vision_msgs::msg::Detection3DArray::UniquePtr msg);
  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr detection_sub_;
  vision_msgs::msg::Detection3DArray::UniquePtr msg;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  bool person_;
};

}  // // namespace follow_person_cpp

#endif  // FOLLOW_PERSON_CPP__TF_PUBLISHER_HPP_