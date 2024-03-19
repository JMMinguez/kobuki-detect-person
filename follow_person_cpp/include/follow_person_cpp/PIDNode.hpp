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

#ifndef FOLLOW_PERSON_CPP__PID_HPP_
#define FOLLOW_PERSON_CPP__PID_HPP_

#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "follow_person_cpp/FollowLifeCycle.hpp"


#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace follow_person_cpp
{

class PIDNode : public rclcpp::Node
{
public:
  PIDNode(double min_ref, double max_ref, double min_output, double max_output);

  void set_pid(double n_KP, double n_KI, double n_KD);
  double get_output(double new_reference);
  void transform_callback(const tf2_msgs::msg::TFMessage::ConstSharedPtr & msg);
  const rclcpp_lifecycle::State get_current_state() const;

private:
  double KP_, KI_, KD_;

  double min_ref_, max_ref_;
  double min_output_, max_output_;
  double prev_error_, int_error_;
  double min_ref;
  double max_ref;
  double min_output; 
  double max_output;

  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr transform_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  geometry_msgs::msg::Twist vel;

  //rclcpp::TimerBase::SharedPtr timer_;

  PIDNode* lin_pid_;
  PIDNode* ang_pid_;

  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

}  // namespace follow_person_cpp

#endif  //  FOLLOW_PERSON_CPP__PID_HPP_