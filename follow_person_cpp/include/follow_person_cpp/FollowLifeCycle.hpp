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

#ifndef FOLLOW_PERSON_CPP__FOLLOW_LIFE_CYCLE_HPP_
#define FOLLOW_PERSON_CPP__FOLLOW_LIFE_CYCLE_HPP_

#include "rclcpp/node.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/macros.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "follow_person_cpp/PIDNode.hpp"
#include "follow_person_cpp/TFPublisherNode.hpp"

#include "std_msgs/msg/int32.hpp"

namespace follow_person_cpp
{
  class PIDNode;

class FollowLifeCycle : public rclcpp_lifecycle::LifecycleNode
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(FollowLifeCycle)
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  FollowLifeCycle();
  void checkPersonDetection();

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state);
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state);
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state);

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state);
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state);
  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state);

  double min_ref;
  double max_ref;
  double min_output;
  double max_output;

private:
  std::shared_ptr<PIDNode> pid_node;
  std::shared_ptr<TFPublisherNode> tf_node;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr control_activation_client_;
  const rclcpp_lifecycle::State get_current_state() const;
};

}  // namespace follow_person_cpp

#endif  // FOLLOW_PERSON_CPP__FOLLOW_LIFE_CYCLE_HPP_