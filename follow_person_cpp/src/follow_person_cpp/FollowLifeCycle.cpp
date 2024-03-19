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

#include "rclcpp/rclcpp.hpp"
#include "follow_person_cpp/FollowLifeCycle.hpp"
#include "follow_person_cpp/PIDNode.hpp"
#include "follow_person_cpp/TFPublisherNode.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace follow_person_cpp
{

FollowLifeCycle::FollowLifeCycle()
: rclcpp_lifecycle::LifecycleNode("follow_life_cycle"),
  pid_node(std::make_shared<PIDNode>()),
  tf_node(std::make_shared<TFPublisherNode>())
{
  timer_ = create_wall_timer(
    100ms, std::bind(&FollowLifeCycle::checkPersonDetection, this));
}

void FollowLifeCycle::checkPersonDetection()
{
  if (pid_node->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE && !tf_node->isPersonDetected())
  {
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE;
    control_activation_client_->async_send_request(request);
  }
}

FollowLifeCycle::CallbackReturn
FollowLifeCycle::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(get_logger(), "Configuring");
  return CallbackReturn::SUCCESS;
}

FollowLifeCycle::CallbackReturn
FollowLifeCycle::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(get_logger(), "Activating");
  return CallbackReturn::SUCCESS;
}

FollowLifeCycle::CallbackReturn
FollowLifeCycle::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(get_logger(), "Deactivating");
  return CallbackReturn::SUCCESS;
}

FollowLifeCycle::CallbackReturn
FollowLifeCycle::on_cleanup(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");
  return CallbackReturn::SUCCESS;
}

FollowLifeCycle::CallbackReturn
FollowLifeCycle::on_shutdown(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return CallbackReturn::SUCCESS;
}

}  // namespace follow_person_cpp