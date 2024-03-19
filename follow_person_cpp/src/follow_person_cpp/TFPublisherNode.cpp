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

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "follow_person_cpp/TFPublisherNode.hpp"

#include "tf2/transform_datatypes.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "vision_msgs/msg/detection3_d_array.hpp"

namespace follow_person_cpp
{

using std::placeholders::_1;
using namespace std::chrono_literals;

TFPublisherNode::TFPublisherNode()
: Node ("tf_node"),
  tf_buffer_(),
  tf_listener_(tf_buffer_)
{
  detection_sub_ = create_subscription<vision_msgs::msg::Detection3DArray>(
    "input_detection", rclcpp::SensorDataQoS().reliable(),
    std::bind(&TFPublisherNode::detection_callback, this, _1));
  
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

}

void
TFPublisherNode::detection_callback(vision_msgs::msg::Detection3DArray::UniquePtr msg)
{
  tf2::Transform camera2person;
  camera2person.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
  camera2person.setRotation(tf2::Quaternion(0, 0, 0, 1));

  geometry_msgs::msg::TransformStamped odom2camera_msg;
  tf2::Stamped<tf2::Transform> odom2camera;

  float len = std::size(msg->detections);
  person_ = false;

  for (int i = 0; i < len; i++) {

    if (msg->detections[i].results[0].hypothesis.class_id == "person") {
      
      RCLCPP_INFO(get_logger(), "detected_person");
      person_ = true;

      float x = msg->detections[i].bbox.center.position.x,
        y = msg->detections[i].bbox.center.position.y,
        z = msg->detections[i].bbox.center.position.z;

      camera2person.setOrigin(tf2::Vector3(x, y, z));

      try {
        odom2camera_msg = tf_buffer_.lookupTransform(
          "odom",   "camera_depth_optical_frame",
          tf2::timeFromSec(rclcpp::Time(msg->header.stamp).seconds()));
        tf2::fromMsg(odom2camera_msg, odom2camera);
        RCLCPP_INFO(get_logger(), "Make transform");
      } catch (tf2::TransformException & ex) {
        RCLCPP_WARN(get_logger(), "Person transform not found: %s", ex.what());
        return;
      }

      tf2::Transform odom2person = odom2camera * camera2person;

      geometry_msgs::msg::TransformStamped odom2person_msg;
      odom2person_msg.transform = tf2::toMsg(odom2person);

      odom2person_msg.header.stamp = msg->header.stamp;
      odom2person_msg.header.frame_id = "odom";
      odom2person_msg.child_frame_id = "person";

      tf_broadcaster_->sendTransform(odom2person_msg);
    }
  }
}

bool 
TFPublisherNode::isPersonDetected()
{
  return person_;
}

}  // namespace follow_person_cpp
