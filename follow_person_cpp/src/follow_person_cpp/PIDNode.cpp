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

#include <algorithm>

#include "follow_person_cpp/PIDNode.hpp"
#include "follow_person_cpp/FollowLifeCycle.hpp"

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_msgs/msg/tf_message.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;


namespace follow_person_cpp
{
PIDNode::PIDNode(double min_ref, double max_ref, double min_output, double max_output)
: Node ("pid_node"),
  lin_pid_(new PIDNode(0.0, 5.0, 0.0, 0.5)),
  ang_pid_(new PIDNode(0.0, M_PI / 2, 0.0, 0.5)),
  tf_buffer_(),
  tf_listener_(tf_buffer_)
{

  min_ref_ = min_ref;
  max_ref_ = max_ref;
  min_output_ = min_output;
  max_output_ = max_output;
  prev_error_ = int_error_ = 0.0;

  declare_parameter("KP", 0.41);
  declare_parameter("KI", 0.0);
  declare_parameter("KD", -0.25);

  get_parameter("KP", KP_);
  get_parameter("KI", KI_);
  get_parameter("KD", KD_);

  vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  transform_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
    "tf_static", rclcpp::SensorDataQoS().reliable(),
    std::bind(&PIDNode::transform_callback, this, _1));
}

void
PIDNode::set_pid(double n_KP, double n_KI, double n_KD)
{
  KP_ = n_KP;
  KI_ = n_KI;
  KD_ = n_KD;
}

double
PIDNode::get_output(double new_reference)
{
  double ref = new_reference;
  double output = 0.0;

  // Proportional Error
  double direction = 0.0;
  if (ref != 0.0) {
    direction = ref / fabs(ref);
  }

  if (fabs(ref) < min_ref_) {
    output = 0.0;
  } else if (fabs(ref) > max_ref_) {
    output = direction * max_output_;
  } else {
    output = direction * min_output_ + ref * (max_output_ - min_output_);
  }

  // Integral Error
  int_error_ = (int_error_ + output) * 2.0 / 3.0;

  // Derivative Error
  double deriv_error = output - prev_error_;
  prev_error_ = output;

  output = KP_ * output + KI_ * int_error_ + KD_ * deriv_error;

  return std::clamp(output, -max_output_, max_output_);
}

void
PIDNode::transform_callback(const tf2_msgs::msg::TFMessage::ConstSharedPtr & msg)
{
  lin_pid_->set_pid(0.6, 0.05, 0.35);
  ang_pid_->set_pid(0.6, 0.08, 0.32);


  RCLCPP_INFO(this->get_logger(), "Transform received");

  for (int i = 0; i < std::size(msg->transforms); i++) {
    fprintf(
      stderr, "Received transform: %f, %f, %f, %f",
      msg->transforms[i].transform.translation.x, msg->transforms[i].transform.translation.y,
      msg->transforms[i].transform.rotation.x, msg->transforms[i].transform.rotation.y);

    geometry_msgs::msg::Twist vel;
    geometry_msgs::msg::TransformStamped odom2person_msg;

    try {
      odom2person_msg = tf_buffer_.lookupTransform(
        "base_link", "person",
        tf2::timeFromSec(rclcpp::Time(odom2person_msg.header.stamp).seconds()));
      RCLCPP_INFO(get_logger(), "Cambio de Base_link");
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(), "Person transform not found: %s", ex.what());
      return;
    }

    const auto & x = odom2person_msg.transform.translation.x;
    const auto & y = odom2person_msg.transform.translation.y;

    // Cálculo de distancia y ángulo
    double distancia = sqrt(pow(x, 2) + pow(y, 2) );
    double angulo = atan2(y, x);

    // Si la distancia es menor de 1m se para
    if (distancia <= 0.9) {   //distancia menor o igual a un metro
      RCLCPP_INFO(get_logger(), "Persona encontrada");
      vel.linear.x = 0;
      vel.angular.z = 0;
      vel_pub_->publish(vel);
      return;
    }

    else if (distancia >= 1){
      // Sino se establecen v.angular y v.lineal
      vel.angular.z = ang_pid_->get_output(angulo);
      RCLCPP_INFO(
        this->get_logger(), "Velocidad angular(x:%f,y:%f =%f): %f", x, y, 
        atan2(y,x ),
        vel.angular.z);
      vel.linear.x = lin_pid_->get_output(distancia);
      RCLCPP_INFO(this->get_logger(), "Velocidad lineal: %f", vel.linear.x);

      // Se publican velocidades
      vel_pub_->publish(vel);

    }
    else {
      RCLCPP_INFO(get_logger(), "Buscando personas");
      vel.angular.z = 0.2;
      vel_pub_->publish(vel);
    }
    //  out_sound.value = kobuki_ros_interfaces::msg::Sound::OFF;
    
  }

const rclcpp_lifecycle::State PIDNode::get_current_state() const
{
  return this->get_current_state();
}
}
}  // namespace follow_person_cpp