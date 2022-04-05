// Copyright 2021 The Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#include "wall_follower/wall_follower_node.hpp"

namespace autoware
{
namespace laboratories
{
namespace wall_follower
{

WallFollowerNode::WallFollowerNode(const rclcpp::NodeOptions & options)
:  Node("wall_follower", options),
   m_control_pub{create_publisher<ControlCommand>(
       "ctrl_cmd", rclcpp::QoS{20})},
   m_laser_scan_sub{create_subscription<LaserScan>(
       "scan", rclcpp::QoS{20},
       [this](const LaserScan::SharedPtr msg) {
         ControlCommand control_msg = m_wall_follower->on_laser_scan(msg);
         m_control_pub->publish(control_msg);
       })}
{
  const auto kp = declare_parameter("pid.kp").get<float64_t>();
  const auto ki = declare_parameter("pid.ki").get<float64_t>();
  const auto kd = declare_parameter("pid.kd").get<float64_t>();
  const auto min_acc = declare_parameter("min_acc").get<float64_t>();
  const auto max_acc = declare_parameter("max_acc").get<float64_t>();
  const auto min_speed_threshold = declare_parameter("min_speed_threshold").get<float64_t>();
  const auto speed_coeff = declare_parameter("speed_coeff").get<float64_t>();
  m_wall_follower = std::make_unique<wall_follower::WallFollower>();
  m_wall_follower->set_parameters(kp,
                                  ki,
                                  kd,
                                  min_acc,
                                  max_acc,
                                  min_speed_threshold,
                                  speed_coeff);
}

}  // namespace wall_follower
}  // namespace labs
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"

// This acts as an entry point, allowing the component to be
// discoverable when its library is being loaded into a running process
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::laboratories::wall_follower::WallFollowerNode)
