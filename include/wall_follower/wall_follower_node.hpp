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

/// \copyright Copyright 2021 The Autoware Foundation
/// \file
/// \brief This file defines the wall_follower_node class.

#ifndef WALL_FOLLOWER__WALL_FOLLOWER_NODE_HPP_
#define WALL_FOLLOWER__WALL_FOLLOWER_NODE_HPP_

#include <wall_follower/wall_follower.hpp>

#include <rclcpp/rclcpp.hpp>

namespace autoware
{
namespace laboratories
{
namespace wall_follower
{
using WallFollowerPtr = std::unique_ptr<autoware::laboratories::wall_follower::WallFollower>;
using ControlCommand = lgsvl_msgs::msg::VehicleControlData;
using LaserScan = sensor_msgs::msg::LaserScan;

/// \class WallFollowerNode
/// \brief ROS 2 Node for PID vehicle control.
class WALL_FOLLOWER_PUBLIC WallFollowerNode : public rclcpp::Node
{
public:
  /// \brief default constructor, starts driver
  /// \throw runtime error if failed to start threads or configure driver
  explicit WallFollowerNode(const rclcpp::NodeOptions & options);

private:
  WallFollowerPtr m_wall_follower{nullptr};
  rclcpp::Publisher<ControlCommand>::SharedPtr m_control_pub{};
  rclcpp::Subscription<LaserScan>::SharedPtr m_laser_scan_sub{};

};
}  // namespace wall_follower
}  // namespace laboratories
}  // namespace autoware

#endif  // WALL_FOLLOWER__WALL_FOLLOWER_NODE_HPP_
