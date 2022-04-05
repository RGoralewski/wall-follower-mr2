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
/// \brief This file defines the wall_follower class.

#ifndef WALL_FOLLOWER__WALL_FOLLOWER_HPP_
#define WALL_FOLLOWER__WALL_FOLLOWER_HPP_

#include <wall_follower/visibility_control.hpp>
#include <iostream>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <lgsvl_msgs/msg/vehicle_control_data.hpp>
#include "trajectory_follower/pid.hpp"
#include "common/types.hpp"
#include <limits>
#include <cmath>
#include <deque>
#include <numeric>


using PID = autoware::motion::control::trajectory_follower::PIDController;
using autoware::common::types::float64_t;
using autoware::common::types::float32_t;

namespace autoware
{
namespace laboratories
{
namespace wall_follower
{
using LaserScanPtr = sensor_msgs::msg::LaserScan::SharedPtr;
using ControlCommand = lgsvl_msgs::msg::VehicleControlData;

class WALL_FOLLOWER_PUBLIC WallFollower
{
public:
  WallFollower();
  void set_parameters(float64_t kp,
                      float64_t ki,
                      float64_t kd,
                      float64_t min_acc,
                      float64_t max_acc,
                      float64_t min_speed_threshold,
                      float64_t speed_coeff);
  lgsvl_msgs::msg::VehicleControlData on_laser_scan(const LaserScanPtr& msg);

private:
  PID m_pid;
  lgsvl_msgs::msg::VehicleControlData m_command{};
  std::vector<float64_t> m_pid_contributions;

  // Default parameters
  float64_t m_min_acc = 0.1;
  float64_t m_max_acc = 1.0;

  double last_time = 0.0;
  float64_t m_min_speed_threshold = 0.1;
  float64_t m_speed_coeff = 2.0;
};

}  // namespace wall_follower
}  // namespace laboratories
}  // namespace autoware

#endif  // WALL_FOLLOWER__WALL_FOLLOWER_HPP_
