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

#include "wall_follower/wall_follower.hpp"

namespace autoware
{
namespace laboratories
{
namespace wall_follower
{

WallFollower::WallFollower() {
   m_pid.setLimits(1.2, -1.2, 1.2, -1.2, 1.2, -1.2, 1.2, -1.2);
}

void WallFollower::set_parameters(float64_t kp,
                                  float64_t ki,
                                  float64_t kd,
                                  float64_t min_acc,
                                  float64_t max_acc,
                                  float64_t min_speed_threshold,
                                  float64_t speed_coeff)
                                  {
  m_pid.setGains(kp, ki, kd);
  m_min_acc = min_acc;
  m_max_acc = max_acc;
  m_min_speed_threshold = min_speed_threshold;
  m_speed_coeff = speed_coeff;
}


ControlCommand WallFollower::on_laser_scan(const LaserScanPtr &msg) {
  ControlCommand control_msg;

  std::cout << "------------------------------------------------------" << std::endl;

  double time_secs = msg->header.stamp.sec + msg->header.stamp.nanosec / 1e9;
  std::cout << "time diff: " << time_secs - last_time << std::endl;
  last_time = time_secs;

  unsigned left_sum = 0;
  unsigned right_sum = 0;
  for (unsigned i=0; i < unsigned(msg->ranges.size()); i++) {
      float measure = msg->ranges[i];
      if (measure == std::numeric_limits<float>::infinity())
          measure = 10.0;

      if (i < unsigned(msg->ranges.size() / 2))
          right_sum += static_cast<unsigned>(measure);
      else
          left_sum += static_cast<unsigned>(measure);
  }

  std::cout << "left side sum: " << left_sum << std::endl;
  std::cout << "right side sum: " << right_sum << std::endl;

  double error = static_cast<int>(right_sum - left_sum) / double(left_sum + right_sum);
  std::cout << "error: " << error << std::endl;

  std::vector<float64_t> pid_contributions;
  double pid_output = (m_pid.calculate(error, 0.05, false, pid_contributions));

  std::cout << "pid output: " << pid_output << std::endl;

  double speed = m_min_acc;
  if (fabs(error) < m_min_speed_threshold){
      double err_diff = m_min_speed_threshold - fabs(error);
      std::cout << "err diff: " << err_diff << std::endl;
      speed += m_speed_coeff * err_diff;
  }

  std::cout << "speed: " << speed << std::endl;

  control_msg.target_wheel_angle = static_cast<float32_t>(pid_output);
  control_msg.acceleration_pct = static_cast<float32_t>(speed);

  return control_msg;
}

}  // namespace wall_follower
}  // namespace laboratories
}  // namespace autoware
