/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <memory>

#include "ros1_rmf_gazebo/utils.h"
#include <ros1_rmf_gazebo/door_common.h>

using namespace std::chrono_literals;

namespace rmf_building_sim_common {

void DoorCommon::logger() { std::cout << "door_ " << _state.door_name << "\n"; }

std::vector<std::string> DoorCommon::joint_names() const {
  std::vector<std::string> joint_names;
  for (const auto &door : _doors)
    joint_names.push_back(door.first);

  return joint_names;
}

MotionParams &DoorCommon::params() { return _params; }

DoorMode DoorCommon::requested_mode() const { return _request.requested_mode; }

void DoorCommon::publish_state(const uint32_t door_value,
                               const ros::Time &time) {
  if (!_initialized)
    return;

  _state.current_mode.value = door_value;
  _state.door_time = time;
  _door_state_pub.publish(_state);
}

void DoorCommon::doorRequestCallback(const DoorRequest::ConstPtr &msg) {
  if (msg->door_name == _state.door_name)
    _request = *msg;
}

DoorCommon::DoorCommon(const std::string &door_name, ros::NodeHandle &node,
                       const MotionParams &params, const Doors &doors)
    : _params(params), _doors(doors) {
  _state.door_name = door_name;
  _request.requested_mode.value = DoorMode::MODE_CLOSED;

  _door_state_pub = node.advertise<DoorState>("/door_states", 10);
  _door_request_sub = node.subscribe<DoorRequest>(
      "/door_requests", 10, &DoorCommon::doorRequestCallback, this);

  _initialized = true;
}

double DoorCommon::calculate_target_velocity(const double target,
                                             const double current_position,
                                             const double current_velocity,
                                             const double dt) {
  double dx = target - current_position;
  if (std::abs(dx) < _params.dx_min / 2.0)
    dx = 0.0;

  double door_v =
      compute_desired_rate_of_change(dx, current_velocity, _params, dt);

  return door_v;
}

bool DoorCommon::all_doors_open() {
  for (const auto &door : _doors)
    if (std::abs(door.second.open_position - door.second.current_position) >
        _params.dx_min)
      return false;

  return true;
}

bool DoorCommon::all_doors_closed() {
  for (const auto &door : _doors)
    if (std::abs(door.second.closed_position - door.second.current_position) >
        _params.dx_min)
      return false;

  return true;
}

std::vector<DoorCommon::DoorUpdateResult>
DoorCommon::update(const double time,
                   const std::vector<DoorCommon::DoorUpdateRequest> &requests) {
  double dt = time - _last_update_time;
  _last_update_time = time;

  // Update simulation position and velocity of each joint and
  // calcuate target velocity for the same
  std::vector<DoorCommon::DoorUpdateResult> results;
  for (const auto &request : requests) {
    const auto it = _doors.find(request.joint_name);
    if (it != _doors.end()) {
      it->second.current_position = request.position;
      it->second.current_velocity = request.velocity;
      DoorCommon::DoorUpdateResult result;
      result.joint_name = request.joint_name;
      result.fmax = _params.f_max;
      if (requested_mode().value == DoorMode::MODE_OPEN) {
        result.velocity = calculate_target_velocity(
            it->second.open_position, request.position, request.velocity, dt);
      } else {
        result.velocity = calculate_target_velocity(
            it->second.closed_position, request.position, request.velocity, dt);
      }
      results.push_back(result);
    } else {
      ROS_INFO("Received update request for uninitialized joint[%s]",
               request.joint_name.c_str());
    }
  }

  // Publishing door states
  if (time - _last_pub_time >= 1.0) {
    _last_pub_time = time;
    const int32_t t_sec = static_cast<int32_t>(time);
    const uint32_t t_nsec =
        static_cast<uint32_t>((time - static_cast<double>(t_sec)) * 1e9);
    const ros::Time now = ros::Time(t_sec, t_nsec);

    if (all_doors_open()) {
      publish_state(DoorMode::MODE_OPEN, now);
    } else if (all_doors_closed()) {
      publish_state(DoorMode::MODE_CLOSED, now);
    } else {
      publish_state(DoorMode::MODE_MOVING, now);
    }
  }

  return results;
}

} // namespace rmf_building_sim_common
