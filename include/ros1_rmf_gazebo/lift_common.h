#ifndef LIFT_COMMMON_H
#define LIFT_COMMMON_H

#include <ros/console.h>
#include <ros/ros.h>

#include "ros1_rmf_gazebo/DoorMode.h"
#include "ros1_rmf_gazebo/DoorRequest.h"
#include "ros1_rmf_gazebo/DoorState.h"
#include "ros1_rmf_gazebo/LiftRequest.h"
#include "ros1_rmf_gazebo/LiftState.h"

#include "ros1_rmf_gazebo/utils.h"

#include <iostream>
#include <unordered_map>
#include <utility>
#include <vector>

namespace rmf_building_sim_common {

using LiftState = ros1_rmf_gazebo::LiftState;
using LiftRequest = ros1_rmf_gazebo::LiftRequest;
using DoorRequest = ros1_rmf_gazebo::DoorRequest;
using DoorState = ros1_rmf_gazebo::DoorState;
using DoorMode = ros1_rmf_gazebo::DoorMode;

//==============================================================================
class LiftCommon {

public:
  struct LiftUpdateResult {
    double velocity;
    double fmax;
  };

  template <typename SdfPtrT>
  static std::unique_ptr<LiftCommon>
  make(ros::NodeHandle &nh, const std::string &lift_name, SdfPtrT &sdf);

  void logger();

  // a lot of dependencies. dont port this first
  LiftUpdateResult update(const double time, const double position,
                          const double velocity);

  std::string get_joint_name() const;

  double get_elevation() const;

  bool motion_state_changed();

private:
  ros::Subscriber _lift_request_sub;
  ros::Subscriber _door_state_sub;

  ros::Publisher _lift_state_pub;
  ros::Publisher _door_request_pub;

  std::string _lift_name;
  std::string _cabin_joint_name;

  MotionParams _cabin_motion_params;
  LiftState::_motion_state_type _old_motion_state;

  std::vector<std::string> _floor_names;
  std::unordered_map<std::string, double> _floor_name_to_elevation;
  std::unordered_map<std::string, std::vector<std::string>>
      _floor_name_to_shaft_door_name;
  std::unordered_map<std::string, std::vector<std::string>>
      _floor_name_to_cabin_door_name;
  std::unordered_map<std::string, DoorState::ConstPtr> _shaft_door_states;
  std::unordered_map<std::string, DoorState::ConstPtr> _cabin_door_states;

  LiftState _lift_state;
  LiftRequest::ConstPtr _lift_request;

  double _last_update_time = 0.0;
  // random start time offset to prevent state message crossfire
  double _last_pub_time = ((double)std::rand()) / ((double)(RAND_MAX));

  void publish_door_request(const double time, std::string door_name,
                            uint32_t door_state);

  double get_step_velocity(const double dt, const double position,
                           const double velocity);

  void update_cabin_state(const double position, const double velocity);

  void move_doors(const double time, uint32_t door_mode);

  void open_doors(const double time);

  void close_doors(const double time);

  uint32_t get_door_state(
      const std::unordered_map<std::string, std::vector<std::string>>
          &floor_to_door_map,
      const std::unordered_map<std::string, DoorState::ConstPtr> &door_states);

  void update_lift_door_state();

  LiftCommon(
      ros::NodeHandle &nh, const std::string &lift_name,
      const std::string &joint_name, const MotionParams &cabin_motion_params,
      const std::vector<std::string> &floor_names,
      const std::unordered_map<std::string, double> &floor_name_to_elevation,
      std::unordered_map<std::string, std::vector<std::string>>
          floor_name_to_shaft_door_name,
      std::unordered_map<std::string, std::vector<std::string>>
          floor_name_to_cabin_door_name,
      std::unordered_map<std::string, DoorState::ConstPtr> shaft_door_states,
      std::unordered_map<std::string, DoorState::ConstPtr> cabin_door_states,
      std::string initial_floor_name);

  void liftRequestCallback(const LiftRequest::ConstPtr &msg);
  void doorStateCallback(const DoorState::ConstPtr &msg);

  void pub_lift_state(const double time);
};

template <typename SdfPtrT>

std::unique_ptr<LiftCommon> LiftCommon::make(ros::NodeHandle &nh,
                                             const std::string &lift_name,
                                             SdfPtrT &sdf) {
  MotionParams cabin_motion_params;
  std::string joint_name;
  std::vector<std::string> floor_names;
  std::unordered_map<std::string, double> floor_name_to_elevation;
  std::unordered_map<std::string, std::vector<std::string>>
      floor_name_to_shaft_door_name;
  std::unordered_map<std::string, std::vector<std::string>>
      floor_name_to_cabin_door_name;
  std::unordered_map<std::string, DoorState::ConstPtr> shaft_door_states;
  std::unordered_map<std::string, DoorState::ConstPtr> cabin_door_states;

  auto sdf_clone = sdf->Clone();

  // load lift cabin motion parameters
  get_sdf_param_if_available<double>(sdf_clone, "v_max_cabin",
                                     cabin_motion_params.v_max);
  get_sdf_param_if_available<double>(sdf_clone, "a_max_cabin",
                                     cabin_motion_params.a_max);
  get_sdf_param_if_available<double>(sdf_clone, "a_nom_cabin",
                                     cabin_motion_params.a_nom);
  get_sdf_param_if_available<double>(sdf_clone, "dx_min_cabin",
                                     cabin_motion_params.dx_min);
  get_sdf_param_if_available<double>(sdf_clone, "f_max_cabin",
                                     cabin_motion_params.f_max);
  if (!get_sdf_param_required(sdf_clone, "cabin_joint_name", joint_name))
    return nullptr;

  // load the floor name and elevation for each floor
  auto floor_element = sdf_clone;
  if (!get_element_required(sdf, "floor", floor_element)) {
    ROS_INFO(" -- Missing required floor element for [%s] plugin",
             lift_name.c_str());
    return nullptr;
  }

  while (floor_element) {
    std::string floor_name;
    double floor_elevation;
    if (!get_sdf_attribute_required<std::string>(floor_element, "name",
                                                 floor_name) ||
        !get_sdf_attribute_required<double>(floor_element, "elevation",
                                            floor_elevation)) {
      ROS_INFO("-- Missing required floor name or elevation attributes for "
               "[%s] plugin",
               lift_name.c_str());
      return nullptr;
    }
    floor_names.push_back(floor_name);
    floor_name_to_elevation.insert({floor_name, floor_elevation});

    auto door_pair_element = floor_element;
    if (get_element_required(floor_element, "door_pair", door_pair_element)) {
      while (door_pair_element) {
        std::string shaft_door_name;
        std::string cabin_door_name;
        if (!get_sdf_attribute_required<std::string>(
                door_pair_element, "cabin_door", cabin_door_name) ||
            !get_sdf_attribute_required<std::string>(
                door_pair_element, "shaft_door", shaft_door_name)) {
          ROS_INFO(" -- Missing required lift door attributes for [%s] plugin",
                   lift_name.c_str());
          return nullptr;
        }
        floor_name_to_cabin_door_name[floor_name].push_back(cabin_door_name);
        floor_name_to_shaft_door_name[floor_name].push_back(shaft_door_name);
        shaft_door_states.insert({shaft_door_name, nullptr});
        cabin_door_states.insert({cabin_door_name, nullptr});

        door_pair_element = door_pair_element->GetNextElement("door_pair");
      }
    }
    floor_element = floor_element->GetNextElement("floor");
  }

  assert(!floor_names.empty());
  std::string initial_floor_name = floor_names[0];
  get_sdf_param_if_available<std::string>(sdf_clone, "initial_floor",
                                          initial_floor_name);

  if (std::find(floor_names.begin(), floor_names.end(), initial_floor_name) ==
      floor_names.end()) {

    ROS_INFO("Initial floor [%s] is not available, changing to default",
             initial_floor_name.c_str());

    initial_floor_name = floor_names[0];
  }

  std::unique_ptr<LiftCommon> lift(new LiftCommon(
      nh, lift_name, joint_name, cabin_motion_params, floor_names,
      floor_name_to_elevation, floor_name_to_shaft_door_name,
      floor_name_to_cabin_door_name, shaft_door_states, cabin_door_states,
      initial_floor_name));

  return lift;
}

} // namespace rmf_building_sim_common

#endif // RMF_BUILDING_SIM_COMMON__LIFT_COMMON_HPP
