#ifndef DOOR_COMMON_H
#define DOOR_COMMON_H

#include <ros/console.h>
#include <ros/ros.h>

#include "ros1_rmf_gazebo/DoorMode.h"
#include "ros1_rmf_gazebo/DoorRequest.h"
#include "ros1_rmf_gazebo/DoorState.h"

#include "ros1_rmf_gazebo/utils.h"
#include <gazebo_plugins/gazebo_ros_utils.h>

#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace rmf_building_sim_common {

using DoorRequest = ros1_rmf_gazebo::DoorRequest;
using DoorState = ros1_rmf_gazebo::DoorState;
using DoorMode = ros1_rmf_gazebo::DoorMode;

//==============================================================================
class DoorCommon {

public:
  struct DoorUpdateRequest {
    std::string joint_name;
    double position;
    double velocity;
  };

  struct DoorUpdateResult {
    std::string joint_name;
    double velocity;
    double fmax;
  };

  template <typename SdfPtrT>
  static std::shared_ptr<DoorCommon> make(const std::string &door_name,
                                          ros::NodeHandle &node, SdfPtrT &sdf);

  void logger();

  std::vector<std::string> joint_names() const;

  MotionParams &params();

  std::vector<DoorUpdateResult>
  update(const double time, const std::vector<DoorUpdateRequest> &request);

private:
  struct DoorElement {
    double closed_position;
    double open_position;
    double current_position;
    double current_velocity;

    DoorElement() {}

    DoorElement(const double lower_limit, const double upper_limit,
                const bool flip_direction = false)
        : current_position(0.0), current_velocity(0.0) {
      if (flip_direction) {
        closed_position = lower_limit;
        open_position = upper_limit;
      } else {
        closed_position = upper_limit;
        open_position = lower_limit;
      }
    }
  };

  // Map joint name to its DoorElement
  using Doors = std::unordered_map<std::string, DoorElement>;

  DoorMode requested_mode() const;

  void publish_state(const uint32_t door_value, const ros::Time &time);

  void doorRequestCallback(const DoorRequest::ConstPtr &msg);

  DoorCommon(const std::string &door_name, ros::NodeHandle &node,
             const MotionParams &params, const Doors &doors);

  double calculate_target_velocity(const double target,
                                   const double current_position,
                                   const double current_velocity,
                                   const double dt);

  bool all_doors_open();

  bool all_doors_closed();

  ros::Publisher _door_state_pub;
  ros::Subscriber _door_request_sub;

  DoorState _state;
  DoorRequest _request;

  MotionParams _params;

  double _last_update_time = 0.0;
  // random start time offset to prevent state message crossfire
  double _last_pub_time = ((double)std::rand()) / ((double)(RAND_MAX));

  bool _initialized = false;

  // Map of joint_name and corresponding DoorElement
  Doors _doors;
};

template <typename SdfPtrT>
std::shared_ptr<DoorCommon> DoorCommon::make(const std::string &door_name,
                                             ros::NodeHandle &node,
                                             SdfPtrT &sdf) {
  // We work with a clone to avoid const correctness issues with
  // get_sdf_param functions in utils.hpp
  auto sdf_clone = sdf->Clone();

  MotionParams params;
  get_sdf_param_if_available<double>(sdf_clone, "v_max_door", params.v_max);
  get_sdf_param_if_available<double>(sdf_clone, "a_max_door", params.a_max);
  get_sdf_param_if_available<double>(sdf_clone, "a_nom_door", params.a_nom);
  get_sdf_param_if_available<double>(sdf_clone, "dx_min_door", params.dx_min);
  get_sdf_param_if_available<double>(sdf_clone, "f_max_door", params.f_max);

  auto door_element = sdf_clone;
  std::string left_door_joint_name;
  std::string right_door_joint_name;
  std::string door_type;

  // Get the joint names and door type
  if (!get_element_required(sdf_clone, "door", door_element) ||
      !get_sdf_attribute_required<std::string>(door_element, "left_joint_name",
                                               left_door_joint_name) ||
      !get_sdf_attribute_required<std::string>(door_element, "right_joint_name",
                                               right_door_joint_name) ||
      !get_sdf_attribute_required<std::string>(door_element, "type",
                                               door_type)) {
    ROS_INFO(" -- Missing required parameters for [%s] plugin",
             door_name.c_str());
    return nullptr;
  }

  if ((left_door_joint_name == "empty_joint" &&
       right_door_joint_name == "empty_joint") ||
      (left_door_joint_name.empty() && right_door_joint_name.empty())) {
    ROS_INFO(" -- Both door joint names are missing for [%s] plugin, at least "
             "one is required. ",
             door_name.c_str());
    return nullptr;
  }

  std::unordered_set<std::string> joint_names;
  if (!left_door_joint_name.empty() && left_door_joint_name != "empty_joint")
    joint_names.insert(left_door_joint_name);
  if (!right_door_joint_name.empty() && right_door_joint_name != "empty_joint")
    joint_names.insert(right_door_joint_name);

  Doors doors;

  auto extract_door = [&](SdfPtrT &joint_sdf) {
    auto joint_sdf_clone = joint_sdf->Clone();
    std::string joint_name;
    get_sdf_attribute_required<std::string>(joint_sdf_clone, "name",
                                            joint_name);
    const auto it = joint_names.find(joint_name);
    if (it != joint_names.end()) {
      auto element = joint_sdf_clone;
      get_element_required(joint_sdf_clone, "axis", element);
      get_element_required(element, "limit", element);
      double lower_limit = -1.57;
      double upper_limit = 0.0;
      get_sdf_param_if_available<double>(element, "lower", lower_limit);
      get_sdf_param_if_available<double>(element, "upper", upper_limit);
      DoorCommon::DoorElement door_element;
      if (joint_name == right_door_joint_name)
        door_element = DoorCommon::DoorElement{lower_limit, upper_limit, true};
      else if (joint_name == left_door_joint_name)
        door_element = DoorCommon::DoorElement{lower_limit, upper_limit};
      doors.insert({joint_name, door_element});
    }
  };

  // Get the joint limits from parent sdf
  auto parent = sdf->GetParent();
  if (!parent) {
    std::cout << "Unable to access parent sdf to retrieve joint limits\n";
    return nullptr;
  }

  auto joint_element = parent->GetElement("joint");
  if (!joint_element) {
    std::cout << "Parent sdf missing required joint element\n";
    return nullptr;
  }

  extract_door(joint_element);
  // Find next joint element if present
  while (joint_element) {
    extract_door(joint_element);
    joint_element = joint_element->GetNextElement("joint");
  }

  std::shared_ptr<DoorCommon> door_common(
      new DoorCommon(door_name, node, params, doors));

  return door_common;
}

} // namespace rmf_building_sim_common

#endif // RMF_BUILDING_SIM_COMMON__DOOR_COMMON_HPP
