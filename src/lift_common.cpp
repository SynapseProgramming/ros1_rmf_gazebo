#include <memory>

#include "ros1_rmf_gazebo/lift_common.h"
#include "ros1_rmf_gazebo/utils.h"

namespace rmf_building_sim_common {

void LiftCommon::logger() {
  std::cout << "lift_" + _lift_state.lift_name << "\n";
}

std::string LiftCommon::get_joint_name() const { return _cabin_joint_name; }

double LiftCommon::get_elevation() const {
  return _floor_name_to_elevation.at(_lift_state.destination_floor);
}

// Compares current lift motion state to the last time this function was called
bool LiftCommon::motion_state_changed() {
  bool changed = _lift_state.motion_state != _old_motion_state;
  _old_motion_state = _lift_state.motion_state;
  return changed;
}

void LiftCommon::publish_door_request(const double time, std::string door_name,
                                      uint32_t door_state) {
  DoorRequest request;
  request.request_time = ros::Time::now();
  request.requester_id = _lift_name;
  request.door_name = door_name;
  request.requested_mode.value = door_state;

  _door_request_pub.publish(request);
}

double LiftCommon::get_step_velocity(const double dt, const double position,
                                     const double velocity) {
  double desired_elevation = get_elevation();
  double dz = desired_elevation - position;

  if (abs(dz) < _cabin_motion_params.dx_min / 2.0)
    dz = 0;

  return compute_desired_rate_of_change(dz, velocity, _cabin_motion_params, dt);
}

void LiftCommon::update_cabin_state(const double position,
                                    const double velocity) {
  // TODO update current_floor only when lift reaches its destination
  double smallest_error = std::numeric_limits<double>::max();
  std::string closest_floor_name;
  for (const auto &floor : _floor_name_to_elevation) {
    double tmp_error = abs(position - floor.second);
    if (tmp_error < smallest_error) {
      smallest_error = tmp_error;
      closest_floor_name = floor.first;
    }
  }
  _lift_state.current_floor = closest_floor_name;

  // Set motion state
  if (abs(velocity) < 0.01)
    _lift_state.motion_state = LiftState::MOTION_STOPPED;
  else if (velocity > 0)
    _lift_state.motion_state = LiftState::MOTION_UP;
  else
    _lift_state.motion_state = LiftState::MOTION_DOWN;
}

void LiftCommon::move_doors(const double time, uint32_t door_mode) {
  auto cabin_door_names =
      _floor_name_to_cabin_door_name[_lift_state.current_floor];
  for (const auto &cabin_door : cabin_door_names) {
    const auto it = _cabin_door_states.find(cabin_door);
    if (it == _cabin_door_states.end())
      continue;
    if (it->second && it->second->current_mode.value != door_mode)
      publish_door_request(time, cabin_door, door_mode);
  }
  auto shaft_door_names =
      _floor_name_to_shaft_door_name[_lift_state.current_floor];
  for (const auto &shaft_door : shaft_door_names) {
    const auto it = _shaft_door_states.find(shaft_door);
    if (it == _shaft_door_states.end())
      continue;
    if (it->second && it->second->current_mode.value != door_mode)
      publish_door_request(time, shaft_door, door_mode);
  }
}

void LiftCommon::open_doors(const double time) {
  move_doors(time, DoorMode::MODE_OPEN);
}

void LiftCommon::close_doors(const double time) {
  move_doors(time, DoorMode::MODE_CLOSED);
}

uint32_t LiftCommon::get_door_state(
    const std::unordered_map<std::string, std::vector<std::string>>
        &floor_to_door_map,
    const std::unordered_map<std::string, DoorState::ConstPtr> &door_states) {
  std::size_t open_count = 0;
  std::size_t closed_count = 0;
  const auto doors = floor_to_door_map.find(_lift_state.current_floor)->second;
  const std::size_t num = doors.size();
  for (const std::string &door : doors) {
    const auto &door_state = door_states.find(door)->second;
    if ((door_state) &&
        (door_state->current_mode.value == DoorMode::MODE_CLOSED))
      closed_count++;

    else if ((door_state) &&
             (door_state->current_mode.value == DoorMode::MODE_OPEN))
      open_count++;
  }
  if (open_count == num)
    return DoorMode::MODE_OPEN;

  else if (closed_count == num)
    return DoorMode::MODE_CLOSED;

  else
    return DoorMode::MODE_MOVING;
}

void LiftCommon::update_lift_door_state() {
  uint32_t cabin_door_state =
      get_door_state(_floor_name_to_cabin_door_name, _cabin_door_states);
  uint32_t shaft_door_state =
      get_door_state(_floor_name_to_shaft_door_name, _shaft_door_states);

  _lift_state.door_state = (cabin_door_state == shaft_door_state)
                               ? cabin_door_state
                               : LiftState::DOOR_MOVING;
}

void LiftCommon::liftRequestCallback(const LiftRequest::ConstPtr &msg) {

  if (msg->lift_name != _lift_name)
    return;

  if (_floor_name_to_elevation.find(msg->destination_floor) ==
      _floor_name_to_elevation.end()) {
    ROS_INFO("Received request for unavailable floor [%s]",
             msg->destination_floor.c_str());
    return;
  }

  if (_lift_request) // Lift is still processing a previous request
  {
    ROS_INFO("Failed to request: [%s] is busy at the moment",
             _lift_name.c_str());
    return;
  }

  _lift_request = std::move(msg);
  ROS_INFO("Lift [%s] requested at level [%s]", _lift_name.c_str(),
           _lift_request->destination_floor.c_str());
}

void LiftCommon::doorStateCallback(const DoorState::ConstPtr &msg) {

  std::string name = msg->door_name;
  if (_cabin_door_states.find(name) != _cabin_door_states.end())
    _cabin_door_states[name] = std::move(msg);
  else if (_shaft_door_states.find(name) != _shaft_door_states.end())
    _shaft_door_states[name] = std::move(msg);
}

LiftCommon::LiftCommon(
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
    std::string initial_floor_name)
    : _lift_name(lift_name), _cabin_joint_name(joint_name),
      _cabin_motion_params(cabin_motion_params), _floor_names(floor_names),
      _floor_name_to_elevation(floor_name_to_elevation),
      _floor_name_to_shaft_door_name(floor_name_to_shaft_door_name),
      _floor_name_to_cabin_door_name(floor_name_to_cabin_door_name),
      _shaft_door_states(shaft_door_states),
      _cabin_door_states(cabin_door_states) {

  // prints out available floors for this lift
  std::cout << "Loaded lift: " << _lift_name << std::endl;
  std::cout << "Names  |  Elevations" << std::endl;
  for (const auto &it : _floor_name_to_elevation)
    std::cout << it.first << "  |  " << it.second << std::endl;

  // initialize pub & sub
  _lift_state_pub = nh.advertise<LiftState>("/lift_states", 10);

  _door_request_pub = nh.advertise<DoorRequest>("/door_requests", 10);

  _lift_request_sub = nh.subscribe<LiftRequest>(
      "/lift_requests", 10, &LiftCommon::liftRequestCallback, this);

  _door_state_sub = nh.subscribe<DoorState>(
      "/door_states", 10, &LiftCommon::doorStateCallback, this);

  // Initial lift state
  _lift_state.lift_name = _lift_name;
  _lift_state.current_floor = _floor_names[0];
  _lift_state.destination_floor = initial_floor_name;
  _lift_state.door_state = LiftState::DOOR_CLOSED;
  _lift_state.motion_state = LiftState::MOTION_STOPPED;
  _lift_state.current_mode = LiftState::MODE_AGV;
  for (const std::string &floor_name : _floor_names)
    _lift_state.available_floors.push_back(floor_name);
}

void LiftCommon::pub_lift_state(const double time) {
  _last_pub_time = time;
  const int32_t t_sec = static_cast<int32_t>(time);
  const uint32_t t_nsec =
      static_cast<uint32_t>((time - static_cast<double>(t_sec)) * 1e9);
  ros::Time now = ros::Time(t_sec, t_nsec);
  _lift_state.lift_time = now;
  _lift_state_pub.publish(_lift_state);
}

LiftCommon::LiftUpdateResult LiftCommon::update(const double time,
                                                const double position,
                                                const double velocity) {
  const double dt = time - _last_update_time;
  _last_update_time = time;

  // Update lift state
  update_cabin_state(position, velocity);
  update_lift_door_state();

  // Construct LiftUpdateResult
  LiftCommon::LiftUpdateResult result;
  result.velocity = 0.0;
  result.fmax = _cabin_motion_params.f_max;

  // Handle lift request
  if (_lift_request) {
    std::string desired_floor = _lift_request->destination_floor;
    uint8_t desired_door_state = _lift_request->door_state;
    if (_lift_request->request_type == LiftRequest::REQUEST_END_SESSION)
      _lift_state.session_id = "";
    else
      _lift_state.session_id = _lift_request->session_id;

    if ((_lift_state.current_floor == desired_floor) &&
        (_lift_state.door_state == desired_door_state) &&
        (_lift_state.motion_state == LiftState::MOTION_STOPPED)) {
      ROS_INFO("Reached floor %s with doors %s", desired_floor.c_str(),
               (desired_door_state == 0 ? "closed" : "open"));
      _lift_request = nullptr;
    } else {
      _lift_state.destination_floor = desired_floor;

      if (_lift_state.current_floor != _lift_state.destination_floor) {
        if (_lift_state.door_state != LiftState::DOOR_CLOSED) {
          close_doors(time);
        } else {
          result.velocity = get_step_velocity(dt, position, velocity);
        }
      } else {
        if (_lift_state.motion_state != LiftState::MOTION_STOPPED) {
          result.velocity = get_step_velocity(dt, position, velocity);
        } else {
          if (desired_door_state == LiftState::DOOR_OPEN) {
            open_doors(time);
          } else if (desired_door_state == LiftState::DOOR_CLOSED) {
            close_doors(time);
          }
        }
      }
    }
  }
  // Publish lift state at 1 Hz
  if (time - _last_pub_time >= 1.0)
    pub_lift_state(time);

  return result;
}

} // namespace rmf_building_sim_common
