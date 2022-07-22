#include "ros1_rmf_gazebo/lift_server.h"

LiftServer::LiftServer()
    : sc(n.serviceClient<ros1_rmf_gazebo::LiftNames>("liftnames")),
      ss(n.advertiseService("do_you_lift", &LiftServer::getLiftCallback, this)),
      lift_request_pub(
          n.advertise<ros1_rmf_gazebo::LiftRequest>("lift_requests", 100)),
      lift_state_sub(n.subscribe<ros1_rmf_gazebo::LiftState>(
          "/rmf_lift_states", 5, &LiftServer::liftStateCallback, this)) {
  ROS_INFO("LiftServer object has been created!");
  // call lift names service
  ros1_rmf_gazebo::LiftNames srv;
  if (sc.call(srv)) {
    ROS_INFO("Obtained lift names!");
    lift_names = srv.response.lift_names;
  } else {
    ROS_INFO("No lifts available");
  }
  current_status = 0;
  is_closing = false;

  // lift request constants
  current_request.session_id = "1";
  current_request.request_type = 1;
  close_door_signal = false;
  start_sequence = false;
  lift_state = nullptr;

  door_timer = n.createTimer(
      ros::Duration(10.0), &LiftServer::closeDoorsCallback, this, true, false);

  sequence_timer =
      n.createTimer(ros::Duration(0.1), &LiftServer::sequenceCallback, this);
}

void LiftServer::liftStateCallback(
    const ros1_rmf_gazebo::LiftState::ConstPtr &msg) {

  lift_current_floor_ = msg->current_floor;
  // close doors if the lift doors are fully opened and lift is stopped
  if (msg->door_state == 2 && msg->motion_state == 0 &&
      close_door_signal == false) {
    closeDoors();
    close_door_signal = true;
  }
  if (msg->door_state != 2) {
    close_door_signal = false;
  }
  if (msg->door_state == 0)
    is_closing = false;
  lift_state = std::move(msg);
}

bool LiftServer::getLiftCallback(
    lb_navigation_msgs::LiftCommand::Request &req,
    lb_navigation_msgs::LiftCommand::Response &res) {
  ROS_INFO("received lift request!");
  // Initialise the current lift request
  // TODO: always use the first lift in the vector for now.
  current_request.lift_name = lift_names[0];
  res.lift_allocated = current_request.lift_name;

  robot_current_floor = req.source_map;
  robot_destination_floor = req.destination_map;

  // start the lift sequence
  start_sequence = true;

  return true;
}

void LiftServer::closeDoors() {
  door_timer.stop();
  door_timer.setPeriod(ros::Duration(5.0));
  door_timer.start();
}

void LiftServer::closeDoorsCallback(const ros::TimerEvent &event) {
  ROS_INFO("closing lift doors!");
  is_closing = true;
  current_request.door_state = 0;
  lift_request_pub.publish(current_request);
}

void LiftServer::sequenceCallback(const ros::TimerEvent &event) {
  if (start_sequence == true) {
    if (current_status == 0) {
      // move to the current floor of the robot
      ROS_INFO("Lift moving to robot floor!");
      current_request.destination_floor = robot_current_floor;
      current_request.door_state = 2;
      lift_request_pub.publish(current_request);
      current_status = 1;
    } else if (current_status == 1 && is_closing == true) {
      current_status = 2;
    } else if (current_status == 2 && is_closing == false) {
      // move to the destination floor
      ROS_INFO("Lift moving to destination floor!");
      current_request.destination_floor = robot_destination_floor;
      current_request.door_state = 2;
      is_closing = false;
      lift_request_pub.publish(current_request);
      current_status = 3;
    } else if (current_status == 3 && lift_state->door_state == 2 &&
               lift_state->motion_state == 0) {
      // done with sequence. reset start sequence flag
      ROS_INFO("Lift Sequence complete!");
      current_status = 0;
      start_sequence = false;
    }
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "lift_server");

  LiftServer obj;

  ros::spin();
  return 0;
}
