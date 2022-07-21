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
    // TODO: remove this print out code
    for (int i = 0; i < lift_names.size(); i++) {
      std::cout << lift_names[i] << "\n";
    }
  } else {
    std::cout << "YES\n";
  }

  // lift request constants
  current_request.session_id = 1;
  // to ensure door is open when floor is reached
  current_request.door_state = 2;
  current_request.request_type = 1;
}

void LiftServer::liftStateCallback(
    const ros1_rmf_gazebo::LiftState::ConstPtr &msg) {

  std::cout << "lift state received!\n";
  std::cout << msg->current_floor << "\n";
}

bool LiftServer::getLiftCallback(
    lb_navigation_msgs::LiftCommand::Request &req,
    lb_navigation_msgs::LiftCommand::Response &res) {
  ROS_INFO("received lift request!");
  // current_request.lift_name = lift_names[0];
  // fill up current request
  // TODO: always use the first lift in the vector for now.

  // std::string robot_current_floor = req.request.source_map;
  // std::string robot_destination_floor = req.request.destination_map;
  //
  //
  // current_request.destination_floor = req.request.
  //
  //                                     return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "lift_server");

  LiftServer obj;

  ros::spin();
  return 0;
}
