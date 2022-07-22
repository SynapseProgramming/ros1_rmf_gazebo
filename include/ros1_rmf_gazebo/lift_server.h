#ifndef LIFT_SERVER_H
#define LIFT_SERVER_H

#include <iostream>
#include <ros/console.h>
#include <ros/ros.h>
#include <string>

#include "ros1_rmf_gazebo/LiftState.h"
#include <lb_navigation_msgs/LiftCommand.h>
#include <ros1_rmf_gazebo/LiftNames.h>
#include <ros1_rmf_gazebo/LiftRequest.h>

#include <boost/shared_ptr.hpp>

class LiftServer {
public:
  LiftServer();

  bool getLiftCallback(lb_navigation_msgs::LiftCommand::Request &req,
                       lb_navigation_msgs::LiftCommand::Response &res);

  void liftStateCallback(const ros1_rmf_gazebo::LiftState::ConstPtr &msg);

  void closeDoors();
  void closeDoorsCallback(const ros::TimerEvent &event);

  void sequenceCallback(const ros::TimerEvent &event);

private:
  ros1_rmf_gazebo::LiftRequest current_request;
  ros1_rmf_gazebo::LiftState::ConstPtr lift_state;

  std::string lift_current_floor_;

  int current_status;

  std::string robot_current_floor;
  std::string robot_destination_floor;
  std::vector<std::string> lift_names;

  bool close_door_signal;
  bool is_closing;
  bool start_sequence;

  ros::Timer door_timer;
  ros::Timer sequence_timer;

  ros::NodeHandle n;
  ros::ServiceClient sc;
  ros::ServiceServer ss;

  ros::Subscriber lift_state_sub;
  ros::Publisher lift_request_pub;
};

#endif /*                                                                      \
          LIFT_SER */
