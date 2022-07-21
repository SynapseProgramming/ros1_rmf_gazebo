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

private:
  ros1_rmf_gazebo::LiftRequest current_request;

  ros::NodeHandle n;
  ros::ServiceClient sc;
  ros::ServiceServer ss;

  ros::Subscriber lift_state_sub;
  ros::Publisher lift_request_pub;
  std::vector<std::string> lift_names;
};

#endif /*                                                                      \
          LIFT_SER */
