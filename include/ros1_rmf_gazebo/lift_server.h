#ifndef LIFT_SERVER_H
#define LIFT_SERVER_H

#include <iostream>
#include <ros/console.h>
#include <ros/ros.h>
#include <string>

#include <ros1_rmf_gazebo/LiftNames.h>

#include <boost/shared_ptr.hpp>

class LiftServer {
public:
  LiftServer();

  // void pclCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);

private:
  ros::NodeHandle n;
  ros::ServiceClient sc;

  // ros::Subscriber pointcloud_sub;
  // ros::Publisher pointcloud_pub;
  std::vector<std::string> lift_names;
};

#endif /*                                                                      \
          LIFT_SER */
