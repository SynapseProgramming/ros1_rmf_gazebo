#include "ros1_rmf_gazebo/lift_server.h"

LiftServer::LiftServer()
    : sc(n.serviceClient<ros1_rmf_gazebo::LiftNames>("liftnames")),
      ss(n.advertiseService("do_you_lift", &LiftServer::getLiftCallback,
                            this)) {

  ROS_INFO("LiftServer object has been created!");
  // call lift names service
  ros1_rmf_gazebo::LiftNames srv;
  if (sc.call(srv)) {
    ROS_INFO("Service Call successful!");
    lift_names = srv.response.lift_names;
    for (int i = 0; i < lift_names.size(); i++) {
      std::cout << lift_names[i] << "\n";
    }
  } else {
    std::cout << "YES\n";
  }
}

bool LiftServer::getLiftCallback(
    lb_navigation_msgs::LiftCommand::Request &req,
    lb_navigation_msgs::LiftCommand::Response &res) {

  ROS_INFO("received lift request!");
  return true;
}

// void LiftServer::pclCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
//
//   pointcloud_msg_ = *msg;
//   pcl_conversions::toPCL(pointcloud_msg_, pointcloud);
//
//   pcl::PCLPointCloud2ConstPtr inpPclPtr =
//       boost::make_shared<pcl::PCLPointCloud2>(pointcloud);
//
//   crop_box.setInputCloud(inpPclPtr);
//   crop_box.filter(pointcloud);
//   pcl_conversions::fromPCL(pointcloud, pointcloud_msg_);
//   // publish filtered pointcloud
//   pointcloud_pub.publish(pointcloud_msg_);
// }

int main(int argc, char **argv) {
  ros::init(argc, argv, "lift_server");

  LiftServer obj;

  ros::spin();
  return 0;
}
