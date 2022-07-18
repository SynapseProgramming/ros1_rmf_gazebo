#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>

#include "ros1_rmf_gazebo/door_common.h"
#include "ros1_rmf_gazebo/utils.h"
#include <gazebo_plugins/gazebo_ros_utils.h>
#include <ros/ros.h>

using namespace rmf_building_sim_common;

namespace rmf_building_sim_gazebo_plugins {
//==============================================================================

class DoorPlugin : public gazebo::ModelPlugin {
private:
  gazebo::event::ConnectionPtr _update_connection;
  gazebo::physics::ModelPtr _model;
  std::unordered_map<std::string, gazebo::physics::JointPtr> _joints;

  std::shared_ptr<DoorCommon> _door_common = nullptr;

  bool _initialized = false;

  gazebo::GazeboRosPtr gazebo_ros_;

  ros::NodeHandle n;
  boost::shared_ptr<ros::NodeHandle> nptr;

public:
  DoorPlugin() {
    // Do nothing
  }

  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override {
    // auto _ros_node = gazebo_ros::Node::Get(sdf);
    std::cout << "BEFORE ERROR\n";
    gazebo_ros_ =
        gazebo::GazeboRosPtr(new gazebo::GazeboRos(model, sdf, "Doorz"));

    gazebo_ros_->isInitialized();
    std::cout << "PLUGIN LOADING!\n";
    _model = model;
    std::cout << "loaded Door plugin!\n";

    nptr = gazebo_ros_->node();
    n = *nptr;

    // RCLCPP_INFO(_ros_node->get_logger(), "Loading DoorPlugin for [%s]",
    //             _model->GetName().c_str());

    _door_common = DoorCommon::make(_model->GetName(), n, sdf);

    if (!_door_common)
      return;

    for (const auto &joint_name : _door_common->joint_names()) {
      const auto joint = _model->GetJoint(joint_name);
      if (!joint) {
        std::cout << " -- Model is missing the joint: " << joint_name.c_str()
                  << "\n";
        // RCLCPP_ERROR(_ros_node->get_logger(),
        //              " -- Model is missing the joint [%s]",
        //              joint_name.c_str());
        return;
      }
      _joints.insert(std::make_pair(joint_name, joint));
    }

    _initialized = true;

    _update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&DoorPlugin::on_update, this));
    std::cout << "Finished loading!\n";

    // RCLCPP_INFO(_ros_node->get_logger(), "Finished loading [%s]",
    //             _model->GetName().c_str());
  }

private:
  void on_update() {
    if (!_initialized)
      return;

    const double t = _model->GetWorld()->SimTime().Double();

    // Create DoorUpdateRequest
    std::vector<DoorCommon::DoorUpdateRequest> requests;
    for (const auto &joint : _joints) {
      DoorCommon::DoorUpdateRequest request;
      request.joint_name = joint.first;
      request.position = joint.second->Position(0);
      request.velocity = joint.second->GetVelocity(0);
      requests.push_back(request);
    }

    auto results = _door_common->update(t, requests);

    // Apply motions to the joints
    for (const auto &result : results) {
      const auto it = _joints.find(result.joint_name);
      assert(it != _joints.end());
      it->second->SetParam("vel", 0, result.velocity);
      it->second->SetParam("fmax", 0, result.fmax);
    }
  }
};

GZ_REGISTER_MODEL_PLUGIN(DoorPlugin)

} // namespace rmf_building_sim_gazebo_plugins
