#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <ros/ros.h>

#include "ros1_rmf_gazebo/lift_common.h"
#include "ros1_rmf_gazebo/utils.h"
#include <gazebo_plugins/gazebo_ros_utils.h>

using namespace rmf_building_sim_common;

namespace building_sim_gazebo {
//==============================================================================

class LiftPlugin : public gazebo::ModelPlugin {
private:
  // Gazebo items
  gazebo::event::ConnectionPtr _update_connection;
  gazebo::physics::ModelPtr _model;
  gazebo::physics::JointPtr _cabin_joint_ptr;
  ros::NodeHandle n;
  boost::shared_ptr<ros::NodeHandle> nptr;

  gazebo::GazeboRosPtr gazebo_ros_;

  std::unique_ptr<LiftCommon> _lift_common = nullptr;

  bool _initialized;

public:
  LiftPlugin() { _initialized = false; }

  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override {

    gazebo_ros_ =
        gazebo::GazeboRosPtr(new gazebo::GazeboRos(model, sdf, "Liftz"));

    gazebo_ros_->isInitialized();
    _model = model;

    ROS_INFO("Loading LiftPlugin for [%s]", _model->GetName().c_str());

    nptr = gazebo_ros_->node();
    n = *nptr;

    // load Lift object
    _lift_common = LiftCommon::make(n, _model->GetName(), sdf);
    if (!_lift_common) {
      ROS_INFO("Failed when loading [%s]", _model->GetName().c_str());
      return;
    }

    _cabin_joint_ptr = _model->GetJoint(_lift_common->get_joint_name());
    if (!_cabin_joint_ptr) {
      ROS_INFO(" -- Model is missing the joint [%s]",
               _lift_common->get_joint_name().c_str());
      return;
    }

    _update_connection = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&LiftPlugin::on_update, this));

    _cabin_joint_ptr->SetPosition(0, _lift_common->get_elevation());
    ROS_INFO("Finished loading [%s]", _model->GetName().c_str());

    _initialized = true;
  }

private:
  void on_update() {
    if (!_initialized)
      return;

    const double t = _model->GetWorld()->SimTime().Double();
    const double position = _cabin_joint_ptr->Position(0);
    const double velocity = _cabin_joint_ptr->GetVelocity(0);

    // Send update request
    auto result = _lift_common->update(t, position, velocity);

    _cabin_joint_ptr->SetParam("vel", 0, result.velocity);
    _cabin_joint_ptr->SetParam("fmax", 0, result.fmax);
  }
};

GZ_REGISTER_MODEL_PLUGIN(LiftPlugin)

} // namespace building_sim_gazebo
