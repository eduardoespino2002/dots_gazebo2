// Copyright 2019 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
 * \brief  Simple model controller that uses a twist message to move an entity on the xy plane.
 *
 * \author  Piyush Khandelwal (piyushk@gmail.com)
 *
 * \date  29 July 2013
 */

#include <gazebo/common/Events.hh>
//#include <gazebo/common/common.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_plugins/gazebo_ros_lifter.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
//#include <gazebo_ros/conversions/std_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#ifdef IGN_PROFILER_ENABLE
#include <ignition/common/Profiler.hh>
#endif
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

#include <memory>

#include <string>

namespace gazebo_plugins
{
class GazeboRosLifterPrivate
{
public:
  /// Callback to be called at every simulation iteration.
  /// \param[in] _info Updated simulation info.
  void OnUpdate(const gazebo::common::UpdateInfo & _info);

  /// Callback when a lifter command is received.
  /// \param[in] _msg Bool command message.
  void OnLifter(const std_msgs::msg::Float32::SharedPtr _msg);

  /// A pointer to the GazeboROS node.
  gazebo_ros::Node::SharedPtr ros_node_;

  /// Subscriber to lifter position
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr lifter_sub_;

  /// Lifter command.
  std_msgs::msg::Float32 target_pos_;

  /// Pointer to world.
  gazebo::physics::WorldPtr world_;

  /// Pointer to model.
  gazebo::physics::ModelPtr model_;

  /// Pointer to joint
  gazebo::physics::JointPtr joint_;

  /// Connection to event called at every world iteration.
  gazebo::event::ConnectionPtr update_connection_;

  /// Protect variables accessed on callbacks.
  std::mutex lock_;

  /// Update period in seconds.
  double update_period_;

  /// Last update time.
  gazebo::common::Time last_update_time_;

  // PID variables
  ignition::math::Vector3d pid = {100.0, 0.0, 0.0};
  //double last_pos;
  //double last_vel;
  double int_pos;
  double int_limit;
  double force_offset;

};

GazeboRosLifter::GazeboRosLifter()
: impl_(std::make_unique<GazeboRosLifterPrivate>())
{
}

GazeboRosLifter::~GazeboRosLifter()
{
}

void GazeboRosLifter::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  impl_->model_ = _model;
  impl_->world_ = _model->GetWorld();

  // Initialize ROS node
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  // Get QoS profiles
  const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

  // Get the parameters
  auto joint_name = _sdf->Get<std::string>("joint_name", "undefined").first;

  
  if (!_sdf->GetElement("joint_pid"))
    RCLCPP_INFO(impl_->ros_node_->get_logger(), "No joint PID, using 100.0 0.0 0.0");
  else
    impl_->pid = _sdf->GetElement("joint_pid")->Get<ignition::math::Vector3d>();

  auto update_rate = _sdf->Get<double>("update_rate", 20.0).first;
  auto joint_position = _sdf->Get<double>("joint_position", 0.0).first;
  auto topic_name = _sdf->Get<std::string>("topic", "undefined").first;
  impl_->int_limit = _sdf->Get<double>("integrator_limit", 0.0).first;
  impl_->force_offset = _sdf->Get<double>("force_offset", 0.0).first;


  // Joint
  impl_->joint_ = _model->GetJoint(joint_name);
  if (!impl_->joint_)
  {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Joint %s does not exist!", joint_name.c_str());
    return;
  }
  RCLCPP_INFO(impl_->ros_node_->get_logger(), "Using joint %s", joint_name.c_str());

  // Update rate
  if (update_rate > 0.0) {
    impl_->update_period_ = 1.0 / update_rate;
  } else {
    impl_->update_period_ = 0.0;
  }
  impl_->last_update_time_ = impl_->world_->SimTime();

  // Topic to subscribe to
  impl_->lifter_sub_ = impl_->ros_node_->create_subscription<std_msgs::msg::Float32>(
    topic_name, qos.get_subscription_qos(topic_name, rclcpp::QoS(1)),
    std::bind(&GazeboRosLifterPrivate::OnLifter, impl_.get(), std::placeholders::_1));

  RCLCPP_INFO(
    impl_->ros_node_->get_logger(), "Subscribed to [%s]",
    impl_->lifter_sub_->get_topic_name());


  // Initial joint position
  impl_->target_pos_.data = joint_position;

  RCLCPP_INFO(impl_->ros_node_->get_logger(),
    "Lifter using gains: p:%f i:%f d:%f", impl_->pid[0], impl_->pid[1], impl_->pid[2]);

  // Listen to the update event (broadcast every simulation iteration)
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRosLifterPrivate::OnUpdate, impl_.get(), std::placeholders::_1));
}

void GazeboRosLifter::Reset()
{
  impl_->last_update_time_ = impl_->world_->SimTime();
  //impl_->last_pos = impl_->joint_->Position(0);
  //impl_->last_vel = impl_->joint_->GetVelocity(0);
  impl_->int_pos = 0.0;
}

void GazeboRosLifterPrivate::OnUpdate(const gazebo::common::UpdateInfo & _info)
{
  double dt = (_info.simTime - last_update_time_).Double();
  std::lock_guard<std::mutex> scoped_lock(lock_);


  double pos = joint_->Position(0);
  double vel = joint_->GetVelocity(0);
  double error = target_pos_.data - pos;

  // Calculate force
  double force = pid[0] * error + pid[1] * int_pos - pid[2] * vel + force_offset;

  // Integrate error and do anti-windup
  int_pos += error * dt;
  if (int_pos < -int_limit) int_pos = -int_limit;
  if (int_pos >  int_limit) int_pos =  int_limit;

  // Apply force to joint
  joint_->SetForce(0, force);


  last_update_time_ = _info.simTime;

  //RCLCPP_INFO(ros_node_->get_logger(), "% 8f % 8f % 8f % 8f % 8f % 8f % 8f % 8f % 8f", pos, vel, error, int_pos, force, target_pos_.data, pid[0], pid[1], pid[2]);

}

void GazeboRosLifterPrivate::OnLifter(const std_msgs::msg::Float32::SharedPtr _msg)
{
  std::lock_guard<std::mutex> scoped_lock(lock_);
  target_pos_ = *_msg;
}


GZ_REGISTER_MODEL_PLUGIN(GazeboRosLifter)
}  // namespace gazebo_plugins
