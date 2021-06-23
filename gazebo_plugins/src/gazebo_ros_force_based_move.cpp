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
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_plugins/gazebo_ros_force_based_move.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/twist.hpp>
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
class GazeboRosForceBasedMovePrivate
{
public:
  /// Callback to be called at every simulation iteration.
  /// \param[in] _info Updated simulation info.
  void OnUpdate(const gazebo::common::UpdateInfo & _info);

  /// Callback when a velocity command is received.
  /// \param[in] _msg Twist command message.
  void OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr _msg);

  /// Update odometry.
  /// \param[in] _current_time Current simulation time
  void UpdateOdometry(const gazebo::common::Time & _current_time);

  /// Publish odometry transforms
  /// \param[in] _current_time Current simulation time
  void PublishOdometryTf(const gazebo::common::Time & _current_time);

  /// A pointer to the GazeboROS node.
  gazebo_ros::Node::SharedPtr ros_node_;

  /// Subscriber to command velocities
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  /// Odometry publisher
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;

  /// To broadcast TF
  std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;

  /// Velocity received on command.
  geometry_msgs::msg::Twist target_cmd_vel_;

  /// Keep latest odometry message
  nav_msgs::msg::Odometry odom_;

  /// Pointer to world.
  gazebo::physics::WorldPtr world_;

  /// Pointer to model.
  gazebo::physics::ModelPtr model_;

  /// Pointer to link
  gazebo::physics::LinkPtr link_;

  /// Connection to event called at every world iteration.
  gazebo::event::ConnectionPtr update_connection_;

  /// Protect variables accessed on callbacks.
  std::mutex lock_;

  /// Update period in seconds.
  double update_period_;

  /// Publish period in seconds.
  double publish_period_;

  /// Last update time.
  gazebo::common::Time last_update_time_;

  /// Last publish time.
  gazebo::common::Time last_publish_time_;

  /// Odometry frame ID
  std::string odometry_frame_;

  /// Robot base frame ID
  std::string robot_base_frame_;

  /// True to publish odometry messages.
  bool publish_odom_;

  /// True to publish odom-to-world transforms.
  bool publish_odom_tf_;

  /// Control loop gains
  double omega_p_gain_, omega_i_gain_, omega_i_lim_;
  double vel_p_gain_, vel_i_gain_, vel_i_lim_;
  double torque, force_x, force_y;
  double force_lim_;
  double torque_lim_;
  /// Control loop integrals
  double wi, xi, yi;
  double last_error_x, last_error_y, last_error_w;
};

GazeboRosForceBasedMove::GazeboRosForceBasedMove()
: impl_(std::make_unique<GazeboRosForceBasedMovePrivate>())
{
}

GazeboRosForceBasedMove::~GazeboRosForceBasedMove()
{
}

void GazeboRosForceBasedMove::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  impl_->model_ = _model;

  impl_->world_ = _model->GetWorld();

  // Initialize ROS node
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  // Get QoS profiles
  const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

  // Odometry
  impl_->odometry_frame_ = _sdf->Get<std::string>("odometry_frame", "odom").first;
  impl_->robot_base_frame_ = _sdf->Get<std::string>("robot_base_frame", "base_footprint").first;

  // Link
  impl_->link_ = _model->GetLink(impl_->robot_base_frame_);

  // Update rate
  auto update_rate = _sdf->Get<double>("update_rate", 20.0).first;
  if (update_rate > 0.0) {
    impl_->update_period_ = 1.0 / update_rate;
  } else {
    impl_->update_period_ = 0.0;
  }
  impl_->last_update_time_ = impl_->world_->SimTime();

  // Publish rate
  auto publish_rate = _sdf->Get<double>("publish_rate", 20.0).first;
  if (update_rate > 0.0) {
    impl_->publish_period_ = 1.0 / publish_rate;
  } else {
    impl_->publish_period_ = 0.0;
  }
  impl_->last_publish_time_ = impl_->world_->SimTime();

  impl_->cmd_vel_sub_ = impl_->ros_node_->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", qos.get_subscription_qos("cmd_vel", rclcpp::QoS(1)),
    std::bind(&GazeboRosForceBasedMovePrivate::OnCmdVel, impl_.get(), std::placeholders::_1));

  RCLCPP_INFO(
    impl_->ros_node_->get_logger(), "Subscribed to [%s]",
    impl_->cmd_vel_sub_->get_topic_name());

  // Advertise odometry topic
  impl_->publish_odom_ = _sdf->Get<bool>("publish_odom", true).first;
  if (impl_->publish_odom_) {
    impl_->odometry_pub_ = impl_->ros_node_->create_publisher<nav_msgs::msg::Odometry>(
      "odom", qos.get_publisher_qos("odom", rclcpp::QoS(1)));

    RCLCPP_INFO(
      impl_->ros_node_->get_logger(), "Advertise odometry on [%s]",
      impl_->odometry_pub_->get_topic_name());
  }

  // Broadcast TF
  impl_->publish_odom_tf_ = _sdf->Get<bool>("publish_odom_tf", true).first;
  if (impl_->publish_odom_tf_) {
    impl_->transform_broadcaster_ =
      std::make_shared<tf2_ros::TransformBroadcaster>(impl_->ros_node_);

    RCLCPP_INFO(
      impl_->ros_node_->get_logger(),
      "Publishing odom transforms between [%s] and [%s]", impl_->odometry_frame_.c_str(),
      impl_->robot_base_frame_.c_str());
  }

  impl_->vel_p_gain_ = _sdf->Get<double>("vel_p_gain", 1000.0).first;
  impl_->vel_i_gain_ = _sdf->Get<double>("vel_i_gain", 0.0).first;
  impl_->vel_i_lim_ = _sdf->Get<double>("vel_i_lim", 1.0).first;
  impl_->force_lim_ = _sdf->Get<double>("force_lim", 1.0).first;
  impl_->omega_p_gain_ = _sdf->Get<double>("omega_p_gain", 100.0).first;
  impl_->omega_i_gain_ = _sdf->Get<double>("omega_i_gain", 0.0).first;
  impl_->omega_i_lim_ = _sdf->Get<double>("omega_i_lim", 1.0).first;
  impl_->torque_lim_ = _sdf->Get<double>("torque_lim", 1.0).first;
  impl_->force_x = 0.0;
  impl_->force_y = 0.0;
  impl_->torque = 0.0;
  impl_->xi = 0.0;
  impl_->yi = 0.0;
  impl_->wi = 0.0;
  impl_->last_error_x = 0.0;
  impl_->last_error_y = 0.0;
  impl_->last_error_w = 0.0;
  RCLCPP_INFO(impl_->ros_node_->get_logger(),
    "ForceBasedMove using gains: vel pi: %f,%f omega pi: %f,%f", 
    impl_->vel_p_gain_, impl_->vel_i_gain_, impl_->vel_i_lim_,
    impl_->omega_p_gain_, impl_->omega_i_gain_, impl_->omega_i_lim_);

  auto covariance_x = _sdf->Get<double>("covariance_x", 0.00001).first;
  auto covariance_y = _sdf->Get<double>("covariance_y", 0.00001).first;
  auto covariance_yaw = _sdf->Get<double>("covariance_yaw", 0.001).first;

  // Set covariance
  impl_->odom_.pose.covariance[0] = covariance_x;
  impl_->odom_.pose.covariance[7] = covariance_y;
  impl_->odom_.pose.covariance[14] = 1000000000000.0;
  impl_->odom_.pose.covariance[21] = 1000000000000.0;
  impl_->odom_.pose.covariance[28] = 1000000000000.0;
  impl_->odom_.pose.covariance[35] = covariance_yaw;

  impl_->odom_.twist.covariance[0] = covariance_x;
  impl_->odom_.twist.covariance[7] = covariance_y;
  impl_->odom_.twist.covariance[14] = 1000000000000.0;
  impl_->odom_.twist.covariance[21] = 1000000000000.0;
  impl_->odom_.twist.covariance[28] = 1000000000000.0;
  impl_->odom_.twist.covariance[35] = covariance_yaw;

  // Set header
  impl_->odom_.header.frame_id = impl_->odometry_frame_;
  impl_->odom_.child_frame_id = impl_->robot_base_frame_;

  // Listen to the update event (broadcast every simulation iteration)
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRosForceBasedMovePrivate::OnUpdate, impl_.get(), std::placeholders::_1));
}

void GazeboRosForceBasedMove::Reset()
{
  impl_->last_update_time_ = impl_->world_->SimTime();
  impl_->target_cmd_vel_.linear.x = 0;
  impl_->target_cmd_vel_.linear.y = 0;
  impl_->target_cmd_vel_.angular.z = 0;
}

void GazeboRosForceBasedMovePrivate::OnUpdate(const gazebo::common::UpdateInfo & _info)
{
  double dt = (_info.simTime - last_update_time_).Double();
  //RCLCPP_INFO(ros_node_->get_logger(), "%10f", dt); 

  std::lock_guard<std::mutex> scoped_lock(lock_);
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE("GazeboRosForceBasedMovePrivate::OnUpdate");
  IGN_PROFILE_BEGIN("fill ROS message");
#endif

  ignition::math::Pose3d pose = model_->WorldPose();
  ignition::math::Vector3d angular_vel = model_->RelativeAngularVel();
  ignition::math::Vector3d linear_vel = model_->RelativeLinearVel();


  double error_x = target_cmd_vel_.linear.x - linear_vel.X();
  double error_y = target_cmd_vel_.linear.y - linear_vel.Y();
  double error_w = target_cmd_vel_.angular.z - angular_vel.Z();

  // Calculate forces and limit
  force_x = vel_p_gain_ * error_x + vel_i_gain_ * xi;
  force_y = vel_p_gain_ * error_y + vel_i_gain_ * yi;
  torque  = omega_p_gain_ * error_w + omega_i_gain_ * wi;
  if (force_x > force_lim_)  force_x = force_lim_;
  if (force_y > force_lim_)  force_y = force_lim_;
  if (torque > torque_lim_)  torque  = torque_lim_;
  if (force_x < -force_lim_) force_x = -force_lim_;
  if (force_y < -force_lim_) force_y = -force_lim_;
  if (torque < -torque_lim_) torque  = -torque_lim_;

  // Integrate error
  xi += error_x * dt;
  yi += error_y * dt;
  wi += error_w * dt;
  // Anti-windup. Prevent integrator exceeding limit
  if (xi < -vel_i_lim_)   xi = -vel_i_lim_;
  if (yi < -vel_i_lim_)   yi = -vel_i_lim_;
  if (wi < -omega_i_lim_) wi = -omega_i_lim_;
  if (xi > vel_i_lim_)    xi = vel_i_lim_;
  if (yi > vel_i_lim_)    yi = vel_i_lim_;
  if (wi > omega_i_lim_)  wi = omega_i_lim_;
  // if (error_x * last_error_x < 0) xi = 0;
  // if (error_y * last_error_y < 0) yi = 0;
  // if (error_w * last_error_w < 0) wi = 0;


  // RCLCPP_INFO(ros_node_->get_logger(), "c:% 8f % 8f % 8f v:% 8f % 8f % 8f f:% 8f % 8f % 8f e:% 8f % 8f % 8f i:% 8f % 8f % 8f", 
  //   target_cmd_vel_.linear.x,
  //   target_cmd_vel_.linear.y,
  //   target_cmd_vel_.angular.z,
  //   linear_vel.X(), linear_vel.Y(), angular_vel.Z(),  
  //   force_x, force_y, torque,
  //   error_x, error_y, error_w,
  //   xi, yi, wi
  //   );

  last_update_time_ = _info.simTime;
  last_error_x = error_x;
  last_error_y = error_y;
  last_error_w = error_w;

  link_->AddRelativeTorque(ignition::math::Vector3d(0.0, 0.0, torque));
  link_->AddRelativeForce(ignition::math::Vector3d(force_x, force_y, 0.0));


#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
#endif
  if (publish_odom_ || publish_odom_tf_) {
    double seconds_since_last_publish = (_info.simTime - last_publish_time_).Double();

    if (seconds_since_last_publish < publish_period_) {
      return;
    }
#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_BEGIN("UpdateOdometry");
#endif
    UpdateOdometry(_info.simTime);
#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_END();
#endif
    if (publish_odom_) {
#ifdef IGN_PROFILER_ENABLE
      IGN_PROFILE_BEGIN("publish odometry");
#endif
      odometry_pub_->publish(odom_);
#ifdef IGN_PROFILER_ENABLE
      IGN_PROFILE_END();
#endif
    }
    if (publish_odom_tf_) {
#ifdef IGN_PROFILER_ENABLE
      IGN_PROFILE_BEGIN("publish odometryTF");
#endif
      PublishOdometryTf(_info.simTime);
#ifdef IGN_PROFILER_ENABLE
      IGN_PROFILE_END();
#endif
    }

    last_publish_time_ = _info.simTime;
  }
}

void GazeboRosForceBasedMovePrivate::OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr _msg)
{
  std::lock_guard<std::mutex> scoped_lock(lock_);
  target_cmd_vel_ = *_msg;
  //RCLCPP_INFO(ros_node_->get_logger(), "OnCmdVel %f %f %f", target_cmd_vel_.linear.x, target_cmd_vel_.linear.y, target_cmd_vel_.angular.z);
}

void GazeboRosForceBasedMovePrivate::UpdateOdometry(const gazebo::common::Time & _current_time)
{
  auto pose = model_->WorldPose();
  odom_.pose.pose = gazebo_ros::Convert<geometry_msgs::msg::Pose>(pose);

  // Get velocity in odom frame
  odom_.twist.twist.angular.z = model_->WorldAngularVel().Z();

  // Convert velocity to child_frame_id(aka base_footprint)
  auto linear = model_->WorldLinearVel();
  auto yaw = static_cast<float>(pose.Rot().Yaw());
  odom_.twist.twist.linear.x = cosf(yaw) * linear.X() + sinf(yaw) * linear.Y();
  odom_.twist.twist.linear.y = cosf(yaw) * linear.Y() - sinf(yaw) * linear.X();

  // Set timestamp
  odom_.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);
}

void GazeboRosForceBasedMovePrivate::PublishOdometryTf(const gazebo::common::Time & _current_time)
{
  geometry_msgs::msg::TransformStamped msg;
  msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);
  msg.header.frame_id = odometry_frame_;
  msg.child_frame_id = robot_base_frame_;
  msg.transform = gazebo_ros::Convert<geometry_msgs::msg::Transform>(odom_.pose.pose);

  transform_broadcaster_->sendTransform(msg);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosForceBasedMove)
}  // namespace gazebo_plugins
