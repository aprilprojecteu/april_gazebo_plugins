/*
 * Copyright 2013 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

/*
   Desc: GazeboRosVel plugin for manipulating objects in Gazebo
   Author: John Hsu
   Date: 24 Sept 2008
   
   Modified on 14.04.2021 by Oscar Lima (oscar.lima@dfki.de) to adapt it to the needs of the EU april project
   
 */

#include <algorithm>
#include <assert.h>

#include <gazebo_ros_vel.h>
#ifdef ENABLE_PROFILER
#include <ignition/common/Profiler.hh>
#endif

namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(GazeboRosVel);

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosVel::GazeboRosVel()
{
  this->vel_msg_.linear.x = 0;
  this->vel_msg_.linear.y = 0;
  this->vel_msg_.linear.z = 0;
  this->vel_msg_.angular.x = 0;
  this->vel_msg_.angular.y = 0;
  this->vel_msg_.angular.z = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosVel::~GazeboRosVel()
{
  this->update_connection_.reset();

  // Custom Callback Queue
  this->queue_.clear();
  this->queue_.disable();
  this->rosnode_->shutdown();
  this->callback_queue_thread_.join();

  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosVel::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Get the world name.
  this->world_ = _model->GetWorld();

  // load parameters
  this->robot_namespace_ = "";
  if (_sdf->HasElement("robotNamespace"))
    this->robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

  if (!_sdf->HasElement("bodyName"))
  {
    ROS_FATAL_NAMED("gazebo_ros_vel", "gazebo_ros_vel plugin missing <bodyName>, cannot proceed");
    return;
  }
  else
    this->link_name_ = _sdf->GetElement("bodyName")->Get<std::string>();

  this->link_ = _model->GetLink(this->link_name_);
  if (!this->link_)
  {
    ROS_FATAL_NAMED("gazebo_ros_vel", "gazebo_ros_vel plugin error: link named: %s does not exist\n",this->link_name_.c_str());
    return;
  }

  if (!_sdf->HasElement("topicName"))
  {
    ROS_FATAL_NAMED("gazebo_ros_vel", "gazebo_ros_vel plugin missing <topicName>, cannot proceed");
    return;
  }
  else
    this->topic_name_ = _sdf->GetElement("topicName")->Get<std::string>();


  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("gazebo_ros_vel", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

  // Custom Callback Queue
  ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::Twist>(
    this->topic_name_,1,
    boost::bind( &GazeboRosVel::UpdateObjectVel,this,_1),
    ros::VoidPtr(), &this->queue_);
  this->sub_ = this->rosnode_->subscribe(so);

  // Custom Callback Queue
  this->callback_queue_thread_ = boost::thread( boost::bind( &GazeboRosVel::QueueThread,this ) );

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboRosVel::UpdateChild, this));
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosVel::UpdateObjectVel(const geometry_msgs::Twist::ConstPtr& _msg)
{
  this->vel_msg_.linear.x = _msg->linear.x;
  this->vel_msg_.linear.y = _msg->linear.y;
  this->vel_msg_.linear.z = _msg->linear.z;
  this->vel_msg_.angular.x = _msg->angular.x;
  this->vel_msg_.angular.y = _msg->angular.y;
  this->vel_msg_.angular.z = _msg->angular.z;

//     this->link_->SetLinearVel({0, 0, 1});
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosVel::UpdateChild()
{
#ifdef ENABLE_PROFILER
  IGN_PROFILE("GazeboRosVel::OnNewFrame");
  IGN_PROFILE_BEGIN("fill ROS message");
#endif
  this->lock_.lock();
  ignition::math::Vector3d lin_vel(this->vel_msg_.linear.x,this->vel_msg_.linear.y,this->vel_msg_.linear.z);
  ignition::math::Vector3d angular_vel(this->vel_msg_.angular.x,this->vel_msg_.angular.y,this->vel_msg_.angular.z);
  this->link_->SetLinearVel(lin_vel);
  this->link_->SetAngularVel(angular_vel);
  this->lock_.unlock();
#ifdef ENABLE_PROFILER
  IGN_PROFILE_END();
#endif
}

// Custom Callback Queue
////////////////////////////////////////////////////////////////////////////////
// custom callback queue thread
void GazeboRosVel::QueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->queue_.callAvailable(ros::WallDuration(timeout));
  }
}

}
