/*
 * Copyright 2016 Open Source Robotics Foundation
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
#include "ROSConveyorBeltPlugin.hh"
#include "conveyor_belt_sim/ConveyorBeltState.h"

#include <cstdlib>
#include <string>

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(ROSConveyorBeltPlugin);

/////////////////////////////////////////////////
ROSConveyorBeltPlugin::ROSConveyorBeltPlugin()
{
}

/////////////////////////////////////////////////
ROSConveyorBeltPlugin::~ROSConveyorBeltPlugin()
{
  this->rosnode_->shutdown();
}

/////////////////////////////////////////////////
void ROSConveyorBeltPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  // load parameters
  this->robotNamespace_ = "";
  if (_sdf->HasElement("robot_namespace"))
  {
    this->robotNamespace_ += _sdf->GetElement("robot_namespace")->Get<std::string>() + "/";
  }

  if (_sdf->HasElement("conveyor_name"))
  {
    this->robotNamespace_ += _sdf->GetElement("conveyor_name")->Get<std::string>() + "/";
  }

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized,"
        << "unable to load plugin. Load the Gazebo system plugin "
        << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  std::string controlTopic = "control";
  if (_sdf->HasElement("control_topic"))
    controlTopic = _sdf->Get<std::string>("control_topic");

  std::string stateTopic = "state";
  if (_sdf->HasElement("state_topic"))
    stateTopic = _sdf->Get<std::string>("state_topic");

  ConveyorBeltPlugin::Load(_parent, _sdf);

  this->rosnode_ = new ros::NodeHandle(this->robotNamespace_);

  this->controlService_ = this->rosnode_->advertiseService(controlTopic,
    &ROSConveyorBeltPlugin::OnControlCommand, this);

  // Message used for publishing the state of the conveyor.
  this->statePub = this->rosnode_->advertise<
    conveyor_belt_sim::ConveyorBeltState>(stateTopic, 1000);
}

/////////////////////////////////////////////////
void ROSConveyorBeltPlugin::Publish() const
{
  conveyor_belt_sim::ConveyorBeltState stateMsg;
  stateMsg.enabled = this->IsEnabled();
  stateMsg.power = this->Power();
  this->statePub.publish(stateMsg);
}

/////////////////////////////////////////////////
bool ROSConveyorBeltPlugin::OnControlCommand(ros::ServiceEvent<
  conveyor_belt_sim::ConveyorBeltControl::Request, conveyor_belt_sim::ConveyorBeltControl::Response> & event)
{
  const conveyor_belt_sim::ConveyorBeltControl::Request& req = event.getRequest();
  conveyor_belt_sim::ConveyorBeltControl::Response& res = event.getResponse();
  gzdbg << "Conveyor control service called with: " << req.power << std::endl;

  const std::string& callerName = event.getCallerName();
  gzdbg << "Conveyor control service called by: " << callerName << std::endl;

  // During the competition, this environment variable will be set.
  bool is_ariac_task_manager = callerName.compare("/gazebo") == 0;
  auto compRunning = std::getenv("ARIAC_COMPETITION");
  if (compRunning && !is_ariac_task_manager)
  {
    std::string errStr = "Competition is running so this service is not enabled.";
    gzerr << errStr << std::endl;
    ROS_ERROR_STREAM(errStr);
    res.success = false;
    return true;
  }

  if (is_ariac_task_manager && !this->IsEnabled())
  {
    ROS_WARN_STREAM("Force enabling conveyor because power set by task manager.");
    this->enabled = true;
  }

//   if (!this->IsEnabled())
//   {
//     std::string errStr = "Belt is not currently enabled so power cannot be set. It may be congested.";
//     gzerr << errStr << std::endl;
//     ROS_ERROR_STREAM(errStr);
//     res.success = false;
//     return true;
//   }

  if (!(0 == req.power || (req.power >= 50 && req.power <= 100)))
  {
    std::string errStr = "Requested belt power is invalid. Accepted values are 0 or in the range [50, 100].";
    gzerr << errStr << std::endl;
    ROS_ERROR_STREAM(errStr);
    res.success = false;
    return true;
  }

  this->SetPower(req.power);
  res.success = true;
  return true;
}
