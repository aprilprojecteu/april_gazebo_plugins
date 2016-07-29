/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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

#include <boost/algorithm/string/replace.hpp>
#include <string>

#include "ConveyorBeltPlugin.hh"
#include <gazebo/transport/Node.hh>
#include <gazebo/transport/Publisher.hh>
#include <ignition/math/Vector3.hh>

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(ConveyorBeltPlugin)

/////////////////////////////////////////////////
ConveyorBeltPlugin::ConveyorBeltPlugin() : SideContactPlugin()
{
}

/////////////////////////////////////////////////
ConveyorBeltPlugin::~ConveyorBeltPlugin()
{
  this->parentSensor.reset();
  this->world.reset();
}

//////////////////////////////////////////////////
std::string ConveyorBeltPlugin::Topic(std::string topicName) const
{
  std::string globalTopicName = "~/";
  globalTopicName += this->parentSensor->Name() + "/" + this->GetHandle() + "/" + topicName;
  boost::replace_all(globalTopicName, "::", "/");

  return globalTopicName;
}

/////////////////////////////////////////////////
void ConveyorBeltPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  SideContactPlugin::Load(_sensor, _sdf);

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->world->GetName());

  if (_sdf->HasElement("belt_velocity"))
  {
    this->beltVelocity = _sdf->Get<double>("belt_velocity");
  }
  else {
    this->beltVelocity = 0.5;
  }
  gzdbg << "Using belt velocity of: " << this->beltVelocity << " m/s\n";

  std::string controlCommandTopic;
  if (_sdf->HasElement("control_command_topic"))
  {
      controlCommandTopic = _sdf->Get<std::string>("control_command_topic");
  }
  else {
      controlCommandTopic = this->Topic("control_command");
  }
  this->controlCommandSub = this->node->Subscribe(controlCommandTopic,
      &ConveyorBeltPlugin::OnControlCommand, this);

  // TODO: get this from SDF
  this->velocityAxis = math::Vector3(0, 1, 0);
}

/////////////////////////////////////////////////
void ConveyorBeltPlugin::OnUpdate()
{
  auto prevNumberContactingLinks = this->contactingLinks.size();
  this->CalculateContactingLinks();
  if (prevNumberContactingLinks != this->contactingLinks.size()) {
    gzdbg << "Number of links ontop of belt: " << this->contactingLinks.size() << "\n";
  }

  std::lock_guard<std::mutex> lock(this->mutex);
  double velocity = this->beltVelocity;
  this->ActOnContactingLinks(velocity);
}

/////////////////////////////////////////////////
void ConveyorBeltPlugin::ActOnContactingLinks(double velocity)
{
  ignition::math::Vector3d velocity_beltFrame = velocity * \
    ignition::math::Vector3d(this->velocityAxis.x, this->velocityAxis.y, this->velocityAxis.z);
  auto beltPose = this->parentLink->GetWorldPose().Ign();
  math::Vector3 velocity_worldFrame = beltPose.Rot().RotateVector(velocity_beltFrame);
  for (auto linkPtr : this->contactingLinks) {
    if (linkPtr) {
      linkPtr->SetLinearVel(velocity_worldFrame);
    }
  }
}

/////////////////////////////////////////////////
void ConveyorBeltPlugin::OnControlCommand(ConstHeaderPtr& _msg)
{
  double requestedVelocity = std::stod(_msg->str_id());
  gzdbg << "Received control command of: " << requestedVelocity << "\n";
  this->SetVelocity(requestedVelocity);
}

/////////////////////////////////////////////////
void ConveyorBeltPlugin::SetVelocity(double velocity)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  gzdbg << "Setting velocity to: " << velocity << "\n";
  this->beltVelocity = velocity;
}
