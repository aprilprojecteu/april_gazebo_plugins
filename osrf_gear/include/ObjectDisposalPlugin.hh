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
/*
 * Desc: Object disposal plugin
 * Author: Deanna Hood
 */
#ifndef _GAZEBO_OBJECT_DISPOSAL_PLUGIN_HH_
#define _GAZEBO_OBJECT_DISPOSAL_PLUGIN_HH_

#include <string>

#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/util/system.hh>

namespace gazebo
{
  /// \brief A plugin for a contact sensor attached to an model disposal unit.
  class GAZEBO_VISIBLE ObjectDisposalPlugin : public SensorPlugin
  {
    /// \brief Constructor.
    public: ObjectDisposalPlugin();

    /// \brief Destructor.
    public: virtual ~ObjectDisposalPlugin();

    /// \brief Load the sensor plugin.
    /// \param[in] _sensor Pointer to the sensor that loaded this plugin.
    /// \param[in] _sdf SDF element that describes the plugin.
    public: virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

    /// \brief Callback that recieves the contact sensor's update signal.
    protected: void OnUpdate();

    /// \brief Pointer to the contact sensor
    protected: sensors::ContactSensorPtr parentSensor;

    /// \brief Connection that maintains a link between the contact sensor's
    /// updated signal and the OnUpdate callback.
    protected: event::ConnectionPtr updateConnection;

    /// \brief Pointer to the world
    protected: physics::WorldPtr world;

    /// \brief Pointer to this node for publishing/subscribing
    protected: transport::NodePtr node;

    /// \brief Name of the collision of the link
    protected: std::string collisionName;

    /// \brief Pointer to the link
    protected: physics::LinkPtr link;

    /// \brief Set of pointers to links which have collisions with the belt
    protected: std::set<physics::ModelPtr> contactingModels;

    /// \brief Determine which models are ontop of the sensor's link
    protected: void CalculateContactingModels();

    /// \brief Act on models that are ontop of the sensor's link
    protected: void ActOnContactingModels();
  };
}
#endif
