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
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

#include <string>

// ROS
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

namespace gazebo
{
class ROSConveyorController : public WorldPlugin
{
  private: ros::NodeHandle* rosnode;
  private: ros::Publisher controlPub;
  private: ros::Subscriber sensorSub;
  private: physics::WorldPtr world;

  public: ~ROSConveyorController()
  {
    this->rosnode->shutdown();
  }

  public: void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
  {
    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    this->world = _parent;
    this->rosnode = new ros::NodeHandle("");

    // Create a subscriber for the proximity sensor output 
    std::string sensorStateChangeTopic = "sensor_output_change";
    this->sensorSub = 
      this->rosnode->subscribe(sensorStateChangeTopic, 1000,
        &ROSConveyorController::OnSensorStateChange, this);

    // Create a publisher for the conveyor control commands 
    std::string conveyorControlTopic = "conveyor_control";
    this->controlPub = this->rosnode->advertise<std_msgs::String>(conveyorControlTopic, 1, true);

    // Turn belt on
    std_msgs::String controlMsg;
    controlMsg.data = "1";
    this->controlPub.publish(controlMsg);
  }

  private: void OnSensorStateChange(const std_msgs::Bool::ConstPtr &_msg)
  {
    gzdbg << "Sensor state changed\n";

    bool sensorValue = _msg->data;
    bool controlCommand;
    std::string outputFunction = "normally_open";
    if ("normally_open" == outputFunction) {
      controlCommand = !sensorValue;
    }
    else if ("normally_closed" == outputFunction) {
      controlCommand = sensorValue;
    }
    else {
      gzerr << "output_function can only be either normally_open or normally_closed" << std::endl;
      return;
    }

    std_msgs::String controlMsg;
    controlMsg.data = std::to_string(controlCommand);
    this->controlPub.publish(controlMsg);
  }
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(ROSConveyorController)
}
