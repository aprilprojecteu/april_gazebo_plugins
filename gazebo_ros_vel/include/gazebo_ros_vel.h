/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
 * Desc: A dynamic controller plugin that performs generic force interface.
 * Author: John Hsu
 * Date: 24 Sept 2008
 * 
 * Modified on 14.04.2021 by Oscar Lima (oscar.lima@dfki.de) to adapt it to the needs of the EU april project
 * 
 */

#ifndef GAZEBO_ROS_VEL_HH
#define GAZEBO_ROS_VEL_HH

#include <string>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <geometry_msgs/Twist.h>

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>


namespace gazebo
{
/// @addtogroup gazebo_dynamic_plugins Gazebo ROS Dynamic Plugins
/// @{
/** \defgroup GazeboRosVel Plugin XML Reference and Example

  \brief Ros Vel Plugin.

  This is a Plugin that collects data from a ROS topic and applies wrench to a body accordingly.

  Example Usage:
  \verbatim
      <gazebo>
        <plugin filename="libgazebo_ros_vel.so" name="gazebo_ros_vel">
          <bodyName>box_body</bodyName>
          <topicName>cmd_vel</topicName>
        </plugin>
      </gazebo>
  \endverbatim

\{
*/

class GazeboRosVel : public ModelPlugin
{
  /// \brief Constructor
  public: GazeboRosVel();

  /// \brief Destructor
  public: virtual ~GazeboRosVel();

  // Documentation inherited
  protected: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  // Documentation inherited
  protected: virtual void UpdateChild();

  /// \brief call back when a Wrench message is published
  /// \param[in] _msg The Incoming ROS message representing the new velocity to exert.
  private: void UpdateObjectVel(const geometry_msgs::Twist::ConstPtr& _msg);

  /// \brief The custom callback queue thread function.
  private: void QueueThread();

  /// \brief A pointer to the gazebo world.
  private: physics::WorldPtr world_;

  /// \brief A pointer to the Link, where velocity is applied
  private: physics::LinkPtr link_;

  /// \brief A pointer to the ROS node.  A node will be instantiated if it does not exist.
  private: ros::NodeHandle* rosnode_;
  private: ros::Subscriber sub_;

  /// \brief A mutex to lock access to fields that are used in ROS message callbacks
  private: boost::mutex lock_;

  /// \brief ROS Wrench topic name inputs
  private: std::string topic_name_;
  /// \brief The Link this plugin is attached to, and will exert forces on.
  private: std::string link_name_;

  /// \brief for setting ROS name space
  private: std::string robot_namespace_;

  // Custom Callback Queue
  private: ros::CallbackQueue queue_;
  /// \brief Thead object for the running callback Thread.
  private: boost::thread callback_queue_thread_;
  /// \brief Container for the twist vel that this plugin exerts on the body.
  private: geometry_msgs::Twist vel_msg_;

  // Pointer to the update event connection
  private: event::ConnectionPtr update_connection_;
};
/** \} */
/// @}
}
#endif
