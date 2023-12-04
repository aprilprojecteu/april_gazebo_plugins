#include <functional>
#include <gazebo/gui/GuiIface.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/gazebo.hh>
#include <ignition/math/Pose3.hh>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include "get_set_camera_pose/SetCameraPose.h"
#include <std_srvs/Empty.h>

namespace gazebo
{
  class GazeboGetSetCameraPose : public SystemPlugin
  {
    public: virtual ~GazeboGetSetCameraPose()
    {
      this->connections.clear();
      if (this->nh_) {
        this->nh_->shutdown();
      }
    }

    public: void Load(int /*_argc*/, char ** /*_argv*/) {}

    private: void Init() {
      this->nh_.reset(new ros::NodeHandle("~"));

      // Define service to get and set the camera pose
      this->updateService = this->nh_->advertiseService("update_camera_pose", &GazeboGetSetCameraPose::UpdateService, this);
      this->getPoseService = this->nh_->advertiseService("print_current_camera_pose", &GazeboGetSetCameraPose::GetPoseService, this);
    }

    private: bool GetPoseService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
    {
      std::cout << "Received request to get camera pose" << std::endl;

      if (!this->userCam)
      {
        this->userCam = gui::get_active_camera();
        if (!this->userCam)
        {
          ROS_ERROR("Unable to get user camera.");
          return false;
        }
      }

      ignition::math::Pose3d currentPose = this->userCam->WorldPose();
      std::cout << "Current Camera Pose:\n"
                << "Position\nx: " << currentPose.Pos().X()
                << ", y: " << currentPose.Pos().Y()
                << ", z: " << currentPose.Pos().Z()
                << "\nOrientation\nroll: " << currentPose.Rot().Roll()
                << ", pitch: " << currentPose.Rot().Pitch()
                << ", yaw: " << currentPose.Rot().Yaw()
                << "\nQuaternion\nx: " << currentPose.Rot().X()
                << ", y: " << currentPose.Rot().Y()
                << ", z: " << currentPose.Rot().Z()
                << ", w: " << currentPose.Rot().W() << std::endl;

      return true;
    }
    
    private: bool UpdateService(get_set_camera_pose::SetCameraPose::Request &req, get_set_camera_pose::SetCameraPose::Response &res)
    {
      ignition::math::Pose3d newPose(
        req.pose.position.x, 
        req.pose.position.y, 
        req.pose.position.z, 
        req.pose.orientation.w, 
        req.pose.orientation.x, 
        req.pose.orientation.y, 
        req.pose.orientation.z
      );

      this->Update(newPose);
      res.success = true;
      return true;
    }

    private: void Update(const ignition::math::Pose3d &newPose)
    {
      std::cout << "Setting custom camera pose" << std::endl;

      if (!this->userCam)
      {
        // Get a pointer to the active user camera
        this->userCam = gui::get_active_camera();
      }

      // Get scene pointer
      rendering::ScenePtr scene = rendering::get_scene();

      // Wait until the scene is initialized.
      if (!scene || !scene->Initialized())
        return;

      // Set camera pose
      this->userCam->SetWorldPose(newPose);
    }

    /// Pointer to the user camera.
    private: rendering::UserCameraPtr userCam;

    /// All the event connections.
    private: std::vector<event::ConnectionPtr> connections;

    /// ROS node handle.
    private: std::unique_ptr<ros::NodeHandle> nh_;

    /// ROS service for updating the camera pose.
    private: ros::ServiceServer updateService;
    private: ros::ServiceServer getPoseService;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_SYSTEM_PLUGIN(GazeboGetSetCameraPose)
}
