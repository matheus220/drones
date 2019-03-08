#ifndef COMMAND_CREATOR_H
#define COMMAND_CREATOR_H

#include <ros/ros.h>
#include <stdio.h>

#include <math.h>
#include <eigen3/Eigen/Eigen>
#include <eigen_conversions/eigen_msg.h>
#include <vector>

#include <drones/EstimatedDronePosition.h>
#include <drones/EstimatedDronePositionArray.h>
#include <std_msgs/Float32.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

namespace rosdrone_Command
{
  class commandCreator{
    public:
      // constructor
      commandCreator(const ros::NodeHandle& ng, const ros::NodeHandle& np);
      // destructor
      ~commandCreator();

      // major functions
      void spinCommand();
      void updateOwnMeasures(const drones::EstimatedDronePositionArray& msg);
      Eigen::Vector4d getCommand();


    private:

      // private functions
      void getROSParameters();
      void setRelativeBearingDesired();
      void calculateVelocityCommand();
      void publishError();
      void plotDesiredBearings();
      double getYawFromQuaternion(const geometry_msgs::Quaternion& q);
      double DistanceController(double distance);
      void addMarker(const int& frame_drone_ID,
                     const int& vector_ID,
                     const Eigen::Vector3d& vector,
                     std::string ns,
                     Eigen::Vector3i color,
                     double length = -1,
                     double thickness = 0.02);

      // callback functions
      void bearingMeasuresCallback(const drones::EstimatedDronePositionArray& msg);
      void posesCallback(const gazebo_msgs::ModelStates& poses);

      // private structures
      struct ControlParams
      {
        double kc=0.7;
        double kp_dist=0.2;
        double kd_dist=0.1;
      } controlParams;

      struct VelocityCommand
      {
        Eigen::Vector3d u;
        double w;
      } velocityCommand;

      struct Measure
      {
        Eigen::Vector3d bearing;
        double yaw;
        double distance;
      };

      struct PoseStructure
      {
        Eigen::Vector3d p;
        Eigen::Quaterniond q;
        Eigen::Matrix3d R;
        double psi;
      };

      struct DistController
      {
        double last_time_measure = 0.0;
        double last_measure = 0.0;
        double distDesired = 2.0;
      } distController;

      std::map<int, std::map<int, Measure>> relativeBearing;
      std::map<int, std::map<int, Eigen::Vector3d>> relativeBearingDesired;
      std::map<int, PoseStructure> posesGazebo;
      Eigen::Matrix3d S;
      Eigen::Matrix3d I;

      // ROS Communication
      ros::NodeHandle nh, nhp;
      ros::Publisher errorFPub, errorDist, desiredDist, markers_pub;
      ros::Subscriber poseSub;
      std::vector<ros::Subscriber> bearingSub;

      // private variables
      int drone_ID;
      std::vector<std::pair<int, int> > desired_edges;

      visualization_msgs::MarkerArray markers;
  };
}
#endif // COMMAND_CREATOR_H
