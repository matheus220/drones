#ifndef COMMAND_CREATOR_H
#define COMMAND_CREATOR_H

#include <ros/ros.h>
#include <stdio.h>

#include <math.h>
#include <eigen3/Eigen/Eigen>
#include <eigen_conversions/eigen_msg.h>
#include <vector>

#include <drones/Formation.h>
#include <drones/FormationLink.h>
#include <drones/FormationControl.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

extern geometry_msgs::Twist sharedTwist;

namespace rosdrone
{
  //static geometry_msgs::Twist getCommand(){ return twist; };

  class commandCreator{

    public:
      // constructor
      commandCreator(const ros::NodeHandle& ng, const ros::NodeHandle& np);
      // destructor
      ~commandCreator();

      // major functions
      void spinCommand();

      //

    private:

      // private functions
      void getROSParameters();
      void setRelativeBearingDesired();
      void calculateVelocityCommand();
      void updateTwist();
      void publishError();
      double getYawFromQuaternion(const geometry_msgs::Quaternion& q);
      double distanceController(double distance);
      void nullSpaceMotions(Eigen::Vector3d& u, double& w);

      // callback functions
      void bearingMeasuresCallback(const drones::Formation& measures);
      void posesCallback(const gazebo_msgs::ModelStates& poses);
      void formationControlCallback(const drones::FormationControl& control);

      // private structures
      struct ControlParams
      {
        double kc=0.7;
        double kp_dist=0.15;
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
        double distDesired = 2.0;
      } distController;

      std::map<int, std::map<int, Measure>> relativeBearing;
      std::map<int, std::map<int, Eigen::Vector3d>> relativeBearingDesired;
      std::map<int, PoseStructure> posesGazebo;
      Eigen::Matrix3d S;
      Eigen::Matrix3d I;
      double start_time;
      double _rotation, _scale;
      Eigen::Vector3d _position;
      bool formation_control_active = false;

      // ROS Communication
      ros::NodeHandle nh, nhp;
      ros::Publisher errorFPub, errorDist, desiredDist;
      ros::Subscriber poseSub, bearings_sub, formationControlSub;

      // private variables
      int drone_ID;
      std::vector<std::pair<int, int> > desired_edges;
  };
}
#endif // COMMAND_CREATOR_H
