#ifndef OUTERLOOP_CONTROLLER_H
#define OUTERLOOP_CONTROLLER_H

#include <ros/ros.h>
#include <stdio.h>

#include <math.h>
#include <eigen3/Eigen/Eigen>
#include <eigen_conversions/eigen_msg.h>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/PoseStamped.h>

namespace rosdrone_Controller
{
  class outerLoopRT{
    public:
      // constructor
      outerLoopRT(const ros::NodeHandle& nh);
      // destructor
      ~outerLoopRT();

      // major functions
      void spinControl();
      bool takeoff();

      // getters
      inline bool isArmed(){return current_state.armed;}
      inline bool isOffboard(){return current_state.mode=="OFFBOARD";}

    private:

      // private functions
      void controlStartingPosition();
      void setControlOutput();
      void spinUp(double fraction);

      // callback functions
      void stateCallBack(const mavros_msgs::State::ConstPtr& msg);
      void poseCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg);

      struct PoseStructure
      {
        bool initialized = false;
        Eigen::Vector3d p;
        Eigen::Quaterniond q;
        Eigen::Matrix3d R;
      } pose;

      struct Setpoint
      {
        Eigen::Vector3d vel;
        Eigen::Vector3d pos;
        double yaw_rate;
      } setpoint;

      // ROS Communication
      ros::NodeHandle nh;
      ros::Publisher controlPub, control2Pub;
      ros::Subscriber poseSub, stateSub;

      ros::ServiceClient set_mode_client, arming_client;
      mavros_msgs::SetMode offb_set_mode;
      mavros_msgs::CommandBool arm_cmd;

      // Messages
      geometry_msgs::PoseStamped poseMsgIn;
      mavros_msgs::State current_state;
      mavros_msgs::PositionTarget pos_target;
      geometry_msgs::Twist vel_comm;

      // private variables
      double last_service_call = 0;
      double serviceDelay = 3;
  };
}
#endif // OUTERLOOP_CONTROLLER_H
