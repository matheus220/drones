#ifndef OUTERLOOP_CONTROLLER_H
#define OUTERLOOP_CONTROLLER_H

#include <ros/ros.h>
#include <stdio.h>

#include <math.h>
#include <eigen3/Eigen/Eigen>
#include <eigen_conversions/eigen_msg.h>
#include <vector>
#include <tf/transform_datatypes.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
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
      outerLoopRT(const ros::NodeHandle& ng, const ros::NodeHandle& np, double mass);
      // destructor
      ~outerLoopRT();

      // major functions
      void spinControl();
      bool takeoff();
      void updateSetpoint(Eigen::Vector4d command);

      // getters
      inline bool isArmed(){return current_state.armed;}
      inline bool isOffboard(){return current_state.mode=="OFFBOARD";}

    private:

      // private functions
      void calculateControlOutput();
      void setControlOutput();
      void spinUp(double fraction);
      void getSetpointParams();

      // callback functions
      void stateCallBack(const mavros_msgs::State::ConstPtr& msg);
      void poseCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg);
      void twistCallBack(const geometry_msgs::TwistStamped::ConstPtr& msg);

      // private structures
      struct ControlParams
      {
        double maxThrust=25;
        double KPxy=0;
        double KPz=5;
        double KPyr=0;
        double KDxy=3;
        double KDz=1.5;
        double KDyr=0;
        double KIxy=0.1;
        double KIz=0.1;
        double KIyr=0;
        int order = 1;
        double dT=0;
      } controlParams;

      struct ControlStructure
      {
        double thrust=0;
        Eigen::Matrix3d R;
        Eigen::Quaterniond q;
        double vyaw;
      } controlStructure;

      struct PoseStructure
      {
        bool initialized = false;
        Eigen::Vector3d p;
        Eigen::Quaterniond q;
        Eigen::Matrix3d R;
      } pose;

      struct TwistStructure
      {
        bool initialized = false;
        Eigen::Vector3d v;
        Eigen::Vector3d omega;
      } twist;

      struct Setpoint
      {
        Eigen::Vector3d vel;
        Eigen::Vector3d pos;
        double yaw_rate;
      } setpoint;

      // ROS Communication
      ros::NodeHandle nh, nhp;
      ros::Publisher controlPub, control2Pub;
      ros::Subscriber twistSub, poseSub, stateSub;

      ros::ServiceClient set_mode_client, arming_client;
      mavros_msgs::SetMode offb_set_mode;
      mavros_msgs::CommandBool arm_cmd;

      // Messages
      geometry_msgs::PoseStamped poseMsgIn;
      geometry_msgs::TwistStamped twistMsgIn;
      mavros_msgs::State current_state;
      mavros_msgs::PositionTarget pos_target;
      geometry_msgs::Twist vel_comm;

      // private variables
      double m;
      double last_service_call = 0;
      double serviceDelay = 3;
  };
}
#endif // OUTERLOOP_CONTROLLER_H
