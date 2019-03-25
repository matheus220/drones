#include "outerloop_controller.h"
#include "command_creator.h"

namespace rosdrone
{
  // constructor
  outerLoopRT::outerLoopRT(const ros::NodeHandle& n) : nh(n)
  {
    // initialize communications
    controlPub = nh.advertise<geometry_msgs::Twist>("mavros/setpoint_velocity/cmd_vel_unstamped",2);

    poseSub = nh.subscribe("mavros/local_position/pose",2,&outerLoopRT::poseCallBack, this);
    stateSub = nh.subscribe("mavros/state",2,&outerLoopRT::stateCallBack, this);

    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    // initialize values
    offb_set_mode.request.custom_mode = "OFFBOARD";
    arm_cmd.request.value = true;

    ROS_INFO("Drone initialized");
  }

  // destructor
  outerLoopRT::~outerLoopRT()
  {
    // destructor contents
  }

  void outerLoopRT::spinControl()
  {
    setVelocityCommand();
    setControlOutput();
    return;
  }

  bool outerLoopRT::takeoff()
  {
    outerLoopRT::spinUp(0.2);
    if(last_service_call == 0)
    {
      last_service_call=ros::Time::now().toSec();
      return 0;
    }
    if(ros::Time::now().toSec() - last_service_call < serviceDelay)
      return 0;
    else
    {
      last_service_call = ros::Time::now().toSec();

      if(!isOffboard())
      {
        if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
          ROS_INFO("Offboard enabled");
        return 0;
      }
      else if(!isArmed())
      {
        if( arming_client.call(arm_cmd) && arm_cmd.response.success)
          ROS_INFO("Armed");
        return 0;
      }
      return 1;
    }

  }

  void outerLoopRT::spinUp(double fraction)
  {
    if(fraction < 0.1)
      fraction = 0.1;
    else if(fraction > 0.35)
      fraction = 0.35;

    tf::vectorEigenToMsg(Eigen::Vector3d(0,0,0), vel_command.linear);
    tf::vectorEigenToMsg(Eigen::Vector3d(0,0,0), vel_command.angular);
    setControlOutput();
  }

  void outerLoopRT::setVelocityCommand()
  {
    if(ros::Time::now().toSec() - last_service_call > 15.0)
    {
      ROS_INFO_ONCE("Bearing control enabled!");
      vel_command = sharedTwist;
    }
    else
    {
      double u = 1.5*(2.0 - pose.p[2]);
      tf::vectorEigenToMsg(Eigen::Vector3d(0,0,u), vel_command.linear);
      tf::vectorEigenToMsg(Eigen::Vector3d(0,0,0), vel_command.angular);
    }
  }

  void outerLoopRT::setControlOutput()
  {
    controlPub.publish(vel_command);
  }

  void outerLoopRT::poseCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    poseMsgIn = *msg;

    pose.p << poseMsgIn.pose.position.x,
              poseMsgIn.pose.position.y,
              poseMsgIn.pose.position.z;
    pose.q.x()=poseMsgIn.pose.orientation.x;
    pose.q.y()=poseMsgIn.pose.orientation.y;
    pose.q.z()=poseMsgIn.pose.orientation.z;
    pose.q.w()=poseMsgIn.pose.orientation.w;
    pose.q.normalize();
    pose.R = pose.q.toRotationMatrix();
    return;
  }

  void outerLoopRT::stateCallBack(const mavros_msgs::State::ConstPtr& msg)
  {
    current_state = *msg;
    return;
  }

// end of namespace rosdrone_Controller
}
