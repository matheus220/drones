#include "outerloop_controller.h"

#define G 9.81

namespace rosdrone_Controller
{
  // constructor
  outerLoopRT::outerLoopRT(const ros::NodeHandle& ng, const ros::NodeHandle& np, double mass) : nh(ng), nhp(np), m(mass)
  {
    // initialize communications
    //controlPub = nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude",2);
    //controlPub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local",2);
    controlPub = nh.advertise<geometry_msgs::Twist>("mavros/setpoint_velocity/cmd_vel_unstamped",2);


    poseSub = nh.subscribe("mavros/local_position/pose",2,&outerLoopRT::poseCallBack, this);
    twistSub = nh.subscribe("mavros/local_position/velocity",2,&outerLoopRT::twistCallBack, this);
    stateSub = nh.subscribe("mavros/state",2,&outerLoopRT::stateCallBack, this);

    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    // initialize values
    controlStructure.thrust = m * G;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    arm_cmd.request.value = true;
    pos_target.type_mask = 0b011111000111;
    pos_target.coordinate_frame = pos_target.FRAME_BODY_NED;
    //pos_target.coordinate_frame = 1;
    getSetpointParams();

    ROS_INFO("Drone initialized");
  }

  // destructor
  outerLoopRT::~outerLoopRT()
  {
    // destructor contents
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

    setpoint.vel << 0.0, 0.0, 0.0;
    setpoint.pos << 0.0, 0.0, 0.0;
    setpoint.yaw_rate = 0.0;
    setControlOutput();
  }

  void outerLoopRT::spinControl()
  {
    calculateControlOutput();
    setControlOutput();
    return;
  }

  void outerLoopRT::updateSetpoint(Eigen::Vector4d command)
  {
    setpoint.vel << command.x(), command.y(), command.z();
    setpoint.yaw_rate = command.w();
    setpoint.pos.z() = 2.0;
  }

  void outerLoopRT::calculateControlOutput()
  {
    if(ros::Time::now().toSec() - last_service_call > 10.0)
    {
      ROS_INFO_ONCE("Bearing control enabled!");
      /*auto aux = setpoint.vel.x();
      setpoint.vel.x() = -setpoint.vel.y();
      setpoint.vel.y() = aux;*/
      /*Eigen::Vector3d aux;
      aux << 1.0, 1.0, 0.0;
      setpoint.vel = aux;*/
      //setpoint.vel.z() += controlParams.KDz*(setpoint.pos.z() - pose.p[2]);
    }
    else
    {
      setpoint.vel.x() = 0;
      setpoint.vel.y() = 0;
      setpoint.vel.z() = controlParams.KDz*(setpoint.pos.z() - pose.p[2]);
      setpoint.yaw_rate = 0;
    }

  }

  void outerLoopRT::setControlOutput()
  {
    /*pos_target.header.seq++;
    pos_target.header.stamp=ros::Time::now();

    tf::vectorEigenToMsg(setpoint.vel, pos_target.velocity);
    pos_target.yaw_rate = setpoint.yaw_rate;
    controlPub.publish(pos_target);*/

    tf::vectorEigenToMsg(setpoint.vel, vel_comm.linear);
    vel_comm.angular.z = setpoint.yaw_rate;
    controlPub.publish(vel_comm);

  }

  void outerLoopRT::getSetpointParams()
  {
    std::string my_name;
    if (nhp.getParam("uav_name", my_name))
    {
      std::string path = "/uavs_info/uav_" + std::to_string(my_name[my_name.length()-1]-48) + "/setpoint/";
      nhp.getParam(path + "x", setpoint.pos[0]);
      nhp.getParam(path + "y", setpoint.pos[1]);
      nhp.getParam(path + "z", setpoint.pos[2]);
      nhp.getParam(path + "vx", setpoint.vel[0]);
      nhp.getParam(path + "vy", setpoint.vel[1]);
      nhp.getParam(path + "vz", setpoint.vel[2]);
    }
    else
      ROS_ERROR("Some ROS parameters uav_name was not found!");
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

  void outerLoopRT::twistCallBack(const geometry_msgs::TwistStamped::ConstPtr& msg)
  {
    twistMsgIn = *msg;
    twist.v <<  twistMsgIn.twist.linear.x,
                twistMsgIn.twist.linear.y,
                twistMsgIn.twist.linear.z;
    twist.omega <<  twistMsgIn.twist.angular.x,
                    twistMsgIn.twist.angular.y,
                    twistMsgIn.twist.angular.z;
    return;
  }

  void outerLoopRT::stateCallBack(const mavros_msgs::State::ConstPtr& msg)
  {
    current_state = *msg;
    return;
  }

// end of namespace rosdrone_Controller
}
