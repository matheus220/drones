#include "target_detector.h"
#include "outerloop_controller.h"
#include "command_creator.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "drone_operator_node");
  ros::NodeHandle nhg, nhp("~");
  ROS_INFO("Main Hover Node Launched");

  rosdrone::outerLoopRT controller(nhg);
  rosdrone::commandCreator command(nhg, nhp);

  ros::Rate rate(20.0);
  ROS_INFO("Outer loop starting at 20 hz");

  double time = ros::Time::now().toSec();

  while(!controller.takeoff())
  {
    ros::spinOnce();
    rate.sleep();
  }

  while (ros::ok())
  {
    ROS_INFO_THROTTLE(5,"Hover Node Running");
    ros::spinOnce();

    command.spinCommand();
    controller.spinControl();

    rate.sleep();
  }
}
