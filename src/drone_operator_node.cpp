#include "target_detector.h"
#include "outerloop_controller.h"
#include "command_creator.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "target_detector_node");
  ros::NodeHandle nhg, nhp("~");
  ROS_INFO("Main Hover Node Launched");

  rosdrone_Detector::targetDetector detector(nhg, nhp);

  double mass = 1.5;
  rosdrone_Controller::outerLoopRT controller(nhg, nhp, mass);

  rosdrone_Command::commandCreator command(nhg, nhp);

  ros::Rate rate(15.0);
  ROS_INFO("Outer loop starting at 15 hz");

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
    detector.detectTargets();

    command.updateOwnMeasures(detector.getMeasures());
    command.spinCommand();

    controller.updateSetpoint(command.getCommand());
    controller.spinControl();

    rate.sleep();
  }
}
