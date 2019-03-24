#include "ball_detector.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ball_detector_node");
  ros::NodeHandle nhg, nhp("~");

  rosdrone_Detector::ballDetector detector(nhg, nhp);

  ros::Rate rate(15.0);

  while (ros::ok())
  {
    ros::spinOnce();

    detector.spinDetector();

    rate.sleep();
  }
}
