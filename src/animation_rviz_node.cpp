#include "animation_rviz.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "animation_rviz_node");
  ros::NodeHandle nh;

  rosdrone_Animation::animationRviz animation(nh);

  ros::Rate rate(60.0);

  while (ros::ok())
  {
    ros::spinOnce();

    animation.addMeasuresArrows();
    animation.addDesiredArrows();
    animation.addVelocityArrows();
    animation.addCentroid();

    animation.publishMarkers();

    rate.sleep();
  }
}
