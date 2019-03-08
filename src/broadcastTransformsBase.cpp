#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <stdio.h>
#include <string.h>

std::vector<std::string> dronesNames = {"iris_1", "iris_2", "iris_3"};

void poseCallback(const gazebo_msgs::ModelStates& msg)
{

  for (int i = 0; i < msg.name.size(); i++)
  {
    if (std::find(dronesNames.begin(), dronesNames.end(), msg.name[i]) !=
        dronesNames.end())
    {
      static tf::TransformBroadcaster br;
      tf::Transform transform;
      transform.setOrigin(tf::Vector3(msg.pose[i].position.x,
                                      msg.pose[i].position.y,
                                      msg.pose[i].position.z));
      transform.setRotation(
          tf::Quaternion(msg.pose[i].orientation.x, msg.pose[i].orientation.y,
                         msg.pose[i].orientation.z, msg.pose[i].orientation.w));
      std::string drone_name = msg.name[i];
      char ID_c = drone_name.at(msg.name[i].length() - 1);
      std::string ID(1, ID_c);
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                            "local_origin",
                                            "uav" + ID + "/base_link"));
    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "my_tf_broadcaster");

  ros::NodeHandle node;
  ros::Subscriber sub =
      node.subscribe("/gazebo/model_states", 10, &poseCallback);

  ros::spin();
  return 0;
};
