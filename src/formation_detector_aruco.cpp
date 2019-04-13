#include <ros/ros.h>

#include <eigen3/Eigen/Eigen>
#include <eigen_conversions/eigen_msg.h>
#include <map>

#include <fiducial_msgs/FiducialTransform.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <geometry_msgs/Transform.h>
#include <drones/Formation.h>
#include <drones/FormationLink.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>

Eigen::Vector3d position;
Eigen::Quaterniond orientation;
Eigen::Matrix3d R_measure2camera;
Eigen::Matrix3d R_camera2drone;
Eigen::Vector3d t_camera2drone;
Eigen::Vector3d t_target2baseLink;
Eigen::Vector3d bearing;

drones::Formation outputMsg;
std::map<int, fiducial_msgs::FiducialTransformArray> inputsMsg;

void measureCallback1(const fiducial_msgs::FiducialTransformArray& msg)
{
  inputsMsg[1] = msg;
}

void measureCallback2(const fiducial_msgs::FiducialTransformArray& msg)
{
  inputsMsg[2] = msg;
}

void measureCallback3(const fiducial_msgs::FiducialTransformArray& msg)
{
  inputsMsg[3] = msg;
}

void processInputs()
{
  std::vector<int> matching_tags_id = {1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3};
  for(auto input : inputsMsg)
  {
    int drone_id = input.first;
    auto measures = input.second;
    if(measures.transforms.size())
    {
      std::map<int, double> fiducial_area;
      drones::FormationLink formationLink;
      formationLink.drone_name = "drone" + std::to_string(drone_id);
      for(auto transform : measures.transforms)
      {
        int target_id = matching_tags_id[transform.fiducial_id];
        geometry_msgs::Transform T = transform.transform;

        tf::vectorMsgToEigen(T.translation, position);
        tf::quaternionMsgToEigen(T.rotation, orientation);

        R_measure2camera = orientation.toRotationMatrix();

        position += R_measure2camera * t_target2baseLink;

        bearing = R_camera2drone * position + t_camera2drone;

        if(fiducial_area.count(target_id) == 0 || fiducial_area[target_id] < transform.fiducial_area)
        {
          formationLink.targets.push_back("drone" + std::to_string(target_id));

          std_msgs::Float64 distanceMsg;
          distanceMsg.data = bearing.norm();
          formationLink.distances.push_back(distanceMsg);

          geometry_msgs::Vector3 bearingMsg;
          tf::vectorEigenToMsg(bearing.normalized(), bearingMsg);
          formationLink.bearings.push_back(bearingMsg);

          fiducial_area[target_id] = transform.fiducial_area;
        }
      }
      outputMsg.drones.push_back(formationLink.drone_name);
      outputMsg.links.push_back(formationLink);
    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "formation_detector_aruco");
  ros::NodeHandle nh;

  ros::Subscriber measure_sub_1 = nh.subscribe("measures_drone1", 1, measureCallback1);
  ros::Subscriber measure_sub_2 = nh.subscribe("measures_drone2", 1, measureCallback2);
  ros::Subscriber measure_sub_3 = nh.subscribe("measures_drone3", 1, measureCallback3);

  ros::Publisher bearingPub = nh.advertise<drones::Formation>("/bearings", 1);

  R_camera2drone << 0, 0, 1, -1, 0, 0, 0, -1, 0;
  t_camera2drone << 0.07, 0.0, 0.055;
  t_target2baseLink << 0.0, -0.16, -0.085;

  ros::Rate rate(15.0);

  while (ros::ok())
  {
    ros::spinOnce();

    processInputs();

    bearingPub.publish(outputMsg);

    outputMsg.drones.clear();
    outputMsg.links.clear();
    inputsMsg.clear();

    rate.sleep();
  }
}
