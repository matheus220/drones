#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <drones/EstimatedDronePosition.h>
#include <drones/EstimatedDronePositionArray.h>
#include <geometry_msgs/Pose.h>
#include <eigen3/Eigen/Eigen>
#include <string.h>

struct MsgEstimatedDronePosition
{
  std::string estimator;
  std::string estimated;
  double distance;
  Eigen::Matrix3d estimatorRot;
  Eigen::Vector3d estimatorPos;
  Eigen::Vector3d bearing;
};

std::vector<MsgEstimatedDronePosition> vectorEstPosition;
float dist_arrow_to_drone = 0.25;
float length_arrow_percentage = 0.45;

void estPositionsCallback(const drones::EstimatedDronePositionArray& msg)
{
  for (int i = 0; i < msg.estPosVector.size(); i++)
  {
    MsgEstimatedDronePosition estPosition;
    estPosition.estimator = msg.estPosVector[i].estimator;
    estPosition.estimated = msg.estPosVector[i].estimated;
    estPosition.distance = msg.estPosVector[i].distance;
    estPosition.estimatorPos[0] = msg.estPosVector[i].poseEstimator.position.x;
    estPosition.estimatorPos[1] = msg.estPosVector[i].poseEstimator.position.y;
    estPosition.estimatorPos[2] = msg.estPosVector[i].poseEstimator.position.z;
    estPosition.bearing[0] = msg.estPosVector[i].bearingVector.x;
    estPosition.bearing[1] = msg.estPosVector[i].bearingVector.y;
    estPosition.bearing[2] = msg.estPosVector[i].bearingVector.z;

    Eigen::Quaterniond q(msg.estPosVector[i].poseEstimator.orientation.w,
                         msg.estPosVector[i].poseEstimator.orientation.x,
                         msg.estPosVector[i].poseEstimator.orientation.y,
                         msg.estPosVector[i].poseEstimator.orientation.z);
    estPosition.estimatorRot = q.normalized().toRotationMatrix();

    vectorEstPosition.push_back(estPosition);
  }
}

void createMessage(visualization_msgs::MarkerArray& markers)
{
  markers.markers.clear();
  for (int i = 0; i < vectorEstPosition.size(); i++)
  {
    visualization_msgs::Marker marker;

    marker.header.frame_id = "local_origin";
    marker.header.stamp = ros::Time();
    marker.ns = vectorEstPosition[i].estimator;
    marker.id = (int)vectorEstPosition[i].estimated.at(
                    vectorEstPosition[i].estimated.length() - 1) - 48;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;

    Eigen::Vector3d bearingWorldFrame;

    bearingWorldFrame =
        vectorEstPosition[i].estimatorRot * vectorEstPosition[i].bearing;

    marker.pose.position.x = vectorEstPosition[i].estimatorPos[0] +
                             bearingWorldFrame[0] * dist_arrow_to_drone;
    marker.pose.position.y = vectorEstPosition[i].estimatorPos[1] +
                             bearingWorldFrame[1] * dist_arrow_to_drone;
    marker.pose.position.z = vectorEstPosition[i].estimatorPos[2] +
                             bearingWorldFrame[2] * dist_arrow_to_drone;

    Eigen::Matrix3d bearingRotation;
    Eigen::Vector3d auxVec, col0, col1;

    auxVec = Eigen::Vector3d(0, 0, 1);
    col0 = auxVec.cross(bearingWorldFrame);
    col1 = bearingWorldFrame.cross(col0);

    bearingWorldFrame.normalize();
    col0.normalize();
    col1.normalize();

    bearingRotation.col(0) = bearingWorldFrame;
    bearingRotation.col(1) = col0;
    bearingRotation.col(2) = col1;

    Eigen::Quaterniond orientation(bearingRotation);
    // orientation.setFromTwoVectors(bearingWorldFrame, Eigen::Vector3d(1,0,0));

    orientation.normalize();

    marker.pose.orientation.x = orientation.x();
    marker.pose.orientation.y = orientation.y();
    marker.pose.orientation.z = orientation.z();
    marker.pose.orientation.w = orientation.w();
    marker.scale.x = (vectorEstPosition[i].distance - 2 * dist_arrow_to_drone) *
                     length_arrow_percentage;
    marker.scale.y = 0.03;
    marker.scale.z = 0.03;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;

    marker.lifetime = ros::Duration(0.5);

    markers.markers.push_back(marker);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "addMarkers");
  ros::NodeHandle nhg;
  ros::Rate r(60.0);
  ros::Publisher markers_pub = nhg.advertise<visualization_msgs::MarkerArray>(
      "visualization_marker_array", 5);
  ros::Subscriber est_pos_sub_1 = nhg.subscribe("/uav1/relative_bearing", 10, estPositionsCallback);
  ros::Subscriber est_pos_sub_2 = nhg.subscribe("/uav2/relative_bearing", 10, estPositionsCallback);
  ros::Subscriber est_pos_sub_3 = nhg.subscribe("/uav3/relative_bearing", 10, estPositionsCallback);

  while (ros::ok())
  {
    visualization_msgs::MarkerArray markers;

    createMessage(markers);

    // Publish the marker
    while (markers_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the markers");
      sleep(1);
    }

    markers_pub.publish(markers);
    vectorEstPosition.clear();
    ros::spinOnce();
    r.sleep();
  }
}
