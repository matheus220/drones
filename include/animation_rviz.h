#ifndef ANIMATION_RVIZ_H
#define ANIMATION_RVIZ_H

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <drones/Formation.h>
#include <drones/FormationLink.h>
#include <tf/transform_broadcaster.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <eigen3/Eigen/Eigen>
#include <eigen_conversions/eigen_msg.h>
#include <string.h>

namespace rosdrone_Animation
{

  class animationRviz{

    public:
      // constructor
      animationRviz(const ros::NodeHandle& n);
      // destructor
      ~animationRviz();

      // major functions
      void addMeasuresArrows();
      void addDesiredArrows();
      void addVelocityArrows();
      void addCentroid();
      void publishMarkers();

    private:

      // private functions
      void addMeasureMarkers(visualization_msgs::MarkerArray& markers);
      void addMarker(const int& frame_drone_ID, const int& vector_ID, const Eigen::Vector3d& vector, std::string ns, Eigen::Vector3i color, double length, double thickness);
      void setRelativeBearingDesired();

      // callback functions
      void broadcastingTransformsCallback(const gazebo_msgs::ModelStates& msg);
      void measuresCallback(const drones::Formation& msg);
      void twistCommandCallBack(const ros::MessageEvent<geometry_msgs::Twist const>& event, const int drone_ID);

      // private structures
      struct MsgEstimatedDronePosition
      {
        std::string drone_name;
        std::string target_name;
        Eigen::Vector3d bearing;
        double distance;
      };

      struct PoseStructure
      {
        Eigen::Vector3d p;
        Eigen::Quaterniond q;
        Eigen::Matrix3d R;
      };

      struct TwistStructure
      {
        bool initialized = false;
        Eigen::Vector3d v;
        Eigen::Vector3d omega;
      };

      // ROS Communication
      ros::NodeHandle nh;
      ros::Publisher markers_pub;
      ros::Subscriber poses_sub;
      std::vector<ros::Subscriber> twistSub;
      ros::Subscriber mesures_sub;

      // private variables
      std::vector<MsgEstimatedDronePosition> vectorMeasures;
      visualization_msgs::MarkerArray markers;
      std::map<int, TwistStructure> twists;
      float dist_arrow_to_drone = 0.25;
      float length_arrow_percentage = 0.45;
      std::map<int, std::map<int, Eigen::Vector3d>> relativeBearingDesired;
      std::map<int, PoseStructure> posesGazebo;
  };
}
#endif // ANIMATION_RVIZ_H
