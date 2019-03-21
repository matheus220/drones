#include "animation_rviz.h"

namespace rosdrone_Animation
{
// constructor
animationRviz::animationRviz(const ros::NodeHandle& n) : nh(n)
{
  markers_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 5);

  for(int i = 1; i <=3 ; i++)
  {
    twistSub.push_back(
          nh.subscribe<geometry_msgs::Twist>(
            "/uav" + std::to_string(i) + "/mavros/setpoint_velocity/cmd_vel_unstamped", 2,
            boost::bind(&animationRviz::twistCommandCallBack, this, _1, i))
          );
  }
  mesures_sub = nh.subscribe("/bearings", 2, &animationRviz::measuresCallback, this);

  poses_sub = nh.subscribe("/gazebo/model_states", 10, &animationRviz::broadcastingTransformsCallback, this);

  setRelativeBearingDesired();
}

// destructor
animationRviz::~animationRviz()
{
  // destructor contents
}

void animationRviz::setRelativeBearingDesired()
{
  std::vector<Eigen::Vector3d> drones;
  drones.emplace_back(-1, 0, 0);
  drones.emplace_back(sqrt(2)/2, sqrt(2)/2, 0);
  drones.emplace_back(sqrt(2)/2, -sqrt(2)/2, 0);

  Eigen::Matrix3d Rtemp;
  std::vector<Eigen::Matrix3d> R;
  Rtemp = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ());
  R.push_back(Rtemp);
  Rtemp = Eigen::AngleAxisd(M_PI*7/8, Eigen::Vector3d::UnitZ());
  R.push_back(Rtemp);
  Rtemp = Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitZ());
  R.push_back(Rtemp);

  std::vector<std::pair<int, int> > pairs = { {1, 2}, {1,3}, {2,1}, {3,2} };

  for(auto pair : pairs)
    relativeBearingDesired[pair.first][pair.second] =
        (R[pair.first-1]*(drones[pair.second-1] - drones[pair.first-1])).normalized();
}

void animationRviz::addMarker(const int& frame_drone_ID, const int& vector_ID, const Eigen::Vector3d& vector, std::string ns, Eigen::Vector3i color, double length, double thickness)
{
  visualization_msgs::Marker marker;

  marker.header.frame_id = "local_origin";
  marker.header.stamp = ros::Time();
  marker.ns = ns + "_" + std::to_string(frame_drone_ID);
  marker.id = vector_ID;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;

  Eigen::Vector3d bearingWorldFrame;

  bearingWorldFrame = posesGazebo[frame_drone_ID].R * vector;

  tf::pointEigenToMsg(bearingWorldFrame * dist_arrow_to_drone + posesGazebo[frame_drone_ID].p, marker.pose.position);

  Eigen::Matrix3d bearingRotation;
  Eigen::Vector3d auxVec, col0, col1;

  auxVec = Eigen::Vector3d(0, 0, 1);
  col0 = auxVec.cross(bearingWorldFrame);
  col1 = bearingWorldFrame.cross(col0);

  bearingRotation.col(0) = bearingWorldFrame.normalized();
  bearingRotation.col(1) = col0.normalized();
  bearingRotation.col(2) = col1.normalized();

  Eigen::Quaterniond orientation(bearingRotation);

  tf::quaternionEigenToMsg(orientation.normalized(), marker.pose.orientation);

  if(length == -1) length = vector.norm();
  tf::vectorEigenToMsg(Eigen::Vector3d(length_arrow_percentage*(length-2*dist_arrow_to_drone), thickness, thickness), marker.scale);

  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = color.x();
  marker.color.g = color.y();
  marker.color.b = color.z();

  marker.lifetime = ros::Duration(0.5);

  markers.markers.push_back(marker);
}

void animationRviz::addMeasuresArrows()
{
  for (int i = 0; i < vectorMeasures.size(); i++)
  {
    int id_drone = vectorMeasures[i].drone_name.at(vectorMeasures[i].drone_name.length() - 1) - 48 - 3;
    int id_target = vectorMeasures[i].target_name.at(vectorMeasures[i].target_name.length() - 1) - 48 - 3;
    Eigen::Vector3i color(1, 0, 1);
    addMarker(id_drone, id_target, vectorMeasures[i].bearing, "measure", color, vectorMeasures[i].distance, 0.03);
  }
}

void animationRviz::addDesiredArrows()
{
  for(auto measures_drone : relativeBearingDesired)
  {
    for(auto measure : measures_drone.second)
    {
      Eigen::Vector3i color((measure.first == 1), (measure.first == 2), (measure.first == 3));
      addMarker(measures_drone.first, measure.first, measure.second, "desired", color, 1.5, 0.05);
    }
  }
}

void animationRviz::addVelocityArrows()
{
  for(auto twist : twists)
  {
      Eigen::Vector3i color(1, 1, 0);
      addMarker(twist.first, 1, twist.second.v, "velocity", color, 1.5, 0.02);
  }
}

void animationRviz::addCentroid()
{
  Eigen::Vector3d translation, centroid;
  for (auto drone : posesGazebo)
  {
    centroid += drone.second.p;
  }
  centroid /= posesGazebo.size();

  visualization_msgs::Marker marker;

  marker.header.frame_id = "local_origin";
  marker.header.stamp = ros::Time();
  marker.ns = "centroid";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;

  tf::pointEigenToMsg(centroid, marker.pose.position);
  tf::quaternionEigenToMsg(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0), marker.pose.orientation);
  tf::vectorEigenToMsg(Eigen::Vector3d(0.12,0.12,0.12), marker.scale);

  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 0.82;
  marker.color.g = 0.38;
  marker.color.b = 0.21;

  marker.lifetime = ros::Duration(0.5);

  markers.markers.push_back(marker);

  translation << 0,0,0;

  Eigen::Vector3i color(0, 1, 1);
  addMarker(1, 1, translation, "tranlation", color, -1, 0.02);
}

void animationRviz::publishMarkers()
{
  if(markers_pub.getNumSubscribers() > 0)
  {
    markers_pub.publish(markers);
    vectorMeasures.clear();
    markers.markers.clear();
  }
  else
  {
    ROS_WARN_ONCE("Please create a subscriber to the markers");
  }
}

void animationRviz::broadcastingTransformsCallback(const gazebo_msgs::ModelStates& poses)
{
  for (int i = 0; i < poses.name.size(); i++)
  {
    if (poses.name[i].find("iris_") != std::string::npos)
    {
      int id = poses.name[i].at(poses.name[i].length() - 1) - 48;

      tf::pointMsgToEigen(poses.pose[i].position, posesGazebo[id].p);
      tf::quaternionMsgToEigen(poses.pose[i].orientation, posesGazebo[id].q);
      posesGazebo[id].q.normalize();
      posesGazebo[id].R = posesGazebo[id].q.toRotationMatrix();

      static tf::TransformBroadcaster br;
      tf::Transform transform;
      transform.setOrigin(tf::Vector3(poses.pose[i].position.x,
                                      poses.pose[i].position.y,
                                      poses.pose[i].position.z));
      transform.setRotation(
          tf::Quaternion(poses.pose[i].orientation.x, poses.pose[i].orientation.y,
                         poses.pose[i].orientation.z, poses.pose[i].orientation.w));

      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                            "local_origin",
                                            "uav" + std::to_string(id) + "/base_link"));
    }
  }
}

void animationRviz::measuresCallback(const formation_control_lib::Formation& measures)
{
  MsgEstimatedDronePosition measure;
  for (int i=0; i<measures.links.size(); i++)
  {
    measure.drone_name = measures.links[i].drone_name;
    for (int j=0; j<measures.links[i].targets.size(); j++)
    {
      measure.target_name = measures.links[i].targets[j];

      Eigen::Vector3d bearing;
      tf::vectorMsgToEigen(measures.links[i].bearings[j], measure.bearing);

      measure.distance = measures.links[i].distances[j].data;

      vectorMeasures.push_back(measure);
    }
  }
}

void animationRviz::twistCommandCallBack(const ros::MessageEvent<geometry_msgs::Twist const>& event, const int drone_ID)
{
  const geometry_msgs::Twist::ConstPtr& msg = event.getMessage();
  auto twistMsgIn = *msg.get();
  twists[drone_ID].v <<   twistMsgIn.linear.x,
                          twistMsgIn.linear.y,
                          twistMsgIn.linear.z;
  twists[drone_ID].omega <<   twistMsgIn.angular.x,
                              twistMsgIn.angular.y,
                              twistMsgIn.angular.z;
  return;
}


// end of namespace rosdrone_Animation
}
