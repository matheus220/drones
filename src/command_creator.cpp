#include "command_creator.h"

using namespace std;

namespace rosdrone_Command
{
// constructor
commandCreator::commandCreator(const ros::NodeHandle& ng, const ros::NodeHandle& np) : nh(ng), nhp(np)
{
  // initialize values
  getROSParameters();
  desired_edges = { {1, 2}, {1,3}, {2,1}, {3,2} };
  S <<  0, -1, 0, 1, 0, 0, 0, 0, 0;
  I = Eigen::Matrix3d::Identity();
  setRelativeBearingDesired();

  // initialize communications
  for(auto edge : desired_edges)
  {
    if(edge.second == drone_ID)
    {
      std::string subsName = "/uav" + std::to_string(edge.first) + "/relative_bearing";
      ros::Subscriber aux = nh.subscribe(subsName, 10, &commandCreator::bearingMeasuresCallback, this);
      bearingSub.push_back(aux);
    }
  }
  poseSub = nh.subscribe("/gazebo/model_states", 2, &commandCreator::posesCallback, this);
  markers_pub = nh.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 5);
  if(drone_ID == 2) errorFPub = nh.advertise<std_msgs::Float32>("errorF", 2);
  if(drone_ID == 1) errorDist = nh.advertise<std_msgs::Float32>("errorDist", 2);
  if(drone_ID == 1) desiredDist = nh.advertise<std_msgs::Float32>("desiredDist", 2);

  ROS_INFO("Command initialized");
}

// destructor
commandCreator::~commandCreator()
{
  // destructor contents
}

void commandCreator::spinCommand()
{
  calculateVelocityCommand();
  publishError();
  plotDesiredBearings();
  return;
}

void commandCreator::updateTwist()
{
  twist.linear.x = velocityCommand.u.x();
  twist.linear.y = velocityCommand.u.y();
  twist.linear.z = velocityCommand.u.z();

  twist.angular.x = 0;
  twist.angular.y = 0;
  twist.angular.z = velocityCommand.w;
}

void commandCreator::updateOwnMeasures(const drones::EstimatedDronePositionArray& msg)
{
  bearingMeasuresCallback(msg);
}

void commandCreator::publishError()
{
  std_msgs::Float32 sum;
  sum.data = 0;
  for (auto& drone_measures : relativeBearing)
  {
    int i = drone_measures.first;
    for(auto& measure : drone_measures.second)
    {
      int j = measure.first;
      if(relativeBearingDesired[i].count(j))
        sum.data += (measure.second.bearing - relativeBearingDesired[i][j]).norm();
    }
  }
  if(drone_ID == 2) errorFPub.publish(sum);

  std_msgs::Float32 dist;
  dist.data = relativeBearing[1][2].distance;
  if(drone_ID == 1) errorDist.publish(dist);

  std_msgs::Float32 desDist;
  desDist.data = 2.0;
  if(drone_ID == 1) desiredDist.publish(desDist);
}

void commandCreator::setRelativeBearingDesired()
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

void commandCreator::calculateVelocityCommand()
{
  Eigen::Matrix3d Rij;

  auto my_measures = relativeBearing[drone_ID];

  int i = drone_ID;
  Eigen::Vector3d u2 = Eigen::Vector3d::Zero();
  Eigen::Vector3d u = Eigen::Vector3d::Zero();
  double w = 0;

  for(auto measure : my_measures)
  {
    int j = measure.first;
    if(relativeBearingDesired[i].count(j))
    {
      Eigen::Vector3d bearing_ij = measure.second.bearing;
      u -= controlParams.kc * (I - bearing_ij*bearing_ij.transpose()) * relativeBearingDesired[i][j];
      w += controlParams.kc * bearing_ij.transpose() * S * relativeBearingDesired[i][j];
      if(i == 1 && j == 2) u -= DistanceController(measure.second.distance) * bearing_ij;
    }
  }

  for (auto drone_measures : relativeBearing)
  {
    if(drone_measures.first != i && drone_measures.second.count(i))
    {
      int j = drone_measures.first;
      if(relativeBearingDesired[j].count(i))
      {
        Eigen::Vector3d bearing_ji = drone_measures.second[i].bearing;
        Rij = Eigen::AngleAxisd(posesGazebo[j].psi - posesGazebo[i].psi, Eigen::Vector3d::UnitZ());
        u += controlParams.kc * Rij * (I - bearing_ji*bearing_ji.transpose()) * relativeBearingDesired[j][i];
        if(i == 2 && j == 1) u += DistanceController(drone_measures.second[i].distance) * Rij * bearing_ji;
      }
    }
  }

  velocityCommand = {u, w};
  updateTwist();
}

double commandCreator::DistanceController(double distance)
{
  if (distController.last_time_measure == 0)
  {
    distController.last_time_measure = ros::Time::now().toSec();
    distController.last_measure = distance;
    return 0;
  }

  double time = ros::Time::now().toSec();
  double dT = time - distController.last_time_measure;
  distController.last_time_measure = time;

  double e = distController.distDesired - distance;
  double e_dot = (distance - distController.last_measure) / dT;

  distController.last_measure = distance;

  return controlParams.kp_dist * e - controlParams.kd_dist * e_dot;
}

void commandCreator::getROSParameters()
{
  std::string my_name;
  int num_uavs;
  if (nhp.getParam("uav_name", my_name) && nh.getParam("/uavs_info/num_uavs", num_uavs))
  {
    drone_ID = my_name[my_name.length() - 1] - 48;
  }
  else
  {
    ROS_ERROR("Some ROS parameters were not found! (uav_name and/or "
              "/uavs_info/num_uavs)");
  }
}

void commandCreator::plotDesiredBearings()
{
  for(auto measure : relativeBearingDesired[drone_ID])
  {
    if (relativeBearing[drone_ID].find(measure.first) != relativeBearing[drone_ID].end())
    {
      Eigen::Vector3i color((measure.first == 1), (measure.first == 2), (measure.first == 3));
      addMarker(drone_ID, measure.first, measure.second, "desired", color, 1.5, 0.05);
    }
  }
  markers_pub.publish(markers);
  markers.markers.clear();
}

void commandCreator::addMarker(const int& frame_drone_ID, const int& vector_ID, const Eigen::Vector3d& vector, std::string ns, Eigen::Vector3i color, double length, double thickness)
{
  float dist_arrow_to_drone = 0.25;
  float weight_distance = 0.45;

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
  tf::vectorEigenToMsg(Eigen::Vector3d(weight_distance*(length-2*dist_arrow_to_drone), thickness, thickness), marker.scale);

  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = color.x();
  marker.color.g = color.y();
  marker.color.b = color.z();

  marker.lifetime = ros::Duration(0.5);

  markers.markers.push_back(marker);
}

double commandCreator::getYawFromQuaternion(const geometry_msgs::Quaternion& q)
{
  double siny_cosp = +2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return atan2(siny_cosp, cosy_cosp);
}

void commandCreator::bearingMeasuresCallback(const drones::EstimatedDronePositionArray& msg)
{
  for (auto measure : msg.estPosVector)
  {
    int drone_id;
    int measured_id;
    drone_id = measure.estimator[measure.estimator.length()-1] - 48;
    measured_id = measure.estimated[measure.estimated.length()-1] - 48;

    Eigen::Vector3d bearing;
    tf::vectorMsgToEigen(measure.bearingVector, bearing);

    relativeBearing[drone_id][measured_id].bearing = bearing;
    relativeBearing[drone_id][measured_id].distance = measure.distance;
  }
}

void commandCreator::posesCallback(const gazebo_msgs::ModelStates& poses)
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
      posesGazebo[id].psi = getYawFromQuaternion(poses.pose[i].orientation);
    }
  }
}
// end of namespace rosdrone_Command
}
