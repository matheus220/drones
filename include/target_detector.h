#ifndef TARGET_DETECTOR_H
#define TARGET_DETECTOR_H

#include <ros/ros.h>

#include <math.h>
#include <eigen3/Eigen/Eigen>
#include <eigen_conversions/eigen_msg.h>
#include <string.h>
#include <map>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/ModelStates.h>
#include <drones/EstimatedDronePosition.h>
#include <drones/EstimatedDronePositionArray.h>

namespace rosdrone_Detector
{

struct KalmanFilter
{
  KalmanFilter(int stateSize, int measSize)
    : kf(stateSize, measSize, 0, CV_32F),
      state(stateSize, 1, CV_32F),
      meas(measSize, 1, CV_32F)
  {
    // cv::Mat procNoise(stateSize, 1, type)

    cv::setIdentity(kf.transitionMatrix);

    kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, CV_32F);
    kf.measurementMatrix.at<float>(0) = 1.0f;
    kf.measurementMatrix.at<float>(6) = 1.0f;
    kf.measurementMatrix.at<float>(12) = 1.0f;

    kf.processNoiseCov = cv::Mat::zeros(stateSize, stateSize, CV_32F);
    kf.processNoiseCov.at<float>(0) = 1e-3;
    kf.processNoiseCov.at<float>(6) = 1e-3;
    kf.processNoiseCov.at<float>(12) = 1e-2;
    kf.processNoiseCov.at<float>(18) = 1e-3;
    kf.processNoiseCov.at<float>(24) = 1e-3;

    kf.measurementNoiseCov = cv::Mat::zeros(measSize, measSize, CV_32F);
    kf.measurementNoiseCov.at<float>(0) = 7e-2;
    kf.measurementNoiseCov.at<float>(4) = 7e-2;
    kf.measurementNoiseCov.at<float>(8) = 1.3;
  }
  KalmanFilter() : KalmanFilter(5,3) {}

  cv::KalmanFilter kf;
  cv::Mat state;
  cv::Mat meas;
  double last_estimation = ros::Time::now().toSec();
  double current_estimation = ros::Time::now().toSec();
  double last_measure = 0;

  bool found = false;

};

typedef std::shared_ptr<KalmanFilter> KalmanFilterPtr;

class targetDetector
{
  public:
    targetDetector(const ros::NodeHandle& ng, const ros::NodeHandle& nl);

    void detectTargets();
    inline drones::EstimatedDronePositionArray getMeasures() { return arrayOutputMessage; };

  private:
    // private functions
    void getParametersROS();
    void colorRGB2HUE(int r, int g, int b);
    void imgBGRtoimgHUE(cv::Mat& img);
    bool detectColorfulCirclesHUE(cv::Mat& img, cv::Vec3f& circle, bool write_circle = false);
    bool bestRadiusEstimation(double& radius, const cv::Mat& image, const cv::Vec3f& circle);

    void kalmanFilterProcess(const bool measure,
                             cv::Vec3f& circle,
                             const std::string &_uav_detected );
    bool getKalmanFilterStructure(KalmanFilterPtr& KFPtr,
                                  const bool measure,
                                  const std::string &_uav_detected);

    drones::EstimatedDronePosition createMessageOutput(const std::string& nameDetector,
                                                       const std::string& nameDetected,
                                                       const cv::Vec3f& circle);

    // callback functions
    void camInfoCallback(const sensor_msgs::CameraInfo& camInfo);
    void imageCallback(const sensor_msgs::ImageConstPtr& image);
    void posesCallback(const gazebo_msgs::ModelStates& poses);

    // ROS Communication
    ros::NodeHandle nhg, nhl;
    ros::Publisher imagePub, outputPub, bearingPub;
    ros::Subscriber camInfoSub, posesSub;
    image_transport::Subscriber imageSub;

    // private structures
    struct CamInfo
    {
      Eigen::Matrix3d K;
      Eigen::Matrix3d R;
      double width, height;
      bool received = false;
    } camInfo;

    struct RosParameters
    {
      std::string my_name;
      int num_uavs = 0;
      std::vector<int> uav_colors;
      std::vector<std::string> uav_names;
    } paramsROS;

    // private variables
    geometry_msgs::Pose poses_gazebo;
    drones::EstimatedDronePositionArray arrayOutputMessage;
    std::map<std::string, KalmanFilterPtr> measuresKF;

    cv::Mat img, img_processed;
    bool img_received = false;
    std::vector<int> hue_;
    int sat_ = 100;
    int val_ = 100;

    bool show_segment_ = false;
};
}

#endif // TARGET_DETECTOR_H
