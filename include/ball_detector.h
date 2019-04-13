#ifndef BALL_DETECTOR_H
#define BALL_DETECTOR_H

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
#include <drones/Formation.h>
#include <drones/FormationLink.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>

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
    kf.processNoiseCov.at<float>(0) = 1e-4;
    kf.processNoiseCov.at<float>(6) = 1e-4;
    kf.processNoiseCov.at<float>(12) = 1e-4;
    kf.processNoiseCov.at<float>(18) = 1e-4;
    kf.processNoiseCov.at<float>(24) = 1e-4;

    kf.measurementNoiseCov = cv::Mat::zeros(measSize, measSize, CV_32F);
    kf.measurementNoiseCov.at<float>(0) = 1e-4;
    kf.measurementNoiseCov.at<float>(4) = 1e-4;
    kf.measurementNoiseCov.at<float>(8) = 5e-3;
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

class ballDetector
{
  public:
    ballDetector(const ros::NodeHandle& ng, const ros::NodeHandle& nl);

    void spinDetector();

  private:
    // private functions
    void getParametersROS();
    void colorRGB2HUE(int r, int g, int b);
    void imgBGRtoimgHUE(cv::Mat& img);
    bool detectColorfulCirclesHUE(cv::Mat& img, cv::Vec3f& circle, bool write_circle = false);
    bool bestRadiusEstimation(double& radius, const cv::Mat& image, const cv::Vec3f& circle);

    std::vector<cv::Point> findMainContour(const cv::Mat &_im);
    bool process(const cv::Mat &_im, cv::Vec3f& circle, bool write_output = false);

    void kalmanFilterProcess(const bool measure,
                             cv::Vec3f& circle,
                             const int& uav_detected_id);
    bool getKalmanFilterStructure(KalmanFilterPtr& KFPtr,
                                  const bool measure,
                                  const int& uav_detected_id);

    bool addMeasureToOutput(const int& target_id, const cv::Vec3f& circle);

    // callback functions
    void camInfoCallback(const sensor_msgs::CameraInfo& camInfo);
    void imageCallback(const sensor_msgs::ImageConstPtr& image);

    // ROS Communication
    ros::NodeHandle nhg, nhl;
    ros::Publisher imagePub, bearingPub;
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
      int drone_ID;
      int num_uavs = 0;
      std::map<int, std::vector<int>> drones_color;
    } paramsROS;

    struct InfoDetection
    {
      Eigen::Vector3d t_ball2drone;
      Eigen::Vector3d t_camera2drone;
      double ballRadius;
    } infoDetection;

    // private variables
    drones::FormationLink outputMessage;
    std::map<int, KalmanFilterPtr> measuresKF;

    cv::Mat img, img_processed;
    bool img_received = false;
    std::vector<int> hue_;
    bool show_segment_ = false, show_output_ = false;
    int sat_ = 100;
    int val_ = 100;
};
}

#endif // BALL_DETECTOR_H
