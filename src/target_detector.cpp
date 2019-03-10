#include "target_detector.h"

namespace rosdrone_Detector
{

// constructor
targetDetector::targetDetector(const ros::NodeHandle& ng, const ros::NodeHandle& nl) : nhg(ng), nhl(nl)
{
  // initialize communications
  image_transport::ImageTransport it(nhg);
  imageSub = it.subscribe("/image", 2, &targetDetector::imageCallback, this);
  camInfoSub = nhg.subscribe("/camera_info", 2, &targetDetector::camInfoCallback, this);
  posesSub = nhg.subscribe("/gazebo/model_states", 2, &targetDetector::posesCallback, this);

  imagePub = nhg.advertise<sensor_msgs::Image>("processed_image", 10);
  outputPub = nhg.advertise<drones::EstimatedDronePositionArray>("relative_bearing", 10);

  // initialize values
  sat_ = 100;
  val_ = 100;
  infoDetection.ballRadius = 0.09;
  infoDetection.t_ball2drone = Eigen::Vector3d(0,0,0.15);
  infoDetection.t_camera2drone = Eigen::Vector3d(0.07, 0.0, 0.055);
  getParametersROS();

  ROS_INFO_STREAM(paramsROS.my_name << " Target Detector initialized");
}

void targetDetector::spinDetector()
{
  if (img_received)
  {
    arrayOutputMessage.estPosVector.clear();
    for (int i = 0; i < paramsROS.uav_names.size(); i++)
    {
      colorRGB2HUE(paramsROS.uav_colors[3 * i], paramsROS.uav_colors[3 * i + 1], paramsROS.uav_colors[3 * i + 2]);

      cv::Vec3f circle;
      bool measure = detectColorfulCirclesHUE(img, circle);

      kalmanFilterProcess(measure, circle, paramsROS.uav_names[i]);

      if (measure)
      {
        arrayOutputMessage.estPosVector.push_back( createMessageOutput(paramsROS.my_name, paramsROS.uav_names[i], circle) );

        cv::Point center(std::round(circle[0]), std::round(circle[1]));
        int radius = std::round(circle[2]);
        cv::circle(img, center, radius, cv::Scalar(255, 0, 128), 2);
      }
    }
    cv::cvtColor(img, img_processed, cv::COLOR_BGR2RGB);
    imagePub.publish(cv_bridge::CvImage(std_msgs::Header(), "rgb8", img_processed).toImageMsg());
    outputPub.publish(arrayOutputMessage);
  }
}

void targetDetector::colorRGB2HUE(int r, int g, int b)
{
  hue_.clear();
  // convert color to HSV
  const float cmax = std::max(r, std::max(g, b));
  const float cmin = std::min(r, std::min(g, b));
  const float d = cmax - cmin;

  int h = 0;
  if (d)
  {
    if (cmax == r)
      h = 30 * (fmod((g - b) / d, 6));
    else if (cmax == g)
      h = 30 * ((b - r) / d + 2);
    else
      h = 30 * ((r - g) / d + 4);
  }

  // build inRange bounds for hue
  int hthr = 10;
  hue_ = {std::max(h - hthr, 0), std::min(h + hthr, 179)};

  // other segmentation for h
  if (h < hthr)
  {
    hue_.push_back(179 + h - hthr);
    hue_.push_back(179);
  }
  else if (h + hthr > 179)
  {
    hue_.push_back(0);
    hue_.push_back(h + hthr - 179);
  }
}

void targetDetector::imgBGRtoimgHUE(cv::Mat& img)
{
  // Convert input image to HSV
  cv::Mat hsv_image, hue_image;
  cv::cvtColor(img, hsv_image, cv::COLOR_BGR2HSV);

  // Threshold the HSV image, keep only the red pixels
  cv::inRange(hsv_image, cv::Scalar(hue_[0], sat_, val_), cv::Scalar(hue_[1], 255, 255), img);

  if (hue_.size() == 4)
  {
    cv::inRange(hsv_image, cv::Scalar(hue_[2], sat_, val_), cv::Scalar(hue_[3], 255, 255), hue_image);
    cv::addWeighted(img, 1.0, hue_image, 1.0, 0.0, img); // Combine the above two images
  }
}

bool targetDetector::detectColorfulCirclesHUE(cv::Mat& img,
                                              cv::Vec3f& circle,
                                              bool write_circle)
{
  if (hue_.size())
  {
    // process image
    cv::Mat hue_image = img.clone();

    imgBGRtoimgHUE(hue_image);

    cv::GaussianBlur(hue_image, hue_image, cv::Size(7, 7), 1.5, 1.5);

    // Use the Hough transform to detect circles in the combined threshold image
    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(hue_image, circles, CV_HOUGH_GRADIENT, 1, hue_image.rows / 8, 150, 20, 0, 0);

    // Loop over all detected circles and outline them on the original image
    if (circles.size() == 0) return false;

    auto largest = std::max_element(circles.begin(), circles.end(),
                                    [](const cv::Vec3f& c1, const cv::Vec3f& c2)
                                    { return c1[2] < c2[2]; });
    int idx = std::distance(circles.begin(), largest);
    circle = circles[idx];

    double new_radius;
    if (bestRadiusEstimation(new_radius, img, circle) && new_radius <= circle[2])
      circle[2] = new_radius;

    if (write_circle)
    {
      cv::Point center(std::round(circle[0]), std::round(circle[1]));
      int radius = std::round(circle[2]);
      cv::circle(img, center, radius, cv::Scalar(255, 0, 144), 2);
    }

    return true;
  }

  return false;
}

bool targetDetector::bestRadiusEstimation(double& radius,
                                          const cv::Mat& image,
                                          const cv::Vec3f& circle)
{

  int x = circle[0];
  int y = circle[1];
  int r = std::round(circle[2]);

  int startX = std::max(0, (int)std::round(x - 1.5 * r));
  int startY = std::max(0, (int)std::round(y - 1.5 * r));
  int endX = std::min(startX + 3 * r, image.cols);
  int endY = std::min(startY + 3 * r, image.rows);

  cv::Mat ROI(image, cv::Rect(startX, startY, endX - startX, endY - startY));
  cv::Mat hue_image;
  ROI.copyTo(hue_image);

  imgBGRtoimgHUE(hue_image);

  cv::GaussianBlur(hue_image, hue_image, cv::Size(3, 3), 0.3, 0.3);

  std::vector<cv::Vec3f> circles;
  cv::HoughCircles(hue_image, circles, CV_HOUGH_GRADIENT, 1, hue_image.rows / 4, 150, 25, 8, 0);

  if (circles.size())
  {
    auto largest = std::max_element(circles.begin(), circles.end(),
                                    [](const cv::Vec3f& c1, const cv::Vec3f& c2)
                                    {
                                      return c1[2] < c2[2];
                                    });
    int idx = std::distance(circles.begin(), largest);

    radius = circles[idx][2];

    return true;
  }

  return false;
}

void targetDetector::kalmanFilterProcess(const bool measure,
                                         cv::Vec3f& circle,
                                         const std::string& _uav_detected )
{
  KalmanFilterPtr KF;

  if(getKalmanFilterStructure(KF, measure, _uav_detected))
  {
    double dT;
    KF->last_estimation = KF->current_estimation;
    KF->current_estimation = ros::Time::now().toSec();
    dT = KF->current_estimation - KF->last_estimation;

    if(KF->found)
    {
      KF->kf.transitionMatrix.at<float>(3) = dT;
      KF->kf.transitionMatrix.at<float>(9) = dT;
      KF->state = KF->kf.predict();
    }

    if (!measure)
    {
      if (ros::Time::now().toSec() - KF->last_measure > 3.0)
        measuresKF.erase(_uav_detected);
    }
    else
    {
      KF->last_measure = ros::Time::now().toSec();

      KF->meas.at<float>(0) = circle[0];
      KF->meas.at<float>(1) = circle[1];
      KF->meas.at<float>(2) = circle[2];

      if (!KF->found) // First detection!
      {
        // Initialization
        KF->kf.errorCovPre.at<float>(0) = 0.01;
        KF->kf.errorCovPre.at<float>(6) = 0.01;
        KF->kf.errorCovPre.at<float>(12) = 0.01;
        KF->kf.errorCovPre.at<float>(18) = 0.001;
        KF->kf.errorCovPre.at<float>(24) = 0.001;

        KF->state.at<float>(0) = KF->meas.at<float>(0);
        KF->state.at<float>(1) = KF->meas.at<float>(1);
        KF->state.at<float>(2) = KF->meas.at<float>(2);
        KF->state.at<float>(3) = 0;
        KF->state.at<float>(4) = 0;

        KF->kf.statePost = KF->state;

        KF->found = true;
      }
      else
        KF->kf.correct(KF->meas); // Kalman Correction
    }

    circle[0] = KF->state.at<float>(0);
    circle[1] = KF->state.at<float>(1);
    circle[2] = KF->state.at<float>(2);
  }

}

bool targetDetector::getKalmanFilterStructure(KalmanFilterPtr& KFPtr,
                                              const bool measure,
                                              const std::string& _uav_detected)
{
  std::map<std::string, KalmanFilterPtr>::iterator it = measuresKF.find(_uav_detected);

  if (it == measuresKF.end())
  {
    if(measure)
      measuresKF[_uav_detected] = std::make_shared<KalmanFilter>();
    else
      return false;
  }

  KFPtr =  measuresKF[_uav_detected];
  return true;
}

drones::EstimatedDronePosition targetDetector::createMessageOutput(const std::string& nameDetector,
                                                                   const std::string& nameDetected,
                                                                   const cv::Vec3f& circle)
{
  drones::EstimatedDronePosition outputMessage;

  if (!camInfo.received) { ROS_ERROR("Camera information not found!"); return outputMessage; }

  outputMessage.estimator = nameDetector;
  outputMessage.estimated = nameDetected;
  outputMessage.distance = camInfo.K(0, 0) * infoDetection.ballRadius / circle[2];

  Eigen::Vector3d P;
  P << circle[0], circle[1], 1;

  Eigen::Vector3d bearingRaw;
  bearingRaw = camInfo.R * camInfo.K.inverse() * P;

  // Transformation from ball frame to drone frame
  Eigen::Vector3d bearing = bearingRaw * outputMessage.distance - infoDetection.t_ball2drone;

  // Transformation from camera frame to drone frame
  bearing = infoDetection.t_camera2drone + bearing;

  // Update distance
  outputMessage.distance = bearing.norm();

  bearing.normalize();

  tf::vectorEigenToMsg(bearing, outputMessage.bearingVector);

  outputMessage.poseEstimator = poses_gazebo;

  return outputMessage;
}

void targetDetector::getParametersROS()
{
  if (nhl.getParam("uav_name", paramsROS.my_name) && nhg.getParam("/uavs_info/num_uavs", paramsROS.num_uavs))
  {
    for (int i = 0; i < paramsROS.num_uavs; i++)
    {
      std::string uav_name;
      int r, g, b;
      nhg.getParam("/uavs_info/uav_" + std::to_string(i + 1) + "/name",
                   uav_name);
      if (uav_name != paramsROS.my_name)
      {
        paramsROS.uav_names.push_back(uav_name);
        nhg.getParam("/uavs_info/uav_" + std::to_string(i + 1) + "/r", r);
        nhg.getParam("/uavs_info/uav_" + std::to_string(i + 1) + "/g", g);
        nhg.getParam("/uavs_info/uav_" + std::to_string(i + 1) + "/b", b);
        paramsROS.uav_colors.push_back(r);
        paramsROS.uav_colors.push_back(g);
        paramsROS.uav_colors.push_back(b);
      }
    }
  }
  else
  {
    ROS_ERROR("Some ROS parameters were not found! (uav_name and/or "
              "/uavs_info/num_uavs)");
  }
}

// callback functions

void targetDetector::camInfoCallback(const sensor_msgs::CameraInfo& _camInfo)
{
  camInfo.K <<  _camInfo.K[0], _camInfo.K[1], _camInfo.K[2],
                _camInfo.K[3], _camInfo.K[4], _camInfo.K[5],
                _camInfo.K[6], _camInfo.K[7], _camInfo.K[8];

  camInfo.R << 0, 0, 1,
              -1, 0, 0,
              0, -1, 0;

  camInfo.width = _camInfo.width;
  camInfo.height = _camInfo.height;

  camInfo.received = true;
}

void targetDetector::imageCallback(const sensor_msgs::ImageConstPtr& image)
{
  try
  {
    img = cv_bridge::toCvCopy(image, "bgr8")->image;
    if (!img.empty())
      img_received = true;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", image->encoding.c_str());
  }
}

void targetDetector::posesCallback(const gazebo_msgs::ModelStates& poses)
{
  if (paramsROS.my_name.length() && poses.name.size())
  {
    for (int i = 0; i < poses.name.size(); i++)
    {
      if (poses.name[i] == paramsROS.my_name)
        poses_gazebo = poses.pose[i];
    }
  }
}

}
